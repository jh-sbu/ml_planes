use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use std::f32::consts::PI;

use crate::camera::CameraMode;
use crate::controllers::{
    ActiveController, AscentController, ControllerKind, HeadingHoldController, L1Controller,
    L1Status, LevelHoldController, ModelLibrary, OrbitController, OrbitDirection, PlaneTuning,
    SelectedModel, SelectedTuningProfile, WingmanController,
};
use crate::plane::{
    ControlInputs, FlightState, PlaneConfig, PlaneConfigHandle, PlaneId, PlaneIndex,
    PlaneTuningHandle, Powerplant,
};
use crate::ui::file_load::{self, PendingLoads};
use crate::ui::map::MapState;

#[allow(unused_variables, unused_mut)]
pub fn draw_flight_hud(
    mode: Res<CameraMode>,
    map: Res<MapState>,
    mut contexts: EguiContexts,
    mut plane_query: Query<(
        Entity,
        &FlightState,
        &ControlInputs,
        &mut ControllerKind,
        // Optional: a networked client renders replicated planes, which carry
        // neither the (server-only) `ActiveController` nor a `PlaneConfigHandle`.
        // The flight readout is driven entirely by replicated state; the per-kind
        // control panels and the fuel readout below degrade gracefully when absent.
        Option<&mut ActiveController>,
        Option<&mut SelectedTuningProfile>,
        Option<&PlaneTuningHandle>,
        Option<&mut SelectedModel>,
        Option<&PlaneConfigHandle>,
    )>,
    all_planes: Query<(Entity, &PlaneId, &PlaneIndex), With<FlightState>>,
    plane_configs: Res<Assets<PlaneConfig>>,
    tuning_assets: Res<Assets<PlaneTuning>>,
    model_lib: Res<ModelLibrary>,
    mut pending: ResMut<PendingLoads>,
) {
    // The full-screen map replaces the 3D view (and this HUD) while open.
    if map.open {
        return;
    }

    // Determine which entity to display
    let result = match *mode {
        CameraMode::Follow(entity) => plane_query.get_mut(entity).ok(),
        CameraMode::FreeLook => plane_query.iter_mut().next(),
    };

    let Some((
        current_entity,
        state,
        inputs,
        mut kind,
        mut controller,
        mut profile,
        tuning_handle,
        mut selected_model,
        config_handle,
    )) = result
    else {
        return;
    };

    let Ok(ctx) = contexts.ctx_mut() else { return };

    let mut pairs: Vec<(Entity, PlaneId, u32)> = all_planes
        .iter()
        .map(|(e, pid, idx)| (e, *pid, idx.0))
        .collect();
    pairs.sort_by_key(|&(_, _, i)| i);
    let camera_label = match *mode {
        CameraMode::FreeLook => "Camera: Free Look".to_string(),
        CameraMode::Follow(entity) => {
            let n = camera_follow_index(&pairs, entity);
            format!("Camera: Follow Plane {}", n)
        }
    };

    egui::Window::new("Flight Data")
        .anchor(egui::Align2::LEFT_TOP, egui::vec2(10.0, 10.0))
        .collapsible(false)
        .show(ctx, |ui| {
            ui.label(&camera_label);
            ui.separator();
            let knots = state.airspeed * 1.944;
            ui.label(format!(
                "Airspeed:  {:.1} m/s  ({:.0} kts)",
                state.airspeed, knots
            ));
            ui.label(format!("Altitude:  {:.1} m", state.altitude));
            ui.label(format!(
                "Position:  ({:.1}, {:.1}, {:.1})",
                state.position.x, state.position.y, state.position.z
            ));

            let alpha_deg = state.alpha * 180.0 / PI;
            let beta_deg = state.beta * 180.0 / PI;
            ui.label(format!("Alpha:     {:.1}°", alpha_deg));
            ui.label(format!("Beta:      {:.1}°", beta_deg));

            let av = state.angular_velocity;
            let p_deg = av.x * 180.0 / PI;
            let q_deg = av.y * 180.0 / PI;
            let r_deg = av.z * 180.0 / PI;
            ui.label(format!(
                "p/q/r:     {:.1} / {:.1} / {:.1} °/s",
                p_deg, q_deg, r_deg
            ));

            // Fuel / charge — label and units depend on the plane's powerplant.
            // `config_handle` is absent on a networked client (config isn't
            // replicated), so the readout is simply omitted there.
            if let Some(cfg) = config_handle.and_then(|h| plane_configs.get(&h.0)) {
                let cap = cfg.powerplant.capacity();
                let rem = state.consumable_remaining;
                if rem.is_finite() && cap > 0.0 {
                    let frac = (rem / cap).clamp(0.0, 1.0);
                    let pct = frac * 100.0;
                    ui.separator();
                    match cfg.powerplant {
                        Powerplant::JetFuel { fuel_type, .. } => {
                            ui.label(format!(
                                "Fuel:      {:.0} / {:.0} kg ({:.0}%)  [{}]",
                                rem,
                                cap,
                                pct,
                                fuel_type.label()
                            ));
                        }
                        Powerplant::Electric { .. } => {
                            ui.label(format!(
                                "Charge:    {:.1} / {:.1} kWh ({:.0}%)",
                                rem, cap, pct
                            ));
                        }
                    }
                    add_surface_bar(ui, "Level", frac, 0.0..=1.0);
                }
            }

            ui.separator();

            ui.label("Controls:");
            add_surface_bar(ui, "Aileron", inputs.aileron, -1.0..=1.0);
            add_surface_bar(ui, "Elevator", inputs.elevator, -1.0..=1.0);
            add_surface_bar(ui, "Rudder", inputs.rudder, -1.0..=1.0);
            add_surface_bar(ui, "Throttle", inputs.throttle, 0.0..=1.0);

            ui.separator();
            let current = *kind;
            let mut selected = current;
            egui::ComboBox::from_label("Controller")
                .selected_text(selected.name())
                .show_ui(ui, |ui| {
                    for &k in ControllerKind::ALL {
                        ui.selectable_value(&mut selected, k, k.name());
                    }
                });
            if selected != current {
                *kind = selected;
            }
            ui.label("(C to cycle)");

            // Everything below edits the live controller. On a networked client the
            // `ActiveController` isn't replicated, so skip the per-kind panels and
            // the RL model picker — the flight data above is already live. (Phase 6
            // converts these into network commands; see `plans/client_server.md`.)
            let Some(mut controller) = controller else {
                return;
            };

            if *kind == ControllerKind::Wingman {
                if let Some(wc) = controller
                    .0
                    .as_any_mut()
                    .downcast_mut::<WingmanController>()
                {
                    let current_leader_id = wc.leader_id;
                    let mut selected_leader_id = current_leader_id;
                    let leader_label = pairs
                        .iter()
                        .find(|&&(_, pid, _)| pid == current_leader_id)
                        .map(|&(_, _, idx)| format!("Plane {}", idx))
                        .unwrap_or_else(|| "Unknown".to_string());
                    egui::ComboBox::from_label("Leader")
                        .selected_text(&leader_label)
                        .show_ui(ui, |ui| {
                            for &(entity, pid, idx) in &pairs {
                                if entity == current_entity {
                                    continue;
                                }
                                ui.selectable_value(
                                    &mut selected_leader_id,
                                    pid,
                                    format!("Plane {}", idx),
                                );
                            }
                        });
                    if selected_leader_id != current_leader_id {
                        wc.leader_id = selected_leader_id;
                    }

                    let d = &wc.diagnostics;
                    if d.leader_found {
                        ui.label(format!("Pos error: {:.1} m", d.pos_error_mag));
                        ui.label(format!("  Cross-track: {:+.1} m", d.cross_track));
                        ui.label(format!("  Fore-aft:    {:+.1} m", d.range_error));
                        ui.label(format!("  Vertical:    {:+.1} m", d.altitude_error));
                    } else {
                        ui.label("Leader: lost (holding)");
                    }
                }
            }

            if *kind == ControllerKind::Ascent {
                if let Some(ascent) = controller.0.as_any_mut().downcast_mut::<AscentController>() {
                    let prev = ascent.target_altitude;
                    ui.horizontal(|ui| {
                        ui.label("Target Alt:");
                        ui.add(
                            egui::DragValue::new(&mut ascent.target_altitude)
                                .speed(10.0)
                                .range(100.0..=15000.0)
                                .suffix(" m"),
                        );
                    });
                    if ascent.target_altitude != prev {
                        ascent.complete = false;
                    }
                    let status = if ascent.complete {
                        "Complete"
                    } else {
                        "Climbing"
                    };
                    ui.label(format!("Status: {}", status));
                }
            }

            if *kind == ControllerKind::LevelHold {
                if let Some(lh) = controller
                    .0
                    .as_any_mut()
                    .downcast_mut::<LevelHoldController>()
                {
                    ui.horizontal(|ui| {
                        ui.label("Target Alt:");
                        ui.add(
                            egui::DragValue::new(&mut lh.target_altitude)
                                .speed(10.0)
                                .range(100.0..=15000.0)
                                .suffix(" m"),
                        );
                    });
                    let tgt_kts = lh.target_airspeed * 1.944;
                    ui.horizontal(|ui| {
                        ui.label("Target Spd:");
                        ui.add(
                            egui::DragValue::new(&mut lh.target_airspeed)
                                .speed(1.0)
                                .range(30.0..=200.0)
                                .suffix(" m/s"),
                        );
                        ui.label(format!("({:.0} kts)", tgt_kts));
                    });
                }
                if let (Some(ref mut profile), Some(handle)) =
                    (profile.as_mut().map(|p| p.reborrow()), tuning_handle)
                {
                    if let Some(pt) = tuning_assets.get(&handle.0) {
                        let mut profiles: Vec<&str> =
                            pt.level_hold.keys().map(|s| s.as_str()).collect();
                        profiles.sort();
                        let current_profile = profile.0.clone();
                        let mut selected_profile = current_profile.clone();
                        egui::ComboBox::from_label("Tune Profile")
                            .selected_text(&selected_profile)
                            .show_ui(ui, |ui| {
                                for &p in &profiles {
                                    ui.selectable_value(&mut selected_profile, p.to_string(), p);
                                }
                            });
                        if selected_profile != current_profile {
                            profile.0 = selected_profile;
                        }
                        ui.label("(T / Shift+T to cycle)");
                    }
                }
                if ui.button("Load tuning…").clicked() {
                    file_load::spawn_tuning_load(current_entity, false, &mut pending);
                }
            }

            if *kind == ControllerKind::HeadingHold {
                if let Some(hh) = controller
                    .0
                    .as_any_mut()
                    .downcast_mut::<HeadingHoldController>()
                {
                    // Display current heading in degrees for reference.
                    let speed_xz = (state.velocity.x.powi(2) + state.velocity.z.powi(2)).sqrt();
                    let cur_heading_deg = if speed_xz > 1.0 {
                        state.velocity.z.atan2(state.velocity.x).to_degrees()
                    } else {
                        0.0
                    };
                    ui.label(format!("Heading: {:.1}°", cur_heading_deg));

                    // Target heading: edit in degrees, store in radians.
                    let mut tgt_deg = hh.target_heading.to_degrees();
                    ui.horizontal(|ui| {
                        ui.label("Target Hdg:");
                        ui.add(egui::DragValue::new(&mut tgt_deg).speed(1.0).suffix("°"));
                    });
                    hh.target_heading = tgt_deg.to_radians();

                    ui.horizontal(|ui| {
                        ui.label("Target Alt:");
                        ui.add(
                            egui::DragValue::new(&mut hh.inner.target_altitude)
                                .speed(10.0)
                                .range(100.0..=15000.0)
                                .suffix(" m"),
                        );
                    });
                    let tgt_kts = hh.inner.target_airspeed * 1.944;
                    ui.horizontal(|ui| {
                        ui.label("Target Spd:");
                        ui.add(
                            egui::DragValue::new(&mut hh.inner.target_airspeed)
                                .speed(1.0)
                                .range(30.0..=200.0)
                                .suffix(" m/s"),
                        );
                        ui.label(format!("({:.0} kts)", tgt_kts));
                    });
                }
                if let (Some(ref mut profile), Some(handle)) =
                    (profile.as_mut().map(|p| p.reborrow()), tuning_handle)
                {
                    if let Some(pt) = tuning_assets.get(&handle.0) {
                        let mut profiles: Vec<&str> =
                            pt.heading_hold.keys().map(|s| s.as_str()).collect();
                        profiles.sort();
                        let current_profile = profile.0.clone();
                        let mut selected_profile = current_profile.clone();
                        egui::ComboBox::from_label("Tune Profile")
                            .selected_text(&selected_profile)
                            .show_ui(ui, |ui| {
                                for &p in &profiles {
                                    ui.selectable_value(&mut selected_profile, p.to_string(), p);
                                }
                            });
                        if selected_profile != current_profile {
                            profile.0 = selected_profile;
                        }
                        ui.label("(T / Shift+T to cycle)");
                    }
                }
                if ui.button("Load tuning…").clicked() {
                    file_load::spawn_tuning_load(current_entity, false, &mut pending);
                }
            }

            if *kind == ControllerKind::Orbit {
                if let Some(orbit) = controller.0.as_any_mut().downcast_mut::<OrbitController>() {
                    if draw_orbit_controls(
                        ui,
                        state,
                        &mut orbit.center_x,
                        &mut orbit.center_z,
                        &mut orbit.target_radius,
                        &mut orbit.target_altitude,
                        &mut orbit.target_airspeed,
                        &mut orbit.direction,
                    ) {
                        orbit.radial_pid.reset();
                        orbit.heading_pid.reset();
                    }
                }
                if let (Some(ref mut profile), Some(handle)) =
                    (profile.as_mut().map(|p| p.reborrow()), tuning_handle)
                {
                    if let Some(pt) = tuning_assets.get(&handle.0) {
                        let mut profiles: Vec<&str> = pt.orbit.keys().map(|s| s.as_str()).collect();
                        profiles.sort();
                        let current_profile = profile.0.clone();
                        let mut selected_profile = current_profile.clone();
                        egui::ComboBox::from_label("Tune Profile")
                            .selected_text(&selected_profile)
                            .show_ui(ui, |ui| {
                                for &p in &profiles {
                                    ui.selectable_value(&mut selected_profile, p.to_string(), p);
                                }
                            });
                        if selected_profile != current_profile {
                            profile.0 = selected_profile;
                        }
                        ui.label("(T / Shift+T to cycle)");
                    }
                }
                if ui.button("Load tuning…").clicked() {
                    file_load::spawn_tuning_load(current_entity, true, &mut pending);
                }
            }

            if *kind == ControllerKind::FlightPlan {
                ui.separator();
                if let Some(l1) = controller.0.as_any_mut().downcast_mut::<L1Controller>() {
                    ui.label(format!("Leg {} / {}", l1.leg_index + 1, l1.plan.legs.len()));
                    match l1.status {
                        L1Status::Waypoint {
                            x,
                            z,
                            distance,
                            capture_radius,
                            eta,
                            xtrack,
                        } => {
                            ui.label(format!("Seeking: Waypoint ({:.0}, {:.0})", x, z));
                            ui.label(format!(
                                "Distance: {:.0} m  (capture {:.0} m)",
                                distance, capture_radius
                            ));
                            ui.label(format!("Eta (η):  {:.1}°", eta.to_degrees()));
                            ui.label(format!("Cross-track: {:+.0} m", xtrack));
                        }
                        L1Status::Orbit {
                            center_x,
                            center_z,
                            radius,
                            radial_error,
                            direction,
                            turns_done,
                            turns_total,
                        } => {
                            let dir = match direction {
                                OrbitDirection::Clockwise => "CW",
                                OrbitDirection::CounterClockwise => "CCW",
                            };
                            ui.label(format!(
                                "Seeking: Orbit ({:.0}, {:.0})  r={:.0} m  {}",
                                center_x, center_z, radius, dir
                            ));
                            let turns = match turns_total {
                                Some(t) => format!("{:.2} / {:.1}", turns_done, t),
                                None => format!("{:.2} / ∞", turns_done),
                            };
                            ui.label(format!("Turns: {}", turns));
                            ui.label(format!("Radial err: {:+.0} m", radial_error));
                        }
                        L1Status::Finished => {
                            ui.label("Plan complete — holding level");
                        }
                    }
                } else {
                    // Pre-swap: still the PID-orbit fallback until apply_flight_plan
                    // installs the real L1Controller once the .plan.ron asset loads.
                    ui.label("Loading flight plan…");
                }
            }

            #[cfg(feature = "inference")]
            if *kind == ControllerKind::RlLevelHold {
                use crate::controllers::RlLevelHoldController;

                // During the transition frames before the RL model is loaded the active
                // controller may still be a LevelHoldController (fallback from build()); try
                // both so targets are always editable regardless of which is present.
                if let Some(rl) = controller
                    .0
                    .as_any_mut()
                    .downcast_mut::<RlLevelHoldController>()
                {
                    ui.horizontal(|ui| {
                        ui.label("Target Alt:");
                        ui.add(
                            egui::DragValue::new(&mut rl.target_altitude)
                                .speed(10.0)
                                .range(100.0..=15000.0)
                                .suffix(" m"),
                        );
                    });
                    let tgt_kts = rl.target_airspeed * 1.944;
                    ui.horizontal(|ui| {
                        ui.label("Target Spd:");
                        ui.add(
                            egui::DragValue::new(&mut rl.target_airspeed)
                                .speed(1.0)
                                .range(30.0..=200.0)
                                .suffix(" m/s"),
                        );
                        ui.label(format!("({:.0} kts)", tgt_kts));
                    });
                } else if let Some(lh) = controller
                    .0
                    .as_any_mut()
                    .downcast_mut::<LevelHoldController>()
                {
                    ui.horizontal(|ui| {
                        ui.label("Target Alt:");
                        ui.add(
                            egui::DragValue::new(&mut lh.target_altitude)
                                .speed(10.0)
                                .range(100.0..=15000.0)
                                .suffix(" m"),
                        );
                    });
                    let tgt_kts = lh.target_airspeed * 1.944;
                    ui.horizontal(|ui| {
                        ui.label("Target Spd:");
                        ui.add(
                            egui::DragValue::new(&mut lh.target_airspeed)
                                .speed(1.0)
                                .range(30.0..=200.0)
                                .suffix(" m/s"),
                        );
                        ui.label(format!("({:.0} kts)", tgt_kts));
                    });
                }
            }

            #[cfg(feature = "inference")]
            if *kind == ControllerKind::RlOrbit {
                use crate::controllers::RlOrbitController;

                if let Some(rl) = controller
                    .0
                    .as_any_mut()
                    .downcast_mut::<RlOrbitController>()
                {
                    draw_orbit_controls(
                        ui,
                        state,
                        &mut rl.center_x,
                        &mut rl.center_z,
                        &mut rl.target_radius,
                        &mut rl.target_altitude,
                        &mut rl.target_airspeed,
                        &mut rl.direction,
                    );
                } else if let Some(orbit) =
                    controller.0.as_any_mut().downcast_mut::<OrbitController>()
                {
                    if draw_orbit_controls(
                        ui,
                        state,
                        &mut orbit.center_x,
                        &mut orbit.center_z,
                        &mut orbit.target_radius,
                        &mut orbit.target_altitude,
                        &mut orbit.target_airspeed,
                        &mut orbit.direction,
                    ) {
                        orbit.radial_pid.reset();
                        orbit.heading_pid.reset();
                    }
                }
            }

            #[cfg(feature = "inference")]
            if *kind == ControllerKind::RlOrbitResidual {
                use crate::controllers::RlOrbitResidualController;

                if let Some(rl) = controller
                    .0
                    .as_any_mut()
                    .downcast_mut::<RlOrbitResidualController>()
                {
                    draw_orbit_controls(
                        ui,
                        state,
                        &mut rl.center_x,
                        &mut rl.center_z,
                        &mut rl.target_radius,
                        &mut rl.target_altitude,
                        &mut rl.target_airspeed,
                        &mut rl.direction,
                    );
                } else if let Some(orbit) =
                    controller.0.as_any_mut().downcast_mut::<OrbitController>()
                {
                    if draw_orbit_controls(
                        ui,
                        state,
                        &mut orbit.center_x,
                        &mut orbit.center_z,
                        &mut orbit.target_radius,
                        &mut orbit.target_altitude,
                        &mut orbit.target_airspeed,
                        &mut orbit.direction,
                    ) {
                        orbit.radial_pid.reset();
                        orbit.heading_pid.reset();
                    }
                }
            }

            #[cfg(feature = "inference")]
            if *kind == ControllerKind::RlLstmOrbit {
                use crate::controllers::RlLstmOrbitController;

                if let Some(rl) = controller
                    .0
                    .as_any_mut()
                    .downcast_mut::<RlLstmOrbitController>()
                {
                    draw_orbit_controls(
                        ui,
                        state,
                        &mut rl.center_x,
                        &mut rl.center_z,
                        &mut rl.target_radius,
                        &mut rl.target_altitude,
                        &mut rl.target_airspeed,
                        &mut rl.direction,
                    );
                } else if let Some(orbit) =
                    controller.0.as_any_mut().downcast_mut::<OrbitController>()
                {
                    if draw_orbit_controls(
                        ui,
                        state,
                        &mut orbit.center_x,
                        &mut orbit.center_z,
                        &mut orbit.target_radius,
                        &mut orbit.target_altitude,
                        &mut orbit.target_airspeed,
                        &mut orbit.direction,
                    ) {
                        orbit.radial_pid.reset();
                        orbit.heading_pid.reset();
                    }
                }
            }

            #[cfg(feature = "inference")]
            if let Some(dir_key) = kind.model_dir() {
                if let Some(ref mut sel) = selected_model {
                    if let Some(available) = model_lib.0.get(dir_key) {
                        let current_path = sel.0.clone();
                        let mut chosen = if available.iter().any(|p| p == &current_path) {
                            current_path.clone()
                        } else {
                            available
                                .first()
                                .cloned()
                                .unwrap_or_else(|| current_path.clone())
                        };
                        egui::ComboBox::from_label("Model")
                            .selected_text(path_stem(&chosen))
                            .show_ui(ui, |ui| {
                                for path in available {
                                    ui.selectable_value(&mut chosen, path.clone(), path_stem(path));
                                }
                            });
                        if chosen != current_path {
                            sel.0 = chosen;
                        }
                        ui.label("(T / Shift+T to cycle)");
                    }
                }
                #[cfg(not(target_arch = "wasm32"))]
                if ui.button("Load model…").clicked() {
                    file_load::spawn_model_load(current_entity, &mut pending);
                }
            }
        });
}

/// Display number for the followed plane: its stable `PlaneIndex` (the `u32` in
/// each `pairs` tuple), **not** its position in the sorted list. Using the index
/// keeps the top-HUD camera label consistent with the bottom Planes panel, the
/// map, and the leader combo when indices are non-contiguous (e.g. after a
/// lower-indexed plane is removed). Returns `0` if `entity` is not live.
fn camera_follow_index(pairs: &[(Entity, PlaneId, u32)], entity: Entity) -> u32 {
    pairs
        .iter()
        .find(|&&(e, _, _)| e == entity)
        .map(|&(_, _, idx)| idx)
        .unwrap_or(0)
}

/// Returns the filename stem from a path like `"models/level_hold/ppo_level_hold"`.
#[cfg(feature = "inference")]
fn path_stem(path: &str) -> &str {
    path.rsplit('/').next().unwrap_or(path)
}

fn draw_orbit_controls(
    ui: &mut egui::Ui,
    state: &FlightState,
    center_x: &mut f32,
    center_z: &mut f32,
    target_radius: &mut f32,
    target_altitude: &mut f32,
    target_airspeed: &mut f32,
    direction: &mut OrbitDirection,
) -> bool {
    ui.horizontal(|ui| {
        ui.label("Center X:");
        ui.add(
            egui::DragValue::new(center_x)
                .speed(10.0)
                .range(-50_000.0..=50_000.0)
                .suffix(" m"),
        );
    });
    ui.horizontal(|ui| {
        ui.label("Center Z:");
        ui.add(
            egui::DragValue::new(center_z)
                .speed(10.0)
                .range(-50_000.0..=50_000.0)
                .suffix(" m"),
        );
    });
    ui.horizontal(|ui| {
        ui.label("Radius:");
        ui.add(
            egui::DragValue::new(target_radius)
                .speed(10.0)
                .range(500.0..=20_000.0)
                .suffix(" m"),
        );
    });
    ui.horizontal(|ui| {
        ui.label("Target Alt:");
        ui.add(
            egui::DragValue::new(target_altitude)
                .speed(10.0)
                .range(100.0..=15000.0)
                .suffix(" m"),
        );
    });
    let tgt_kts = *target_airspeed * 1.944;
    ui.horizontal(|ui| {
        ui.label("Target Spd:");
        ui.add(
            egui::DragValue::new(target_airspeed)
                .speed(1.0)
                .range(30.0..=200.0)
                .suffix(" m/s"),
        );
        ui.label(format!("({:.0} kts)", tgt_kts));
    });
    let dir_label = match *direction {
        OrbitDirection::Clockwise => "CW",
        OrbitDirection::CounterClockwise => "CCW",
    };
    let direction_changed = if ui.button(format!("Dir: {}", dir_label)).clicked() {
        *direction = match *direction {
            OrbitDirection::Clockwise => OrbitDirection::CounterClockwise,
            OrbitDirection::CounterClockwise => OrbitDirection::Clockwise,
        };
        true
    } else {
        false
    };
    let rx = state.position.x - *center_x;
    let rz = state.position.z - *center_z;
    let r = (rx * rx + rz * rz).sqrt();
    ui.label(format!("Radius err: {:.1} m", r - *target_radius));
    direction_changed
}

fn add_surface_bar(
    ui: &mut egui::Ui,
    label: &str,
    value: f32,
    range: std::ops::RangeInclusive<f32>,
) {
    ui.horizontal(|ui| {
        ui.label(format!("{:>8}:", label));
        let mut v = value;
        ui.add(egui::Slider::new(&mut v, range).show_value(true));
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entity(i: u32) -> Entity {
        Entity::from_raw_u32(i).expect("valid test entity index")
    }

    #[test]
    fn camera_follow_index_uses_plane_index_not_list_position() {
        // State after plane #1 was removed: live indices are non-contiguous [2, 3].
        let e2 = entity(20);
        let e3 = entity(30);
        let pairs = vec![(e2, PlaneId(2), 2u32), (e3, PlaneId(3), 3u32)];

        // e2 is first in the (sorted) list but its stable index is 2, not 1.
        assert_eq!(camera_follow_index(&pairs, e2), 2);
        assert_eq!(camera_follow_index(&pairs, e3), 3);
    }

    #[test]
    fn camera_follow_index_missing_entity_returns_zero() {
        let pairs = vec![(entity(20), PlaneId(2), 2u32)];
        assert_eq!(camera_follow_index(&pairs, entity(99)), 0);
    }
}
