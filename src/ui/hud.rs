use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use std::f32::consts::PI;

use crate::camera::CameraMode;
use crate::controllers::{
    ActiveController, AscentController, ControllerKind, LeaderRef, LevelHoldController,
    ModelLibrary, OrbitController, OrbitDirection, PlaneTuning, SelectedModel,
    SelectedTuningProfile,
};
use crate::plane::{ControlInputs, FlightState, PlaneIndex, PlaneTuningHandle};

#[allow(unused_variables, unused_mut)]
pub fn draw_flight_hud(
    mode: Res<CameraMode>,
    mut contexts: EguiContexts,
    mut plane_query: Query<(
        Entity,
        &FlightState,
        &ControlInputs,
        &mut ControllerKind,
        &mut ActiveController,
        Option<&mut SelectedTuningProfile>,
        Option<&PlaneTuningHandle>,
        Option<&mut SelectedModel>,
        Option<&mut LeaderRef>,
    )>,
    all_planes: Query<(Entity, &PlaneIndex), With<FlightState>>,
    tuning_assets: Res<Assets<PlaneTuning>>,
    model_lib: Res<ModelLibrary>,
) {
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
        profile,
        tuning_handle,
        mut selected_model,
        mut leader_ref,
    )) = result
    else {
        return;
    };

    let Ok(ctx) = contexts.ctx_mut() else { return };

    let mut pairs: Vec<(Entity, u32)> = all_planes.iter().map(|(e, idx)| (e, idx.0)).collect();
    pairs.sort_by_key(|&(_, i)| i);
    let camera_label = match *mode {
        CameraMode::FreeLook => "Camera: Free Look".to_string(),
        CameraMode::Follow(entity) => {
            let n = pairs
                .iter()
                .position(|&(e, _)| e == entity)
                .map(|i| i + 1)
                .unwrap_or(0);
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

            if *kind == ControllerKind::Wingman {
                if let Some(ref mut lref) = leader_ref {
                    let current_leader = lref.0;
                    let mut selected_leader = current_leader;
                    let leader_label = pairs
                        .iter()
                        .find(|&&(e, _)| e == current_leader)
                        .map(|&(_, idx)| format!("Plane {}", idx))
                        .unwrap_or_else(|| "Unknown".to_string());
                    egui::ComboBox::from_label("Leader")
                        .selected_text(&leader_label)
                        .show_ui(ui, |ui| {
                            for &(entity, idx) in &pairs {
                                if entity == current_entity {
                                    continue;
                                }
                                ui.selectable_value(
                                    &mut selected_leader,
                                    entity,
                                    format!("Plane {}", idx),
                                );
                            }
                        });
                    if selected_leader != current_leader {
                        lref.0 = selected_leader;
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
                if let (Some(ref mut profile), Some(handle)) = (profile, tuning_handle) {
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
            }

            #[cfg(feature = "training")]
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

            #[cfg(feature = "training")]
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

            #[cfg(feature = "training")]
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

            #[cfg(feature = "training")]
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
            }
        });
}

/// Returns the filename stem from a path like `"models/level_hold/ppo_level_hold"`.
#[cfg(feature = "training")]
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
