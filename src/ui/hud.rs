use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use std::f32::consts::PI;

use crate::camera::CameraMode;
use crate::controllers::{
    ActiveController, AscentController, ControllerKind, ControllerTargets, ControllerTelemetry,
    L1Controller, L1Status, ModelLibrary, OrbitDirection, OrbitParams, PlaneTuning, SelectedModel,
    SelectedTuningProfile, WingmanController,
};
use crate::plane::{
    ControlInputs, FlightState, PlaneConfig, PlaneConfigHandle, PlaneId, PlaneIndex,
    PlaneTuningHandle, Powerplant,
};
use crate::ui::file_load::{self, PendingLoads};
use crate::ui::map::MapState;

// On the networked client the HUD sends commands instead of mutating local
// (replicated) components.
#[cfg(all(feature = "net", feature = "inference"))]
use crate::net::SetModelCommand;
#[cfg(feature = "net")]
use crate::net::{SetControllerTargetsCommand, SetTuningProfileCommand, SwitchControllerCommand};
#[cfg(feature = "net")]
use bevy_replicon::prelude::ClientTriggerExt;

/// Client-side shadow of an in-flight controller-target edit. While it is live (and
/// matches the followed entity + the replicated value's variant) the HUD draws its
/// target-editor widgets against this instead of the freshly-replicated
/// `ControllerTargets` snapshot, so a snapshot arriving mid-drag can't snap a
/// `DragValue` backwards. Only meaningful on the networked client — the local-sim
/// path edits the live controller directly and never touches this resource.
// Only meaningfully read/written on a networked client (`cfg(feature = "net")`
// branches below); on a non-net local-sim build the fields/methods/helper below are
// legitimately unused, exactly like `commands`/`pending_target` in `draw_flight_hud`
// (see its `#[allow(unused_variables, unused_mut)]`).
#[cfg_attr(not(feature = "net"), allow(dead_code))]
#[derive(Resource, Default)]
pub struct PendingTargetEdit {
    entity: Option<Entity>,
    targets: ControllerTargets,
    last_sent: ControllerTargets,
    last_edit_secs: f64,
}

/// How long an edited value is trusted over the replicated snapshot before falling
/// back — comfortably longer than a round trip plus the ~2-tick client render delay
/// (`NetInterpolation`), so by expiry the server's echo should have caught up.
#[cfg_attr(not(feature = "net"), allow(dead_code))]
const TARGET_EDIT_HOLD_SECS: f64 = 0.75;

#[cfg_attr(not(feature = "net"), allow(dead_code))]
impl PendingTargetEdit {
    /// Record a fresh edit for `entity` at time `now`.
    fn record(&mut self, entity: Entity, targets: ControllerTargets, now: f64) {
        self.entity = Some(entity);
        self.targets = targets;
        self.last_edit_secs = now;
    }

    /// `true` (and updates `last_sent`) iff `targets` differs from the last value
    /// sent. Guards against an out-of-range replicated value — e.g. an orbit radius
    /// a `DragValue::range()` would clamp — causing the widget to report `changed()`
    /// (and thus resend the same command) on every single frame forever.
    fn take_if_new(&mut self, targets: ControllerTargets) -> bool {
        if self.last_sent == targets {
            return false;
        }
        self.last_sent = targets;
        true
    }
}

/// Which value the target-editor widgets should be drawn against this frame: the
/// pending edit (if it's still within its hold window, for the same entity, and the
/// same `ControllerTargets` variant as the replicated snapshot), or `replicated`
/// itself. A variant mismatch means the server rebuilt the controller into a
/// different kind since the edit — the replicated value must win. Pure, so it's
/// unit-tested without egui.
#[cfg_attr(not(feature = "net"), allow(dead_code))]
fn seed_targets(
    replicated: ControllerTargets,
    pending: &PendingTargetEdit,
    entity: Entity,
    now: f64,
) -> ControllerTargets {
    let live = pending.entity == Some(entity)
        && now - pending.last_edit_secs < TARGET_EDIT_HOLD_SECS
        && std::mem::discriminant(&pending.targets) == std::mem::discriminant(&replicated);
    if live {
        pending.targets
    } else {
        replicated
    }
}

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
        // Replicated read-only controller status — the only telemetry source on a
        // networked client (where `ActiveController` isn't replicated).
        Option<&ControllerTelemetry>,
        // Replicated editable setpoints — the seed for the target-editor widgets on
        // a networked client (where `ActiveController` isn't replicated either).
        Option<&ControllerTargets>,
    )>,
    all_planes: Query<(Entity, &PlaneId, &PlaneIndex), With<FlightState>>,
    plane_configs: Res<Assets<PlaneConfig>>,
    tuning_assets: Res<Assets<PlaneTuning>>,
    model_lib: Res<ModelLibrary>,
    mut pending: ResMut<PendingLoads>,
    mut pending_target: ResMut<PendingTargetEdit>,
    real_time: Res<Time<Real>>,
    // Used only by the networked client to send commands; unused in local-sim
    // (covered by the `#[allow(unused_variables, unused_mut)]` on this system).
    mut commands: Commands,
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
        telemetry,
        replicated_targets,
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
                // Local sim mutates the component (SimControlPlugin rebuilds); the
                // networked client asks the authoritative server to switch.
                #[cfg(not(feature = "net"))]
                {
                    *kind = selected;
                }
                #[cfg(feature = "net")]
                if let Some(plane) = plane_id_of(&pairs, current_entity) {
                    commands.client_trigger(SwitchControllerCommand {
                        plane,
                        kind: selected,
                    });
                }
            }
            ui.label("(C to cycle)");

            // Networked client: the tuning/model combos below live inside the
            // `ActiveController` block, which never runs on a client (that component
            // isn't replicated). Draw command-sending equivalents here instead,
            // reading the replicated selection + client-reconstructed handles.
            #[cfg(feature = "net")]
            net_selection_combos(
                ui,
                &pairs,
                current_entity,
                *kind,
                profile.as_deref(),
                tuning_handle,
                &tuning_assets,
                #[cfg(feature = "inference")]
                selected_model.as_deref(),
                #[cfg(feature = "inference")]
                &model_lib,
                &mut commands,
            );

            // Networked client: `ActiveController` isn't replicated, so the per-kind
            // panels below never run. Draw the read-only controller status from the
            // replicated `ControllerTelemetry` snapshot instead (orbit radial error,
            // L1 leg/status, wingman formation errors, ascent progress).
            #[cfg(feature = "net")]
            if controller.is_none() {
                draw_replicated_telemetry(ui, telemetry);
            }

            // Controller-target editor: one widget body serves both the local-sim
            // path (edits the live controller directly, below) and the networked
            // client (shadows the edit locally and sends a `SetControllerTargetsCommand`
            // to the authoritative server).
            let now = real_time.elapsed_secs_f64();
            let mut edited = match controller.as_ref() {
                Some(c) => c.0.targets(),
                None => {
                    #[cfg(feature = "net")]
                    {
                        seed_targets(
                            replicated_targets.copied().unwrap_or_default(),
                            &pending_target,
                            current_entity,
                            now,
                        )
                    }
                    #[cfg(not(feature = "net"))]
                    {
                        ControllerTargets::None
                    }
                }
            };
            if draw_controller_targets(ui, state, &pairs, current_entity, &mut edited) {
                match controller.as_mut() {
                    Some(c) => c.0.apply_targets(&edited, state),
                    None =>
                    {
                        #[cfg(feature = "net")]
                        if let Some(plane) = plane_id_of(&pairs, current_entity) {
                            pending_target.record(current_entity, edited, now);
                            if pending_target.take_if_new(edited) {
                                commands.client_trigger(SetControllerTargetsCommand {
                                    plane,
                                    targets: edited,
                                });
                            }
                        }
                    }
                }
            }

            // Everything below is live-controller-only extras that the shared editor
            // above doesn't cover: read-only diagnostics/status the client already
            // gets via `draw_replicated_telemetry`, the L1 flight-plan display, local
            // tuning-profile combos + file-dialog buttons, and the local RL model
            // picker. On a networked client `ActiveController` isn't replicated, so
            // all of it is skipped here — the flight data and target editor above are
            // already live.
            let Some(mut controller) = controller else {
                return;
            };

            if *kind == ControllerKind::Wingman {
                if let Some(wc) = controller
                    .0
                    .as_any_mut()
                    .downcast_mut::<WingmanController>()
                {
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
                    let status = if ascent.complete {
                        "Complete"
                    } else {
                        "Climbing"
                    };
                    ui.label(format!("Status: {}", status));
                }
            }

            if *kind == ControllerKind::LevelHold {
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

            // RL and PID controllers share the same target variants, so
            // `RlLevelHoldController`/`RlOrbitController`/`RlOrbitResidualController`/
            // `RlLstmOrbitController` all report through the same `LevelHold`/`Orbit`
            // `ControllerTargets` variants as their PID counterparts (see
            // `controllers::targets`), so the shared editor above already handles
            // them — including the transition frames before an RL model finishes
            // loading, when the active controller is still the PID fallback.

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

/// The `PlaneId` for a followed entity (looked up from the sorted roster).
#[cfg(feature = "net")]
fn plane_id_of(pairs: &[(Entity, PlaneId, u32)], entity: Entity) -> Option<PlaneId> {
    pairs
        .iter()
        .find(|&&(e, _, _)| e == entity)
        .map(|&(_, pid, _)| pid)
}

/// Sorted tuning-profile names for a controller kind's family (mirrors
/// `main::tuning_profile_names`), or `None` if the kind has no tuning pool.
#[cfg(feature = "net")]
fn net_tuning_names(kind: ControllerKind, pt: &PlaneTuning) -> Option<Vec<String>> {
    let mut names: Vec<String> = match kind {
        ControllerKind::LevelHold | ControllerKind::RlLevelHold => {
            pt.level_hold.keys().cloned().collect()
        }
        ControllerKind::Orbit | ControllerKind::RlOrbit => pt.orbit.keys().cloned().collect(),
        ControllerKind::HeadingHold | ControllerKind::RlHeadingHold => {
            pt.heading_hold.keys().cloned().collect()
        }
        _ => return None,
    };
    names.sort();
    (!names.is_empty()).then_some(names)
}

/// Networked-client tuning/model selection combos. Reads the replicated
/// `SelectedTuningProfile` / `SelectedModel` and the client-reconstructed
/// `PlaneTuningHandle`, and sends a command on change (the server is
/// authoritative). Mirrors the local-sim combos that live in the
/// `ActiveController` block (which never runs on a client).
#[cfg(feature = "net")]
#[allow(clippy::too_many_arguments)]
fn net_selection_combos(
    ui: &mut egui::Ui,
    pairs: &[(Entity, PlaneId, u32)],
    current_entity: Entity,
    kind: ControllerKind,
    profile: Option<&SelectedTuningProfile>,
    tuning_handle: Option<&PlaneTuningHandle>,
    tuning_assets: &Assets<PlaneTuning>,
    #[cfg(feature = "inference")] selected_model: Option<&SelectedModel>,
    #[cfg(feature = "inference")] model_lib: &ModelLibrary,
    commands: &mut Commands,
) {
    let Some(plane) = plane_id_of(pairs, current_entity) else {
        return;
    };

    // Tuning profile.
    if let (Some(profile), Some(handle)) = (profile, tuning_handle) {
        if let Some(pt) = tuning_assets.get(&handle.0) {
            if let Some(names) = net_tuning_names(kind, pt) {
                let current = profile.0.clone();
                let mut selected = current.clone();
                egui::ComboBox::from_label("Tune Profile")
                    .selected_text(&selected)
                    .show_ui(ui, |ui| {
                        for name in &names {
                            ui.selectable_value(&mut selected, name.clone(), name);
                        }
                    });
                if selected != current {
                    commands.client_trigger(SetTuningProfileCommand {
                        plane,
                        profile: selected,
                    });
                }
                ui.label("(T / Shift+T to cycle)");
            }
        }
    }

    // RL model.
    #[cfg(feature = "inference")]
    if let (Some(dir_key), Some(sel)) = (kind.model_dir(), selected_model) {
        if let Some(available) = model_lib.0.get(dir_key) {
            let current = sel.0.clone();
            let mut chosen = if available.iter().any(|p| p == &current) {
                current.clone()
            } else {
                available
                    .first()
                    .cloned()
                    .unwrap_or_else(|| current.clone())
            };
            egui::ComboBox::from_label("Model")
                .selected_text(path_stem(&chosen))
                .show_ui(ui, |ui| {
                    for path in available {
                        ui.selectable_value(&mut chosen, path.clone(), path_stem(path));
                    }
                });
            if chosen != current {
                commands.client_trigger(SetModelCommand {
                    plane,
                    model_stem: chosen,
                });
            }
            ui.label("(T / Shift+T to cycle)");
        }
    }
}

/// Draw the editable setpoint widgets for whatever variant `targets` holds, mutating
/// it in place. Returns `true` iff a widget actually changed a value this frame.
///
/// Shared by both paths: the local-sim caller writes the result straight back into
/// the live controller (`FlightController::apply_targets`); the networked-client
/// caller shadows it in `PendingTargetEdit` and sends a `SetControllerTargetsCommand`
/// — see the call site in `draw_flight_hud`. `pairs` + `current_entity` are only used
/// by the `Wingman` leader combo.
fn draw_controller_targets(
    ui: &mut egui::Ui,
    state: &FlightState,
    pairs: &[(Entity, PlaneId, u32)],
    current_entity: Entity,
    targets: &mut ControllerTargets,
) -> bool {
    match targets {
        ControllerTargets::None => false,
        ControllerTargets::LevelHold { altitude, airspeed } => draw_alt_spd(ui, altitude, airspeed),
        ControllerTargets::Ascent { altitude } => {
            drag_row(ui, "Target Alt:", altitude, 10.0, 100.0..=15000.0, " m")
        }
        ControllerTargets::HeadingHold {
            heading,
            altitude,
            airspeed,
        } => {
            // Current heading display for reference — read-only, from live state.
            let speed_xz = (state.velocity.x.powi(2) + state.velocity.z.powi(2)).sqrt();
            let cur_heading_deg = if speed_xz > 1.0 {
                state.velocity.z.atan2(state.velocity.x).to_degrees()
            } else {
                0.0
            };
            ui.label(format!("Heading: {:.1}°", cur_heading_deg));

            let mut changed = draw_heading_row(ui, heading);
            changed |= draw_alt_spd(ui, altitude, airspeed);
            changed
        }
        ControllerTargets::Orbit(params) => draw_orbit_targets(ui, state, params),
        ControllerTargets::Wingman { leader } => {
            draw_leader_combo(ui, pairs, current_entity, leader)
        }
    }
}

/// A single labeled `DragValue` row. Returns whether the value changed this frame.
fn drag_row(
    ui: &mut egui::Ui,
    label: &str,
    value: &mut f32,
    speed: f32,
    range: std::ops::RangeInclusive<f32>,
    suffix: &str,
) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label(label);
        changed = ui
            .add(
                egui::DragValue::new(value)
                    .speed(speed)
                    .range(range)
                    .suffix(suffix),
            )
            .changed();
    });
    changed
}

/// The "Target Alt:" + "Target Spd:" row pair shared by every altitude/airspeed-based
/// controller kind (level-hold, heading-hold's inner loop, orbit).
fn draw_alt_spd(ui: &mut egui::Ui, altitude: &mut f32, airspeed: &mut f32) -> bool {
    let mut changed = drag_row(ui, "Target Alt:", altitude, 10.0, 100.0..=15000.0, " m");
    let tgt_kts = *airspeed * 1.944;
    ui.horizontal(|ui| {
        ui.label("Target Spd:");
        let resp = ui.add(
            egui::DragValue::new(airspeed)
                .speed(1.0)
                .range(30.0..=f32::MAX)
                .suffix(" m/s"),
        );
        ui.label(format!("({:.0} kts)", tgt_kts));
        changed |= resp.changed();
    });
    changed
}

/// Target heading, edited in degrees and stored in radians.
fn draw_heading_row(ui: &mut egui::Ui, heading: &mut f32) -> bool {
    let mut tgt_deg = heading.to_degrees();
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label("Target Hdg:");
        changed = ui
            .add(egui::DragValue::new(&mut tgt_deg).speed(1.0).suffix("°"))
            .changed();
    });
    if changed {
        *heading = tgt_deg.to_radians();
    }
    changed
}

/// The wingman formation-leader combo. Returns whether a different leader was picked.
fn draw_leader_combo(
    ui: &mut egui::Ui,
    pairs: &[(Entity, PlaneId, u32)],
    current_entity: Entity,
    leader: &mut PlaneId,
) -> bool {
    let current_leader_id = *leader;
    let mut selected_leader_id = current_leader_id;
    let leader_label = pairs
        .iter()
        .find(|&&(_, pid, _)| pid == current_leader_id)
        .map(|&(_, _, idx)| format!("Plane {}", idx))
        .unwrap_or_else(|| "Unknown".to_string());
    egui::ComboBox::from_label("Leader")
        .selected_text(&leader_label)
        .show_ui(ui, |ui| {
            for &(entity, pid, idx) in pairs {
                if entity == current_entity {
                    continue;
                }
                ui.selectable_value(&mut selected_leader_id, pid, format!("Plane {}", idx));
            }
        });
    if selected_leader_id != current_leader_id {
        *leader = selected_leader_id;
        true
    } else {
        false
    }
}

/// Orbit geometry editor (center/radius/target alt/spd/direction) shared by
/// `OrbitController` and all three RL orbit variants via `ControllerTargets::Orbit`.
fn draw_orbit_targets(ui: &mut egui::Ui, state: &FlightState, params: &mut OrbitParams) -> bool {
    let mut changed = drag_row(
        ui,
        "Center X:",
        &mut params.center_x,
        10.0,
        -50_000.0..=50_000.0,
        " m",
    );
    changed |= drag_row(
        ui,
        "Center Z:",
        &mut params.center_z,
        10.0,
        -50_000.0..=50_000.0,
        " m",
    );
    changed |= drag_row(
        ui,
        "Radius:",
        &mut params.target_radius,
        10.0,
        500.0..=20_000.0,
        " m",
    );
    changed |= draw_alt_spd(ui, &mut params.target_altitude, &mut params.target_airspeed);

    let dir_label = match params.direction {
        OrbitDirection::Clockwise => "CW",
        OrbitDirection::CounterClockwise => "CCW",
    };
    if ui.button(format!("Dir: {}", dir_label)).clicked() {
        params.direction = match params.direction {
            OrbitDirection::Clockwise => OrbitDirection::CounterClockwise,
            OrbitDirection::CounterClockwise => OrbitDirection::Clockwise,
        };
        changed = true;
    }

    let rx = state.position.x - params.center_x;
    let rz = state.position.z - params.center_z;
    let r = (rx * rx + rz * rz).sqrt();
    ui.label(format!("Radius err: {:.1} m", r - params.target_radius));
    changed
}

/// Render the read-only controller status on a networked client from the replicated
/// [`ControllerTelemetry`] snapshot. Mirrors the label text the local-sim per-kind
/// panels draw off the live controller, so the client HUD looks the same.
#[cfg(feature = "net")]
fn draw_replicated_telemetry(ui: &mut egui::Ui, telemetry: Option<&ControllerTelemetry>) {
    let Some(telemetry) = telemetry else { return };
    match telemetry {
        ControllerTelemetry::None => {}
        ControllerTelemetry::Ascent { complete } => {
            ui.separator();
            ui.label(if *complete {
                "Status: Complete"
            } else {
                "Status: Climbing"
            });
        }
        ControllerTelemetry::Orbit { radial_error } => {
            ui.separator();
            ui.label(format!("Radius err: {:.1} m", radial_error));
        }
        ControllerTelemetry::Wingman(d) => {
            ui.separator();
            if d.leader_found {
                ui.label(format!("Pos error: {:.0} m", d.pos_error_mag));
                ui.label(format!("Cross-track: {:+.0} m", d.cross_track));
                ui.label(format!("Fore-aft: {:+.0} m", d.range_error));
                ui.label(format!("Vertical: {:+.0} m", d.altitude_error));
            } else {
                ui.label("Leader: lost (holding)");
            }
        }
        ControllerTelemetry::FlightPlan {
            leg_index,
            leg_count,
            status,
        } => {
            ui.separator();
            ui.label(format!("Leg {} / {}", leg_index + 1, leg_count));
            match status {
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
        }
    }
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

    use crate::controllers::ControllerTargets;
    use crate::controllers::OrbitParams;

    #[test]
    fn seed_targets_prefers_pending_within_hold_window() {
        let e = entity(1);
        let mut pending = PendingTargetEdit::default();
        let edited = ControllerTargets::LevelHold {
            altitude: 1600.0,
            airspeed: 110.0,
        };
        pending.record(e, edited, 10.0);
        let replicated = ControllerTargets::LevelHold {
            altitude: 1000.0,
            airspeed: 80.0,
        };
        assert_eq!(seed_targets(replicated, &pending, e, 10.1), edited);
    }

    #[test]
    fn seed_targets_falls_back_to_replicated_after_hold_expires() {
        let e = entity(1);
        let mut pending = PendingTargetEdit::default();
        pending.record(
            e,
            ControllerTargets::LevelHold {
                altitude: 1600.0,
                airspeed: 110.0,
            },
            10.0,
        );
        let replicated = ControllerTargets::LevelHold {
            altitude: 1000.0,
            airspeed: 80.0,
        };
        let now = 10.0 + TARGET_EDIT_HOLD_SECS + 0.01;
        assert_eq!(seed_targets(replicated, &pending, e, now), replicated);
    }

    #[test]
    fn seed_targets_discards_pending_when_variant_changes() {
        let e = entity(1);
        let mut pending = PendingTargetEdit::default();
        pending.record(
            e,
            ControllerTargets::LevelHold {
                altitude: 1600.0,
                airspeed: 110.0,
            },
            10.0,
        );
        // Server rebuilt the controller into Orbit — a variant mismatch must defer
        // to the replicated value, not the stale pending edit for the old kind.
        let replicated = ControllerTargets::Orbit(OrbitParams {
            center_x: 0.0,
            center_z: 0.0,
            target_radius: 1000.0,
            target_altitude: 1000.0,
            target_airspeed: 100.0,
            direction: OrbitDirection::CounterClockwise,
        });
        assert_eq!(seed_targets(replicated, &pending, e, 10.1), replicated);
    }

    #[test]
    fn seed_targets_discards_pending_for_a_different_entity() {
        let e1 = entity(1);
        let e2 = entity(2);
        let mut pending = PendingTargetEdit::default();
        pending.record(
            e1,
            ControllerTargets::LevelHold {
                altitude: 1600.0,
                airspeed: 110.0,
            },
            10.0,
        );
        let replicated = ControllerTargets::LevelHold {
            altitude: 900.0,
            airspeed: 85.0,
        };
        assert_eq!(seed_targets(replicated, &pending, e2, 10.1), replicated);
    }

    #[test]
    fn pending_edit_suppresses_duplicate_sends() {
        let mut pending = PendingTargetEdit::default();
        let t = ControllerTargets::LevelHold {
            altitude: 1600.0,
            airspeed: 110.0,
        };
        assert!(pending.take_if_new(t), "first send should go out");
        assert!(!pending.take_if_new(t), "unchanged value must not resend");
        let t2 = ControllerTargets::LevelHold {
            altitude: 1650.0,
            airspeed: 110.0,
        };
        assert!(pending.take_if_new(t2), "changed value should send again");
    }

    /// Lay `draw_controller_targets` out in a headless egui context (mirrors
    /// `lifecycle_panel`'s `layout_panel` pattern) and return whether any widget
    /// reported a change over the run.
    fn run_draw_controller_targets(
        targets: &mut ControllerTargets,
        state: &FlightState,
        pairs: &[(Entity, PlaneId, u32)],
        current_entity: Entity,
    ) -> bool {
        let ctx = egui::Context::default();
        let mut changed = false;
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::pos2(0.0, 0.0),
                egui::vec2(400.0, 400.0),
            )),
            ..Default::default()
        };
        let _ = ctx.run(input, |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| {
                changed = draw_controller_targets(ui, state, pairs, current_entity, targets);
            });
        });
        changed
    }

    /// Rendering widgets without input must not report a change or mutate the
    /// value — otherwise the networked client would spam
    /// `SetControllerTargetsCommand` every single frame.
    #[test]
    fn drawing_targets_without_input_reports_no_change() {
        let state = FlightState::default();
        let current_entity = entity(1);
        let pairs = vec![(current_entity, PlaneId(1), 1u32)];

        let variants = [
            ControllerTargets::None,
            ControllerTargets::LevelHold {
                altitude: 1000.0,
                airspeed: 80.0,
            },
            ControllerTargets::Ascent { altitude: 1500.0 },
            ControllerTargets::HeadingHold {
                heading: 0.3,
                altitude: 1000.0,
                airspeed: 80.0,
            },
            ControllerTargets::Orbit(OrbitParams {
                center_x: 0.0,
                center_z: 0.0,
                target_radius: 1000.0,
                target_altitude: 1000.0,
                target_airspeed: 100.0,
                direction: OrbitDirection::CounterClockwise,
            }),
            ControllerTargets::Wingman { leader: PlaneId(1) },
        ];

        for original in variants {
            let mut targets = original;
            for frame in 0..3 {
                let changed =
                    run_draw_controller_targets(&mut targets, &state, &pairs, current_entity);
                assert!(
                    !changed,
                    "frame {frame}: empty input should never report changed for {original:?}"
                );
            }
            assert_eq!(
                targets, original,
                "empty input must not mutate targets for {original:?}"
            );
        }
    }

    /// The airspeed range has no upper bound (only a 30 m/s stall floor) — a target set
    /// above 200 m/s (e.g. via MCP or a scenario) must survive an unattended
    /// render pass unchanged in both value and reported `changed` state.
    #[test]
    fn target_airspeed_above_200_survives_a_render_pass() {
        let state = FlightState::default();
        let current_entity = entity(1);
        let pairs = vec![(current_entity, PlaneId(1), 1u32)];

        let variants = [
            ControllerTargets::LevelHold {
                altitude: 5000.0,
                airspeed: 250.0,
            },
            ControllerTargets::HeadingHold {
                heading: 0.3,
                altitude: 5000.0,
                airspeed: 250.0,
            },
            ControllerTargets::Orbit(OrbitParams {
                center_x: 0.0,
                center_z: 0.0,
                target_radius: 1000.0,
                target_altitude: 5000.0,
                target_airspeed: 250.0,
                direction: OrbitDirection::CounterClockwise,
            }),
        ];

        for original in variants {
            let mut targets = original;
            let changed = run_draw_controller_targets(&mut targets, &state, &pairs, current_entity);
            assert!(
                !changed,
                "an above-200 airspeed should not be reported as changed for {original:?}"
            );
            assert_eq!(
                targets, original,
                "an above-200 airspeed should survive a render pass unchanged for {original:?}"
            );
        }
    }

    #[test]
    fn none_variant_draws_nothing() {
        let state = FlightState::default();
        let current_entity = entity(1);
        let pairs = vec![(current_entity, PlaneId(1), 1u32)];
        let mut targets = ControllerTargets::None;

        let ctx = egui::Context::default();
        let mut before = f32::NAN;
        let mut after = f32::NAN;
        let input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::pos2(0.0, 0.0),
                egui::vec2(400.0, 400.0),
            )),
            ..Default::default()
        };
        let _ = ctx.run(input, |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| {
                before = ui.cursor().top();
                draw_controller_targets(ui, &state, &pairs, current_entity, &mut targets);
                after = ui.cursor().top();
            });
        });
        assert_eq!(
            before, after,
            "None variant should not advance the layout cursor (i.e. draw nothing)"
        );
    }
}
