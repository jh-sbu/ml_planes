//! Headless controller-rebuild systems (`SimControlPlugin`).
//!
//! These systems react to `Changed<ControllerKind>` / `Changed<SelectedTuningProfile>`
//! / `Changed<SelectedModel>` and rebuild a plane's [`ActiveController`] accordingly.
//! They previously lived in `main.rs` behind `#[cfg(feature = "visual")]` +
//! `run_if(in_state(AppState::InGame))`, but the client/server split (see
//! `plans/client_server.md`) needs them to run on a headless server too: a client's
//! `SwitchControllerCommand` / `SetTuningProfileCommand` / `SetModelCommand` is applied
//! server-side simply by mutating the corresponding component, after which these systems
//! rebuild the controller.
//!
//! The input-polling / hotkey half (`poll_controller_inputs`, `switch_controller`,
//! `cycle_tune_profile`, `cycle_rl_model`) stays client-only in `main.rs` — it becomes a
//! set of command senders in a later phase.

use bevy::prelude::*;

use crate::controllers::{
    ActiveController, ControllerKind, ControllerTuning, FlightPlan, FormationOffset, L1Controller,
    LevelHoldController, ModelLibrary, OrbitController, OrbitParams, PidController, PlaneTuning,
    SelectedTuningProfile, TuningApplied, WingmanController,
};
use crate::plane::{ControlInputs, FlightPlanHandle, FlightState, PlaneId, PlaneTuningHandle};

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
use crate::controllers::heading_hold::ground_track_heading;
#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
use crate::controllers::{
    HeadingHoldController, ModelLoadError, RlHeadingHoldController, RlLevelHoldController,
    RlLstmOrbitController, RlOrbitController, RlOrbitResidualController, SelectedModel,
};
#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
use crate::controllers::{OrbitTuning, RlLstmOrbitConfig, RlOrbitConfig, RlOrbitResidualConfig};
#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
use crate::notifications::Notifications;

/// Registers the controller-rebuild systems on `PostUpdate` so they run after the
/// client's hotkey/input systems (and the menu's spawn flow) have written their
/// changes. Render-neutral: adds no `AppState` gate, so the headless server applies
/// client commands by mutating components and letting these systems rebuild.
pub struct SimControlPlugin;

impl Plugin for SimControlPlugin {
    fn build(&self, app: &mut App) {
        // Available headlessly so the RL rebuild systems (and the visual HUD model
        // dropdown) share one model library; populated by `scan_models` at startup.
        app.init_resource::<ModelLibrary>();

        app.add_systems(
            PostUpdate,
            (
                apply_initial_tuning,
                apply_controller_switch.after(apply_initial_tuning),
                apply_flight_plan.after(apply_controller_switch),
            ),
        );

        #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
        {
            // Idempotent — the visual `UiPlugin` also inits this for its HUD banner.
            app.init_resource::<Notifications>();
            app.add_systems(Startup, scan_models);
            app.add_systems(
                PostUpdate,
                (
                    apply_rl_controller_switch.after(apply_controller_switch),
                    apply_model_switch,
                ),
            );
        }
    }
}

/// Scan `models/<category>/` subdirectories at startup and populate `ModelLibrary`.
/// `pub(crate)` so the networked client (which omits `SimControlPlugin`) can run it
/// too, giving its HUD model dropdown / cycler data to enumerate.
#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
pub(crate) fn scan_models(mut commands: Commands) {
    let mut lib: std::collections::HashMap<String, Vec<String>> = Default::default();
    if let Ok(categories) = std::fs::read_dir("models/") {
        for cat in categories.flatten() {
            if !cat.file_type().map(|t| t.is_dir()).unwrap_or(false) {
                continue;
            }
            let cat_name = match cat.file_name().into_string() {
                Ok(s) => s,
                Err(_) => continue,
            };
            if let Ok(files) = std::fs::read_dir(cat.path()) {
                let mut stems: Vec<String> = files
                    .flatten()
                    .filter_map(|f| {
                        let name = f.file_name().into_string().ok()?;
                        let stem = name.strip_suffix(".mpk")?;
                        Some(format!("models/{cat_name}/{stem}"))
                    })
                    .collect();
                stems.sort();
                if !stems.is_empty() {
                    lib.insert(cat_name, stems);
                }
            }
        }
    }
    commands.insert_resource(ModelLibrary(lib));
}

/// Reload `ActiveController` whenever `SelectedModel` changes (HUD model dropdown).
#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
fn apply_model_switch(
    mut query: Query<
        (
            &FlightState,
            &mut ActiveController,
            &ControllerKind,
            &SelectedModel,
            Option<&PlaneTuningHandle>,
            Option<&SelectedTuningProfile>,
        ),
        Changed<SelectedModel>,
    >,
    tuning_assets: Res<Assets<PlaneTuning>>,
    mut notes: ResMut<Notifications>,
) {
    for (state, mut ctrl, kind, sel, tuning_handle, profile) in query.iter_mut() {
        let Some(dir) = kind.model_dir() else {
            continue;
        };
        if !model_path_matches_dir(&sel.0, dir) {
            warn!("Ignoring model '{}' for controller {}", sel.0, kind.name());
            continue;
        }

        match *kind {
            ControllerKind::RlLevelHold => {
                let (target_alt, target_spd) = level_hold_targets_from_controller(&mut ctrl, state);
                match RlLevelHoldController::load(&sel.0, target_alt, target_spd) {
                    Ok(new_ctrl) => ctrl.0 = Box::new(new_ctrl),
                    Err(e) => report_skipped_model(&mut notes, &sel.0, &e),
                }
            }
            ControllerKind::RlOrbit => {
                let config = orbit_config_from_controller(&mut ctrl, state);
                match RlOrbitController::load(&sel.0, config) {
                    Ok(new_ctrl) => ctrl.0 = Box::new(new_ctrl),
                    Err(e) => report_skipped_model(&mut notes, &sel.0, &e),
                }
            }
            ControllerKind::RlOrbitResidual => {
                let config = residual_config_from_controller(&mut ctrl, state);
                let profile_name = profile.map(|p| p.0.as_str()).unwrap_or("normal");
                let orbit_tuning: Option<&OrbitTuning> = tuning_handle
                    .and_then(|h| tuning_assets.get(&h.0))
                    .and_then(|pt| pt.get_orbit(profile_name));
                match RlOrbitResidualController::load(&sel.0, config, state, orbit_tuning) {
                    Ok(new_ctrl) => ctrl.0 = Box::new(new_ctrl),
                    Err(e) => report_skipped_model(&mut notes, &sel.0, &e),
                }
            }
            ControllerKind::RlLstmOrbit => {
                let config = lstm_orbit_config_from_controller(&mut ctrl, state);
                match RlLstmOrbitController::load(&sel.0, config) {
                    Ok(new_ctrl) => ctrl.0 = Box::new(new_ctrl),
                    Err(e) => report_skipped_model(&mut notes, &sel.0, &e),
                }
            }
            ControllerKind::RlHeadingHold => {
                let (heading, altitude, airspeed) =
                    heading_hold_targets_from_controller(&mut ctrl, state);
                let config = crate::controllers::RlHeadingHoldConfig {
                    target_heading: heading,
                    target_altitude: altitude,
                    target_airspeed: airspeed,
                };
                match RlHeadingHoldController::load(&sel.0, config) {
                    Ok(new_ctrl) => ctrl.0 = Box::new(new_ctrl),
                    Err(e) => report_skipped_model(&mut notes, &sel.0, &e),
                }
            }
            _ => {}
        }
    }
}

/// Log a model-load failure and surface a transient HUD banner. Used when an
/// incompatible (e.g. stale-dimension) checkpoint is skipped and the controller
/// keeps its current (PID fallback) behavior.
#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
fn report_skipped_model(notes: &mut Notifications, path: &str, err: &ModelLoadError) {
    warn!("Skipping model '{path}': {err}");
    let name = std::path::Path::new(path)
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or(path);
    notes.push(format!("Model '{name}' skipped — {err}"));
}

/// Whether `apply_rl_controller_switch` should (re)load a model for a plane
/// whose `ControllerKind` was just observed as changed.
///
/// Only RL kinds are handled here (non-RL kinds keep the PID path). For RL kinds
/// we load on any genuine kind change, and on the spawn frame only when no
/// `SelectedModel` is wired yet: runtime (panel/hotkey) spawns arrive on the PID
/// fallback with no model and need loading, whereas startup-spawned RL planes
/// already carry both the loaded controller and a `SelectedModel`, so they're
/// skipped to avoid a redundant reload. That skip is only safe because
/// `preserve_rl_controller` stops `apply_initial_tuning`/`apply_controller_switch` from
/// later clobbering the already-loaded controller with the PID fallback once the plane's
/// tuning asset loads — before that guard existed, a startup-spawned RL plane's policy was
/// silently replaced by PID the moment its `.tuning.ron` resolved, with nothing here to
/// notice and reload it (`Changed<ControllerKind>` doesn't fire on a tuning-asset load).
#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
fn rl_kind_needs_load_on_change(
    kind: ControllerKind,
    kind_added: bool,
    has_selected_model: bool,
) -> bool {
    let is_rl = matches!(
        kind,
        ControllerKind::RlLevelHold
            | ControllerKind::RlOrbit
            | ControllerKind::RlOrbitResidual
            | ControllerKind::RlLstmOrbit
            | ControllerKind::RlHeadingHold
    );
    is_rl && (!kind_added || !has_selected_model)
}

/// When `ControllerKind` changes to an RL kind, load the actual RL model,
/// overriding the PID fallback that `apply_controller_switch` produces.
/// If the entity lacks `SelectedModel`, inserts a default so `apply_model_switch`
/// loads the controller on the next frame.
#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
fn apply_rl_controller_switch(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            &FlightState,
            &mut ActiveController,
            Mut<ControllerKind>,
            Option<&SelectedModel>,
            Option<&PlaneTuningHandle>,
            Option<&SelectedTuningProfile>,
        ),
        Changed<ControllerKind>,
    >,
    model_lib: Res<ModelLibrary>,
    tuning_assets: Res<Assets<PlaneTuning>>,
    mut notes: ResMut<Notifications>,
) {
    for (entity, state, mut controller, mut kind, sel, tuning_handle, profile) in query.iter_mut() {
        if !rl_kind_needs_load_on_change(*kind, kind.is_added(), sel.is_some()) {
            continue;
        }

        let Some(path) = selected_or_default_model_path(*kind, sel, &model_lib) else {
            // No checkpoint available: revert the label to the PID baseline the
            // controller is actually running, so a spawned RL plane doesn't show
            // an "RL" label while flying PID.
            match *kind {
                ControllerKind::RlOrbit
                | ControllerKind::RlOrbitResidual
                | ControllerKind::RlLstmOrbit => {
                    kind.set_if_neq(ControllerKind::Orbit);
                }
                ControllerKind::RlLevelHold => {
                    kind.set_if_neq(ControllerKind::LevelHold);
                }
                ControllerKind::RlHeadingHold => {
                    kind.set_if_neq(ControllerKind::HeadingHold);
                }
                _ => {}
            }
            continue;
        };

        if sel.map(|s| s.0.as_str()) != Some(path.as_str()) {
            commands.entity(entity).insert(SelectedModel(path.clone()));
        }

        match *kind {
            ControllerKind::RlLevelHold => {
                let (tgt_alt, tgt_spd) = level_hold_targets_from_controller(&mut controller, state);
                match RlLevelHoldController::load(&path, tgt_alt, tgt_spd) {
                    Ok(rl) => controller.0 = Box::new(rl),
                    Err(e) => {
                        report_skipped_model(&mut notes, &path, &e);
                        kind.set_if_neq(ControllerKind::LevelHold);
                    }
                }
            }
            ControllerKind::RlOrbit => {
                let config = orbit_config_from_controller(&mut controller, state);
                match RlOrbitController::load(&path, config) {
                    Ok(rl) => controller.0 = Box::new(rl),
                    Err(e) => {
                        report_skipped_model(&mut notes, &path, &e);
                        kind.set_if_neq(ControllerKind::Orbit);
                    }
                }
            }
            ControllerKind::RlOrbitResidual => {
                let config = residual_config_from_controller(&mut controller, state);
                let profile_name = profile.map(|p| p.0.as_str()).unwrap_or("normal");
                let orbit_tuning: Option<&OrbitTuning> = tuning_handle
                    .and_then(|h| tuning_assets.get(&h.0))
                    .and_then(|pt| pt.get_orbit(profile_name));
                match RlOrbitResidualController::load(&path, config, state, orbit_tuning) {
                    Ok(rl) => controller.0 = Box::new(rl),
                    Err(e) => {
                        report_skipped_model(&mut notes, &path, &e);
                        kind.set_if_neq(ControllerKind::Orbit);
                    }
                }
            }
            ControllerKind::RlLstmOrbit => {
                let config = lstm_orbit_config_from_controller(&mut controller, state);
                match RlLstmOrbitController::load(&path, config) {
                    Ok(rl) => controller.0 = Box::new(rl),
                    Err(e) => {
                        report_skipped_model(&mut notes, &path, &e);
                        kind.set_if_neq(ControllerKind::Orbit);
                    }
                }
            }
            ControllerKind::RlHeadingHold => {
                let (heading, altitude, airspeed) =
                    heading_hold_targets_from_controller(&mut controller, state);
                let config = crate::controllers::RlHeadingHoldConfig {
                    target_heading: heading,
                    target_altitude: altitude,
                    target_airspeed: airspeed,
                };
                match RlHeadingHoldController::load(&path, config) {
                    Ok(rl) => controller.0 = Box::new(rl),
                    Err(e) => {
                        report_skipped_model(&mut notes, &path, &e);
                        kind.set_if_neq(ControllerKind::HeadingHold);
                    }
                }
            }
            _ => {}
        }
    }
}

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
fn selected_or_default_model_path(
    kind: ControllerKind,
    selected: Option<&SelectedModel>,
    model_lib: &ModelLibrary,
) -> Option<String> {
    let dir = kind.model_dir()?;
    if let Some(sel) = selected {
        if model_path_matches_dir(&sel.0, dir) {
            return Some(sel.0.clone());
        }
    }
    model_lib
        .0
        .get(dir)
        .and_then(|paths| paths.first().cloned())
}

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
/// Whether a `SelectedModel` stem is a legitimate path inside `models/<dir>/`.
///
/// The stem arrives from a client (`SetModelCommand`) and reaches burn's
/// `File::open`, so a bare `starts_with` is not enough: `models/orbit/../../..`
/// satisfies the prefix while walking straight back out. Rejects any `..` /
/// absolute / backslash component, matching `sanitize_asset_path`'s contract.
fn model_path_matches_dir(path: &str, dir: &str) -> bool {
    if !path.starts_with(&format!("models/{dir}/")) {
        return false;
    }
    if path.contains('\0') || path.contains('\\') {
        return false;
    }
    std::path::Path::new(path)
        .components()
        .all(|c| matches!(c, std::path::Component::Normal(_)))
}

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
fn level_hold_targets_from_controller(
    controller: &mut ActiveController,
    state: &FlightState,
) -> (f32, f32) {
    if let Some(rl) = controller
        .0
        .as_any_mut()
        .downcast_mut::<RlLevelHoldController>()
    {
        return (rl.target_altitude, rl.target_airspeed);
    }
    if let Some(lh) = controller
        .0
        .as_any_mut()
        .downcast_mut::<LevelHoldController>()
    {
        return (lh.target_altitude, lh.target_airspeed);
    }
    (state.altitude, state.airspeed)
}

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
fn heading_hold_targets_from_controller(
    controller: &mut ActiveController,
    state: &FlightState,
) -> (f32, f32, f32) {
    if let Some(rl) = controller
        .0
        .as_any_mut()
        .downcast_mut::<RlHeadingHoldController>()
    {
        return (rl.target_heading, rl.target_altitude, rl.target_airspeed);
    }
    if let Some(hh) = controller
        .0
        .as_any_mut()
        .downcast_mut::<HeadingHoldController>()
    {
        return (
            hh.target_heading,
            hh.inner.target_altitude,
            hh.inner.target_airspeed,
        );
    }
    (ground_track_heading(state), state.altitude, state.airspeed)
}

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
fn orbit_config_from_controller(
    controller: &mut ActiveController,
    state: &FlightState,
) -> RlOrbitConfig {
    if let Some(rl) = controller
        .0
        .as_any_mut()
        .downcast_mut::<RlOrbitController>()
    {
        return rl.config();
    }
    if let Some(orbit) = controller.0.as_any_mut().downcast_mut::<OrbitController>() {
        return RlOrbitConfig::from_orbit(orbit);
    }
    RlOrbitConfig::from_state(state)
}

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
fn residual_config_from_controller(
    controller: &mut ActiveController,
    state: &FlightState,
) -> RlOrbitResidualConfig {
    if let Some(rl) = controller
        .0
        .as_any_mut()
        .downcast_mut::<RlOrbitResidualController>()
    {
        return rl.config();
    }
    if let Some(orbit) = controller.0.as_any_mut().downcast_mut::<OrbitController>() {
        return RlOrbitResidualConfig::from_orbit(orbit);
    }
    RlOrbitResidualConfig::from_state(state)
}

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
fn lstm_orbit_config_from_controller(
    controller: &mut ActiveController,
    state: &FlightState,
) -> RlLstmOrbitConfig {
    if let Some(rl) = controller
        .0
        .as_any_mut()
        .downcast_mut::<RlLstmOrbitController>()
    {
        return rl.config();
    }
    if let Some(orbit) = controller.0.as_any_mut().downcast_mut::<OrbitController>() {
        return RlLstmOrbitConfig::from_orbit(orbit);
    }
    RlLstmOrbitConfig::from_state(state)
}

/// Extract orbit geometry from whatever controller variant is currently active.
///
/// Returns `None` when the active controller is not an orbit variant (e.g. LevelHold,
/// Manual), so the caller can fall back to `from_state()` geometry as before.
fn extract_orbit_params(ctrl: &mut ActiveController) -> Option<OrbitParams> {
    if let Some(orbit) = ctrl.0.as_any_mut().downcast_mut::<OrbitController>() {
        return Some(OrbitParams {
            center_x: orbit.center_x,
            center_z: orbit.center_z,
            target_radius: orbit.target_radius,
            target_altitude: orbit.target_altitude,
            target_airspeed: orbit.target_airspeed,
            direction: orbit.direction,
        });
    }
    #[cfg(feature = "inference")]
    {
        if let Some(rl) = ctrl.0.as_any_mut().downcast_mut::<RlOrbitController>() {
            let cfg = rl.config();
            return Some(OrbitParams {
                center_x: cfg.center_x,
                center_z: cfg.center_z,
                target_radius: cfg.target_radius,
                target_altitude: cfg.target_altitude,
                target_airspeed: cfg.target_airspeed,
                direction: cfg.direction,
            });
        }
        if let Some(rl) = ctrl
            .0
            .as_any_mut()
            .downcast_mut::<RlOrbitResidualController>()
        {
            let cfg = rl.config();
            return Some(OrbitParams {
                center_x: cfg.center_x,
                center_z: cfg.center_z,
                target_radius: cfg.target_radius,
                target_altitude: cfg.target_altitude,
                target_airspeed: cfg.target_airspeed,
                direction: cfg.direction,
            });
        }
        if let Some(rl) = ctrl.0.as_any_mut().downcast_mut::<RlLstmOrbitController>() {
            let cfg = rl.config();
            return Some(OrbitParams {
                center_x: cfg.center_x,
                center_z: cfg.center_z,
                target_radius: cfg.target_radius,
                target_altitude: cfg.target_altitude,
                target_airspeed: cfg.target_airspeed,
                direction: cfg.direction,
            });
        }
    }
    None
}

/// Whether the active controller is already running its RL policy and the tuning-rebuild
/// systems should leave it alone (or, for `RlOrbitResidual`, just re-tune its inner PID
/// baseline) instead of calling `kind.build()`.
///
/// `ControllerKind::build()` falls back to a PID controller for every RL kind — it has no
/// model path to load one from (`kind.rs`, "RlLevelHold requires a model path — fall back to
/// LevelHold like Wingman"). Without this check, `apply_initial_tuning`/`apply_controller_switch`
/// would silently replace a live RL policy with that PID fallback the moment a `.tuning.ron`
/// asset loads or a profile switches, exactly as they once did for `Wingman` before
/// `extract_wingman_params`/`restore_wingman` closed that gap — see the "Adding a New
/// `ControllerKind`" checklist in `CLAUDE.md`.
///
/// Returns `true` when handled here (the caller must skip `kind.build()`); `false` when the
/// active controller isn't actually running the RL policy yet (e.g. the kind just changed *to*
/// an RL kind this frame and the PID fallback is still in place) — the caller falls through to
/// the normal rebuild, and `apply_rl_controller_switch` loads the model afterward as usual.
///
/// `RlLevelHold`/`RlOrbit`/`RlLstmOrbit` carry no PID gains at all, so "preserve" is a no-op
/// beyond the downcast check. `RlOrbitResidual` is the one RL kind with a real inner PID
/// baseline (see `RlOrbitResidualController::retune`), so it alone needs the tuning applied.
#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
fn preserve_rl_controller(
    ctrl: &mut ActiveController,
    kind: ControllerKind,
    state: &FlightState,
    orbit_tuning: Option<&OrbitTuning>,
) -> bool {
    match kind {
        ControllerKind::RlLevelHold => ctrl
            .0
            .as_any_mut()
            .downcast_mut::<RlLevelHoldController>()
            .is_some(),
        ControllerKind::RlOrbit => ctrl
            .0
            .as_any_mut()
            .downcast_mut::<RlOrbitController>()
            .is_some(),
        ControllerKind::RlLstmOrbit => ctrl
            .0
            .as_any_mut()
            .downcast_mut::<RlLstmOrbitController>()
            .is_some(),
        ControllerKind::RlHeadingHold => ctrl
            .0
            .as_any_mut()
            .downcast_mut::<RlHeadingHoldController>()
            .is_some(),
        ControllerKind::RlOrbitResidual => {
            match ctrl
                .0
                .as_any_mut()
                .downcast_mut::<RlOrbitResidualController>()
            {
                Some(rl) => {
                    rl.retune(orbit_tuning, state);
                    true
                }
                None => false,
            }
        }
        _ => false,
    }
}

/// Formation state a tuning rebuild would otherwise destroy.
///
/// `ControllerKind::Wingman.build()` returns a plain `LevelHoldController` (the
/// generic factory has no leader reference), so the rebuild systems snapshot
/// this first and re-wrap the freshly tuned inner controller via
/// `WingmanController::from_inner` — the wingman analogue of
/// `extract_orbit_params` + `OrbitController::apply_params`. The outer PIDs are
/// carried over too, so a profile switch doesn't discard accumulated
/// range/lateral/heading integrator state.
struct WingmanParams {
    leader_id: PlaneId,
    offset: FormationOffset,
    range_pid: PidController,
    lateral_pid: PidController,
    heading_pid: PidController,
}

/// Snapshot the formation state of the active controller, if it's a
/// `WingmanController`. `None` when the kind says `Wingman` but no wingman law
/// is actually installed (e.g. a fresh `SpawnPlaneCommand`/`SwitchControllerCommand`
/// fallback) — the caller just proceeds with the plain rebuild in that case.
fn extract_wingman_params(ctrl: &mut ActiveController) -> Option<WingmanParams> {
    let wingman = ctrl.0.as_any_mut().downcast_mut::<WingmanController>()?;
    Some(WingmanParams {
        leader_id: wingman.leader_id,
        offset: wingman.offset.clone(),
        range_pid: wingman.range_pid.clone(),
        lateral_pid: wingman.lateral_pid.clone(),
        heading_pid: wingman.heading_pid.clone(),
    })
}

/// Re-wrap the freshly rebuilt (plain `LevelHoldController`) active controller
/// back into a `WingmanController`, restoring the formation state `extract_wingman_params`
/// captured before the rebuild.
fn restore_wingman(ctrl: &mut ActiveController, params: WingmanParams) {
    let Some(inner) = ctrl.0.as_any_mut().downcast_mut::<LevelHoldController>() else {
        // Shouldn't happen — `Wingman.build()` always yields a LevelHoldController —
        // but leave the controller alone rather than panic if it ever does.
        return;
    };
    let inner = inner.clone();
    let mut wingman = WingmanController::from_inner(params.leader_id, params.offset, inner);
    wingman.range_pid = params.range_pid;
    wingman.lateral_pid = params.lateral_pid;
    wingman.heading_pid = params.heading_pid;
    ctrl.0 = Box::new(wingman);
}

/// Kinds whose `build()` re-seeds every setpoint from the *current* `FlightState`
/// (`LevelHoldController::with_tuning`/`from_state` capture `state.altitude`/
/// `state.airspeed`; `HeadingHoldController::from_state` captures
/// `ground_track_heading(state)`; `AscentController::new` re-targets
/// `state.altitude + 1000.0`). A tuning-asset load or profile switch would
/// otherwise silently cancel a scenario- or pilot-commanded setpoint the moment
/// the plane's tuning finishes loading — exactly the bug class `ControllerTargets`
/// exists to prevent on the HUD side, just hitting it from the rebuild side instead.
///
/// `Orbit`/`RlOrbit*` and `Wingman` are deliberately excluded: they already have
/// their own extract/restore pairs above (`extract_orbit_params`,
/// `extract_wingman_params`), and their `apply_targets` impls have side effects
/// (PID resets, auto-centering) a blind snapshot/replay must not trigger.
fn rebuild_preserves_targets(kind: ControllerKind) -> bool {
    matches!(
        kind,
        ControllerKind::LevelHold
            | ControllerKind::RlLevelHold
            | ControllerKind::HeadingHold
            | ControllerKind::RlHeadingHold
            | ControllerKind::Ascent
    )
}

/// Snapshot `controller`'s editable setpoints if its kind is one
/// `rebuild_preserves_targets` covers, so they can be replayed via
/// `restore_targets` after `kind.build()` re-seeds them from live state.
/// `None` both when the kind is out of scope and when the controller reports
/// `ControllerTargets::None` (nothing to preserve either way).
fn extract_targets(
    ctrl: &mut ActiveController,
    kind: ControllerKind,
) -> Option<crate::controllers::targets::ControllerTargets> {
    if !rebuild_preserves_targets(kind) {
        return None;
    }
    match ctrl.0.targets() {
        crate::controllers::targets::ControllerTargets::None => None,
        targets => Some(targets),
    }
}

/// Replay a snapshot taken by `extract_targets` onto the freshly rebuilt
/// controller. `FlightController::apply_targets` already no-ops on a mismatched
/// variant, so this is safe even if `kind.build()`'s fallback ever changes.
fn restore_targets(
    ctrl: &mut ActiveController,
    targets: crate::controllers::targets::ControllerTargets,
    state: &FlightState,
) {
    ctrl.0.apply_targets(&targets, state);
}

/// Apply the named tuning profile to a controller that was spawned before the PlaneTuning asset
/// finished loading. Runs once per entity (guarded by `Without<TuningApplied>`) and fires before
/// `apply_controller_switch` so any explicit profile switch in the same frame takes precedence.
fn apply_initial_tuning(
    mut query: Query<
        (
            Entity,
            &FlightState,
            &mut ActiveController,
            &ControllerKind,
            &ControlInputs,
            &PlaneTuningHandle,
            &SelectedTuningProfile,
        ),
        Without<TuningApplied>,
    >,
    tuning_assets: Res<Assets<PlaneTuning>>,
    mut commands: Commands,
) {
    for (entity, state, mut controller, kind, prev_inputs, tuning_handle, profile) in
        query.iter_mut()
    {
        let Some(pt) = tuning_assets.get(&tuning_handle.0) else {
            continue;
        };
        let profile_name = profile.0.as_str();
        let tuning: Option<&dyn ControllerTuning> = match *kind {
            ControllerKind::Orbit
            | ControllerKind::RlOrbit
            | ControllerKind::RlOrbitResidual
            | ControllerKind::RlLstmOrbit => pt
                .get_orbit(profile_name)
                .map(|t| t as &dyn ControllerTuning),
            // Bug fix: this arm was missing entirely, so a heading-hold plane got
            // `level_hold` gains here on the tuning-asset-load frame while
            // `apply_controller_switch` (below) correctly used `heading_hold` gains —
            // divergent tuning depending on which system fired first.
            ControllerKind::HeadingHold | ControllerKind::RlHeadingHold => pt
                .get_heading_hold(profile_name)
                .map(|t| t as &dyn ControllerTuning),
            _ => pt
                .get_level_hold(profile_name)
                .map(|t| t as &dyn ControllerTuning),
        };

        // Preserve orbit geometry: extract before rebuild, restore after.
        let orbit_params = if matches!(
            *kind,
            ControllerKind::Orbit
                | ControllerKind::RlOrbit
                | ControllerKind::RlOrbitResidual
                | ControllerKind::RlLstmOrbit
        ) {
            extract_orbit_params(&mut controller)
        } else {
            None
        };
        // Preserve wingman formation state the same way: `Wingman.build()` can't
        // reconstruct it (no leader reference in the generic factory).
        let wingman_params = if *kind == ControllerKind::Wingman {
            extract_wingman_params(&mut controller)
        } else {
            None
        };
        // Preserve LevelHold/HeadingHold/Ascent setpoints the same way: `build()`
        // re-seeds them from `state`, which would silently cancel a scenario- or
        // pilot-commanded target the moment tuning finishes loading.
        let saved_targets = extract_targets(&mut controller, *kind);

        // Preserve a live RL policy the same way: `kind.build()` has no model path and
        // would fall back to a PID controller for any RL kind.
        #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
        {
            let orbit_tuning = pt.get_orbit(profile_name);
            if preserve_rl_controller(&mut controller, *kind, state, orbit_tuning) {
                commands.entity(entity).insert(TuningApplied);
                continue;
            }
        }

        controller.0 = kind.build(state, tuning, prev_inputs);

        if let Some(params) = orbit_params {
            if let Some(orbit) = controller.0.as_any_mut().downcast_mut::<OrbitController>() {
                orbit.apply_params(&params, state.airspeed);
            }
        }
        if let Some(params) = wingman_params {
            restore_wingman(&mut controller, params);
        }
        if let Some(targets) = saved_targets {
            restore_targets(&mut controller, targets, state);
        }

        commands.entity(entity).insert(TuningApplied);
    }
}

/// Rebuild `ActiveController` whenever `ControllerKind` or `SelectedTuningProfile` changes.
/// Runs in `PostUpdate` so both `switch_controller` (Update) and `draw_flight_hud` (Update)
/// have already written their changes before this system fires.
fn apply_controller_switch(
    mut query: Query<
        (
            &FlightState,
            &mut ActiveController,
            Ref<ControllerKind>,
            &ControlInputs,
            Option<&PlaneTuningHandle>,
            Option<Ref<SelectedTuningProfile>>,
        ),
        Or<(Changed<ControllerKind>, Changed<SelectedTuningProfile>)>,
    >,
    tuning_assets: Res<Assets<PlaneTuning>>,
) {
    for (state, mut controller, kind, prev_inputs, tuning_handle, profile) in query.iter_mut() {
        // Skip entities that matched only because components were just inserted at spawn —
        // their controllers were already constructed correctly before spawning.
        let kind_mutated = kind.is_changed() && !kind.is_added();
        let profile_mutated = profile
            .as_ref()
            .map(|p| p.is_changed() && !p.is_added())
            .unwrap_or(false);
        if !kind_mutated && !profile_mutated {
            continue;
        }
        let profile_name = profile.as_deref().map(|p| p.0.as_str()).unwrap_or("normal");
        let tuning: Option<&dyn ControllerTuning> = tuning_handle
            .and_then(|h| tuning_assets.get(&h.0))
            .and_then(|pt| match *kind {
                ControllerKind::Orbit
                | ControllerKind::RlOrbit
                | ControllerKind::RlOrbitResidual
                | ControllerKind::RlLstmOrbit => pt
                    .get_orbit(profile_name)
                    .map(|t| t as &dyn ControllerTuning),
                ControllerKind::HeadingHold | ControllerKind::RlHeadingHold => pt
                    .get_heading_hold(profile_name)
                    .map(|t| t as &dyn ControllerTuning),
                _ => pt
                    .get_level_hold(profile_name)
                    .map(|t| t as &dyn ControllerTuning),
            });

        // Preserve orbit geometry when switching between orbit variants.
        // None when the current controller is not an orbit type (e.g. LevelHold → Orbit
        // stays with the from_state() auto-center default).
        let orbit_params = if matches!(
            *kind,
            ControllerKind::Orbit
                | ControllerKind::RlOrbit
                | ControllerKind::RlOrbitResidual
                | ControllerKind::RlLstmOrbit
        ) {
            extract_orbit_params(&mut controller)
        } else {
            None
        };
        // Preserve wingman formation state across a profile switch the same way.
        // `None` when the kind just changed *to* Wingman this frame (the current
        // controller isn't a wingman yet) — the LevelHold fallback stands, and
        // `cleanup_orphaned_wingmen` will demote the kind honestly next tick.
        let wingman_params = if *kind == ControllerKind::Wingman {
            extract_wingman_params(&mut controller)
        } else {
            None
        };
        // Preserve LevelHold/HeadingHold/Ascent setpoints across a profile switch the
        // same way (see `apply_initial_tuning` above for why).
        let saved_targets = extract_targets(&mut controller, *kind);

        // Preserve a live RL policy the same way: `kind.build()` has no model path and
        // would fall back to a PID controller for any RL kind.
        #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
        {
            let orbit_tuning = tuning_handle
                .and_then(|h| tuning_assets.get(&h.0))
                .and_then(|pt| pt.get_orbit(profile_name));
            if preserve_rl_controller(&mut controller, *kind, state, orbit_tuning) {
                continue;
            }
        }

        controller.0 = kind.build(state, tuning, prev_inputs);

        // Apply preserved geometry to the freshly-built OrbitController (always the PID
        // fallback produced by build() for orbit kinds). No-op when orbit_params is None.
        if let Some(params) = orbit_params {
            if let Some(orbit) = controller.0.as_any_mut().downcast_mut::<OrbitController>() {
                orbit.apply_params(&params, state.airspeed);
            }
        }
        if let Some(params) = wingman_params {
            restore_wingman(&mut controller, params);
        }
        if let Some(targets) = saved_targets {
            restore_targets(&mut controller, targets, state);
        }
    }
}

/// Swap a `FlightPlan` plane's PID-orbit fallback for an `L1Controller` once its
/// `.plan.ron` asset finishes loading.
///
/// `ControllerKind::build()` cannot construct an `L1Controller` (it lacks the
/// plan asset), so it returns a PID orbit. This system runs after
/// `apply_controller_switch`, detects planes whose kind is `FlightPlan` but
/// whose active controller is not yet an `L1Controller`, and builds the real
/// controller from the loaded plan — preserving leg progress once installed by
/// leaving existing `L1Controller`s untouched.
fn apply_flight_plan(
    mut query: Query<(
        &FlightState,
        &mut ActiveController,
        &ControllerKind,
        &ControlInputs,
        &FlightPlanHandle,
    )>,
    plans: Res<Assets<FlightPlan>>,
) {
    for (state, mut controller, kind, prev_inputs, handle) in query.iter_mut() {
        if *kind != ControllerKind::FlightPlan {
            continue;
        }
        // Already running the plan — keep it so leg progress is preserved.
        if controller
            .0
            .as_any_mut()
            .downcast_mut::<L1Controller>()
            .is_some()
        {
            continue;
        }
        let Some(plan) = plans.get(&handle.0) else {
            continue; // asset still loading; the PID orbit fallback keeps flying
        };
        controller.0 = Box::new(L1Controller::from_plan(state, plan.clone(), prev_inputs));
    }
}

#[cfg(all(test, feature = "inference", not(target_arch = "wasm32")))]
mod tests {
    use super::*;

    #[test]
    fn rl_kind_load_gate() {
        use ControllerKind::*;
        // Runtime (panel/hotkey) spawn: added, no model wired yet → load.
        assert!(rl_kind_needs_load_on_change(RlOrbit, true, false));
        // Startup spawn: added but already carries a SelectedModel → skip.
        assert!(!rl_kind_needs_load_on_change(RlOrbit, true, true));
        // Interactive cycle to an RL kind: not added → load.
        assert!(rl_kind_needs_load_on_change(RlOrbit, false, false));
        // Every RL kind is covered.
        for k in [
            RlLevelHold,
            RlOrbit,
            RlOrbitResidual,
            RlLstmOrbit,
            RlHeadingHold,
        ] {
            assert!(rl_kind_needs_load_on_change(k, true, false));
        }
        // Non-RL kinds are never handled here.
        assert!(!rl_kind_needs_load_on_change(Orbit, true, false));
        assert!(!rl_kind_needs_load_on_change(LevelHold, false, false));
    }

    #[test]
    fn model_path_guard_accepts_legitimate_stem() {
        assert!(model_path_matches_dir("models/orbit/ppo_orbit_1", "orbit"));
        assert!(!model_path_matches_dir("models/level_hold/ppo_x", "orbit"));
    }

    #[test]
    fn model_path_guard_rejects_traversal() {
        // A bare `starts_with` prefix check lets `..` walk straight out of
        // `models/<dir>/`, which then reaches burn's `File::open`.
        for path in [
            "models/orbit/../../../etc/passwd",
            "models/orbit/../../secrets",
            "models/orbit/sub/../../../../etc/shadow",
        ] {
            assert!(
                !model_path_matches_dir(path, "orbit"),
                "traversal must not satisfy the model-dir guard: {path}"
            );
        }
    }

    #[test]
    fn heading_hold_targets_from_controller_reads_heading_hold_controller() {
        let state = FlightState {
            altitude: 1000.0,
            airspeed: 100.0,
            velocity: bevy::math::Vec3::new(100.0, 0.0, 0.0),
            attitude: bevy::math::Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            ..Default::default()
        };
        let mut ctrl = ActiveController(Box::new(HeadingHoldController::new(&state, 0.7)));
        let (heading, alt, spd) = heading_hold_targets_from_controller(&mut ctrl, &state);
        assert!((heading - 0.7).abs() < 1e-5);
        assert_eq!(alt, state.altitude);
        assert_eq!(spd, state.airspeed);
    }

    #[test]
    fn heading_hold_targets_from_controller_falls_back_to_state() {
        let state = FlightState {
            altitude: 2000.0,
            airspeed: 90.0,
            velocity: bevy::math::Vec3::new(90.0, 0.0, 0.0),
            attitude: bevy::math::Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            ..Default::default()
        };
        let mut ctrl = ActiveController(Box::new(crate::controllers::ManualController::new()));
        let (heading, alt, spd) = heading_hold_targets_from_controller(&mut ctrl, &state);
        assert!((heading - ground_track_heading(&state)).abs() < 1e-5);
        assert_eq!(alt, state.altitude);
        assert_eq!(spd, state.airspeed);
    }
}
