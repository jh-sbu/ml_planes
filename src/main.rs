use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

// Always-on surface: the menu spawns scenarios, so the binary itself only needs
// the core plugins + the model library. Gameplay/controller types are gated to
// the visual build that runs the systems using them.
use ml_planes::controllers::ModelLibrary;
use ml_planes::environment::{EnvironmentPlugin, LifecyclePlugin};
use ml_planes::plane::PlanePlugin;

#[cfg(feature = "visual")]
use ml_planes::controllers::{
    ActiveController, ControllerKind, ControllerTuning, FlightPlan, L1Controller, OrbitController,
    OrbitParams, PlaneTuning, SelectedTuningProfile, TuningApplied,
};
#[cfg(feature = "visual")]
use ml_planes::plane::{ControlInputs, FlightPlanHandle, FlightState, PlaneTuningHandle};

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
use ml_planes::controllers::OrbitTuning;
#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
#[allow(unused_imports)]
use ml_planes::controllers::{
    LevelHoldController, ModelLoadError, RlLevelHoldController, RlLstmOrbitConfig,
    RlLstmOrbitController, RlOrbitConfig, RlOrbitController, RlOrbitResidualConfig,
    RlOrbitResidualController, SelectedModel,
};
#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
use ml_planes::ui::notifications::Notifications;

#[cfg(feature = "visual")]
use bevy::asset::{AssetMetaCheck, AssetPlugin};
#[cfg(feature = "visual")]
use bevy_egui::EguiPlugin;
#[cfg(feature = "visual")]
use ml_planes::camera::{CameraMode, CameraPlugin};
#[cfg(feature = "visual")]
use ml_planes::ui::{AppState, UiPlugin};

fn main() {
    let mut app = App::new();

    // This project does not use Bevy .meta sidecar files; skipping the check
    // eliminates spurious HTTP 404 errors in the WASM build and is safe because
    // no asset requires custom loader settings or processing overrides.
    #[cfg(feature = "visual")]
    app.add_plugins(DefaultPlugins.set(AssetPlugin {
        meta_check: AssetMetaCheck::Never,
        ..default()
    }));
    #[cfg(not(feature = "visual"))]
    app.add_plugins((
        MinimalPlugins,
        bevy::asset::AssetPlugin::default(),
        bevy::transform::TransformPlugin,
    ));

    app.insert_resource(TimestepMode::Fixed {
        dt: 1.0 / 64.0,
        substeps: 1,
    });
    app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule());
    app.add_plugins(PlanePlugin);
    app.add_plugins(EnvironmentPlugin);
    app.add_plugins(LifecyclePlugin);

    #[cfg(feature = "visual")]
    {
        app.add_plugins(EguiPlugin::default());
        app.add_plugins(CameraPlugin);
        app.add_plugins(UiPlugin);
    }

    app.init_resource::<ModelLibrary>();
    // The visual app boots into the main menu; the scenario it picks is spawned
    // on `OnEnter(AppState::InGame)` by the `MenuPlugin` (added via `UiPlugin`).
    // The headless build has no menu, so it spawns nothing automatically.
    #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
    app.add_systems(Startup, scan_models);

    // Gameplay control/tuning systems only run while a scenario is flying, so the
    // menu screens don't drive controller switching, input polling, etc.
    #[cfg(feature = "visual")]
    app.add_systems(
        Update,
        (
            poll_controller_inputs,
            switch_controller,
            cycle_tune_profile,
        )
            .run_if(in_state(AppState::InGame)),
    );
    #[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
    app.add_systems(Update, cycle_rl_model.run_if(in_state(AppState::InGame)));

    #[cfg(feature = "visual")]
    app.add_systems(
        PostUpdate,
        (
            apply_initial_tuning,
            apply_controller_switch.after(apply_initial_tuning),
            apply_flight_plan.after(apply_controller_switch),
        )
            .run_if(in_state(AppState::InGame)),
    );
    #[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
    app.add_systems(
        PostUpdate,
        (
            apply_rl_controller_switch.after(apply_controller_switch),
            apply_model_switch,
        )
            .run_if(in_state(AppState::InGame)),
    );

    app.run();
}

#[cfg(all(
    test,
    feature = "visual",
    feature = "inference",
    not(target_arch = "wasm32")
))]
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
        for k in [RlLevelHold, RlOrbit, RlOrbitResidual, RlLstmOrbit] {
            assert!(rl_kind_needs_load_on_change(k, true, false));
        }
        // Non-RL kinds are never handled here.
        assert!(!rl_kind_needs_load_on_change(Orbit, true, false));
        assert!(!rl_kind_needs_load_on_change(LevelHold, false, false));
    }
}

#[cfg(feature = "visual")]
fn poll_controller_inputs(
    mut query: Query<&mut ActiveController, With<FlightState>>,
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
) {
    let dt = time.delta_secs();
    for mut controller in query.iter_mut() {
        controller.0.poll_input(&keys, dt);
    }
}

/// C key: cycle through available controller kinds for every plane.
#[cfg(feature = "visual")]
fn switch_controller(
    mut query: Query<&mut ControllerKind, With<FlightState>>,
    keys: Res<ButtonInput<KeyCode>>,
) {
    if keys.just_pressed(KeyCode::KeyC) {
        for mut kind in query.iter_mut() {
            let next = kind.next();
            kind.set_if_neq(next);
        }
    }
}

/// T / Shift+T: cycle tune profile forward/backward for the followed plane.
#[cfg(feature = "visual")]
fn cycle_tune_profile(
    mode: Res<CameraMode>,
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<(
        &ControllerKind,
        Option<&mut SelectedTuningProfile>,
        Option<&PlaneTuningHandle>,
    )>,
    tuning_assets: Res<Assets<PlaneTuning>>,
) {
    if !keys.just_pressed(KeyCode::KeyT) {
        return;
    }
    let CameraMode::Follow(entity) = *mode else {
        return;
    };
    let Ok((kind, Some(mut profile), Some(handle))) = query.get_mut(entity) else {
        return;
    };
    let Some(pt) = tuning_assets.get(&handle.0) else {
        return;
    };
    let mut names: Vec<&str> = match *kind {
        ControllerKind::LevelHold | ControllerKind::RlLevelHold => {
            pt.level_hold.keys().map(|s| s.as_str()).collect()
        }
        ControllerKind::Orbit | ControllerKind::RlOrbit => {
            pt.orbit.keys().map(|s| s.as_str()).collect()
        }
        ControllerKind::HeadingHold => pt.heading_hold.keys().map(|s| s.as_str()).collect(),
        _ => return,
    };
    names.sort();
    if names.is_empty() {
        return;
    }
    let n = names.len();
    let cur = names.iter().position(|&nm| nm == profile.0).unwrap_or(0);
    let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    let next = if shift {
        (cur + n - 1) % n
    } else {
        (cur + 1) % n
    };
    profile.set_if_neq(SelectedTuningProfile(names[next].to_string()));
}

/// T / Shift+T: cycle RL model forward/backward for the followed plane.
#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
fn cycle_rl_model(
    mode: Res<CameraMode>,
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&ControllerKind, Option<&mut SelectedModel>)>,
    model_lib: Res<ModelLibrary>,
) {
    if !keys.just_pressed(KeyCode::KeyT) {
        return;
    }
    let CameraMode::Follow(entity) = *mode else {
        return;
    };
    let Ok((kind, Some(mut sel))) = query.get_mut(entity) else {
        return;
    };
    let Some(dir) = kind.model_dir() else { return };
    let Some(available) = model_lib.0.get(dir) else {
        return;
    };
    if available.is_empty() {
        return;
    }
    let n = available.len();
    let cur = available.iter().position(|p| p == &sel.0).unwrap_or(0);
    let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    sel.0 = available[if shift {
        (cur + n - 1) % n
    } else {
        (cur + 1) % n
    }]
    .clone();
}

/// Scan `models/<category>/` subdirectories at startup and populate `ModelLibrary`.
#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
fn scan_models(mut commands: Commands) {
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
#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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
            _ => {}
        }
    }
}

/// Log a model-load failure and surface a transient HUD banner. Used when an
/// incompatible (e.g. stale-dimension) checkpoint is skipped and the controller
/// keeps its current (PID fallback) behavior.
#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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
/// skipped to avoid a redundant reload.
#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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
    );
    is_rl && (!kind_added || !has_selected_model)
}

/// When `ControllerKind` changes to an RL kind, load the actual RL model,
/// overriding the PID fallback that `apply_controller_switch` produces.
/// If the entity lacks `SelectedModel`, inserts a default so `apply_model_switch`
/// loads the controller on the next frame.
#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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
                    Err(e) => report_skipped_model(&mut notes, &path, &e),
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
            _ => {}
        }
    }
}

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
fn model_path_matches_dir(path: &str, dir: &str) -> bool {
    path.starts_with(&format!("models/{dir}/"))
}

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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
    if let Some(orbit) = controller
        .0
        .as_any_mut()
        .downcast_mut::<ml_planes::controllers::OrbitController>()
    {
        return RlOrbitConfig::from_orbit(orbit);
    }
    RlOrbitConfig::from_state(state)
}

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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
    if let Some(orbit) = controller
        .0
        .as_any_mut()
        .downcast_mut::<ml_planes::controllers::OrbitController>()
    {
        return RlOrbitResidualConfig::from_orbit(orbit);
    }
    RlOrbitResidualConfig::from_state(state)
}

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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
    if let Some(orbit) = controller
        .0
        .as_any_mut()
        .downcast_mut::<ml_planes::controllers::OrbitController>()
    {
        return RlLstmOrbitConfig::from_orbit(orbit);
    }
    RlLstmOrbitConfig::from_state(state)
}

/// Extract orbit geometry from whatever controller variant is currently active.
///
/// Returns `None` when the active controller is not an orbit variant (e.g. LevelHold,
/// Manual), so the caller can fall back to `from_state()` geometry as before.
#[cfg(feature = "visual")]
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

/// Apply the named tuning profile to a controller that was spawned before the PlaneTuning asset
/// finished loading. Runs once per entity (guarded by `Without<TuningApplied>`) and fires before
/// `apply_controller_switch` so any explicit profile switch in the same frame takes precedence.
#[cfg(feature = "visual")]
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

        controller.0 = kind.build(state, tuning, prev_inputs);

        if let Some(params) = orbit_params {
            if let Some(orbit) = controller.0.as_any_mut().downcast_mut::<OrbitController>() {
                orbit.apply_params(&params, state.airspeed);
            }
        }

        commands.entity(entity).insert(TuningApplied);
    }
}

/// Rebuild `ActiveController` whenever `ControllerKind` or `SelectedTuningProfile` changes.
/// Runs in `PostUpdate` so both `switch_controller` (Update) and `draw_flight_hud` (Update)
/// have already written their changes before this system fires.
#[cfg(feature = "visual")]
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
                ControllerKind::HeadingHold => pt
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

        controller.0 = kind.build(state, tuning, prev_inputs);

        // Apply preserved geometry to the freshly-built OrbitController (always the PID
        // fallback produced by build() for orbit kinds). No-op when orbit_params is None.
        if let Some(params) = orbit_params {
            if let Some(orbit) = controller.0.as_any_mut().downcast_mut::<OrbitController>() {
                orbit.apply_params(&params, state.airspeed);
            }
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
#[cfg(feature = "visual")]
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
