//! Pins that `SimControlPlugin`'s controller-rebuild systems (`apply_initial_tuning` /
//! `apply_controller_switch`, see `src/controllers/sim_control.rs`) preserve a loaded RL
//! policy instead of silently replacing it with the PID fallback `ControllerKind::build()`
//! produces for RL kinds.
//!
//! This mirrors `tests/core/sim_control.rs`'s wingman-survival tests
//! (`initial_tuning_preserves_wingman_controller` / `profile_switch_preserves_wingman_controller`)
//! — the same bug class, documented in `CLAUDE.md` under "A kind whose `build()` can't
//! reconstruct its full state", now closed for the RL kinds too.
//!
//! `RlLstmOrbit` is not covered here: no `models/lstm_orbit/` checkpoint is shipped, and
//! `LstmActorCritic` can't load a plain `ActorCritic` `.mpk` (different architecture), so
//! there's no fixture to embed. Its `preserve_rl_controller` arm is a one-line mirror of
//! `RlOrbit`'s (a plain downcast check, no PID to retune).
//!
//! Run: `cargo test --no-default-features --features inference --test rl rl_sim_control::`

use bevy::math::{Quat, Vec3};
use bevy::prelude::*;
use burn::backend::NdArray;
use burn::module::Module;
use burn::record::{DefaultFileRecorder, FullPrecisionSettings};
use burn::tensor::backend::Backend;

use ml_planes::controllers::orbit::OrbitDirection;
use ml_planes::controllers::{
    ActiveController, ControllerKind, HeadingHoldController, LevelHoldController, LevelHoldTuning,
    ModelLibrary, OrbitTuning, PlaneTuning, RlHeadingHoldConfig, RlHeadingHoldController,
    RlLevelHoldController, RlOrbitConfig, RlOrbitController, RlOrbitResidualConfig,
    RlOrbitResidualController, SelectedModel, SelectedTuningProfile, SimControlPlugin,
    TuningApplied,
};
use ml_planes::plane::{ControlInputs, FlightState, PlaneTuningHandle};
use ml_planes::training::heading_hold_env::HEADING_HOLD_OBS_DIM;
use ml_planes::training::level_hold_env::LEVEL_HOLD_OBS_DIM;
use ml_planes::training::ppo::model::ActorCritic;

use crate::common::build_headless_app_with;

type InfB = NdArray;

/// Save a freshly-initialized `ActorCritic` of `obs_dim` to a unique temp path (without the
/// `.mpk` extension, as the loaders expect) and return that path. Mirrors
/// `tests/rl/rl_inference.rs::save_stale_model`.
fn save_stale_model(obs_dim: usize, tag: &str) -> std::path::PathBuf {
    let device: <InfB as Backend>::Device = Default::default();
    let path = std::env::temp_dir().join(format!(
        "ml_planes_rl_sim_control_stale_{tag}_{obs_dim}_{}",
        std::process::id()
    ));
    ActorCritic::<InfB>::new(&device, obs_dim)
        .save_file(
            path.clone(),
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
        )
        .expect("save stale model");
    path
}

const LEVEL_HOLD_MPK: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/models/level_hold/ppo_level_hold.mpk"
));
const ORBIT_MPK: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/models/orbit/ppo_orbit_1.mpk"
));

fn level_state(altitude: f32, airspeed: f32) -> FlightState {
    let mut state = FlightState {
        position: Vec3::new(0.0, altitude, 0.0),
        velocity: Vec3::new(airspeed, 0.0, 0.0),
        attitude: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
        ..Default::default()
    };
    state.update_air_data();
    state
}

fn orbit_config() -> RlOrbitConfig {
    RlOrbitConfig {
        center_x: 0.0,
        center_z: 0.0,
        target_radius: 3000.0,
        target_altitude: 800.0,
        target_airspeed: 100.0,
        direction: OrbitDirection::CounterClockwise,
    }
}

fn heading_hold_config() -> RlHeadingHoldConfig {
    RlHeadingHoldConfig {
        target_heading: 0.5,
        target_altitude: 1000.0,
        target_airspeed: 120.0,
    }
}

fn residual_config() -> RlOrbitResidualConfig {
    RlOrbitResidualConfig {
        center_x: 0.0,
        center_z: 0.0,
        target_radius: 3000.0,
        target_altitude: 800.0,
        target_airspeed: 100.0,
        direction: OrbitDirection::CounterClockwise,
        residual_scale: 0.3,
    }
}

/// A `.tuning.ron`-equivalent asset carrying both a `level_hold` and an `orbit`
/// "normal" profile, distinguishable from defaults so a rebuild that discards the
/// loaded profile (or the loaded policy) is caught.
fn tuning_asset() -> PlaneTuning {
    let mut tuning = PlaneTuning::default();
    tuning.level_hold.insert(
        "normal".to_string(),
        LevelHoldTuning {
            alt_kp: 9.87,
            ..LevelHoldTuning::default()
        },
    );
    tuning.orbit.insert(
        "normal".to_string(),
        OrbitTuning {
            radial_kp: 0.042,
            ..OrbitTuning::default()
        },
    );
    tuning
}

/// `apply_initial_tuning` must leave an `RlLevelHoldController` in place — it has no PID
/// gains for a tuning profile to apply, so the correct behavior is simply "don't touch it".
///
/// Carries a `SelectedModel` at spawn, mirroring `spawn_resolved_scenario` — this matters
/// for the test, not just realism: without it, `apply_rl_controller_switch`'s spawn-frame
/// reload (see `rl_kind_needs_load_on_change`'s "no model wired yet" branch) would load a
/// *different*, real checkpoint off disk moments after `apply_initial_tuning` clobbers this
/// one, masking the very bug this test exists to catch.
#[test]
fn initial_tuning_preserves_rl_level_hold_controller() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    let state = level_state(1000.0, 100.0);
    let controller =
        RlLevelHoldController::load_bytes(LEVEL_HOLD_MPK, 1000.0, 100.0).expect("load model");

    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneTuning>>()
        .add(tuning_asset());

    let entity = app
        .world_mut()
        .spawn((
            state,
            ControlInputs::default(),
            ActiveController(Box::new(controller)),
            ControllerKind::RlLevelHold,
            SelectedModel("models/level_hold/ppo_level_hold".to_string()),
            PlaneTuningHandle(handle),
            SelectedTuningProfile("normal".to_string()),
        ))
        .id();

    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(entity).unwrap();
    assert!(
        ctrl.0
            .as_any_mut()
            .downcast_mut::<RlLevelHoldController>()
            .is_some(),
        "controller must still be an RlLevelHoldController after the tuning rebuild"
    );
    assert!(
        world.get::<TuningApplied>(entity).is_some(),
        "TuningApplied must be inserted so the rebuild doesn't repeat every frame"
    );
}

/// A later profile switch (`apply_controller_switch`) has the identical clobber risk.
#[test]
fn profile_switch_preserves_rl_level_hold_controller() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    let state = level_state(1000.0, 100.0);
    let controller =
        RlLevelHoldController::load_bytes(LEVEL_HOLD_MPK, 1000.0, 100.0).expect("load model");

    let mut tuning = tuning_asset();
    tuning.level_hold.insert(
        "aggressive".to_string(),
        LevelHoldTuning {
            alt_kp: 1.23,
            ..LevelHoldTuning::default()
        },
    );
    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneTuning>>()
        .add(tuning);

    let entity = app
        .world_mut()
        .spawn((
            state,
            ControlInputs::default(),
            ActiveController(Box::new(controller)),
            ControllerKind::RlLevelHold,
            SelectedModel("models/level_hold/ppo_level_hold".to_string()),
            PlaneTuningHandle(handle),
            SelectedTuningProfile("normal".to_string()),
        ))
        .id();

    // First update: apply_initial_tuning fires and picks up "normal".
    app.update();

    // Switch profiles — exactly what a server command handler / HUD does.
    app.world_mut()
        .get_mut::<SelectedTuningProfile>(entity)
        .unwrap()
        .set_if_neq(SelectedTuningProfile("aggressive".to_string()));
    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(entity).unwrap();
    assert!(
        ctrl.0
            .as_any_mut()
            .downcast_mut::<RlLevelHoldController>()
            .is_some(),
        "controller must still be an RlLevelHoldController after the profile switch"
    );
}

/// Same clobber, heading-hold family: `apply_initial_tuning`'s tuning-family match routes
/// `RlHeadingHold` through the (now-fixed) `heading_hold` tuning pool lookup, but
/// `preserve_rl_controller` must still intercept before `kind.build()` runs, since
/// `RlHeadingHoldController` has no PID gains for a tuning profile to apply to.
///
/// No checkpoint is shipped under `models/heading_hold/` yet (this feature's smoke
/// checkpoint lands separately), so this saves a freshly-initialized (untrained but
/// correctly-dimensioned) model to a temp path rather than embedding one via
/// `include_bytes!`, mirroring `save_stale_model` but at the *current* obs dim.
#[test]
fn initial_tuning_preserves_rl_heading_hold_controller() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    let state = level_state(1000.0, 120.0);
    let model_path = save_stale_model(HEADING_HOLD_OBS_DIM, "heading_hold_valid");
    let controller =
        RlHeadingHoldController::load(model_path.to_str().unwrap(), heading_hold_config())
            .expect("load model");

    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneTuning>>()
        .add(tuning_asset());

    let entity = app
        .world_mut()
        .spawn((
            state,
            ControlInputs::default(),
            ActiveController(Box::new(controller)),
            ControllerKind::RlHeadingHold,
            SelectedModel(model_path.to_str().unwrap().to_string()),
            PlaneTuningHandle(handle),
            SelectedTuningProfile("normal".to_string()),
        ))
        .id();

    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(entity).unwrap();
    assert!(
        ctrl.0
            .as_any_mut()
            .downcast_mut::<RlHeadingHoldController>()
            .is_some(),
        "controller must still be an RlHeadingHoldController after the tuning rebuild"
    );
    assert!(
        world.get::<TuningApplied>(entity).is_some(),
        "TuningApplied must be inserted so the rebuild doesn't repeat every frame"
    );

    let _ = std::fs::remove_file(model_path.with_extension("mpk"));
}

/// A later profile switch (`apply_controller_switch`) has the identical clobber risk.
#[test]
fn profile_switch_preserves_rl_heading_hold_controller() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    let state = level_state(1000.0, 120.0);
    let model_path = save_stale_model(HEADING_HOLD_OBS_DIM, "heading_hold_valid_switch");
    let controller =
        RlHeadingHoldController::load(model_path.to_str().unwrap(), heading_hold_config())
            .expect("load model");

    let mut tuning = tuning_asset();
    tuning.heading_hold.insert(
        "aggressive".to_string(),
        ml_planes::controllers::HeadingHoldTuning::default(),
    );
    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneTuning>>()
        .add(tuning);

    let entity = app
        .world_mut()
        .spawn((
            state,
            ControlInputs::default(),
            ActiveController(Box::new(controller)),
            ControllerKind::RlHeadingHold,
            SelectedModel(model_path.to_str().unwrap().to_string()),
            PlaneTuningHandle(handle),
            SelectedTuningProfile("normal".to_string()),
        ))
        .id();

    app.update();

    app.world_mut()
        .get_mut::<SelectedTuningProfile>(entity)
        .unwrap()
        .set_if_neq(SelectedTuningProfile("aggressive".to_string()));
    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(entity).unwrap();
    assert!(
        ctrl.0
            .as_any_mut()
            .downcast_mut::<RlHeadingHoldController>()
            .is_some(),
        "controller must still be an RlHeadingHoldController after the profile switch"
    );

    let _ = std::fs::remove_file(model_path.with_extension("mpk"));
}

/// Mirrors `rl_level_hold_load_failure_demotes_kind`: a failed heading-hold model load
/// must demote the kind label to `HeadingHold`, not leave it claiming `RlHeadingHold`
/// while actually flying PID.
#[test]
fn rl_heading_hold_load_failure_demotes_kind() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    app.update();

    let stale_path = save_stale_model(HEADING_HOLD_OBS_DIM - 2, "heading_hold");

    app.world_mut().insert_resource(ModelLibrary(
        [(
            "heading_hold".to_string(),
            vec![stale_path.to_str().unwrap().to_string()],
        )]
        .into_iter()
        .collect(),
    ));

    let state = level_state(1000.0, 120.0);
    let placeholder = HeadingHoldController::from_state(&state, &ControlInputs::default());

    let entity = app
        .world_mut()
        .spawn((
            state,
            ControlInputs::default(),
            ActiveController(Box::new(placeholder)),
            ControllerKind::RlHeadingHold,
        ))
        .id();

    app.update();

    let kind = app.world_mut().get::<ControllerKind>(entity).unwrap();
    assert_eq!(
        *kind,
        ControllerKind::HeadingHold,
        "a failed RL heading-hold model load must demote the kind label to HeadingHold"
    );

    let _ = std::fs::remove_file(stale_path.with_extension("mpk"));
}

/// Same clobber, orbit family: `apply_initial_tuning`'s orbit branch also routes RL orbit
/// kinds through `kind.build()`, which returns a PID `OrbitController`.
#[test]
fn initial_tuning_preserves_rl_orbit_controller() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    let state = level_state(800.0, 100.0);
    let controller = RlOrbitController::load_bytes(ORBIT_MPK, orbit_config()).expect("load model");

    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneTuning>>()
        .add(tuning_asset());

    let entity = app
        .world_mut()
        .spawn((
            state,
            ControlInputs::default(),
            ActiveController(Box::new(controller)),
            ControllerKind::RlOrbit,
            SelectedModel("models/orbit/ppo_orbit_1".to_string()),
            PlaneTuningHandle(handle),
            SelectedTuningProfile("normal".to_string()),
        ))
        .id();

    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(entity).unwrap();
    assert!(
        ctrl.0
            .as_any_mut()
            .downcast_mut::<RlOrbitController>()
            .is_some(),
        "controller must still be an RlOrbitController after the tuning rebuild"
    );
}

/// `RlOrbitResidual` is the one RL kind that genuinely owns PID gains (its inner PID
/// baseline). The rebuild must both preserve the policy *and* apply the newly loaded
/// `OrbitTuning` to that inner baseline — unlike the other RL kinds, "do nothing" is the
/// wrong behavior here.
#[test]
fn initial_tuning_retunes_rl_orbit_residual_baseline() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    let state = level_state(800.0, 100.0);
    // `load_bytes` seeds the inner PID from `tuning: None` (default gains) — the rebuild
    // must move it to the loaded profile's 0.042.
    let controller =
        RlOrbitResidualController::load_bytes(ORBIT_MPK, residual_config(), &state, None)
            .expect("load model");

    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneTuning>>()
        .add(tuning_asset());

    let entity = app
        .world_mut()
        .spawn((
            state,
            ControlInputs::default(),
            ActiveController(Box::new(controller)),
            ControllerKind::RlOrbitResidual,
            // No `models/orbit_residual/` checkpoint is shipped; the path just needs to
            // match the `model_dir()` prefix so `apply_rl_controller_switch` treats this
            // as "already carries its loaded controller" and doesn't touch it.
            SelectedModel("models/orbit_residual/fake".to_string()),
            PlaneTuningHandle(handle),
            SelectedTuningProfile("normal".to_string()),
        ))
        .id();

    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(entity).unwrap();
    let rl = ctrl
        .0
        .as_any_mut()
        .downcast_mut::<RlOrbitResidualController>()
        .expect("controller must still be an RlOrbitResidualController after the tuning rebuild");
    assert!(
        (rl.pid.radial_pid.kp - 0.042).abs() < 1e-6,
        "the loaded orbit tuning profile must reach the inner PID baseline, got {}",
        rl.pid.radial_pid.kp
    );
}

/// `apply_rl_controller_switch`'s three orbit-family arms demote `ControllerKind` back to
/// `Orbit` when a *resolved* checkpoint path fails to load (e.g. dimension mismatch), so the
/// HUD stops claiming "RL". The `RlLevelHold` arm was missing the equivalent demotion to
/// `LevelHold` — this reproduces that path via `ModelLibrary`, since `selected_or_default_model_path`
/// must actually resolve *some* path for `RlLevelHoldController::load` to be attempted (the
/// "no checkpoint available at all" case already demotes correctly, both before and after this
/// fix — see `selected_or_default_model_path`'s `None` arm in `sim_control.rs`).
#[test]
fn rl_level_hold_load_failure_demotes_kind() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    // Flush the `Startup`-scheduled `scan_models` (populates `ModelLibrary` from the real
    // `models/` dir) before any plane exists, so it can't race the entity spawned below.
    app.update();

    // A dimensionally-stale checkpoint (wrong obs dim) fails `check_obs_dim` inside `load`.
    let stale_path = save_stale_model(LEVEL_HOLD_OBS_DIM - 2, "level_hold");

    // Replace the real scanned library with just the stale path, so
    // `selected_or_default_model_path`'s `ModelLibrary` fallback resolves to it instead of a
    // real (valid) checkpoint on disk.
    app.world_mut().insert_resource(ModelLibrary(
        [(
            "level_hold".to_string(),
            vec![stale_path.to_str().unwrap().to_string()],
        )]
        .into_iter()
        .collect(),
    ));

    let state = level_state(1000.0, 100.0);
    let placeholder = LevelHoldController::from_state(&state, &ControlInputs::default());

    // Spawned on the PID fallback with no `SelectedModel`, mirroring a runtime
    // `SwitchControllerCommand` to `RlLevelHold` — `apply_rl_controller_switch` loads on the
    // spawn frame precisely because no `SelectedModel` is wired yet.
    let entity = app
        .world_mut()
        .spawn((
            state,
            ControlInputs::default(),
            ActiveController(Box::new(placeholder)),
            ControllerKind::RlLevelHold,
        ))
        .id();

    app.update();

    let kind = app.world_mut().get::<ControllerKind>(entity).unwrap();
    assert_eq!(
        *kind,
        ControllerKind::LevelHold,
        "a failed RL level-hold model load must demote the kind label to LevelHold, \
         matching the orbit-family arms' Orbit demotion"
    );

    let _ = std::fs::remove_file(stale_path.with_extension("mpk"));
}
