//! RL inference integration tests — requires the `inference` feature. Module of the
//! consolidated `rl` test binary (see plans/test_compile_speed.md).
//!   cargo test --no-default-features --features inference --test rl rl_inference::
use bevy::math::{Quat, Vec3};
use burn::backend::NdArray;
use burn::module::Module;
use burn::record::{DefaultFileRecorder, FullPrecisionSettings, NamedMpkBytesRecorder, Recorder};
use ml_planes::controllers::orbit::{OrbitDirection, OrbitParams, ORBIT_OBS_DIM};
use ml_planes::controllers::{
    ControllerTargets, FlightController, ModelLoadError, RlHeadingHoldConfig,
    RlHeadingHoldController, RlLevelHoldController, RlOrbitConfig, RlOrbitController,
};
use ml_planes::plane::{ControllerContext, FlightState, PlaneId};
use ml_planes::training::heading_hold_env::{heading_hold_observation, HEADING_HOLD_OBS_DIM};
use ml_planes::training::level_hold_env::{level_hold_observation, LEVEL_HOLD_OBS_DIM};
use ml_planes::training::ppo::model::ActorCritic;

type InfB = NdArray;

/// Save a freshly-initialized `ActorCritic` of `obs_dim` to a unique temp path
/// (without the `.mpk` suffix, as the loaders expect) and return that path.
fn save_stale_model(obs_dim: usize, tag: &str) -> std::path::PathBuf {
    let device: <InfB as burn::tensor::backend::Backend>::Device = Default::default();
    let path = std::env::temp_dir().join(format!(
        "ml_planes_stale_{tag}_{obs_dim}_{}",
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

const ORBIT_MPK: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/models/orbit/ppo_orbit_1.mpk"
));

#[test]
fn load_bytes_orbit_produces_valid_controller() {
    let config = RlOrbitConfig {
        center_x: 0.0,
        center_z: 0.0,
        target_radius: 3000.0,
        target_altitude: 800.0,
        target_airspeed: 100.0,
        direction: OrbitDirection::CounterClockwise,
    };
    let _ = RlOrbitController::load_bytes(ORBIT_MPK, config).expect("load_bytes must succeed");
}

/// The embedded orbit checkpoint must match the current observation dimension: a
/// real forward pass feeds an `ORBIT_OBS_DIM`-wide observation through the loaded
/// network. Guards against a stale-dimension checkpoint (which loads but would panic
/// in the matmul) after the fuel observation was appended (13 → 14).
#[test]
fn loaded_orbit_policy_runs_forward_pass() {
    let config = RlOrbitConfig {
        center_x: 0.0,
        center_z: 0.0,
        target_radius: 3000.0,
        target_altitude: 800.0,
        target_airspeed: 100.0,
        direction: OrbitDirection::CounterClockwise,
    };
    let mut ctrl = RlOrbitController::load_bytes(ORBIT_MPK, config).expect("load_bytes");

    let mut state = FlightState {
        position: Vec3::new(3000.0, 800.0, 0.0),
        velocity: Vec3::new(0.0, 0.0, 100.0),
        attitude: Quat::IDENTITY,
        ..Default::default()
    };
    state.update_air_data();

    let inputs = ctrl.update(
        &state,
        &ControllerContext::empty_for(PlaneId::TEST),
        1.0 / 64.0,
    );
    assert!(
        inputs.elevator.is_finite()
            && inputs.throttle.is_finite()
            && inputs.aileron.is_finite()
            && inputs.rudder.is_finite(),
        "controls must be finite: {inputs:?}"
    );
}

/// A stale-dimension orbit checkpoint (13-dim, pre-fuel) must be rejected at load
/// time with `DimensionMismatch` rather than loading `Ok` and panicking later in
/// the forward-pass matmul.
#[test]
fn loading_stale_dim_orbit_model_errors() {
    let path = save_stale_model(ORBIT_OBS_DIM - 1, "orbit");
    let config = RlOrbitConfig {
        center_x: 0.0,
        center_z: 0.0,
        target_radius: 3000.0,
        target_altitude: 800.0,
        target_airspeed: 100.0,
        direction: OrbitDirection::CounterClockwise,
    };
    let result = RlOrbitController::load(path.to_str().unwrap(), config);
    assert!(
        matches!(
            result,
            Err(ModelLoadError::DimensionMismatch {
                expected,
                found,
            }) if expected == ORBIT_OBS_DIM && found == ORBIT_OBS_DIM - 1
        ),
        "expected DimensionMismatch, got {:?}",
        result.as_ref().map(|_| "Ok"),
    );
    let _ = std::fs::remove_file(path.with_extension("mpk"));
}

/// Same guard for the level-hold controller (pre-envelope-randomization
/// 11-dim stale checkpoint, from before `density_ratio`/airspeed were
/// appended to the observation).
#[test]
fn loading_stale_dim_level_hold_model_errors() {
    let path = save_stale_model(LEVEL_HOLD_OBS_DIM - 2, "level_hold");
    let result = RlLevelHoldController::load(path.to_str().unwrap(), 1000.0, 100.0);
    assert!(
        matches!(
            result,
            Err(ModelLoadError::DimensionMismatch { expected, found })
                if expected == LEVEL_HOLD_OBS_DIM && found == LEVEL_HOLD_OBS_DIM - 2
        ),
        "expected DimensionMismatch, got {:?}",
        result.as_ref().map(|_| "Ok"),
    );
    let _ = std::fs::remove_file(path.with_extension("mpk"));
}

/// The `level_hold_observation` builder `RlLevelHoldController::update` calls
/// must stay in lockstep with `LEVEL_HOLD_OBS_DIM` and the density-ratio /
/// airspeed elements the envelope-randomization rework appended — a cheap
/// regression guard on the single shared builder (env and controller no
/// longer keep independent copies of this vector).
#[test]
fn level_hold_controller_obs_matches_env_obs() {
    let mut state = FlightState {
        position: Vec3::new(0.0, 2500.0, 0.0),
        velocity: Vec3::new(110.0, 1.0, 0.0),
        attitude: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
        angular_velocity: Vec3::new(0.01, 0.02, 0.03),
        ..Default::default()
    };
    state.update_air_data();

    let obs = level_hold_observation(&state, 1000.0, 100.0);
    assert_eq!(obs.len(), LEVEL_HOLD_OBS_DIM);
    assert_eq!(
        obs[11],
        ml_planes::aerodynamics::density_ratio(state.altitude),
        "obs[11] must be the raw density ratio"
    );
    assert_eq!(
        obs[12],
        state.airspeed / 100.0,
        "obs[12] must be raw airspeed / AIRSPEED_OBS_SCALE"
    );
}

/// `RlOrbitController` must report its setpoints through the shared `Orbit` variant of
/// `ControllerTargets` — the HUD (and the replicated read path) treats it exactly like a
/// PID `OrbitController` with the same geometry, so the widgets stay reachable across an
/// RL-load transition.
#[test]
fn rl_orbit_targets_use_the_shared_orbit_variant() {
    let config = RlOrbitConfig {
        center_x: 10.0,
        center_z: -20.0,
        target_radius: 3000.0,
        target_altitude: 800.0,
        target_airspeed: 100.0,
        direction: OrbitDirection::CounterClockwise,
    };
    let ctrl = RlOrbitController::load_bytes(ORBIT_MPK, config).expect("load_bytes");
    assert_eq!(
        ctrl.targets(),
        ControllerTargets::Orbit(OrbitParams {
            center_x: 10.0,
            center_z: -20.0,
            target_radius: 3000.0,
            target_altitude: 800.0,
            target_airspeed: 100.0,
            direction: OrbitDirection::CounterClockwise,
        })
    );
}

/// Same guard for the heading-hold controller: a stale-dimension checkpoint
/// (e.g. saved before the turn-rate element was appended) must be rejected
/// at load time, not panic in the forward pass.
#[test]
fn loading_stale_dim_heading_hold_model_errors() {
    let path = save_stale_model(HEADING_HOLD_OBS_DIM - 2, "heading_hold");
    let config = RlHeadingHoldConfig {
        target_heading: 0.5,
        target_altitude: 1000.0,
        target_airspeed: 120.0,
    };
    let result = RlHeadingHoldController::load(path.to_str().unwrap(), config);
    assert!(
        matches!(
            result,
            Err(ModelLoadError::DimensionMismatch { expected, found })
                if expected == HEADING_HOLD_OBS_DIM && found == HEADING_HOLD_OBS_DIM - 2
        ),
        "expected DimensionMismatch, got {:?}",
        result.as_ref().map(|_| "Ok"),
    );
    let _ = std::fs::remove_file(path.with_extension("mpk"));
}

/// `heading_hold_observation` is the single builder shared by `HeadingHoldEnv`
/// and `RlHeadingHoldController::update` — pin its shape and the heading-error
/// sin/cos elements so training and inference can't silently drift apart.
#[test]
fn heading_hold_controller_obs_matches_env_obs() {
    let mut state = FlightState {
        position: Vec3::new(0.0, 2500.0, 0.0),
        velocity: Vec3::new(110.0, 1.0, 0.0),
        attitude: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
        angular_velocity: Vec3::new(0.01, 0.02, 0.03),
        ..Default::default()
    };
    state.update_air_data();

    let target_heading = 1.0_f32;
    let obs = heading_hold_observation(&state, target_heading, 1000.0, 100.0);
    assert_eq!(obs.len(), HEADING_HOLD_OBS_DIM);

    let e = ml_planes::controllers::heading_hold::heading_error(&state, target_heading);
    assert!(
        (obs[13] - e.sin() / 0.5).abs() < 1e-6,
        "obs[13] must be sin(heading_error)/scale"
    );
    assert!(
        (obs[14] - e.cos()).abs() < 1e-6,
        "obs[14] must be cos(heading_error)"
    );
    let expected_turn_rate = (state.attitude * state.angular_velocity).y / 0.2;
    assert!(
        (obs[15] - expected_turn_rate).abs() < 1e-6,
        "obs[15] must be world-vertical turn rate / scale"
    );
}

/// `RlHeadingHoldController` must report its setpoints through the shared
/// `HeadingHold` variant of `ControllerTargets` — the same widget set the PID
/// `HeadingHoldController` uses, so the HUD stays reachable across an RL-load
/// transition.
#[test]
fn rl_heading_hold_targets_use_the_shared_heading_hold_variant() {
    let device: <InfB as burn::tensor::backend::Backend>::Device = Default::default();
    let path = std::env::temp_dir().join(format!(
        "ml_planes_heading_hold_targets_{}",
        std::process::id()
    ));
    ActorCritic::<InfB>::new(&device, HEADING_HOLD_OBS_DIM)
        .save_file(
            path.clone(),
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
        )
        .expect("save untrained model");

    let config = RlHeadingHoldConfig {
        target_heading: 0.7,
        target_altitude: 800.0,
        target_airspeed: 100.0,
    };
    let ctrl =
        RlHeadingHoldController::load(path.to_str().unwrap(), config).expect("load untrained");
    assert_eq!(
        ctrl.targets(),
        ControllerTargets::HeadingHold {
            heading: 0.7,
            altitude: 800.0,
            airspeed: 100.0,
        }
    );
    let _ = std::fs::remove_file(path.with_extension("mpk"));
}

#[test]
fn named_mpk_bytes_recorder_round_trips_orbit_model() {
    let device: <InfB as burn::tensor::backend::Backend>::Device = Default::default();
    let record = NamedMpkBytesRecorder::<FullPrecisionSettings>::default()
        .load(ORBIT_MPK.to_vec(), &device)
        .expect("must deserialize .mpk bytes");
    let _ = ActorCritic::<InfB>::new(&device, ORBIT_OBS_DIM).load_record(record);
}
