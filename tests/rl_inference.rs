//! RL inference integration tests — requires the `inference` feature.
//!   cargo test --no-default-features --features inference --test rl_inference
#![cfg(any(feature = "inference", feature = "training"))]

use bevy::math::{Quat, Vec3};
use burn::backend::NdArray;
use burn::module::Module;
use burn::record::{DefaultFileRecorder, FullPrecisionSettings, NamedMpkBytesRecorder, Recorder};
use ml_planes::controllers::orbit::{OrbitDirection, ORBIT_OBS_DIM};
use ml_planes::controllers::{
    FlightController, ModelLoadError, RlLevelHoldController, RlOrbitConfig, RlOrbitController,
};
use ml_planes::plane::{ControllerContext, FlightState, PlaneId};
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

/// Same guard for the level-hold controller (10-dim stale checkpoint).
#[test]
fn loading_stale_dim_level_hold_model_errors() {
    let path = save_stale_model(10, "level_hold");
    let result = RlLevelHoldController::load(path.to_str().unwrap(), 1000.0, 100.0);
    assert!(
        matches!(
            result,
            Err(ModelLoadError::DimensionMismatch { expected, found })
                if expected == 11 && found == 10
        ),
        "expected DimensionMismatch, got {:?}",
        result.as_ref().map(|_| "Ok"),
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
