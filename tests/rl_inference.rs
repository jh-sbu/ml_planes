//! RL inference integration tests — requires the `inference` feature.
//!   cargo test --no-default-features --features inference --test rl_inference
#![cfg(any(feature = "inference", feature = "training"))]

use bevy::math::{Quat, Vec3};
use burn::backend::NdArray;
use burn::module::Module;
use burn::record::{FullPrecisionSettings, NamedMpkBytesRecorder, Recorder};
use ml_planes::controllers::orbit::{OrbitDirection, ORBIT_OBS_DIM};
use ml_planes::controllers::{FlightController, RlOrbitConfig, RlOrbitController};
use ml_planes::plane::{ControllerContext, FlightState, PlaneId};
use ml_planes::training::ppo::model::ActorCritic;

type InfB = NdArray;

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

#[test]
fn named_mpk_bytes_recorder_round_trips_orbit_model() {
    let device: <InfB as burn::tensor::backend::Backend>::Device = Default::default();
    let record = NamedMpkBytesRecorder::<FullPrecisionSettings>::default()
        .load(ORBIT_MPK.to_vec(), &device)
        .expect("must deserialize .mpk bytes");
    let _ = ActorCritic::<InfB>::new(&device, ORBIT_OBS_DIM).load_record(record);
}
