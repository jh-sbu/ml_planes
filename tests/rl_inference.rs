//! RL inference integration tests — requires the `inference` feature.
//!   cargo test --no-default-features --features inference --test rl_inference
#![cfg(any(feature = "inference", feature = "training"))]

use burn::backend::NdArray;
use burn::module::Module;
use burn::record::{FullPrecisionSettings, NamedMpkBytesRecorder, Recorder};
use ml_planes::controllers::orbit::{OrbitDirection, ORBIT_OBS_DIM};
use ml_planes::controllers::{RlOrbitConfig, RlOrbitController};
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

#[test]
fn named_mpk_bytes_recorder_round_trips_orbit_model() {
    let device: <InfB as burn::tensor::backend::Backend>::Device = Default::default();
    let record = NamedMpkBytesRecorder::<FullPrecisionSettings>::default()
        .load(ORBIT_MPK.to_vec(), &device)
        .expect("must deserialize .mpk bytes");
    let _ = ActorCritic::<InfB>::new(&device, ORBIT_OBS_DIM).load_record(record);
}
