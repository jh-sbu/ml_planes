//! RL level-hold controller: loads a trained PPO policy and implements FlightController.
//!
//! Uses the NdArray backend (CPU-only, no GPU required) for inference.
//! Only compiled when the `training` feature is active — gated via the parent mod declaration.

use std::any::Any;

use bevy::math::Quat;
#[cfg(not(target_arch = "wasm32"))]
use burn::record::DefaultFileRecorder;
use burn::{
    backend::NdArray,
    module::Module,
    record::{FullPrecisionSettings, NamedMpkBytesRecorder, Recorder},
    tensor::{backend::Backend, Tensor, TensorData},
};

use crate::controllers::FlightController;
use crate::plane::{ControlInputs, FlightState};
use crate::training::direct_action_to_inputs;
use crate::training::ppo::model::ActorCritic;

type InfB = NdArray;

/// Observation dimension for the level-hold policy. Must match
/// `LevelHoldEnv::observation_dim` and `build_obs` below.
pub const LEVEL_HOLD_OBS_DIM: usize = 11;

/// Trained PPO level-hold controller that runs inference on the CPU.
///
/// `ActorCritic<NdArray>` is not `Sync` (burn's `Param` uses `OnceCell`),
/// so we wrap in `Mutex` to satisfy `FlightController: Sync`.
pub struct RlLevelHoldController {
    model: std::sync::Mutex<ActorCritic<InfB>>,
    device: <InfB as Backend>::Device,
    pub target_altitude: f32,
    pub target_airspeed: f32,
}

impl RlLevelHoldController {
    /// Load weights from `path` (without `.mpk` extension) saved by `PpoTrainer::save_policy`.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn load(
        path: &str,
        target_altitude: f32,
        target_airspeed: f32,
    ) -> Result<Self, burn::record::RecorderError> {
        let device: <InfB as Backend>::Device = Default::default();
        let model = ActorCritic::<InfB>::new(&device, LEVEL_HOLD_OBS_DIM).load_file(
            path,
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
            &device,
        )?;
        Ok(Self {
            model: std::sync::Mutex::new(model),
            device,
            target_altitude,
            target_airspeed,
        })
    }

    /// Load weights from embedded bytes — for WASM builds where `std::fs` is unavailable.
    pub fn load_bytes(
        bytes: &[u8],
        target_altitude: f32,
        target_airspeed: f32,
    ) -> Result<Self, burn::record::RecorderError> {
        let device: <InfB as Backend>::Device = Default::default();
        let record = NamedMpkBytesRecorder::<FullPrecisionSettings>::default()
            .load(bytes.to_vec(), &device)?;
        let model = ActorCritic::<InfB>::new(&device, LEVEL_HOLD_OBS_DIM).load_record(record);
        Ok(Self {
            model: std::sync::Mutex::new(model),
            device,
            target_altitude,
            target_airspeed,
        })
    }
}

impl FlightController for RlLevelHoldController {
    fn update(
        &mut self,
        state: &FlightState,
        _ctx: &crate::plane::ControllerContext,
        _dt: f32,
    ) -> ControlInputs {
        let obs = build_obs(state, self.target_altitude, self.target_airspeed);
        let obs_t = Tensor::<InfB, 2>::from_data(
            TensorData::new(obs, vec![1, LEVEL_HOLD_OBS_DIM]),
            &self.device,
        );
        // Deterministic inference: use mean action (no sampling noise).
        let action_t = self.model.lock().unwrap().mean_action(obs_t);
        let action = action_t
            .into_data()
            .to_vec::<f32>()
            .expect("rl action data");

        // action = [elevator, throttle_norm, aileron, rudder]
        direct_action_to_inputs(&action)
    }

    fn name(&self) -> &'static str {
        "RlLevelHold"
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

/// Build normalized 10-dim observation matching `LevelHoldEnv::build_observation`.
fn build_obs(state: &FlightState, target_alt: f32, target_spd: f32) -> Vec<f32> {
    let alt_err = state.altitude - target_alt;
    let speed_err = state.airspeed - target_spd;
    let roll = roll_angle(state.attitude);
    let p = state.angular_velocity.x;
    let q = state.angular_velocity.y;
    let r = state.angular_velocity.z;
    vec![
        alt_err / 200.0,
        speed_err / 50.0,
        state.alpha / 0.5,
        q / 1.0,
        roll / 0.5,
        p / 1.0,
        state.beta / 0.5,
        r / 1.0,
        pitch_angle(state.attitude) / 0.5,
        state.velocity.y / 30.0,
        // Remaining fuel/charge fraction in [0, 1] (last element).
        state.fuel_fraction_obs(),
    ]
}

fn roll_angle(attitude: Quat) -> f32 {
    let right_world = attitude * bevy::math::Vec3::Y;
    let up_world = attitude * bevy::math::Vec3::Z;
    right_world.y.atan2(up_world.y)
}

fn pitch_angle(attitude: Quat) -> f32 {
    let fwd_world = attitude * bevy::math::Vec3::X;
    fwd_world
        .y
        .atan2((fwd_world.x * fwd_world.x + fwd_world.z * fwd_world.z).sqrt())
}
