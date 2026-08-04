//! RL level-hold controller: loads a trained PPO policy and implements FlightController.
//!
//! Uses the NdArray backend (CPU-only, no GPU required) for inference.
//! Only compiled when the `training` feature is active — gated via the parent mod declaration.

use std::any::Any;

#[cfg(not(target_arch = "wasm32"))]
use burn::record::DefaultFileRecorder;
use burn::{
    backend::NdArray,
    module::Module,
    record::{FullPrecisionSettings, NamedMpkBytesRecorder, Recorder},
    tensor::{backend::Backend, Tensor, TensorData},
};

use crate::controllers::model_load::ModelLoadError;
use crate::controllers::FlightController;
use crate::plane::{ControlInputs, FlightState};
use crate::training::direct_action_to_inputs;
use crate::training::level_hold_env::{level_hold_observation, LEVEL_HOLD_OBS_DIM};
use crate::training::ppo::model::ActorCritic;

type InfB = NdArray;

/// Reject a checkpoint whose observation dimension does not match
/// `LEVEL_HOLD_OBS_DIM` (a stale pre-fuel model) before it can reach a forward pass.
fn check_obs_dim(model: &ActorCritic<InfB>) -> Result<(), ModelLoadError> {
    let found = model.input_dim();
    if found != LEVEL_HOLD_OBS_DIM {
        return Err(ModelLoadError::DimensionMismatch {
            expected: LEVEL_HOLD_OBS_DIM,
            found,
        });
    }
    Ok(())
}

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
    ) -> Result<Self, ModelLoadError> {
        let device: <InfB as Backend>::Device = Default::default();
        let model = ActorCritic::<InfB>::new(&device, LEVEL_HOLD_OBS_DIM).load_file(
            path,
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
            &device,
        )?;
        check_obs_dim(&model)?;
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
    ) -> Result<Self, ModelLoadError> {
        let device: <InfB as Backend>::Device = Default::default();
        let record = NamedMpkBytesRecorder::<FullPrecisionSettings>::default()
            .load(bytes.to_vec(), &device)?;
        let model = ActorCritic::<InfB>::new(&device, LEVEL_HOLD_OBS_DIM).load_record(record);
        check_obs_dim(&model)?;
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
        let obs = level_hold_observation(state, self.target_altitude, self.target_airspeed);
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

    fn targets(&self) -> crate::controllers::targets::ControllerTargets {
        crate::controllers::targets::ControllerTargets::LevelHold {
            altitude: self.target_altitude,
            airspeed: self.target_airspeed,
        }
    }

    fn apply_targets(
        &mut self,
        targets: &crate::controllers::targets::ControllerTargets,
        _state: &FlightState,
    ) {
        if let crate::controllers::targets::ControllerTargets::LevelHold { altitude, airspeed } =
            targets
        {
            self.target_altitude = *altitude;
            self.target_airspeed = *airspeed;
        }
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
