//! RL level-hold controller: loads a trained PPO policy and implements FlightController.
//!
//! Uses the NdArray backend (CPU-only, no GPU required) for inference.
//! Only compiled when the `training` feature is active — gated via the parent mod declaration.

use std::any::Any;

use burn::backend::NdArray;

#[cfg(not(target_arch = "wasm32"))]
use crate::controllers::policy_batch::load_mlp_file;
use crate::controllers::policy_batch::{load_mlp_bytes, run_single, BatchedPolicy, SharedPolicy};

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
/// The model is shared with every other controller flying the same checkpoint so
/// the live sim can batch their forward passes (see `controllers::policy_batch`).
pub struct RlLevelHoldController {
    policy: SharedPolicy,
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
        let model = load_mlp_file(path)?;
        check_obs_dim(&model.lock().unwrap())?;
        Ok(Self {
            policy: SharedPolicy::Mlp(model),
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
        let model = load_mlp_bytes(bytes)?;
        check_obs_dim(&model.lock().unwrap())?;
        Ok(Self {
            policy: SharedPolicy::Mlp(model),
            target_altitude,
            target_airspeed,
        })
    }
}

impl BatchedPolicy for RlLevelHoldController {
    fn policy(&self) -> &SharedPolicy {
        &self.policy
    }

    fn observe(
        &mut self,
        state: &FlightState,
        _ctx: &crate::plane::ControllerContext,
        _dt: f32,
        obs: &mut Vec<f32>,
    ) {
        obs.extend(level_hold_observation(
            state,
            self.target_altitude,
            self.target_airspeed,
        ));
    }

    fn finish(
        &mut self,
        action: &[f32],
        _hidden: Option<crate::training::ppo::lstm_model::LstmHiddenState>,
    ) -> ControlInputs {
        // action = [elevator, throttle_norm, aileron, rudder]
        direct_action_to_inputs(action)
    }
}

impl FlightController for RlLevelHoldController {
    fn update(
        &mut self,
        state: &FlightState,
        ctx: &crate::plane::ControllerContext,
        dt: f32,
    ) -> ControlInputs {
        run_single(self, state, ctx, dt)
    }

    fn batched(&mut self) -> Option<&mut dyn BatchedPolicy> {
        Some(self)
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
