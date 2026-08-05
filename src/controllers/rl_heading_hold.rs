//! RL heading-hold controller: loads a trained PPO policy and implements FlightController.
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
use crate::training::heading_hold_env::{heading_hold_observation, HEADING_HOLD_OBS_DIM};
use crate::training::ppo::model::ActorCritic;

type InfB = NdArray;

/// Reject a checkpoint whose observation dimension does not match
/// `HEADING_HOLD_OBS_DIM` before it can reach a forward pass.
fn check_obs_dim(model: &ActorCritic<InfB>) -> Result<(), ModelLoadError> {
    let found = model.input_dim();
    if found != HEADING_HOLD_OBS_DIM {
        return Err(ModelLoadError::DimensionMismatch {
            expected: HEADING_HOLD_OBS_DIM,
            found,
        });
    }
    Ok(())
}

/// The three setpoints an `RlHeadingHoldController` is loaded/rebuilt with.
/// Mirrors `RlOrbitConfig` — a bundled struct rather than bare args, since
/// heading hold (unlike level hold) has three independent setpoints.
#[derive(Clone, Copy, Debug)]
pub struct RlHeadingHoldConfig {
    pub target_heading: f32,
    pub target_altitude: f32,
    pub target_airspeed: f32,
}

/// Trained PPO heading-hold controller that runs inference on the CPU.
///
/// `ActorCritic<NdArray>` is not `Sync` (burn's `Param` uses `OnceCell`),
/// so we wrap in `Mutex` to satisfy `FlightController: Sync`.
pub struct RlHeadingHoldController {
    model: std::sync::Mutex<ActorCritic<InfB>>,
    device: <InfB as Backend>::Device,
    pub target_heading: f32,
    pub target_altitude: f32,
    pub target_airspeed: f32,
}

impl RlHeadingHoldController {
    /// Load weights from `path` (without `.mpk` extension) saved by `PpoTrainer::save_policy`.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn load(path: &str, config: RlHeadingHoldConfig) -> Result<Self, ModelLoadError> {
        let device: <InfB as Backend>::Device = Default::default();
        let model = ActorCritic::<InfB>::new(&device, HEADING_HOLD_OBS_DIM).load_file(
            path,
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
            &device,
        )?;
        check_obs_dim(&model)?;
        Ok(Self {
            model: std::sync::Mutex::new(model),
            device,
            target_heading: config.target_heading,
            target_altitude: config.target_altitude,
            target_airspeed: config.target_airspeed,
        })
    }

    /// Load weights from embedded bytes — for WASM builds where `std::fs` is unavailable.
    pub fn load_bytes(bytes: &[u8], config: RlHeadingHoldConfig) -> Result<Self, ModelLoadError> {
        let device: <InfB as Backend>::Device = Default::default();
        let record = NamedMpkBytesRecorder::<FullPrecisionSettings>::default()
            .load(bytes.to_vec(), &device)?;
        let model = ActorCritic::<InfB>::new(&device, HEADING_HOLD_OBS_DIM).load_record(record);
        check_obs_dim(&model)?;
        Ok(Self {
            model: std::sync::Mutex::new(model),
            device,
            target_heading: config.target_heading,
            target_altitude: config.target_altitude,
            target_airspeed: config.target_airspeed,
        })
    }

    pub fn config(&self) -> RlHeadingHoldConfig {
        RlHeadingHoldConfig {
            target_heading: self.target_heading,
            target_altitude: self.target_altitude,
            target_airspeed: self.target_airspeed,
        }
    }
}

impl FlightController for RlHeadingHoldController {
    fn update(
        &mut self,
        state: &FlightState,
        _ctx: &crate::plane::ControllerContext,
        _dt: f32,
    ) -> ControlInputs {
        let obs = heading_hold_observation(
            state,
            self.target_heading,
            self.target_altitude,
            self.target_airspeed,
        );
        let obs_t = Tensor::<InfB, 2>::from_data(
            TensorData::new(obs, vec![1, HEADING_HOLD_OBS_DIM]),
            &self.device,
        );
        // Deterministic inference: use mean action (no sampling noise).
        let action_t = self.model.lock().unwrap().mean_action(obs_t);
        let action = action_t
            .into_data()
            .to_vec::<f32>()
            .expect("rl heading hold action data");

        // action = [elevator, throttle_norm, aileron, rudder]
        direct_action_to_inputs(&action)
    }

    fn name(&self) -> &'static str {
        "RlHeadingHold"
    }

    fn targets(&self) -> crate::controllers::targets::ControllerTargets {
        // Reuses ControllerTargets::HeadingHold — one variant per widget set,
        // not per controller kind (see targets.rs doc). PID HeadingHold and
        // this RL variant share the exact same three-field HUD editor.
        crate::controllers::targets::ControllerTargets::HeadingHold {
            heading: self.target_heading,
            altitude: self.target_altitude,
            airspeed: self.target_airspeed,
        }
    }

    fn apply_targets(
        &mut self,
        targets: &crate::controllers::targets::ControllerTargets,
        _state: &FlightState,
    ) {
        if let crate::controllers::targets::ControllerTargets::HeadingHold {
            heading,
            altitude,
            airspeed,
        } = targets
        {
            self.target_heading = *heading;
            self.target_altitude = *altitude;
            self.target_airspeed = *airspeed;
        }
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::{Quat, Vec3};
    use std::f32::consts::FRAC_PI_2;

    fn level_attitude() -> Quat {
        Quat::from_rotation_x(-FRAC_PI_2)
    }

    fn make_state(position: Vec3, velocity: Vec3) -> FlightState {
        let airspeed = velocity.length();
        FlightState {
            position,
            velocity,
            attitude: level_attitude(),
            angular_velocity: Vec3::ZERO,
            alpha: 0.0,
            beta: 0.0,
            airspeed,
            altitude: position.y,

            consumable_remaining: f32::INFINITY,
        }
    }

    #[test]
    fn update_outputs_finite_controls_with_finite_observation() {
        let device: <InfB as Backend>::Device = Default::default();
        let mut ctrl = RlHeadingHoldController {
            model: std::sync::Mutex::new(ActorCritic::<InfB>::new(&device, HEADING_HOLD_OBS_DIM)),
            device,
            target_heading: 0.5,
            target_altitude: 1000.0,
            target_airspeed: 120.0,
        };
        let state = make_state(Vec3::new(0.0, 1000.0, 0.0), Vec3::new(120.0, 0.0, 0.0));
        let obs = heading_hold_observation(
            &state,
            ctrl.target_heading,
            ctrl.target_altitude,
            ctrl.target_airspeed,
        );
        assert_eq!(obs.len(), HEADING_HOLD_OBS_DIM);
        assert!(
            obs.iter().all(|v| v.is_finite()),
            "obs contains NaN/inf: {obs:?}"
        );

        let inputs = ctrl.update(
            &state,
            &crate::plane::ControllerContext::empty_for(crate::plane::PlaneId::TEST),
            1.0 / 60.0,
        );
        assert!(inputs.aileron.is_finite());
        assert!(inputs.elevator.is_finite());
        assert!(inputs.rudder.is_finite());
        assert!(inputs.throttle.is_finite());
    }

    #[test]
    fn targets_round_trip_through_apply_targets() {
        let device: <InfB as Backend>::Device = Default::default();
        let mut ctrl = RlHeadingHoldController {
            model: std::sync::Mutex::new(ActorCritic::<InfB>::new(&device, HEADING_HOLD_OBS_DIM)),
            device,
            target_heading: 0.0,
            target_altitude: 1000.0,
            target_airspeed: 100.0,
        };
        ctrl.apply_targets(
            &crate::controllers::targets::ControllerTargets::HeadingHold {
                heading: 1.2,
                altitude: 3000.0,
                airspeed: 130.0,
            },
            &FlightState::default(),
        );
        assert_eq!(ctrl.target_heading, 1.2);
        assert_eq!(ctrl.target_altitude, 3000.0);
        assert_eq!(ctrl.target_airspeed, 130.0);

        // Mismatched variant is a no-op.
        ctrl.apply_targets(
            &crate::controllers::targets::ControllerTargets::LevelHold {
                altitude: 1.0,
                airspeed: 1.0,
            },
            &FlightState::default(),
        );
        assert_eq!(ctrl.target_heading, 1.2);
        assert_eq!(ctrl.target_altitude, 3000.0);
        assert_eq!(ctrl.target_airspeed, 130.0);
    }
}
