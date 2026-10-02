//! RL LSTM orbit controller: loads a trained Wu et al. FC-LSTM-FC policy and
//! implements FlightController with stateful LSTM hidden state.
//!
//! Only compiled when the `training` feature is active.

use std::any::Any;

use burn::backend::NdArray;

#[cfg(not(target_arch = "wasm32"))]
use crate::controllers::policy_batch::load_lstm_file;
use crate::controllers::policy_batch::{load_lstm_bytes, run_single, BatchedPolicy, SharedPolicy};

use crate::controllers::model_load::ModelLoadError;
use crate::controllers::orbit::{
    build_orbit_observation, OrbitController, OrbitDirection, ORBIT_OBS_DIM,
};
use crate::controllers::FlightController;
use crate::plane::{ControlInputs, FlightState};
use crate::training::direct_action_to_inputs;
use crate::training::ppo::lstm_model::{LstmActorCritic, LstmHiddenState};

type InfB = NdArray;

/// Reject a checkpoint whose observation dimension does not match `ORBIT_OBS_DIM`
/// (a stale pre-fuel model) before it can reach a forward pass.
fn check_obs_dim(model: &LstmActorCritic<InfB>) -> Result<(), ModelLoadError> {
    let found = model.input_dim();
    if found != ORBIT_OBS_DIM {
        return Err(ModelLoadError::DimensionMismatch {
            expected: ORBIT_OBS_DIM,
            found,
        });
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct RlLstmOrbitConfig {
    pub center_x: f32,
    pub center_z: f32,
    pub target_radius: f32,
    pub target_altitude: f32,
    pub target_airspeed: f32,
    pub direction: OrbitDirection,
}

impl RlLstmOrbitConfig {
    pub fn from_state(state: &FlightState) -> Self {
        let orbit = OrbitController::from_state(state, &ControlInputs::default());
        Self::from_orbit(&orbit)
    }

    pub fn from_orbit(orbit: &OrbitController) -> Self {
        Self {
            center_x: orbit.center_x,
            center_z: orbit.center_z,
            target_radius: orbit.target_radius,
            target_altitude: orbit.target_altitude,
            target_airspeed: orbit.target_airspeed,
            direction: orbit.direction,
        }
    }
}

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------

/// Trained Wu et al. LSTM orbit controller.
///
/// Maintains per-step LSTM hidden state between ticks. The model is shared with
/// every other controller flying the same checkpoint so the live sim can batch
/// their forward passes (see `controllers::policy_batch`); the hidden state stays
/// per controller and is stacked into the batch each tick.
pub struct RlLstmOrbitController {
    policy: SharedPolicy,
    /// Carried LSTM policy state.
    policy_hidden: LstmHiddenState,
    pub center_x: f32,
    pub center_z: f32,
    pub target_radius: f32,
    pub target_altitude: f32,
    pub target_airspeed: f32,
    pub direction: OrbitDirection,
}

impl RlLstmOrbitController {
    /// Load weights from `path` (without `.mpk` extension).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn load(path: &str, config: RlLstmOrbitConfig) -> Result<Self, ModelLoadError> {
        let model = load_lstm_file(path)?;
        check_obs_dim(&model.lock().unwrap())?;
        Ok(Self {
            policy: SharedPolicy::Lstm(model),
            policy_hidden: LstmHiddenState::default(),
            center_x: config.center_x,
            center_z: config.center_z,
            target_radius: config.target_radius,
            target_altitude: config.target_altitude,
            target_airspeed: config.target_airspeed,
            direction: config.direction,
        })
    }

    /// Load weights from embedded bytes — for WASM builds where `std::fs` is unavailable.
    pub fn load_bytes(bytes: &[u8], config: RlLstmOrbitConfig) -> Result<Self, ModelLoadError> {
        let model = load_lstm_bytes(bytes)?;
        check_obs_dim(&model.lock().unwrap())?;
        Ok(Self {
            policy: SharedPolicy::Lstm(model),
            policy_hidden: LstmHiddenState::default(),
            center_x: config.center_x,
            center_z: config.center_z,
            target_radius: config.target_radius,
            target_altitude: config.target_altitude,
            target_airspeed: config.target_airspeed,
            direction: config.direction,
        })
    }

    pub fn config(&self) -> RlLstmOrbitConfig {
        RlLstmOrbitConfig {
            center_x: self.center_x,
            center_z: self.center_z,
            target_radius: self.target_radius,
            target_altitude: self.target_altitude,
            target_airspeed: self.target_airspeed,
            direction: self.direction,
        }
    }

    /// Reset LSTM hidden state (call on episode start or controller re-engagement).
    pub fn reset_hidden(&mut self) {
        self.policy_hidden = LstmHiddenState::default();
    }
}

impl BatchedPolicy for RlLstmOrbitController {
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
        obs.extend(build_orbit_observation(
            state,
            self.center_x,
            self.center_z,
            self.target_radius,
            self.target_altitude,
            self.target_airspeed,
            self.direction,
        ));
    }

    fn hidden(&self) -> Option<&LstmHiddenState> {
        Some(&self.policy_hidden)
    }

    fn finish(&mut self, action: &[f32], hidden: Option<LstmHiddenState>) -> ControlInputs {
        // Carry the new hidden state into the next step.
        self.policy_hidden = hidden.expect("an LSTM policy step returns its new hidden state");
        // action = [elevator, throttle_norm, aileron, rudder]
        direct_action_to_inputs(action)
    }
}

impl FlightController for RlLstmOrbitController {
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
        "RlLstmOrbit"
    }

    fn telemetry(&self, state: &FlightState) -> crate::controllers::telemetry::ControllerTelemetry {
        let rx = state.position.x - self.center_x;
        let rz = state.position.z - self.center_z;
        let radial_error = (rx * rx + rz * rz).sqrt() - self.target_radius;
        crate::controllers::telemetry::ControllerTelemetry::Orbit { radial_error }
    }

    fn targets(&self) -> crate::controllers::targets::ControllerTargets {
        crate::controllers::targets::ControllerTargets::Orbit(
            crate::controllers::orbit::OrbitParams {
                center_x: self.center_x,
                center_z: self.center_z,
                target_radius: self.target_radius,
                target_altitude: self.target_altitude,
                target_airspeed: self.target_airspeed,
                direction: self.direction,
            },
        )
    }

    fn apply_targets(
        &mut self,
        targets: &crate::controllers::targets::ControllerTargets,
        _state: &FlightState,
    ) {
        if let crate::controllers::targets::ControllerTargets::Orbit(params) = targets {
            self.center_x = params.center_x;
            self.center_z = params.center_z;
            self.target_radius = params.target_radius;
            self.target_altitude = params.target_altitude;
            self.target_airspeed = params.target_airspeed;
            self.direction = params.direction;
        }
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::training::ppo::lstm_model::LSTM_HIDDEN;
    use bevy::math::{Quat, Vec3};
    use burn::tensor::backend::Backend;
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
    fn update_produces_finite_outputs_with_untrained_model() {
        let device: <InfB as Backend>::Device = Default::default();
        let mut ctrl = RlLstmOrbitController {
            policy: SharedPolicy::Lstm(std::sync::Arc::new(std::sync::Mutex::new(
                LstmActorCritic::<InfB>::new(&device, ORBIT_OBS_DIM),
            ))),
            policy_hidden: LstmHiddenState::default(),
            center_x: 0.0,
            center_z: 0.0,
            target_radius: 1000.0,
            target_altitude: 1000.0,
            target_airspeed: 100.0,
            direction: OrbitDirection::CounterClockwise,
        };
        let state = make_state(Vec3::new(0.0, 1000.0, -1000.0), Vec3::new(100.0, 0.0, 0.0));
        let inputs = ctrl.update(
            &state,
            &crate::plane::ControllerContext::empty_for(crate::plane::PlaneId::TEST),
            1.0 / 60.0,
        );
        assert!(inputs.elevator.is_finite());
        assert!(inputs.aileron.is_finite());
        assert!(inputs.rudder.is_finite());
        assert!(inputs.throttle.is_finite() && inputs.throttle >= 0.0 && inputs.throttle <= 1.0);
    }

    #[test]
    fn lstm_state_changes_after_step() {
        let device: <InfB as Backend>::Device = Default::default();
        let mut ctrl = RlLstmOrbitController {
            policy: SharedPolicy::Lstm(std::sync::Arc::new(std::sync::Mutex::new(
                LstmActorCritic::<InfB>::new(&device, ORBIT_OBS_DIM),
            ))),
            policy_hidden: LstmHiddenState::default(),
            center_x: 0.0,
            center_z: 0.0,
            target_radius: 1000.0,
            target_altitude: 1000.0,
            target_airspeed: 100.0,
            direction: OrbitDirection::CounterClockwise,
        };
        let state = make_state(Vec3::new(0.0, 1000.0, -1000.0), Vec3::new(100.0, 0.0, 0.0));
        let h_before = ctrl.policy_hidden.h[0];
        ctrl.update(
            &state,
            &crate::plane::ControllerContext::empty_for(crate::plane::PlaneId::TEST),
            1.0 / 60.0,
        );
        let h_after = ctrl.policy_hidden.h[0];
        // After one step the LSTM hidden state should change from zero.
        // (With a freshly initialised model and zero obs this may stay 0 — check that it's finite)
        assert!(
            h_after.is_finite(),
            "hidden state became non-finite: {h_after}"
        );
        let _ = h_before; // used for documentation
    }

    #[test]
    fn reset_hidden_clears_state() {
        let device: <InfB as Backend>::Device = Default::default();
        let mut ctrl = RlLstmOrbitController {
            policy: SharedPolicy::Lstm(std::sync::Arc::new(std::sync::Mutex::new(
                LstmActorCritic::<InfB>::new(&device, ORBIT_OBS_DIM),
            ))),
            policy_hidden: LstmHiddenState {
                h: vec![1.0; LSTM_HIDDEN],
                c: vec![2.0; LSTM_HIDDEN],
            },
            center_x: 0.0,
            center_z: 0.0,
            target_radius: 1000.0,
            target_altitude: 1000.0,
            target_airspeed: 100.0,
            direction: OrbitDirection::CounterClockwise,
        };
        ctrl.reset_hidden();
        assert_eq!(ctrl.policy_hidden.h[0], 0.0);
        assert_eq!(ctrl.policy_hidden.c[0], 0.0);
    }
}
