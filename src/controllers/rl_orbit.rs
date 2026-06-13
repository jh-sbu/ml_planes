//! RL orbit controller: loads a trained PPO policy and implements FlightController.
//!
//! Uses the NdArray backend (CPU-only, no GPU required) for inference.
//! Only compiled when the `training` feature is active via the parent module.

use std::any::Any;

#[cfg(not(target_arch = "wasm32"))]
use burn::record::DefaultFileRecorder;
use burn::{
    backend::NdArray,
    module::Module,
    record::{FullPrecisionSettings, NamedMpkBytesRecorder, Recorder},
    tensor::{backend::Backend, Tensor, TensorData},
};

use crate::controllers::orbit::{
    build_orbit_observation, OrbitController, OrbitDirection, ORBIT_OBS_DIM,
};
use crate::controllers::FlightController;
use crate::plane::{ControlInputs, FlightState};
use crate::training::direct_action_to_inputs;
use crate::training::ppo::model::ActorCritic;

type InfB = NdArray;

#[derive(Clone, Copy, Debug)]
pub struct RlOrbitConfig {
    pub center_x: f32,
    pub center_z: f32,
    pub target_radius: f32,
    pub target_altitude: f32,
    pub target_airspeed: f32,
    pub direction: OrbitDirection,
}

impl RlOrbitConfig {
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

/// Trained PPO orbit controller that runs inference on the CPU.
///
/// `ActorCritic<NdArray>` is not `Sync` (burn's `Param` uses `OnceCell`),
/// so we wrap in `Mutex` to satisfy `FlightController: Sync`.
pub struct RlOrbitController {
    model: std::sync::Mutex<ActorCritic<InfB>>,
    device: <InfB as Backend>::Device,
    pub center_x: f32,
    pub center_z: f32,
    pub target_radius: f32,
    pub target_altitude: f32,
    pub target_airspeed: f32,
    pub direction: OrbitDirection,
}

impl RlOrbitController {
    /// Load weights from `path` (without `.mpk` extension) saved by `PpoTrainer::save_policy`.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn load(path: &str, config: RlOrbitConfig) -> Result<Self, burn::record::RecorderError> {
        let device: <InfB as Backend>::Device = Default::default();
        let model = ActorCritic::<InfB>::new(&device, ORBIT_OBS_DIM).load_file(
            path,
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
            &device,
        )?;
        Ok(Self {
            model: std::sync::Mutex::new(model),
            device,
            center_x: config.center_x,
            center_z: config.center_z,
            target_radius: config.target_radius,
            target_altitude: config.target_altitude,
            target_airspeed: config.target_airspeed,
            direction: config.direction,
        })
    }

    /// Load weights from embedded bytes — for WASM builds where `std::fs` is unavailable.
    pub fn load_bytes(
        bytes: &[u8],
        config: RlOrbitConfig,
    ) -> Result<Self, burn::record::RecorderError> {
        let device: <InfB as Backend>::Device = Default::default();
        let record = NamedMpkBytesRecorder::<FullPrecisionSettings>::default()
            .load(bytes.to_vec(), &device)?;
        let model = ActorCritic::<InfB>::new(&device, ORBIT_OBS_DIM).load_record(record);
        Ok(Self {
            model: std::sync::Mutex::new(model),
            device,
            center_x: config.center_x,
            center_z: config.center_z,
            target_radius: config.target_radius,
            target_altitude: config.target_altitude,
            target_airspeed: config.target_airspeed,
            direction: config.direction,
        })
    }

    pub fn config(&self) -> RlOrbitConfig {
        RlOrbitConfig {
            center_x: self.center_x,
            center_z: self.center_z,
            target_radius: self.target_radius,
            target_altitude: self.target_altitude,
            target_airspeed: self.target_airspeed,
            direction: self.direction,
        }
    }
}

impl FlightController for RlOrbitController {
    fn update(
        &mut self,
        state: &FlightState,
        _ctx: &crate::plane::ControllerContext,
        _dt: f32,
    ) -> ControlInputs {
        let obs = build_orbit_observation(
            state,
            self.center_x,
            self.center_z,
            self.target_radius,
            self.target_altitude,
            self.target_airspeed,
            self.direction,
        );
        let obs_t = Tensor::<InfB, 2>::from_data(
            TensorData::new(obs, vec![1, ORBIT_OBS_DIM]),
            &self.device,
        );
        // Deterministic inference: use mean action (no sampling noise).
        let action_t = self.model.lock().unwrap().mean_action(obs_t);
        let action = action_t
            .into_data()
            .to_vec::<f32>()
            .expect("rl orbit action data");

        // action = [elevator, throttle_norm, aileron, rudder]
        direct_action_to_inputs(&action)
    }

    fn name(&self) -> &'static str {
        "RlOrbit"
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
        let mut ctrl = RlOrbitController {
            model: std::sync::Mutex::new(ActorCritic::<InfB>::new(&device, ORBIT_OBS_DIM)),
            device,
            center_x: 0.0,
            center_z: 0.0,
            target_radius: 1000.0,
            target_altitude: 1000.0,
            target_airspeed: 100.0,
            direction: OrbitDirection::CounterClockwise,
        };
        let state = make_state(Vec3::new(0.0, 1000.0, -1000.0), Vec3::new(100.0, 0.0, 0.0));
        let obs = build_orbit_observation(
            &state,
            ctrl.center_x,
            ctrl.center_z,
            ctrl.target_radius,
            ctrl.target_altitude,
            ctrl.target_airspeed,
            ctrl.direction,
        );
        assert_eq!(obs.len(), ORBIT_OBS_DIM);
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
}
