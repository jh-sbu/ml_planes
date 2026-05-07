//! Residual RL orbit controller: PID cascade + trained PPO correction.
//!
//! The inner `OrbitController` runs every tick to produce a baseline
//! `ControlInputs`. The PPO policy then outputs a small delta in [-1, 1] for
//! each channel; the two are summed and clamped to the legal range.
//!
//! Uses the NdArray backend (CPU-only) for inference.
//! Only compiled when the `training` feature is active.

use std::any::Any;

use burn::{
    backend::NdArray,
    module::Module,
    record::{DefaultFileRecorder, FullPrecisionSettings},
    tensor::{backend::Backend, Tensor, TensorData},
};

use crate::controllers::orbit::{
    build_orbit_observation, OrbitController, OrbitDirection, ORBIT_OBS_DIM,
};
use crate::controllers::FlightController;
use crate::plane::{ControlInputs, FlightState};
use crate::training::ppo::model::ActorCritic;

type InfB = NdArray;

/// Maximum authority the NN residual can add to any single control channel
/// (as a fraction of full deflection). Keeps the PID the dominant signal.
pub const RESIDUAL_SCALE: f32 = 0.3;

#[derive(Clone, Copy, Debug)]
pub struct RlOrbitResidualConfig {
    pub center_x: f32,
    pub center_z: f32,
    pub target_radius: f32,
    pub target_altitude: f32,
    pub target_airspeed: f32,
    pub direction: OrbitDirection,
}

impl RlOrbitResidualConfig {
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

/// Trained PPO orbit controller that runs a PID baseline and adds a learned residual.
///
/// `ActorCritic<NdArray>` is not `Sync` (burn's `Param` uses `OnceCell`),
/// so the model is wrapped in `Mutex` to satisfy `FlightController: Sync`.
pub struct RlOrbitResidualController {
    pid: OrbitController,
    model: std::sync::Mutex<ActorCritic<InfB>>,
    device: <InfB as Backend>::Device,
    pub center_x: f32,
    pub center_z: f32,
    pub target_radius: f32,
    pub target_altitude: f32,
    pub target_airspeed: f32,
    pub direction: OrbitDirection,
}

impl RlOrbitResidualController {
    /// Load weights from `path` (without `.mpk` extension).
    pub fn load(
        path: &str,
        config: RlOrbitResidualConfig,
        state: &FlightState,
    ) -> Result<Self, burn::record::RecorderError> {
        let device: <InfB as Backend>::Device = Default::default();
        let model = ActorCritic::<InfB>::new(&device, ORBIT_OBS_DIM).load_file(
            path,
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
            &device,
        )?;
        let pid = OrbitController::from_state(state, &ControlInputs::default());
        Ok(Self {
            pid,
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

    pub fn config(&self) -> RlOrbitResidualConfig {
        RlOrbitResidualConfig {
            center_x: self.center_x,
            center_z: self.center_z,
            target_radius: self.target_radius,
            target_altitude: self.target_altitude,
            target_airspeed: self.target_airspeed,
            direction: self.direction,
        }
    }
}

impl FlightController for RlOrbitResidualController {
    fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs {
        // Sync PID's orbit parameters with our public fields (allows runtime changes).
        self.pid.center_x = self.center_x;
        self.pid.center_z = self.center_z;
        self.pid.target_radius = self.target_radius;
        self.pid.target_altitude = self.target_altitude;
        self.pid.target_airspeed = self.target_airspeed;
        self.pid.direction = self.direction;

        let pid_inputs = self.pid.update(state, dt);

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
        let action_t = self.model.lock().unwrap().mean_action(obs_t);
        let action = action_t
            .into_data()
            .to_vec::<f32>()
            .expect("rl orbit residual action data");

        let mut inputs = ControlInputs {
            elevator: pid_inputs.elevator + action[0] * RESIDUAL_SCALE,
            throttle: pid_inputs.throttle + action[1] * RESIDUAL_SCALE,
            aileron: pid_inputs.aileron + action[2] * RESIDUAL_SCALE,
            rudder: pid_inputs.rudder + action[3] * RESIDUAL_SCALE,
        };
        inputs.clamp();
        inputs
    }

    fn name(&self) -> &'static str {
        "RlOrbitResidual"
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
        }
    }

    #[test]
    fn update_outputs_finite_controls_with_finite_observation() {
        let state = make_state(Vec3::new(0.0, 1000.0, -1000.0), Vec3::new(100.0, 0.0, 0.0));
        let device: <InfB as Backend>::Device = Default::default();
        let pid = OrbitController::from_state(&state, &ControlInputs::default());
        let mut ctrl = RlOrbitResidualController {
            pid,
            model: std::sync::Mutex::new(ActorCritic::<InfB>::new(&device, ORBIT_OBS_DIM)),
            device,
            center_x: 0.0,
            center_z: 0.0,
            target_radius: 1000.0,
            target_altitude: 1000.0,
            target_airspeed: 100.0,
            direction: OrbitDirection::CounterClockwise,
        };

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
        assert!(obs.iter().all(|v| v.is_finite()), "obs has NaN/inf: {obs:?}");

        let inputs = ctrl.update(&state, 1.0 / 60.0);
        assert!(inputs.aileron.is_finite());
        assert!(inputs.elevator.is_finite());
        assert!(inputs.rudder.is_finite());
        assert!(inputs.throttle.is_finite());
    }
}
