//! Residual RL orbit controller: PID cascade + trained PPO correction.
//!
//! The inner `OrbitController` runs every tick to produce a baseline
//! `ControlInputs`. The PPO policy then outputs a small delta in [-1, 1] for
//! each channel; the two are summed and clamped to the legal range.
//!
//! Uses the NdArray backend (CPU-only) for inference.
//! Only compiled when the `training` feature is active.

use std::any::Any;

use burn::backend::NdArray;

#[cfg(not(target_arch = "wasm32"))]
use crate::controllers::policy_batch::load_mlp_file;
use crate::controllers::policy_batch::{load_mlp_bytes, run_single, BatchedPolicy, SharedPolicy};

use crate::controllers::model_load::ModelLoadError;
use crate::controllers::orbit::{
    build_orbit_observation, OrbitController, OrbitDirection, OrbitParams, ORBIT_OBS_DIM,
};
use crate::controllers::tuning::OrbitTuning;
use crate::controllers::FlightController;
use crate::plane::{ControlInputs, FlightState};
use crate::training::ppo::model::ActorCritic;

type InfB = NdArray;

/// Reject a checkpoint whose observation dimension does not match `ORBIT_OBS_DIM`
/// (a stale pre-fuel model) before it can reach a forward pass.
fn check_obs_dim(model: &ActorCritic<InfB>) -> Result<(), ModelLoadError> {
    let found = model.input_dim();
    if found != ORBIT_OBS_DIM {
        return Err(ModelLoadError::DimensionMismatch {
            expected: ORBIT_OBS_DIM,
            found,
        });
    }
    Ok(())
}

#[derive(Clone, Copy, Debug)]
pub struct RlOrbitResidualConfig {
    pub center_x: f32,
    pub center_z: f32,
    pub target_radius: f32,
    pub target_altitude: f32,
    pub target_airspeed: f32,
    pub direction: OrbitDirection,
    /// Maximum authority the NN residual can add to any single control channel
    /// (as a fraction of full deflection). Keeps the PID the dominant signal.
    pub residual_scale: f32,
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
            residual_scale: 0.3,
        }
    }
}

/// Trained PPO orbit controller that runs a PID baseline and adds a learned residual.
///
/// The model is shared with every other controller flying the same checkpoint so
/// the live sim can batch their forward passes (see `controllers::policy_batch`).
pub struct RlOrbitResidualController {
    /// Inner PID baseline. `center_x`/`target_radius`/etc. are re-synced from this
    /// controller's own public fields every tick (see `update`); its gains are the part a
    /// tuning-rebuild can legitimately change — see `retune`.
    pub pid: OrbitController,
    policy: SharedPolicy,
    /// This tick's PID output, from `observe` to `finish`. Scratch — not state.
    baseline: ControlInputs,
    pub center_x: f32,
    pub center_z: f32,
    pub target_radius: f32,
    pub target_altitude: f32,
    pub target_airspeed: f32,
    pub direction: OrbitDirection,
    pub residual_scale: f32,
}

impl RlOrbitResidualController {
    /// Load weights from `path` (without `.mpk` extension).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn load(
        path: &str,
        config: RlOrbitResidualConfig,
        state: &FlightState,
        tuning: Option<&OrbitTuning>,
    ) -> Result<Self, ModelLoadError> {
        let model = load_mlp_file(path)?;
        check_obs_dim(&model.lock().unwrap())?;
        let pid = match tuning {
            Some(t) => OrbitController::with_tuning(state, t, &ControlInputs::default()),
            None => OrbitController::from_state(state, &ControlInputs::default()),
        };
        Ok(Self {
            pid,
            policy: SharedPolicy::Mlp(model),
            baseline: ControlInputs::default(),
            center_x: config.center_x,
            center_z: config.center_z,
            target_radius: config.target_radius,
            target_altitude: config.target_altitude,
            target_airspeed: config.target_airspeed,
            direction: config.direction,
            residual_scale: config.residual_scale,
        })
    }

    /// Load weights from embedded bytes — for WASM builds where `std::fs` is unavailable.
    pub fn load_bytes(
        bytes: &[u8],
        config: RlOrbitResidualConfig,
        state: &FlightState,
        tuning: Option<&OrbitTuning>,
    ) -> Result<Self, ModelLoadError> {
        let model = load_mlp_bytes(bytes)?;
        check_obs_dim(&model.lock().unwrap())?;
        let pid = match tuning {
            Some(t) => OrbitController::with_tuning(state, t, &ControlInputs::default()),
            None => OrbitController::from_state(state, &ControlInputs::default()),
        };
        Ok(Self {
            pid,
            policy: SharedPolicy::Mlp(model),
            baseline: ControlInputs::default(),
            center_x: config.center_x,
            center_z: config.center_z,
            target_radius: config.target_radius,
            target_altitude: config.target_altitude,
            target_airspeed: config.target_airspeed,
            direction: config.direction,
            residual_scale: config.residual_scale,
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
            residual_scale: self.residual_scale,
        }
    }

    /// Rebuild the inner PID baseline from a freshly loaded tuning profile, leaving the
    /// trained policy untouched.
    ///
    /// Used by the `SimControlPlugin` tuning-rebuild systems (`apply_initial_tuning` /
    /// `apply_controller_switch` in `sim_control.rs`) to apply a `.tuning.ron` load or
    /// profile switch to this controller's PID baseline without falling back to
    /// `ControllerKind::build()`, which would replace the whole controller with a plain
    /// `OrbitController` and drop the policy — the same clobber the wingman
    /// extract/restore pair (`extract_wingman_params`/`restore_wingman`) guards against.
    ///
    /// Mirrors `load`/`load_bytes`'s PID construction; `with_tuning`/`from_state` auto-center
    /// on `state`'s current position, so the controller's own retained geometry is re-applied
    /// afterward via `apply_params` (also recomputing the bank feedforward for the current
    /// airspeed) rather than letting the orbit silently re-center.
    pub fn retune(&mut self, tuning: Option<&OrbitTuning>, state: &FlightState) {
        self.pid = match tuning {
            Some(t) => OrbitController::with_tuning(state, t, &ControlInputs::default()),
            None => OrbitController::from_state(state, &ControlInputs::default()),
        };
        let params = OrbitParams {
            center_x: self.center_x,
            center_z: self.center_z,
            target_radius: self.target_radius,
            target_altitude: self.target_altitude,
            target_airspeed: self.target_airspeed,
            direction: self.direction,
        };
        self.pid.apply_params(&params, state.airspeed);
    }
}

impl BatchedPolicy for RlOrbitResidualController {
    fn policy(&self) -> &SharedPolicy {
        &self.policy
    }

    /// Runs the PID baseline — before the policy, as the unbatched tick did — and
    /// parks its output in `baseline` for `finish` to add the residual onto.
    fn observe(
        &mut self,
        state: &FlightState,
        ctx: &crate::plane::ControllerContext,
        dt: f32,
        obs: &mut Vec<f32>,
    ) {
        // Sync PID's orbit parameters with our public fields (allows runtime changes).
        self.pid.center_x = self.center_x;
        self.pid.center_z = self.center_z;
        self.pid.target_radius = self.target_radius;
        self.pid.target_altitude = self.target_altitude;
        self.pid.target_airspeed = self.target_airspeed;
        self.pid.direction = self.direction;

        self.baseline = self.pid.update(state, ctx, dt);

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

    fn finish(
        &mut self,
        action: &[f32],
        _hidden: Option<crate::training::ppo::lstm_model::LstmHiddenState>,
    ) -> ControlInputs {
        let pid_inputs = &self.baseline;
        let mut inputs = ControlInputs {
            elevator: pid_inputs.elevator + action[0] * self.residual_scale,
            throttle: pid_inputs.throttle + action[1] * self.residual_scale,
            aileron: pid_inputs.aileron + action[2] * self.residual_scale,
            rudder: pid_inputs.rudder + action[3] * self.residual_scale,
        };
        inputs.clamp();
        inputs
    }
}

impl FlightController for RlOrbitResidualController {
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
        "RlOrbitResidual"
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
        // The `pid` baseline's fields are re-synced from these each `update()` tick
        // (see above), so updating them here is sufficient — no separate reset needed.
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

#[cfg(test)]
mod tests {
    use super::*;
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
    fn update_outputs_finite_controls_with_finite_observation() {
        let state = make_state(Vec3::new(0.0, 1000.0, -1000.0), Vec3::new(100.0, 0.0, 0.0));
        let device: <InfB as Backend>::Device = Default::default();
        let pid = OrbitController::from_state(&state, &ControlInputs::default());
        let mut ctrl =
            RlOrbitResidualController {
                pid,
                policy: SharedPolicy::Mlp(std::sync::Arc::new(std::sync::Mutex::new(
                    ActorCritic::<InfB>::new(&device, ORBIT_OBS_DIM),
                ))),
                baseline: ControlInputs::default(),
                center_x: 0.0,
                center_z: 0.0,
                target_radius: 1000.0,
                target_altitude: 1000.0,
                target_airspeed: 100.0,
                direction: OrbitDirection::CounterClockwise,
                residual_scale: 0.3,
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
        assert!(
            obs.iter().all(|v| v.is_finite()),
            "obs has NaN/inf: {obs:?}"
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
