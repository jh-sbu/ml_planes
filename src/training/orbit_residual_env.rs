//! Training environment for residual-RL circular orbit.
//!
//! Observation (dim = 13, normalized) — same as OrbitEnv:
//!   [radial_err/500, heading_err/0.5, bank_ff/60deg,
//!    alt_err/200, speed_err/50, alpha/0.5, pitch/0.5, pitch_rate/1,
//!    roll/0.5, roll_rate/1, beta/0.5, yaw_rate/1, vertical_speed/30]
//!
//! Action (dim = 4, each in [-1, 1]):
//!   [delta_elevator, delta_throttle, delta_aileron, delta_rudder]
//!   Each channel is a residual added to the PID output and clamped.
//!   Throttle delta is symmetric: positive = more thrust, negative = less.

use std::f32::consts::FRAC_PI_3;

use bevy::math::{Mat3, Quat, Vec3};

use crate::controllers::orbit::{
    build_orbit_observation, build_orbit_observation_from_terms, orbit_observation_terms,
    OrbitController, OrbitDirection, OrbitObservationTerms, ORBIT_OBS_DIM,
};
use crate::controllers::FlightController;
use crate::plane::{ControlInputs, ControllerContext, FlightState, PlaneConfig, PlaneId};
use crate::training::flight_env::{integrate_state, roll_angle, Lcg};

use crate::training::reward_config::OrbitRewardConfig;
use crate::training::{Observation, SpawnSpec, StepInfo, TrainingEnv};

const G: f32 = 9.81;
const TWO_PI: f32 = std::f32::consts::PI * 2.0;
const HEADING_PERTURB_RANGE: f32 = 25.0 * std::f32::consts::PI / 180.0;
const ANG_VEL_RANGE: f32 = 5.0 * std::f32::consts::PI / 180.0;
const VVEL_RANGE: f32 = 2.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TerminationReason {
    Failure,
    Timeout,
}

#[derive(Clone)]
pub struct ResidualOrbitEnv {
    pub center_x: f32,
    pub center_z: f32,
    pub target_radius: f32,
    pub target_altitude: f32,
    pub target_airspeed: f32,
    pub max_episode_steps: u32,
    pub radial_offset_range: std::ops::RangeInclusive<f32>,
    pub altitude_perturb_range: std::ops::RangeInclusive<f32>,
    pub airspeed_perturb_range: std::ops::RangeInclusive<f32>,
    pub target_radius_range: std::ops::RangeInclusive<f32>,

    reward_cfg: OrbitRewardConfig,
    cfg: PlaneConfig,
    dt: f32,
    state: FlightState,
    direction: OrbitDirection,
    orbit_controller: OrbitController,
    episode_step: u32,
    rng: Lcg,
    rng_seed: u64,
}

impl ResidualOrbitEnv {
    pub fn new(
        target_altitude: f32,
        target_airspeed: f32,
        target_radius: f32,
        cfg: PlaneConfig,
    ) -> Self {
        let reward_cfg = OrbitRewardConfig::default();
        let max_episode_steps = reward_cfg.max_episode_steps;
        let placeholder_state = FlightState::default();
        let orbit_controller =
            OrbitController::from_state(&placeholder_state, &ControlInputs::default());
        Self {
            center_x: 0.0,
            center_z: 0.0,
            target_radius,
            target_altitude,
            target_airspeed,
            max_episode_steps,
            radial_offset_range: -250.0..=250.0,
            altitude_perturb_range: -150.0..=150.0,
            airspeed_perturb_range: -20.0..=20.0,
            target_radius_range: 2500.0..=4000.0,
            reward_cfg,
            cfg,
            dt: 1.0 / 60.0,
            state: placeholder_state,
            direction: OrbitDirection::CounterClockwise,
            orbit_controller,
            episode_step: 0,
            rng: Lcg::new(4242),
            rng_seed: 4242,
        }
    }

    pub fn with_reward_config(
        target_altitude: f32,
        target_airspeed: f32,
        target_radius: f32,
        cfg: PlaneConfig,
        reward_cfg: OrbitRewardConfig,
    ) -> Self {
        let mut env = Self::new(target_altitude, target_airspeed, target_radius, cfg);
        env.max_episode_steps = reward_cfg.max_episode_steps;
        env.reward_cfg = reward_cfg;
        env
    }

    fn build_observation(&self) -> Observation {
        build_orbit_observation(
            &self.state,
            self.center_x,
            self.center_z,
            self.target_radius,
            self.target_altitude,
            self.target_airspeed,
            self.direction,
        )
    }

    fn current_terms(&self) -> OrbitObservationTerms {
        orbit_observation_terms(
            &self.state,
            self.center_x,
            self.center_z,
            self.target_radius,
            self.direction,
        )
    }

    fn build_observation_from_terms(&self, terms: &OrbitObservationTerms) -> Observation {
        build_orbit_observation_from_terms(
            &self.state,
            self.target_altitude,
            self.target_airspeed,
            terms,
        )
    }

    fn compute_base_reward(&self, terms: &OrbitObservationTerms) -> f32 {
        let c = &self.reward_cfg;
        let radial_err = terms.radial_error.abs();
        let heading_err = terms.guidance_heading_error.abs();
        let alt_err = (self.state.altitude - self.target_altitude).abs();
        let speed_err = (self.state.airspeed - self.target_airspeed).abs();
        let roll = roll_angle(self.state.attitude).abs();
        let beta = self.state.beta.abs();
        let p = self.state.angular_velocity.x.abs();
        let q = self.state.angular_velocity.y.abs();
        let r = self.state.angular_velocity.z.abs();

        -(radial_err / c.radial_reward_scale) * c.radial_reward_weight
            - (heading_err / c.heading_reward_scale) * c.heading_reward_weight
            - (alt_err / c.altitude_reward_scale) * c.altitude_reward_weight
            - (speed_err / c.speed_reward_scale) * c.speed_reward_weight
            - (roll / c.roll_reward_scale) * c.roll_reward_weight
            - (beta / c.beta_reward_scale) * c.beta_reward_weight
            - (q / c.pitch_rate_reward_scale) * c.pitch_rate_reward_weight
            - (p / c.roll_rate_reward_scale) * c.roll_rate_reward_weight
            - (r / c.yaw_rate_reward_scale) * c.yaw_rate_reward_weight
            + c.alive_reward
    }

    fn termination_reason(&self, terms: &OrbitObservationTerms) -> Option<TerminationReason> {
        let c = &self.reward_cfg;
        if self.state.altitude < c.min_altitude
            || (self.state.altitude - self.target_altitude).abs() > c.max_altitude_error
            || terms.radial_error.abs() > self.reward_cfg.max_radial_error
            || self.state.airspeed < c.min_airspeed
        {
            Some(TerminationReason::Failure)
        } else if self.episode_step >= self.max_episode_steps {
            Some(TerminationReason::Timeout)
        } else {
            None
        }
    }
}

impl TrainingEnv for ResidualOrbitEnv {
    fn offset_rng_seed(&mut self, offset: u64) {
        self.rng_seed = self.rng_seed.wrapping_add(offset);
        self.rng = Lcg::new(self.rng_seed);
    }

    fn reset(&mut self) -> (Observation, SpawnSpec) {
        self.rng_seed = self.rng_seed.wrapping_add(1);
        self.rng = Lcg::new(self.rng_seed);

        self.target_radius = self.rng.next_f32(
            *self.target_radius_range.start(),
            *self.target_radius_range.end(),
        );

        self.direction = if self.rng.next_f32(0.0, 1.0) < 0.5 {
            OrbitDirection::CounterClockwise
        } else {
            OrbitDirection::Clockwise
        };

        let angle = self.rng.next_f32(0.0, TWO_PI);
        let radial_offset = self.rng.next_f32(
            *self.radial_offset_range.start(),
            *self.radial_offset_range.end(),
        );
        let radius = (self.target_radius + radial_offset).max(100.0);
        let (sin_a, cos_a) = angle.sin_cos();
        let radial_x = cos_a;
        let radial_z = sin_a;

        let (tang_x, tang_z) = match self.direction {
            OrbitDirection::CounterClockwise => (-radial_z, radial_x),
            OrbitDirection::Clockwise => (radial_z, -radial_x),
        };
        let heading_perturb = self
            .rng
            .next_f32(-HEADING_PERTURB_RANGE, HEADING_PERTURB_RANGE);
        let (sin_h, cos_h) = heading_perturb.sin_cos();
        let head_x = cos_h * tang_x - sin_h * tang_z;
        let head_z = sin_h * tang_x + cos_h * tang_z;

        let altitude = self.target_altitude
            + self.rng.next_f32(
                *self.altitude_perturb_range.start(),
                *self.altitude_perturb_range.end(),
            );
        let airspeed = (self.target_airspeed
            + self.rng.next_f32(
                *self.airspeed_perturb_range.start(),
                *self.airspeed_perturb_range.end(),
            ))
        .max(25.0);
        let dvv = self.rng.next_f32(-VVEL_RANGE, VVEL_RANGE);
        let dp = self.rng.next_f32(-ANG_VEL_RANGE, ANG_VEL_RANGE);
        let dq = self.rng.next_f32(-ANG_VEL_RANGE, ANG_VEL_RANGE);
        let dr = self.rng.next_f32(-ANG_VEL_RANGE, ANG_VEL_RANGE);

        let position = Vec3::new(
            self.center_x + radial_x * radius,
            altitude,
            self.center_z + radial_z * radius,
        );
        let velocity = Vec3::new(head_x * airspeed, dvv, head_z * airspeed);
        let attitude = level_attitude_for_heading(head_x, head_z);
        let angular_velocity = Vec3::new(dp, dq, dr);

        self.state = FlightState {
            position,
            velocity,
            attitude,
            angular_velocity,
            alpha: 0.0,
            beta: 0.0,
            airspeed,
            altitude,
        };
        self.state.update_air_data();
        self.episode_step = 0;

        // Build PID from spawn state, then fix up with the sampled env parameters.
        let mut ctrl = OrbitController::from_state(&self.state, &ControlInputs::default());
        ctrl.center_x = self.center_x;
        ctrl.center_z = self.center_z;
        ctrl.target_radius = self.target_radius;
        ctrl.target_altitude = self.target_altitude;
        ctrl.target_airspeed = self.target_airspeed;
        ctrl.direction = self.direction;
        ctrl.inner.target_altitude = self.target_altitude;
        ctrl.inner.target_airspeed = self.target_airspeed;
        // Reseed bank feedforward with the actual sampled radius and direction.
        let bank_ff =
            self.direction.sign() * (self.state.airspeed.powi(2) / (G * self.target_radius)).atan();
        ctrl.inner.target_roll = bank_ff.clamp(-FRAC_PI_3, FRAC_PI_3);
        self.orbit_controller = ctrl;

        let spawn_spec = SpawnSpec {
            position: Some(self.state.position),
            velocity: Some(self.state.velocity),
            attitude: Some(attitude),
            angular_velocity: Some(angular_velocity),
        };

        (self.build_observation(), spawn_spec)
    }

    fn step(&mut self, action: &[f32]) -> (Observation, f32, bool, StepInfo) {
        let pid_inputs = self.orbit_controller.update(
            &self.state,
            &ControllerContext::empty_for(PlaneId::TEST),
            self.dt,
        );

        let scale = self.reward_cfg.residual_scale;
        let mut final_inputs = ControlInputs {
            elevator: pid_inputs.elevator + action[0] * scale,
            throttle: pid_inputs.throttle + action[1] * scale,
            aileron: pid_inputs.aileron + action[2] * scale,
            rudder: pid_inputs.rudder + action[3] * scale,
        };
        final_inputs.clamp();

        integrate_state(&mut self.state, &final_inputs, &self.cfg, self.dt);
        self.episode_step += 1;

        let terms = self.current_terms();
        let obs = self.build_observation_from_terms(&terms);
        let termination = self.termination_reason(&terms);
        let mut reward = self.compute_base_reward(&terms);
        if termination == Some(TerminationReason::Failure) {
            reward += self.reward_cfg.terminal_failure_penalty;
        }
        let done = termination.is_some();
        let info = StepInfo {
            episode_step: self.episode_step,
            ..Default::default()
        };

        (obs, reward, done, info)
    }

    fn observation_dim(&self) -> usize {
        ORBIT_OBS_DIM
    }

    fn action_dim(&self) -> usize {
        4
    }
}

fn level_attitude_for_heading(head_x: f32, head_z: f32) -> Quat {
    let forward = Vec3::new(head_x, 0.0, head_z).normalize_or_zero();
    let forward = if forward.length_squared() > 0.0 {
        forward
    } else {
        Vec3::X
    };
    let up = Vec3::Y;
    let right = up.cross(forward).normalize_or_zero();
    let right = if right.length_squared() > 0.0 {
        right
    } else {
        Vec3::NEG_Z
    };
    Quat::from_mat3(&Mat3::from_cols(forward, right, up)).normalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn jet_cfg() -> PlaneConfig {
        PlaneConfig {
            wing_area: 20.0,
            mean_chord: 2.0,
            wing_span: 10.0,
            mass: 5000.0,
            inertia: Vec3::new(10000.0, 40000.0, 45000.0),
            cl0: 0.1,
            cl_alpha: 4.5,
            cl_delta_e: 0.4,
            cl_max: 1.4,
            cd0: 0.02,
            cd_induced: 0.05,
            cm0: -0.02,
            cm_alpha: 0.6,
            cm_q: -14.0,
            cm_delta_e: -1.2,
            cl_beta: -0.08,
            cl_p: -0.45,
            cl_r: 0.12,
            cl_delta_a: 0.18,
            cn_beta: 0.10,
            cn_r: -0.12,
            cn_delta_r: -0.10,
            thrust_max: 60000.0,
            aileron_limit: 0.4363,
            elevator_limit: 0.3491,
            rudder_limit: 0.2618,
        }
    }

    fn state_at(radius: f32, heading: Vec3) -> FlightState {
        let velocity = heading.normalize() * 100.0;
        let mut state = FlightState {
            position: Vec3::new(0.0, 1000.0, -radius),
            velocity,
            attitude: level_attitude_for_heading(velocity.x, velocity.z),
            angular_velocity: Vec3::ZERO,
            alpha: 0.0,
            beta: 0.0,
            airspeed: 100.0,
            altitude: 1000.0,
        };
        state.update_air_data();
        state
    }

    #[test]
    fn dimensions_are_correct() {
        let env = ResidualOrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        assert_eq!(ORBIT_OBS_DIM, 13);
        assert_eq!(env.observation_dim(), ORBIT_OBS_DIM);
        assert_eq!(env.action_dim(), 4);
    }

    #[test]
    fn reset_and_step_outputs_are_finite() {
        let mut env = ResidualOrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        let (obs, _) = env.reset();
        assert_eq!(obs.len(), ORBIT_OBS_DIM);
        assert!(
            obs.iter().all(|v| v.is_finite()),
            "reset obs contains NaN/inf: {obs:?}"
        );

        let (obs, reward, _done, info) = env.step(&[0.0, 0.0, 0.0, 0.0]);
        assert_eq!(obs.len(), ORBIT_OBS_DIM);
        assert!(
            obs.iter().all(|v| v.is_finite()),
            "step obs contains NaN/inf: {obs:?}"
        );
        assert!(reward.is_finite(), "reward is not finite: {reward}");
        assert_eq!(info.episode_step, 1);
    }

    #[test]
    fn zero_action_matches_pid_trajectory() {
        // With action=[0,0,0,0] the residual delta is 0, so final_inputs == pid_inputs.
        // We verify by running the residual env and a standalone OrbitController on the
        // same state and checking they produce identical step observations.
        let mut env = ResidualOrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        let (_, spawn) = env.reset();

        // Reconstruct the same initial state for a reference orbit controller.
        let mut ref_state = FlightState {
            position: spawn.position.unwrap(),
            velocity: spawn.velocity.unwrap(),
            attitude: spawn.attitude.unwrap(),
            angular_velocity: spawn.angular_velocity.unwrap(),
            alpha: 0.0,
            beta: 0.0,
            airspeed: spawn.velocity.unwrap().length(),
            altitude: spawn.position.unwrap().y,
        };
        ref_state.update_air_data();

        // Manually advance the reference state using the PID output.
        let mut ref_ctrl = env.orbit_controller.clone();
        let pid_inputs = ref_ctrl.update(
            &ref_state,
            &ControllerContext::empty_for(PlaneId::TEST),
            env.dt,
        );
        integrate_state(&mut ref_state, &pid_inputs, &env.cfg, env.dt);
        ref_state.update_air_data();

        // Step the residual env with zero action.
        let (env_obs, _, _, _) = env.step(&[0.0, 0.0, 0.0, 0.0]);

        let ref_obs = build_orbit_observation(
            &ref_state,
            env.center_x,
            env.center_z,
            env.target_radius,
            env.target_altitude,
            env.target_airspeed,
            env.direction,
        );

        for (i, (e, r)) in env_obs.iter().zip(ref_obs.iter()).enumerate() {
            assert!((e - r).abs() < 1e-4, "obs[{i}] mismatch: env={e}, ref={r}");
        }
    }

    #[test]
    fn residual_scale_zero_matches_pid_output() {
        // With residual_scale=0.0, any action has no effect — same as zero action.
        let mut reward_cfg = OrbitRewardConfig::default();
        reward_cfg.residual_scale = 0.0;
        let mut env =
            ResidualOrbitEnv::with_reward_config(1000.0, 100.0, 1000.0, jet_cfg(), reward_cfg);
        env.offset_rng_seed(99);
        let _ = env.reset();
        let (obs_zero, _, _, _) = env.step(&[0.0, 0.0, 0.0, 0.0]);

        let mut reward_cfg2 = OrbitRewardConfig::default();
        reward_cfg2.residual_scale = 0.0;
        let mut env2 =
            ResidualOrbitEnv::with_reward_config(1000.0, 100.0, 1000.0, jet_cfg(), reward_cfg2);
        env2.offset_rng_seed(99);
        let _ = env2.reset();
        let (obs_full, _, _, _) = env2.step(&[1.0, 0.5, -0.3, 0.8]);

        for (i, (z, f)) in obs_zero.iter().zip(obs_full.iter()).enumerate() {
            assert!(
                (z - f).abs() < 1e-4,
                "obs[{i}] should match when scale=0: zero={z} full={f}"
            );
        }
    }

    #[test]
    fn failure_terminal_reward_includes_penalty() {
        let mut env = ResidualOrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        env.state = state_at(1000.0, Vec3::X);
        // Reset to initialize orbit_controller, then force crash state.
        let _ = env.reset();
        env.state = state_at(1000.0, Vec3::X);
        env.state.position.y = 5.0;
        env.state.altitude = 5.0;

        let (_obs, reward, done, _info) = env.step(&[0.0, 0.0, 0.0, 0.0]);

        assert!(done, "low altitude should terminate");
        let terms = env.current_terms();
        let expected = env.compute_base_reward(&terms) + env.reward_cfg.terminal_failure_penalty;
        assert!(
            (reward - expected).abs() < 1e-4,
            "reward={reward} expected={expected}"
        );
    }
}
