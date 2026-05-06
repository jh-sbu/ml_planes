//! Training environment for direct-control circular orbit.
//!
//! Observation (dim = 13, normalized):
//!   [radial_err/500, heading_err/0.5, bank_ff/60deg,
//!    alt_err/200, speed_err/50, alpha/0.5, pitch/0.5, pitch_rate/1,
//!    roll/0.5, roll_rate/1, beta/0.5, yaw_rate/1, vertical_speed/30]
//!
//! Action (dim = 4, each in [-1, 1]):
//!   [elevator, throttle_norm, aileron, rudder]
//!   Throttle mapping: ControlInputs.throttle = (action[1] + 1) / 2

use bevy::math::{Mat3, Quat, Vec3};

use crate::controllers::orbit::{
    build_orbit_observation, build_orbit_observation_from_terms, orbit_observation_terms,
    OrbitDirection, OrbitObservationTerms, ORBIT_OBS_DIM,
};
use crate::plane::{FlightState, PlaneConfig};
use crate::training::flight_env::{direct_action_to_inputs, integrate_state, roll_angle, Lcg};
use crate::training::reward_config::OrbitRewardConfig;
use crate::training::{Observation, SpawnSpec, StepInfo, TrainingEnv};

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
pub struct OrbitEnv {
    /// World-frame X coordinate of the orbit center [m].
    pub center_x: f32,
    /// World-frame Z coordinate of the orbit center [m].
    pub center_z: f32,
    /// Desired orbit radius [m].
    pub target_radius: f32,
    /// Target altitude [m].
    pub target_altitude: f32,
    /// Target airspeed [m/s].
    pub target_airspeed: f32,
    /// Episode terminates after this many steps.
    pub max_episode_steps: u32,
    /// Initial radial offset range [m].
    pub radial_offset_range: std::ops::RangeInclusive<f32>,
    /// Initial altitude perturbation around target altitude [m].
    pub altitude_perturb_range: std::ops::RangeInclusive<f32>,
    /// Initial airspeed perturbation around target airspeed [m/s].
    pub airspeed_perturb_range: std::ops::RangeInclusive<f32>,

    reward_cfg: OrbitRewardConfig,
    cfg: PlaneConfig,
    dt: f32,
    state: FlightState,
    direction: OrbitDirection,
    episode_step: u32,
    rng: Lcg,
    rng_seed: u64,
}

impl OrbitEnv {
    /// Create an environment using default reward config.
    pub fn new(
        target_altitude: f32,
        target_airspeed: f32,
        target_radius: f32,
        cfg: PlaneConfig,
    ) -> Self {
        let reward_cfg = OrbitRewardConfig::default();
        let max_episode_steps = reward_cfg.max_episode_steps;
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
            reward_cfg,
            cfg,
            dt: 1.0 / 60.0,
            state: FlightState::default(),
            direction: OrbitDirection::CounterClockwise,
            episode_step: 0,
            rng: Lcg::new(4242),
            rng_seed: 4242,
        }
    }

    /// Create an environment with an explicit reward config (e.g. loaded from a RON file).
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
        let heading_err = terms.heading_error.abs();
        let alt_err = (self.state.altitude - self.target_altitude).abs();
        let speed_err = (self.state.airspeed - self.target_airspeed).abs();
        let roll = roll_angle(self.state.attitude).abs();
        let beta = self.state.beta.abs();

        -(radial_err / c.radial_reward_scale) * c.radial_reward_weight
            - (heading_err / c.heading_reward_scale) * c.heading_reward_weight
            - (alt_err / c.altitude_reward_scale) * c.altitude_reward_weight
            - (speed_err / c.speed_reward_scale) * c.speed_reward_weight
            - (roll / c.roll_reward_scale) * c.roll_reward_weight
            - (beta / c.beta_reward_scale) * c.beta_reward_weight
            + c.alive_reward
    }

    #[cfg(test)]
    fn compute_reward(&self) -> f32 {
        let terms = self.current_terms();
        self.compute_base_reward(&terms)
    }

    fn termination_reason(&self, terms: &OrbitObservationTerms) -> Option<TerminationReason> {
        let c = &self.reward_cfg;
        if self.state.altitude < c.min_altitude
            || (self.state.altitude - self.target_altitude).abs() > c.max_altitude_error
            || terms.radial_error.abs() > self.target_radius.max(500.0)
            || self.state.airspeed < c.min_airspeed
        {
            Some(TerminationReason::Failure)
        } else if self.episode_step >= self.max_episode_steps {
            Some(TerminationReason::Timeout)
        } else {
            None
        }
    }

    #[cfg(test)]
    fn is_done(&self) -> bool {
        let terms = self.current_terms();
        self.termination_reason(&terms).is_some()
    }
}

impl TrainingEnv for OrbitEnv {
    fn offset_rng_seed(&mut self, offset: u64) {
        self.rng_seed = self.rng_seed.wrapping_add(offset);
        self.rng = Lcg::new(self.rng_seed);
    }

    fn reset(&mut self) -> (Observation, SpawnSpec) {
        self.rng_seed = self.rng_seed.wrapping_add(1);
        self.rng = Lcg::new(self.rng_seed);

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

        let spawn_spec = SpawnSpec {
            position: Some(self.state.position),
            velocity: Some(self.state.velocity),
            attitude: Some(attitude),
            angular_velocity: Some(angular_velocity),
        };

        (self.build_observation(), spawn_spec)
    }

    fn step(&mut self, action: &[f32]) -> (Observation, f32, bool, StepInfo) {
        let inputs = direct_action_to_inputs(action);
        integrate_state(&mut self.state, &inputs, &self.cfg, self.dt);
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
        let env = OrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        assert_eq!(ORBIT_OBS_DIM, 13);
        assert_eq!(env.observation_dim(), ORBIT_OBS_DIM);
        assert_eq!(env.action_dim(), 4);
    }

    #[test]
    fn reset_and_step_outputs_are_finite() {
        let mut env = OrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
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
    fn reward_prefers_lower_radial_and_heading_error() {
        let mut good = OrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        good.direction = OrbitDirection::CounterClockwise;
        good.state = state_at(1000.0, Vec3::X);

        let mut bad = good.clone();
        bad.state = state_at(1400.0, Vec3::Z);

        assert!(
            good.compute_reward() > bad.compute_reward(),
            "good={} bad={}",
            good.compute_reward(),
            bad.compute_reward()
        );
    }

    #[test]
    fn terminal_conditions_trigger() {
        let mut env = OrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        env.state = state_at(1000.0, Vec3::X);
        env.state.position.y = 5.0;
        env.state.altitude = 5.0;
        assert!(env.is_done(), "low altitude should terminate");

        env.state = state_at(1000.0, Vec3::X);
        env.episode_step = env.max_episode_steps;
        assert!(env.is_done(), "max episode steps should terminate");
    }

    #[test]
    fn failure_terminal_reward_includes_penalty() {
        let mut env = OrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
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

    #[test]
    fn timeout_terminal_reward_does_not_include_failure_penalty() {
        let mut env = OrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        env.state = state_at(1000.0, Vec3::X);
        env.max_episode_steps = 1;

        let (_obs, reward, done, _info) = env.step(&[0.0, 0.0, 0.0, 0.0]);

        assert!(done, "max episode steps should terminate");
        let terms = env.current_terms();
        let expected = env.compute_base_reward(&terms);
        assert!(
            (reward - expected).abs() < 1e-4,
            "reward={reward} expected={expected}"
        );
    }
}
