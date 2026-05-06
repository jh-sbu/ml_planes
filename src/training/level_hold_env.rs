//! Training environment for level altitude + airspeed hold.
//!
//! Self-contained: runs its own Euler-integrated 6-DOF flight model
//! using `compute_aero_forces`.  No Bevy ECS or Rapier required.
//!
//! Observation (dim = 10, normalised to ≈ [-1, 1]):
//!   [alt_err/200, speed_err/50, alpha/0.5, pitch_rate/1,
//!    roll_angle/0.5, roll_rate/1, beta/0.5, yaw_rate/1,
//!    pitch_angle/0.5, vertical_speed/30]
//!
//! Action (dim = 4, each in [-1, 1]):
//!   [elevator, throttle_norm, aileron, rudder]
//!   Throttle mapping: ControlInputs.throttle = (action[1] + 1) / 2

use bevy::math::{Quat, Vec3};

use crate::plane::{ControlInputs, FlightState, PlaneConfig};
use crate::training::flight_env::{
    direct_action_to_inputs, integrate_state, pitch_angle, roll_angle, Lcg,
};
use crate::training::reward_config::LevelHoldRewardConfig;
use crate::training::{Observation, SpawnSpec, StepInfo, TrainingEnv};

// Domain-randomization ranges applied at every reset.
const ROLL_RANGE: f32 = 10.0 * std::f32::consts::PI / 180.0; // ±10°
const PITCH_RANGE: f32 = 5.0 * std::f32::consts::PI / 180.0; // ±5°
const ANG_VEL_RANGE: f32 = 5.0 * std::f32::consts::PI / 180.0; // ±5°/s
const VVEL_RANGE: f32 = 2.0; // ±2 m/s

// ---------------------------------------------------------------------------
// LevelHoldEnv
// ---------------------------------------------------------------------------

/// Training environment for level flight hold.
#[derive(Clone)]
pub struct LevelHoldEnv {
    /// Target altitude [m].
    pub target_altitude: f32,
    /// Target airspeed [m/s].
    pub target_airspeed: f32,
    /// Episode terminates after this many steps.
    pub max_episode_steps: u32,
    /// Spawn altitude range [m].  Initial altitude is drawn uniformly.
    pub alt_spawn_range: std::ops::RangeInclusive<f32>,
    /// Spawn airspeed range [m/s].
    pub airspeed_spawn_range: std::ops::RangeInclusive<f32>,

    /// Reward weights, scales, and termination thresholds.
    reward_cfg: LevelHoldRewardConfig,
    /// Aerodynamic / mass configuration.
    cfg: PlaneConfig,
    /// Fixed-step size [s].
    dt: f32,
    /// Current flight state.
    state: FlightState,
    /// Steps elapsed in current episode.
    episode_step: u32,
    /// RNG state.
    rng: Lcg,
    /// RNG seed counter (incremented each reset so episodes differ).
    rng_seed: u64,
}

impl LevelHoldEnv {
    /// Create an environment using default reward config.
    pub fn new(target_altitude: f32, target_airspeed: f32, cfg: PlaneConfig) -> Self {
        let reward_cfg = LevelHoldRewardConfig::default();
        let max_episode_steps = reward_cfg.max_episode_steps;
        Self {
            target_altitude,
            target_airspeed,
            max_episode_steps,
            alt_spawn_range: (target_altitude - 150.0)..=(target_altitude + 150.0),
            airspeed_spawn_range: (target_airspeed - 20.0)..=(target_airspeed + 20.0),
            reward_cfg,
            cfg,
            dt: 1.0 / 60.0,
            state: FlightState::default(),
            episode_step: 0,
            rng: Lcg::new(42),
            rng_seed: 42,
        }
    }

    /// Create an environment with an explicit reward config (e.g. loaded from a RON file).
    pub fn with_reward_config(
        target_altitude: f32,
        target_airspeed: f32,
        cfg: PlaneConfig,
        reward_cfg: LevelHoldRewardConfig,
    ) -> Self {
        let mut env = Self::new(target_altitude, target_airspeed, cfg);
        env.max_episode_steps = reward_cfg.max_episode_steps;
        env.reward_cfg = reward_cfg;
        env
    }

    // --- Physics step -------------------------------------------------------

    fn integrate(&mut self, inputs: &ControlInputs) {
        integrate_state(&mut self.state, inputs, &self.cfg, self.dt);
    }

    // --- Helpers ------------------------------------------------------------

    fn build_observation(&self) -> Observation {
        let alt_err = self.state.altitude - self.target_altitude;
        let speed_err = self.state.airspeed - self.target_airspeed;
        let roll = roll_angle(self.state.attitude);
        // angular_velocity: body frame (p=roll, q=pitch, r=yaw)
        let p = self.state.angular_velocity.x;
        let q = self.state.angular_velocity.y;
        let r = self.state.angular_velocity.z;
        vec![
            alt_err / 200.0,
            speed_err / 50.0,
            self.state.alpha / 0.5,
            q / 1.0,
            roll / 0.5,
            p / 1.0,
            self.state.beta / 0.5,
            r / 1.0,
            pitch_angle(self.state.attitude) / 0.5,
            self.state.velocity.y / 30.0,
        ]
    }

    fn compute_reward(&self) -> f32 {
        let c = &self.reward_cfg;
        let alt_err = (self.state.altitude - self.target_altitude).abs();
        let speed_err = (self.state.airspeed - self.target_airspeed).abs();
        let roll = roll_angle(self.state.attitude).abs();
        let beta = self.state.beta.abs();

        -(alt_err / c.alt_error_scale) * c.alt_error_weight
            - (speed_err / c.speed_error_scale) * c.speed_error_weight
            - (roll / c.roll_scale) * c.roll_weight
            - (beta / c.beta_scale) * c.beta_weight
            + c.alive_bonus
    }

    fn is_done(&self) -> bool {
        let c = &self.reward_cfg;
        self.state.altitude < c.min_altitude
            || (self.state.altitude - self.target_altitude).abs() > c.max_altitude_error
            || self.episode_step >= self.max_episode_steps
    }

    fn action_to_inputs(action: &[f32]) -> ControlInputs {
        direct_action_to_inputs(action)
    }
}

impl TrainingEnv for LevelHoldEnv {
    fn offset_rng_seed(&mut self, offset: u64) {
        self.rng_seed = self.rng_seed.wrapping_add(offset);
        self.rng = Lcg::new(self.rng_seed);
    }

    fn reset(&mut self) -> (Observation, SpawnSpec) {
        // Advance seed so each episode starts with different conditions.
        self.rng_seed = self.rng_seed.wrapping_add(1);
        self.rng = Lcg::new(self.rng_seed);

        let spawn_alt = self
            .rng
            .next_f32(*self.alt_spawn_range.start(), *self.alt_spawn_range.end());
        let spawn_spd = self.rng.next_f32(
            *self.airspeed_spawn_range.start(),
            *self.airspeed_spawn_range.end(),
        );
        let droll = self.rng.next_f32(-ROLL_RANGE, ROLL_RANGE);
        let dpitch = self.rng.next_f32(-PITCH_RANGE, PITCH_RANGE);
        let dp = self.rng.next_f32(-ANG_VEL_RANGE, ANG_VEL_RANGE);
        let dq = self.rng.next_f32(-ANG_VEL_RANGE, ANG_VEL_RANGE);
        let dr = self.rng.next_f32(-ANG_VEL_RANGE, ANG_VEL_RANGE);
        let dvv = self.rng.next_f32(-VVEL_RANGE, VVEL_RANGE);

        // Base level-flight attitude: body +Z (up) aligns with world +Y (up).
        // Roll (body X) and pitch (body Y) perturbations are applied in body frame.
        let base_attitude = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);
        let attitude =
            (base_attitude * Quat::from_rotation_x(droll) * Quat::from_rotation_y(dpitch))
                .normalize();

        let ang_vel = Vec3::new(dp, dq, dr);

        self.state = FlightState {
            position: Vec3::new(0.0, spawn_alt, 0.0),
            velocity: Vec3::new(spawn_spd, dvv, 0.0),
            attitude,
            angular_velocity: ang_vel,
            alpha: 0.0,
            beta: 0.0,
            airspeed: spawn_spd,
            altitude: spawn_alt,
        };
        self.state.update_air_data();
        self.episode_step = 0;

        let spawn_spec = SpawnSpec {
            position: Some(self.state.position),
            velocity: Some(self.state.velocity),
            attitude: Some(attitude),
            angular_velocity: Some(ang_vel),
        };

        (self.build_observation(), spawn_spec)
    }

    fn step(&mut self, action: &[f32]) -> (Observation, f32, bool, StepInfo) {
        let inputs = Self::action_to_inputs(action);
        self.integrate(&inputs);
        self.episode_step += 1;

        let obs = self.build_observation();
        let reward = self.compute_reward();
        let done = self.is_done();
        let info = StepInfo {
            episode_step: self.episode_step,
            ..Default::default()
        };

        (obs, reward, done, info)
    }

    fn observation_dim(&self) -> usize {
        10
    }
    fn action_dim(&self) -> usize {
        4
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

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

    #[test]
    fn dimensions_are_correct() {
        let env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
        assert_eq!(env.observation_dim(), 10);
        assert_eq!(env.action_dim(), 4);
    }

    #[test]
    fn reset_returns_correct_obs_length() {
        let mut env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
        let (obs, _) = env.reset();
        assert_eq!(obs.len(), 10);
    }

    #[test]
    fn step_returns_correct_obs_length() {
        let mut env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
        env.reset();
        let (obs, _reward, _done, _info) = env.step(&[0.0, 0.0, 0.0, 0.0]);
        assert_eq!(obs.len(), 10);
    }

    #[test]
    fn obs_values_are_finite() {
        let mut env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
        env.reset();
        for _ in 0..60 {
            let (obs, reward, _, _) = env.step(&[0.0, 0.0, 0.0, 0.0]);
            assert!(
                obs.iter().all(|v| v.is_finite()),
                "obs contains NaN/inf: {:?}",
                obs
            );
            assert!(reward.is_finite(), "reward is not finite: {reward}");
        }
    }

    #[test]
    fn episode_terminates_on_ground() {
        // Start just above the termination altitude and apply full nose-down.
        let mut env = LevelHoldEnv::new(50.0, 80.0, jet_cfg());
        env.alt_spawn_range = 20.0..=20.0;
        env.reset();
        let mut done = false;
        for _ in 0..600 {
            let (_, _, d, _) = env.step(&[-1.0, -1.0, 0.0, 0.0]); // nose down, idle
            if d {
                done = true;
                break;
            }
        }
        assert!(done, "episode should have terminated near the ground");
    }

    #[test]
    fn throttle_remapping_is_correct() {
        let inputs = LevelHoldEnv::action_to_inputs(&[0.0, -1.0, 0.0, 0.0]);
        assert!(
            (inputs.throttle - 0.0).abs() < 1e-5,
            "throttle={}",
            inputs.throttle
        );

        let inputs = LevelHoldEnv::action_to_inputs(&[0.0, 1.0, 0.0, 0.0]);
        assert!(
            (inputs.throttle - 1.0).abs() < 1e-5,
            "throttle={}",
            inputs.throttle
        );

        let inputs = LevelHoldEnv::action_to_inputs(&[0.0, 0.0, 0.0, 0.0]);
        assert!(
            (inputs.throttle - 0.5).abs() < 1e-5,
            "throttle={}",
            inputs.throttle
        );
    }
}
