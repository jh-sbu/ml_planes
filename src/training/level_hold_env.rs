//! Training environment for level altitude + airspeed hold.
//!
//! Self-contained: runs its own Euler-integrated 6-DOF flight model
//! using `compute_aero_forces`.  No Bevy ECS or Rapier required.
//!
//! Observation (dim = 13, normalised to ≈ [-1, 1]):
//!   [alt_err/200, speed_err/50, alpha/0.5, pitch_rate/1,
//!    roll_angle/0.5, roll_rate/1, beta/0.5, yaw_rate/1,
//!    pitch_angle/0.5, vertical_speed/30, fuel_fraction,
//!    density_ratio, airspeed/100]
//!
//! The last two entries let the policy generalize across a randomized
//! target-altitude/airspeed envelope (see `with_target_ranges`):
//! `alt_err`/`speed_err` alone cannot distinguish "500 m, 10 m/s low" from
//! "5000 m, 10 m/s low", but the aerodynamics (and thrust) at those two
//! altitudes differ substantially via `density_ratio`. `density_ratio` is
//! exactly the physical quantity that scales `q̄` and thrust
//! (`aerodynamics::atmosphere`), so it also encodes absolute altitude
//! monotonically; raw airspeed disambiguates the absolute operating point
//! the way `alt_err` alone cannot.
//!
//! Action (dim = 4, each in [-1, 1]):
//!   [elevator, throttle_norm, aileron, rudder]
//!   Throttle mapping: ControlInputs.throttle = (action[1] + 1) / 2

use bevy::math::{Quat, Vec3};

use crate::aerodynamics::density_ratio;
use crate::controllers::{FlightController, LevelHoldController};
use crate::plane::{ControlInputs, FlightState, PlaneConfig, PHYSICS_DT};
use crate::training::flight_env::{
    direct_action_to_inputs, integrate_state, pitch_angle, roll_angle, Lcg,
};
use crate::training::reward_config::LevelHoldRewardConfig;
use crate::training::{
    DemonstrationEnv, Observation, SpawnSpec, StepInfo, StepOutcome, TerminationReason, TrainingEnv,
};

/// Observation dimension for the level-hold task. Shared by `LevelHoldEnv` and
/// `RlLevelHoldController` — both must agree with `level_hold_observation` below.
pub const LEVEL_HOLD_OBS_DIM: usize = 13;

/// Scale divisor for the raw-airspeed observation element (index 12).
pub const AIRSPEED_OBS_SCALE: f32 = 100.0;

// Domain-randomization ranges applied at every reset.
const ROLL_RANGE: f32 = 10.0 * std::f32::consts::PI / 180.0; // ±10°
const PITCH_RANGE: f32 = 5.0 * std::f32::consts::PI / 180.0; // ±5°
const ANG_VEL_RANGE: f32 = 5.0 * std::f32::consts::PI / 180.0; // ±5°/s
const VVEL_RANGE: f32 = 2.0; // ±2 m/s

/// Default randomized-target envelope: jointly stall-feasible for the generic
/// jet at full fuel (7000 kg loaded, S=20 m², CL_max=1.4) — the worst corner,
/// 5000 m @ 90 m/s, requires CL≈1.15 with margin to stall.
pub const DEFAULT_TARGET_ALT_MIN: f32 = 500.0;
pub const DEFAULT_TARGET_ALT_MAX: f32 = 5000.0;
pub const DEFAULT_TARGET_AIRSPEED_MIN: f32 = 90.0;
pub const DEFAULT_TARGET_AIRSPEED_MAX: f32 = 140.0;

/// Spawn is never allowed below this altitude/airspeed regardless of the
/// sampled target + offset — guards against a pathological user-supplied
/// range spawning the plane already on the ground or unable to fly.
const MIN_SPAWN_ALTITUDE_MARGIN: f32 = 50.0;
const MIN_SPAWN_AIRSPEED: f32 = 70.0;

/// Build the normalized level-hold observation vector for `state` relative to
/// the given target altitude/airspeed. The single definition shared by
/// `LevelHoldEnv::build_observation` and `RlLevelHoldController::update`, so
/// training and inference cannot silently drift apart.
pub fn level_hold_observation(
    state: &FlightState,
    target_altitude: f32,
    target_airspeed: f32,
) -> Observation {
    let alt_err = state.altitude - target_altitude;
    let speed_err = state.airspeed - target_airspeed;
    let roll = roll_angle(state.attitude);
    // angular_velocity: body frame (p=roll, q=pitch, r=yaw)
    let p = state.angular_velocity.x;
    let q = state.angular_velocity.y;
    let r = state.angular_velocity.z;
    vec![
        alt_err / 200.0,
        speed_err / 50.0,
        state.alpha / 0.5,
        q / 1.0,
        roll / 0.5,
        p / 1.0,
        state.beta / 0.5,
        r / 1.0,
        pitch_angle(state.attitude) / 0.5,
        state.velocity.y / 30.0,
        // Remaining fuel fraction in [0, 1].
        state.fuel_fraction_obs(),
        // Air density ratio ρ(h)/ρ₀ — lets the policy distinguish operating
        // points that share the same alt_err/speed_err but sit at very
        // different altitudes (thinner air ⇒ less lift/drag/thrust).
        density_ratio(state.altitude),
        // Raw airspeed — disambiguates the absolute operating point.
        state.airspeed / AIRSPEED_OBS_SCALE,
    ]
}

// ---------------------------------------------------------------------------
// LevelHoldEnv
// ---------------------------------------------------------------------------

/// Training environment for level flight hold.
#[derive(Clone)]
pub struct LevelHoldEnv {
    /// Target altitude for the *current episode* [m]. Resampled from
    /// `target_altitude_range` on every `reset()`.
    pub target_altitude: f32,
    /// Target airspeed for the *current episode* [m/s]. Resampled from
    /// `target_airspeed_range` on every `reset()`.
    pub target_airspeed: f32,
    /// Range the per-episode target altitude is drawn from [m].
    pub target_altitude_range: std::ops::RangeInclusive<f32>,
    /// Range the per-episode target airspeed is drawn from [m/s].
    pub target_airspeed_range: std::ops::RangeInclusive<f32>,
    /// Episode terminates after this many steps.
    pub max_episode_steps: u32,
    /// Spawn altitude offset from the episode's target altitude [m].
    pub alt_spawn_offset_range: std::ops::RangeInclusive<f32>,
    /// Spawn airspeed offset from the episode's target airspeed [m/s].
    pub airspeed_spawn_offset_range: std::ops::RangeInclusive<f32>,

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
    /// Create an environment with a fixed target altitude/airspeed and the
    /// default reward config.
    pub fn new(target_altitude: f32, target_airspeed: f32, cfg: PlaneConfig) -> Self {
        let reward_cfg = LevelHoldRewardConfig::default();
        let max_episode_steps = reward_cfg.max_episode_steps;
        Self {
            target_altitude,
            target_airspeed,
            target_altitude_range: target_altitude..=target_altitude,
            target_airspeed_range: target_airspeed..=target_airspeed,
            max_episode_steps,
            alt_spawn_offset_range: -150.0..=150.0,
            airspeed_spawn_offset_range: -20.0..=20.0,
            reward_cfg,
            cfg,
            dt: PHYSICS_DT,
            state: FlightState::default(),
            episode_step: 0,
            rng: Lcg::new(42),
            rng_seed: 42,
        }
    }

    /// Create a fixed-target environment with an explicit reward config (e.g.
    /// loaded from a RON file).
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

    /// Create an environment whose target altitude/airspeed are resampled
    /// from `alt_range`/`airspeed_range` on every `reset()`. This is the
    /// constructor `train_ppo`/`train_bc`/`evaluate_policy` use so a single
    /// policy is trained/evaluated across a flight envelope rather than one
    /// fixed operating point.
    pub fn with_target_ranges(
        alt_range: std::ops::RangeInclusive<f32>,
        airspeed_range: std::ops::RangeInclusive<f32>,
        cfg: PlaneConfig,
        reward_cfg: LevelHoldRewardConfig,
    ) -> Self {
        // Seed target_altitude/target_airspeed to the range midpoints so the
        // struct is valid even before the first reset() draws real values.
        let mid_alt = (*alt_range.start() + *alt_range.end()) / 2.0;
        let mid_spd = (*airspeed_range.start() + *airspeed_range.end()) / 2.0;
        let mut env = Self::with_reward_config(mid_alt, mid_spd, cfg, reward_cfg);
        env.target_altitude_range = alt_range;
        env.target_airspeed_range = airspeed_range;
        env
    }

    // --- Physics step -------------------------------------------------------

    fn integrate(&mut self, inputs: &ControlInputs) {
        integrate_state(&mut self.state, inputs, &self.cfg, self.dt);
    }

    // --- Helpers ------------------------------------------------------------

    fn build_observation(&self) -> Observation {
        level_hold_observation(&self.state, self.target_altitude, self.target_airspeed)
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

    /// Why the episode ends here, if it does. Mirrors
    /// [`crate::training::heading_hold_env::HeadingHoldEnv::termination_reason`]:
    /// the failure checks come first so that a plane which crashes on the very last
    /// step is reported as a `Failure`, not a `Timeout`.
    fn termination_reason(&self) -> Option<TerminationReason> {
        let c = &self.reward_cfg;
        if self.state.altitude < c.min_altitude
            || (self.state.altitude - self.target_altitude).abs() > c.max_altitude_error
        {
            Some(TerminationReason::Failure)
        } else if self.episode_step >= self.max_episode_steps {
            Some(TerminationReason::Timeout)
        } else {
            None
        }
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

        // Resample this episode's target altitude/airspeed from the
        // configured ranges (degenerate ranges from `new()` always yield the
        // same fixed target).
        self.target_altitude = self.rng.next_f32(
            *self.target_altitude_range.start(),
            *self.target_altitude_range.end(),
        );
        self.target_airspeed = self.rng.next_f32(
            *self.target_airspeed_range.start(),
            *self.target_airspeed_range.end(),
        );

        let alt_offset = self.rng.next_f32(
            *self.alt_spawn_offset_range.start(),
            *self.alt_spawn_offset_range.end(),
        );
        let spd_offset = self.rng.next_f32(
            *self.airspeed_spawn_offset_range.start(),
            *self.airspeed_spawn_offset_range.end(),
        );
        // Clamp against pathological CLI-supplied ranges. A low-speed spawn
        // against a high target altitude (or vice versa) is a deliberate
        // *recovery* scenario the policy must handle within the reward
        // config's `max_altitude_error` budget — not a bug to avoid.
        let spawn_alt = (self.target_altitude + alt_offset)
            .max(self.reward_cfg.min_altitude + MIN_SPAWN_ALTITUDE_MARGIN);
        let spawn_spd = (self.target_airspeed + spd_offset).max(MIN_SPAWN_AIRSPEED);

        let droll = self.rng.next_f32(-ROLL_RANGE, ROLL_RANGE);
        let dpitch = self.rng.next_f32(-PITCH_RANGE, PITCH_RANGE);
        let dp = self.rng.next_f32(-ANG_VEL_RANGE, ANG_VEL_RANGE);
        let dq = self.rng.next_f32(-ANG_VEL_RANGE, ANG_VEL_RANGE);
        let dr = self.rng.next_f32(-ANG_VEL_RANGE, ANG_VEL_RANGE);
        let dvv = self.rng.next_f32(-VVEL_RANGE, VVEL_RANGE);
        // Randomize the fuel load so the policy observes (and is robust to) a range of
        // fuel fractions / airframe masses. Never start empty.
        let fuel_fraction = self.rng.next_f32(0.2, 1.0);

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
            consumable_remaining: self.cfg.powerplant.capacity() * fuel_fraction,
        };
        self.state.update_air_data();
        self.episode_step = 0;

        let spawn_spec = SpawnSpec {
            position: Some(self.state.position),
            velocity: Some(self.state.velocity),
            attitude: Some(attitude),
            angular_velocity: Some(ang_vel),
            fuel_fraction: Some(fuel_fraction),
        };

        (self.build_observation(), spawn_spec)
    }

    fn step(&mut self, action: &[f32]) -> StepOutcome {
        let inputs = Self::action_to_inputs(action);
        self.integrate(&inputs);
        self.episode_step += 1;

        let obs = self.build_observation();
        let reward = self.compute_reward();
        let info = StepInfo {
            episode_step: self.episode_step,
            ..Default::default()
        };

        StepOutcome {
            obs,
            reward,
            end: self.termination_reason(),
            info,
        }
    }

    fn observation_dim(&self) -> usize {
        LEVEL_HOLD_OBS_DIM
    }
    fn action_dim(&self) -> usize {
        4
    }
}

impl DemonstrationEnv for LevelHoldEnv {
    fn current_state(&self) -> FlightState {
        self.state.clone()
    }

    fn dt(&self) -> f32 {
        self.dt
    }

    fn make_expert(&self) -> Box<dyn FlightController> {
        Box::new(LevelHoldController::new(
            self.target_altitude,
            self.target_airspeed,
        ))
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// The training integrator must use the live simulation timestep.
    #[test]
    fn env_dt_is_the_shared_physics_dt() {
        let env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
        assert_eq!(env.dt, PHYSICS_DT);
    }

    /// The shared frozen test airframe (`fixtures/generic_jet.plane.ron`).
    /// A snapshot, not a mirror of `assets/planes/` — see `fixture_jet_config`.
    fn jet_cfg() -> PlaneConfig {
        crate::plane::config::fixture_jet_config()
    }

    #[test]
    fn dimensions_are_correct() {
        let env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
        assert_eq!(env.observation_dim(), 13);
        assert_eq!(env.action_dim(), 4);
    }

    #[test]
    fn reset_returns_correct_obs_length() {
        let mut env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
        let (obs, _) = env.reset();
        assert_eq!(obs.len(), 13);
    }

    #[test]
    fn step_returns_correct_obs_length() {
        let mut env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
        env.reset();
        let obs = env.step(&[0.0, 0.0, 0.0, 0.0]).obs;
        assert_eq!(obs.len(), 13);
    }

    #[test]
    fn obs_values_are_finite() {
        let mut env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
        env.reset();
        for _ in 0..60 {
            let out = env.step(&[0.0, 0.0, 0.0, 0.0]);
            let (obs, reward) = (out.obs, out.reward);
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
        env.alt_spawn_offset_range = -30.0..=-30.0; // target(50) + offset(-30) = 20
        env.reset();
        let mut end = None;
        for _ in 0..600 {
            let out = env.step(&[-1.0, -1.0, 0.0, 0.0]); // nose down, idle
            if out.done() {
                end = out.end;
                break;
            }
        }
        assert_eq!(
            end,
            Some(TerminationReason::Failure),
            "flying into the ground is a failure — it must not bootstrap a continuation value"
        );
    }

    #[test]
    fn running_out_of_steps_is_a_timeout() {
        // The counterpart to `episode_terminates_on_ground`. Level trim well clear
        // of every failure threshold means the only way this episode can end is the
        // step limit, and that ending must be reported as a truncation so the PPO
        // value target bootstraps rather than treating the state as absorbing.
        let n = 5u32;
        let mut env = LevelHoldEnv::new(1000.0, 100.0, jet_cfg());
        env.alt_spawn_offset_range = 0.0..=0.0;
        env.airspeed_spawn_offset_range = 0.0..=0.0;
        env.max_episode_steps = n;
        env.reset();

        let mut ended = None;
        for _ in 0..(n + 10) {
            let out = env.step(&[0.0, 0.0, 0.0, 0.0]);
            if out.done() {
                ended = Some((out.end, out.info.episode_step));
                break;
            }
        }

        assert_eq!(
            ended,
            Some((Some(TerminationReason::Timeout), n)),
            "must time out at exactly step {n}"
        );
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

    #[test]
    fn observation_appends_density_ratio_and_airspeed() {
        let mut state = FlightState {
            altitude: 3000.0,
            airspeed: 123.0,
            ..Default::default()
        };
        state.position.y = 3000.0;
        let obs = level_hold_observation(&state, 1000.0, 100.0);
        assert_eq!(obs.len(), 13);
        assert!(
            (obs[11] - crate::aerodynamics::density_ratio(3000.0)).abs() < 1e-6,
            "obs[11]={} expected density_ratio(3000)={}",
            obs[11],
            crate::aerodynamics::density_ratio(3000.0)
        );
        assert!(
            (obs[12] - 123.0 / AIRSPEED_OBS_SCALE).abs() < 1e-6,
            "obs[12]={}",
            obs[12]
        );
    }

    #[test]
    fn fixed_target_constructor_pins_targets() {
        let mut env = LevelHoldEnv::new(1000.0, 100.0, jet_cfg());
        for _ in 0..20 {
            env.reset();
            assert_eq!(env.target_altitude, 1000.0);
            assert_eq!(env.target_airspeed, 100.0);
        }
    }

    #[test]
    fn reset_samples_target_within_range() {
        let mut env = LevelHoldEnv::with_target_ranges(
            500.0..=5000.0,
            90.0..=140.0,
            jet_cfg(),
            LevelHoldRewardConfig::default(),
        );
        for _ in 0..200 {
            env.reset();
            assert!(
                (500.0..=5000.0).contains(&env.target_altitude),
                "target_altitude {} out of range",
                env.target_altitude
            );
            assert!(
                (90.0..=140.0).contains(&env.target_airspeed),
                "target_airspeed {} out of range",
                env.target_airspeed
            );
        }
    }

    #[test]
    fn reset_varies_targets_across_episodes() {
        let mut env = LevelHoldEnv::with_target_ranges(
            500.0..=5000.0,
            90.0..=140.0,
            jet_cfg(),
            LevelHoldRewardConfig::default(),
        );
        let mut seen = std::collections::HashSet::new();
        for _ in 0..20 {
            env.reset();
            seen.insert((env.target_altitude.to_bits(), env.target_airspeed.to_bits()));
        }
        assert!(
            seen.len() >= 2,
            "expected varied targets across episodes, got {} distinct",
            seen.len()
        );
    }

    #[test]
    fn spawn_offset_is_relative_to_episode_target() {
        let mut env = LevelHoldEnv::with_target_ranges(
            1000.0..=1000.0,
            100.0..=100.0,
            jet_cfg(),
            LevelHoldRewardConfig::default(),
        );
        env.alt_spawn_offset_range = 100.0..=100.0;
        env.airspeed_spawn_offset_range = 5.0..=5.0;
        env.reset();
        assert!((env.target_altitude - 1000.0).abs() < 1e-6);
        // spawn altitude should be exactly target + 100
        assert!(
            (env.current_state().altitude - 1100.0).abs() < 1e-3,
            "altitude={}",
            env.current_state().altitude
        );
        // Check the forward-velocity component directly rather than the
        // scalar `airspeed` (|velocity|): the random vertical-speed
        // perturbation (`VVEL_RANGE`) also contributes to the magnitude, so
        // `airspeed` alone isn't exactly `target + offset`.
        assert!(
            (env.current_state().velocity.x - 105.0).abs() < 1e-3,
            "velocity.x={}",
            env.current_state().velocity.x
        );
    }

    #[test]
    fn spawn_altitude_is_clamped() {
        // Pathological range: target near min_altitude with a large negative offset.
        let reward_cfg = LevelHoldRewardConfig::default();
        let min_alt = reward_cfg.min_altitude;
        let mut env =
            LevelHoldEnv::with_target_ranges(20.0..=20.0, 90.0..=90.0, jet_cfg(), reward_cfg);
        env.alt_spawn_offset_range = -1000.0..=-1000.0;
        env.reset();
        assert!(
            env.current_state().altitude >= min_alt,
            "altitude {} should be clamped above min_altitude {}",
            env.current_state().altitude,
            min_alt
        );
    }

    #[test]
    fn spawn_airspeed_is_clamped() {
        let mut env = LevelHoldEnv::with_target_ranges(
            1000.0..=1000.0,
            80.0..=80.0,
            jet_cfg(),
            LevelHoldRewardConfig::default(),
        );
        env.airspeed_spawn_offset_range = -1000.0..=-1000.0;
        env.reset();
        assert!(
            env.current_state().airspeed >= MIN_SPAWN_AIRSPEED,
            "airspeed {} should be clamped above {}",
            env.current_state().airspeed,
            MIN_SPAWN_AIRSPEED
        );
    }
}
