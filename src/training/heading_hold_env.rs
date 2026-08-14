//! Training environment for heading hold (coordinated turns + level flight).
//!
//! Self-contained: runs its own Euler-integrated 6-DOF flight model using
//! `compute_aero_forces`. No Bevy ECS or Rapier required.
//!
//! Observation (dim = 16, normalised to ≈ [-1, 1]):
//!   [alt_err/200, speed_err/50, alpha/0.5, pitch_rate/1,
//!    roll_angle/0.5, roll_rate/1, beta/0.5, yaw_rate/1,
//!    pitch_angle/0.5, vertical_speed/30, fuel_fraction,
//!    density_ratio, airspeed/100,
//!    sin(heading_error)/0.5, cos(heading_error), turn_rate/0.2]
//!
//! The first 13 elements are exactly `level_hold_observation` — reused
//! verbatim (not recomputed) so this env and `RlLevelHoldController` never
//! silently drift apart, and so `MetricFamily::LevelHold`'s obs-index
//! assumptions (0/1/4/6) keep meaning what they already mean.
//!
//! The heading error is encoded as `(sin, cos)` rather than a signed
//! `error/π` term: many episodes *start* near the ±π boundary (the target
//! heading is sampled from the full circle), and a raw `error/π` term would
//! jump by ±2.0 across that boundary even though the value function is
//! continuous there. `sin(e)/0.5` also keeps usable resolution at a settled
//! small error (0.02 rad → 0.04, vs 0.006 for `e/π`), matching the
//! `roll/0.5`/`beta/0.5` scale convention already used above.
//!
//! The turn-rate element is the airframe's rotation rate about the *world*
//! vertical (`(attitude * angular_velocity).y`), not body yaw-rate `r` alone:
//! in a coordinated bank φ the true turn rate splits as `q·sinφ + r·cosφ`,
//! so at a 60° bank body `r` alone is only about half the real rate. The
//! world-frame form is singularity-free and a pure function of `FlightState`,
//! so the env and `RlHeadingHoldController::update` compute it identically.
//!
//! Action (dim = 4, each in [-1, 1]):
//!   [elevator, throttle_norm, aileron, rudder]
//!   Throttle mapping: ControlInputs.throttle = (action[1] + 1) / 2
//!
//! # Sampling target heading
//!
//! The dynamics are exactly invariant under rotation about world Y: forces
//! depend on world position only through altitude (`density_ratio`), the
//! only world-frame term in `integrate_state` is gravity along -Y, and there
//! is no wind field. The observation above carries no absolute heading or
//! world XZ position either. So spawning along +X (ground track = 0, per
//! `LevelHoldEnv::reset`'s convention) and sampling an absolute
//! `target_heading` from the full circle is fully general — the sampled
//! value *is* the required heading change.
//!
//! # Suggested curriculum
//!
//! `PpoTrainer` (unlike `LstmPpoTrainer`) never calls `CurriculumEnv`, so
//! there is no in-env curriculum here. Stage training manually via
//! `--target-heading-range` + `--init-from`:
//! ```text
//! train_ppo --task heading_hold --target-heading-range -30:30   --bc-steps 200000 --steps 1000000 --output hh_s1
//! train_ppo --task heading_hold --target-heading-range -90:90   --init-from models/heading_hold/hh_s1 --steps 1000000 --output hh_s2
//! train_ppo --task heading_hold --target-heading-range -180:180 --init-from models/heading_hold/hh_s2 --steps 2000000 --output hh_s3
//! ```

use bevy::math::{Quat, Vec3};

use crate::controllers::heading_hold::heading_error;
use crate::controllers::targets::ControllerTargets;
use crate::controllers::{FlightController, HeadingHoldController};
use crate::plane::{ControlInputs, FlightState, PlaneConfig, PHYSICS_DT};
use crate::training::flight_env::{direct_action_to_inputs, integrate_state, roll_angle, Lcg};
use crate::training::level_hold_env::{level_hold_observation, LEVEL_HOLD_OBS_DIM};
use crate::training::reward_config::HeadingHoldRewardConfig;
use crate::training::{
    DemonstrationEnv, Observation, SpawnSpec, StepInfo, StepOutcome, TerminationReason, TrainingEnv,
};

/// Observation dimension for the heading-hold task: the 13-dim level-hold
/// observation plus `[sin(heading_error)/scale, cos(heading_error), turn_rate/scale]`.
pub const HEADING_HOLD_OBS_DIM: usize = LEVEL_HOLD_OBS_DIM + 3;

/// Scale divisor for the fine (sin) heading-error observation element.
pub const HEADING_ERROR_OBS_SCALE: f32 = 0.5;
/// Scale divisor for the world-vertical turn-rate observation element [rad/s].
pub const TURN_RATE_OBS_SCALE: f32 = 0.2;

// Domain-randomization ranges applied at every reset. Roll perturbation is
// wider than level-hold's ±10° so resets include established-bank, mid-turn
// states, not only wings-level starts.
const ROLL_RANGE: f32 = 30.0 * std::f32::consts::PI / 180.0;
const PITCH_RANGE: f32 = 5.0 * std::f32::consts::PI / 180.0;
const ANG_VEL_RANGE: f32 = 5.0 * std::f32::consts::PI / 180.0;
const VVEL_RANGE: f32 = 2.0;

/// Default randomized target-heading envelope [deg]: the full circle.
pub const DEFAULT_TARGET_HEADING_DEG_MIN: f32 = -180.0;
pub const DEFAULT_TARGET_HEADING_DEG_MAX: f32 = 180.0;

/// Default randomized-target airspeed envelope [m/s] — deliberately identical to
/// level-hold's `90:140`, pinned by `default_airspeed_envelope_matches_level_hold`.
///
/// This used to be `110:140`, on the argument that at the 5000 m / 90 m/s / full-fuel
/// corner the generic jet needs CL≈1.15 of `cl_max=1.4` just for 1 g, leaving little
/// margin to sustain the bank a 180° turn requires. That squeeze is real and is now an
/// **accepted cost**: the tighter floor meant anything flown below 110 m/s was out of
/// distribution, and the live sim routinely runs planes at 100. An `RlHeadingHold`
/// checkpoint accepted at `mean_tail_abs_altitude_m = 0.333` measured 1.47 m live; the
/// same policy re-evaluated at 1000 m / 100 m/s in the *training* physics already showed
/// 1.142 m, so most of that gap was this envelope, not the transfer.
///
/// Two consequences to train around rather than design around:
///   - The bleed margin to `min_airspeed` (60 m/s) narrows from 50 to 30 m/s, so early
///     training fails more often. That is the correct signal, not a regression. Use a BC
///     warm start (`--bc-steps`) so the policy inherits the PID expert's speed
///     management, and stage via `--target-speed-range 110:140` then `--init-from` at
///     `90:140` if a cold run struggles.
///   - Spawn no longer starts near that floor — see [`MIN_SPAWN_AIRSPEED_MARGIN`].
///
/// `--target-speed-range 110:140` remains available for the easier envelope.
pub const DEFAULT_TARGET_AIRSPEED_MIN: f32 = 90.0;
pub const DEFAULT_TARGET_AIRSPEED_MAX: f32 = 140.0;

const MIN_SPAWN_ALTITUDE_MARGIN: f32 = 50.0;
/// Spawn is never closer than this to `min_airspeed`, the instant-termination floor.
///
/// Expressed as a *margin over the reward config's floor* rather than a bare airspeed,
/// mirroring the `min_altitude + MIN_SPAWN_ALTITUDE_MARGIN` clamp below, so it tracks a
/// retuned `min_airspeed` automatically. A bare constant stops being a margin the moment
/// the target envelope moves down: at a 90 m/s target with the ±20 spawn offset, the old
/// flat 70.0 put episode starts 10 m/s from instant failure. A hard turn at the envelope
/// floor bleeds speed fast, so such an episode is unlearnable rather than merely hard.
const MIN_SPAWN_AIRSPEED_MARGIN: f32 = 20.0;

/// Build the normalized heading-hold observation vector for `state` relative
/// to the given target heading/altitude/airspeed. The single definition
/// shared by `HeadingHoldEnv::build_observation` and
/// `RlHeadingHoldController::update`.
pub fn heading_hold_observation(
    state: &FlightState,
    target_heading: f32,
    target_altitude: f32,
    target_airspeed: f32,
) -> Observation {
    let mut obs = level_hold_observation(state, target_altitude, target_airspeed);
    let e = heading_error(state, target_heading);
    let (sin_e, cos_e) = e.sin_cos();
    obs.push(sin_e / HEADING_ERROR_OBS_SCALE);
    obs.push(cos_e);
    let turn_rate = (state.attitude * state.angular_velocity).y;
    obs.push(turn_rate / TURN_RATE_OBS_SCALE);
    obs
}

// ---------------------------------------------------------------------------
// HeadingHoldEnv
// ---------------------------------------------------------------------------

/// Training environment for heading hold.
#[derive(Clone)]
pub struct HeadingHoldEnv {
    /// Target heading for the *current episode* [rad]. Resampled from
    /// `target_heading_range` on every `reset()`.
    pub target_heading: f32,
    /// Target altitude for the *current episode* [m].
    pub target_altitude: f32,
    /// Target airspeed for the *current episode* [m/s].
    pub target_airspeed: f32,
    /// Range the per-episode target heading is drawn from [rad].
    pub target_heading_range: std::ops::RangeInclusive<f32>,
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

    reward_cfg: HeadingHoldRewardConfig,
    cfg: PlaneConfig,
    dt: f32,
    state: FlightState,
    episode_step: u32,
    rng: Lcg,
    rng_seed: u64,
}

impl HeadingHoldEnv {
    /// Create an environment with a **fixed** target heading/altitude/airspeed
    /// (no per-episode randomization) using the default reward config.
    pub fn new(
        target_heading: f32,
        target_altitude: f32,
        target_airspeed: f32,
        cfg: PlaneConfig,
    ) -> Self {
        let reward_cfg = HeadingHoldRewardConfig::default();
        let max_episode_steps = reward_cfg.max_episode_steps;
        Self {
            target_heading,
            target_altitude,
            target_airspeed,
            target_heading_range: target_heading..=target_heading,
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

    /// Create a fixed-target environment with an explicit reward config.
    pub fn with_reward_config(
        target_heading: f32,
        target_altitude: f32,
        target_airspeed: f32,
        cfg: PlaneConfig,
        reward_cfg: HeadingHoldRewardConfig,
    ) -> Self {
        let mut env = Self::new(target_heading, target_altitude, target_airspeed, cfg);
        env.max_episode_steps = reward_cfg.max_episode_steps;
        env.reward_cfg = reward_cfg;
        env
    }

    /// Create an environment whose target heading/altitude/airspeed are
    /// resampled from the given ranges on every `reset()`. `heading_range` is
    /// in **radians**. This is the constructor `train_ppo`/`train_bc`/
    /// `evaluate_policy` use.
    pub fn with_target_ranges(
        heading_range: std::ops::RangeInclusive<f32>,
        alt_range: std::ops::RangeInclusive<f32>,
        airspeed_range: std::ops::RangeInclusive<f32>,
        cfg: PlaneConfig,
        reward_cfg: HeadingHoldRewardConfig,
    ) -> Self {
        let mid_heading = (*heading_range.start() + *heading_range.end()) / 2.0;
        let mid_alt = (*alt_range.start() + *alt_range.end()) / 2.0;
        let mid_spd = (*airspeed_range.start() + *airspeed_range.end()) / 2.0;
        let mut env = Self::with_reward_config(mid_heading, mid_alt, mid_spd, cfg, reward_cfg);
        env.target_heading_range = heading_range;
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
        heading_hold_observation(
            &self.state,
            self.target_heading,
            self.target_altitude,
            self.target_airspeed,
        )
    }

    fn compute_reward(&self) -> f32 {
        let c = &self.reward_cfg;
        let heading_err = heading_error(&self.state, self.target_heading).abs();
        let alt_err = (self.state.altitude - self.target_altitude).abs();
        let speed_err = (self.state.airspeed - self.target_airspeed).abs();
        let beta = self.state.beta.abs();
        let p = self.state.angular_velocity.x.abs();
        let bank_excess = (roll_angle(self.state.attitude).abs() - c.bank_soft_limit).max(0.0);
        let alpha_excess = (self.state.alpha.abs() - c.alpha_soft_limit).max(0.0);

        -(heading_err / c.heading_error_scale) * c.heading_error_weight
            - (alt_err / c.alt_error_scale) * c.alt_error_weight
            - (speed_err / c.speed_error_scale) * c.speed_error_weight
            - (beta / c.beta_scale) * c.beta_weight
            - (p / c.roll_rate_scale) * c.roll_rate_weight
            - (bank_excess / c.bank_excess_scale) * c.bank_excess_weight
            - (alpha_excess / c.alpha_excess_scale) * c.alpha_excess_weight
            + c.alive_bonus
    }

    fn termination_reason(&self) -> Option<TerminationReason> {
        let c = &self.reward_cfg;
        if self.state.altitude < c.min_altitude
            || (self.state.altitude - self.target_altitude).abs() > c.max_altitude_error
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
        self.termination_reason().is_some()
    }

    fn action_to_inputs(action: &[f32]) -> ControlInputs {
        direct_action_to_inputs(action)
    }
}

impl TrainingEnv for HeadingHoldEnv {
    fn offset_rng_seed(&mut self, offset: u64) {
        self.rng_seed = self.rng_seed.wrapping_add(offset);
        self.rng = Lcg::new(self.rng_seed);
    }

    fn reset(&mut self) -> (Observation, SpawnSpec) {
        self.rng_seed = self.rng_seed.wrapping_add(1);
        self.rng = Lcg::new(self.rng_seed);

        self.target_heading = self.rng.next_f32(
            *self.target_heading_range.start(),
            *self.target_heading_range.end(),
        );
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
        let spawn_alt = (self.target_altitude + alt_offset)
            .max(self.reward_cfg.min_altitude + MIN_SPAWN_ALTITUDE_MARGIN);
        let spawn_spd = (self.target_airspeed + spd_offset)
            .max(self.reward_cfg.min_airspeed + MIN_SPAWN_AIRSPEED_MARGIN);

        let droll = self.rng.next_f32(-ROLL_RANGE, ROLL_RANGE);
        let dpitch = self.rng.next_f32(-PITCH_RANGE, PITCH_RANGE);
        let dp = self.rng.next_f32(-ANG_VEL_RANGE, ANG_VEL_RANGE);
        let dq = self.rng.next_f32(-ANG_VEL_RANGE, ANG_VEL_RANGE);
        let dr = self.rng.next_f32(-ANG_VEL_RANGE, ANG_VEL_RANGE);
        let dvv = self.rng.next_f32(-VVEL_RANGE, VVEL_RANGE);
        let fuel_fraction = self.rng.next_f32(0.2, 1.0);

        // Spawn along world +X (ground track = 0) — see module doc for why
        // this is fully general given the sim's rotational symmetry.
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
        let termination = self.termination_reason();
        let mut reward = self.compute_reward();
        if termination == Some(TerminationReason::Failure) {
            reward += self.reward_cfg.terminal_failure_penalty;
        }
        let info = StepInfo {
            episode_step: self.episode_step,
            ..Default::default()
        };

        StepOutcome {
            obs,
            reward,
            end: termination,
            info,
        }
    }

    fn observation_dim(&self) -> usize {
        HEADING_HOLD_OBS_DIM
    }
    fn action_dim(&self) -> usize {
        4
    }
}

impl DemonstrationEnv for HeadingHoldEnv {
    fn current_state(&self) -> FlightState {
        self.state.clone()
    }

    fn dt(&self) -> f32 {
        self.dt
    }

    fn make_expert(&self) -> Box<dyn FlightController> {
        // `HeadingHoldController::new` seeds its inner LevelHoldController
        // from the *spawn* state (altitude/airspeed), but `reset()`
        // deliberately spawns at target ± an offset. Re-apply the episode's
        // actual targets via `apply_targets` so the expert demonstrates
        // holding the right altitude/airspeed/heading, not the spawn ones —
        // otherwise BC would teach the policy to settle at a nonzero
        // alt_err/speed_err matching what the expert actually held.
        let mut ctrl = HeadingHoldController::new(&self.state, self.target_heading);
        ctrl.apply_targets(
            &ControllerTargets::HeadingHold {
                heading: self.target_heading,
                altitude: self.target_altitude,
                airspeed: self.target_airspeed,
            },
            &self.state,
        );
        Box::new(ctrl)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// The self-contained Euler integrator must step at the same rate as the live
    /// Rapier sim. When it did not (60 Hz here vs 64 Hz live), `evaluate_policy`
    /// numbers stopped predicting live behavior and nothing caught it.
    #[test]
    fn env_dt_is_the_shared_physics_dt() {
        let env = HeadingHoldEnv::new(0.0, 1000.0, 120.0, jet_cfg());
        assert_eq!(env.dt, PHYSICS_DT);
    }

    /// Angle-of-attack margin guard: past `alpha_soft_limit` the reward must fall
    /// off linearly, the same shape as the bank-excess guard. Without it nothing in
    /// the reward keeps a policy off `cl_max` — at the low-speed/high-altitude corner
    /// of the envelope (90 m/s at 5000 m, where level flight already needs CL ≈ 1.15
    /// of the generic jet's 1.4 max) a policy that keeps pulling for altitude or turn
    /// rate rides α past the stall and departs, which is exactly what a live-sim
    /// rollout of the pre-guard checkpoints did.
    #[test]
    fn alpha_excess_beyond_soft_limit_reduces_reward() {
        let mut reward_cfg = HeadingHoldRewardConfig::default();
        reward_cfg.alpha_soft_limit = 0.26;
        reward_cfg.alpha_excess_scale = 0.1;
        reward_cfg.alpha_excess_weight = 1.0;
        let mut env = HeadingHoldEnv::with_reward_config(0.0, 1000.0, 120.0, jet_cfg(), reward_cfg);
        env.reset();

        env.state.alpha = 0.26;
        let at_limit = env.compute_reward();
        env.state.alpha = 0.36; // 0.1 rad past the limit = one full `scale`
        let past_limit = env.compute_reward();

        assert!(
            (at_limit - past_limit - 1.0).abs() < 1e-4,
            "0.1 rad past a 0.1-rad scale at weight 1.0 must cost exactly 1.0 \
             (at_limit {at_limit}, past_limit {past_limit})"
        );
    }

    /// Below the soft limit the guard must be completely inert — ordinary slow flight
    /// at altitude legitimately sits at a high-ish α and must not be penalized for it.
    #[test]
    fn alpha_below_soft_limit_is_not_penalized() {
        let mut reward_cfg = HeadingHoldRewardConfig::default();
        reward_cfg.alpha_soft_limit = 0.26;
        reward_cfg.alpha_excess_scale = 0.1;
        reward_cfg.alpha_excess_weight = 1.0;
        let mut env = HeadingHoldEnv::with_reward_config(0.0, 1000.0, 120.0, jet_cfg(), reward_cfg);
        env.reset();

        env.state.alpha = 0.10;
        let low = env.compute_reward();
        env.state.alpha = 0.26;
        let at_limit = env.compute_reward();

        assert!(
            (low - at_limit).abs() < 1e-6,
            "α below the soft limit must cost nothing (low {low}, at_limit {at_limit})"
        );
    }

    /// The alternate constructors route through `new()` today, but nothing pinned
    /// that they preserve dt — and `with_reward_config` is what every training run
    /// actually calls, so a regression there would ship silently.
    #[test]
    fn env_dt_survives_alternate_constructors() {
        let with_reward = HeadingHoldEnv::with_reward_config(
            0.0,
            1000.0,
            120.0,
            jet_cfg(),
            HeadingHoldRewardConfig::default(),
        );
        assert_eq!(with_reward.dt, PHYSICS_DT);

        let with_ranges = HeadingHoldEnv::with_target_ranges(
            -std::f32::consts::PI..=std::f32::consts::PI,
            500.0..=5000.0,
            DEFAULT_TARGET_AIRSPEED_MIN..=DEFAULT_TARGET_AIRSPEED_MAX,
            jet_cfg(),
            HeadingHoldRewardConfig::default(),
        );
        assert_eq!(with_ranges.dt, PHYSICS_DT);
    }

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
            cl_beta: 0.08,
            cl_p: -0.45,
            cl_r: -0.12,
            cl_delta_a: 0.18,
            cn_beta: 0.10,
            cn_r: -0.12,
            cn_delta_r: -0.10,
            cy_beta: -0.80,
            cy_p: 0.05,
            cy_r: 0.25,
            cy_delta_r: 0.18,
            thrust_max: 60000.0,
            powerplant: Default::default(),
            aileron_limit: 0.4363,
            elevator_limit: 0.3491,
            rudder_limit: 0.2618,
        }
    }

    #[test]
    fn dimensions_are_correct() {
        let env = HeadingHoldEnv::new(0.0, 1000.0, 120.0, jet_cfg());
        assert_eq!(env.observation_dim(), 16);
        assert_eq!(env.action_dim(), 4);
    }

    #[test]
    fn reset_returns_correct_obs_length() {
        let mut env = HeadingHoldEnv::new(0.0, 1000.0, 120.0, jet_cfg());
        let (obs, _) = env.reset();
        assert_eq!(obs.len(), 16);
    }

    #[test]
    fn step_returns_correct_obs_length() {
        let mut env = HeadingHoldEnv::new(0.0, 1000.0, 120.0, jet_cfg());
        env.reset();
        let obs = env.step(&[0.0, 0.0, 0.0, 0.0]).obs;
        assert_eq!(obs.len(), 16);
    }

    #[test]
    fn obs_values_are_finite() {
        let mut env = HeadingHoldEnv::new(0.0, 1000.0, 120.0, jet_cfg());
        env.reset();
        for _ in 0..60 {
            let out = env.step(&[0.0, 0.0, 0.3, 0.0]);
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
    fn heading_obs_elements_match_sin_cos_of_error() {
        let mut state = FlightState {
            altitude: 1000.0,
            airspeed: 120.0,
            velocity: Vec3::new(120.0, 0.0, 0.0), // ground track = 0
            attitude: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            ..Default::default()
        };
        state.position.y = 1000.0;
        let target_heading = 1.0_f32; // 1 rad
        let obs = heading_hold_observation(&state, target_heading, 1000.0, 120.0);
        assert_eq!(obs.len(), 16);
        let e = heading_error(&state, target_heading);
        assert!((obs[13] - e.sin() / HEADING_ERROR_OBS_SCALE).abs() < 1e-6);
        assert!((obs[14] - e.cos()).abs() < 1e-6);
    }

    #[test]
    fn episode_terminates_on_ground() {
        let mut env = HeadingHoldEnv::new(0.0, 50.0, 120.0, jet_cfg());
        env.alt_spawn_offset_range = -30.0..=-30.0;
        env.reset();
        let mut done = false;
        for _ in 0..600 {
            let d = env.step(&[-1.0, -1.0, 0.0, 0.0]).done();
            if d {
                done = true;
                break;
            }
        }
        assert!(done, "episode should have terminated near the ground");
    }

    #[test]
    fn failure_termination_applies_penalty() {
        let mut env = HeadingHoldEnv::new(0.0, 50.0, 120.0, jet_cfg());
        env.alt_spawn_offset_range = -30.0..=-30.0;
        env.reset();
        let mut saw_penalized_reward = false;
        for _ in 0..600 {
            let out = env.step(&[-1.0, -1.0, 0.0, 0.0]);
            let (reward, d) = (out.reward, out.done());
            if d {
                assert!(env.is_done());
                // Penalty (-50) should dominate the per-step reward.
                assert!(
                    reward < -10.0,
                    "expected failure-penalized reward, got {reward}"
                );
                saw_penalized_reward = true;
                break;
            }
        }
        assert!(saw_penalized_reward, "episode never terminated");
    }

    #[test]
    fn timeout_does_not_apply_failure_penalty() {
        let mut env = HeadingHoldEnv::new(0.0, 1000.0, 120.0, jet_cfg());
        env.max_episode_steps = 2;
        env.reset();
        env.step(&[0.0, 0.0, 0.0, 0.0]);
        let out = env.step(&[0.0, 0.0, 0.0, 0.0]);
        let (reward, done) = (out.reward, out.done());
        assert!(done);
        // A near-level, near-target step should be a small reward, not a
        // failure-penalized one.
        assert!(
            reward > -5.0,
            "timeout should not incur the failure penalty, got {reward}"
        );
    }

    #[test]
    fn fixed_target_constructor_pins_targets() {
        let mut env = HeadingHoldEnv::new(0.5, 1000.0, 120.0, jet_cfg());
        for _ in 0..20 {
            env.reset();
            assert_eq!(env.target_heading, 0.5);
            assert_eq!(env.target_altitude, 1000.0);
            assert_eq!(env.target_airspeed, 120.0);
        }
    }

    /// heading_hold no longer carries a tighter airspeed floor than level_hold. The
    /// bank-authority squeeze at the 5000 m / 90 m/s / full-fuel corner is a known and
    /// accepted cost — see `DEFAULT_TARGET_AIRSPEED_MIN`'s doc. Asserted against
    /// level_hold's constants rather than a literal so this states the *intent* and
    /// survives a future edit to either env.
    #[test]
    fn default_airspeed_envelope_matches_level_hold() {
        use crate::training::level_hold_env;
        assert_eq!(
            DEFAULT_TARGET_AIRSPEED_MIN,
            level_hold_env::DEFAULT_TARGET_AIRSPEED_MIN
        );
        assert_eq!(
            DEFAULT_TARGET_AIRSPEED_MAX,
            level_hold_env::DEFAULT_TARGET_AIRSPEED_MAX
        );
    }

    /// The behavioral half: the default envelope must actually reach the sub-100 m/s
    /// operating points that were out-of-distribution before. Deterministic — `Lcg`
    /// with the fixed seed 42 set in `new()`.
    #[test]
    fn default_envelope_samples_below_the_old_110_floor() {
        let mut env = HeadingHoldEnv::with_target_ranges(
            DEFAULT_TARGET_HEADING_DEG_MIN.to_radians()
                ..=DEFAULT_TARGET_HEADING_DEG_MAX.to_radians(),
            crate::training::level_hold_env::DEFAULT_TARGET_ALT_MIN
                ..=crate::training::level_hold_env::DEFAULT_TARGET_ALT_MAX,
            DEFAULT_TARGET_AIRSPEED_MIN..=DEFAULT_TARGET_AIRSPEED_MAX,
            jet_cfg(),
            HeadingHoldRewardConfig::default(),
        );
        let mut saw_slow = false;
        for _ in 0..200 {
            env.reset();
            saw_slow |= env.target_airspeed < 100.0;
        }
        assert!(
            saw_slow,
            "default envelope never sampled below 100 m/s — the operating point that \
             measured 1.47 m altitude error live is still out of distribution"
        );
    }

    #[test]
    fn reset_samples_targets_within_range() {
        let mut env = HeadingHoldEnv::with_target_ranges(
            -std::f32::consts::PI..=std::f32::consts::PI,
            500.0..=5000.0,
            DEFAULT_TARGET_AIRSPEED_MIN..=DEFAULT_TARGET_AIRSPEED_MAX,
            jet_cfg(),
            HeadingHoldRewardConfig::default(),
        );
        for _ in 0..200 {
            env.reset();
            assert!(
                (-std::f32::consts::PI..=std::f32::consts::PI).contains(&env.target_heading),
                "target_heading {} out of range",
                env.target_heading
            );
            assert!(
                (500.0..=5000.0).contains(&env.target_altitude),
                "target_altitude {} out of range",
                env.target_altitude
            );
            assert!(
                (DEFAULT_TARGET_AIRSPEED_MIN..=DEFAULT_TARGET_AIRSPEED_MAX)
                    .contains(&env.target_airspeed),
                "target_airspeed {} out of range",
                env.target_airspeed
            );
        }
    }

    #[test]
    fn reset_varies_targets_across_episodes() {
        let mut env = HeadingHoldEnv::with_target_ranges(
            -std::f32::consts::PI..=std::f32::consts::PI,
            500.0..=5000.0,
            DEFAULT_TARGET_AIRSPEED_MIN..=DEFAULT_TARGET_AIRSPEED_MAX,
            jet_cfg(),
            HeadingHoldRewardConfig::default(),
        );
        let mut seen = std::collections::HashSet::new();
        for _ in 0..20 {
            env.reset();
            seen.insert(env.target_heading.to_bits());
        }
        assert!(
            seen.len() >= 2,
            "expected varied target headings across episodes, got {} distinct",
            seen.len()
        );
    }

    #[test]
    fn spawn_ground_track_is_zero() {
        // Spawns along +X per module doc; the target heading IS the required change.
        let mut env = HeadingHoldEnv::new(1.2, 1000.0, 120.0, jet_cfg());
        env.reset();
        let state = env.current_state();
        assert!(state.velocity.z.abs() < 1e-5, "vz={}", state.velocity.z);
        assert!(state.velocity.x > 0.0);
    }

    #[test]
    fn spawn_altitude_is_clamped() {
        let reward_cfg = HeadingHoldRewardConfig::default();
        let min_alt = reward_cfg.min_altitude;
        let mut env = HeadingHoldEnv::with_target_ranges(
            0.0..=0.0,
            20.0..=20.0,
            120.0..=120.0,
            jet_cfg(),
            reward_cfg,
        );
        env.alt_spawn_offset_range = -1000.0..=-1000.0;
        env.reset();
        assert!(
            env.current_state().altitude >= min_alt,
            "altitude {} should be clamped above min_altitude {}",
            env.current_state().altitude,
            min_alt
        );
    }

    /// A hard turn at the envelope floor bleeds speed fast, and `min_airspeed` is an
    /// *instant* episode failure — so an episode that spawns already inside the bleed
    /// budget is unlearnable, not merely hard. The spawn clamp must therefore be
    /// expressed as a margin over the termination floor, not as a bare constant that
    /// silently stops being a margin when the target envelope moves down.
    #[test]
    fn spawn_airspeed_keeps_margin_over_termination_floor() {
        let reward_cfg = HeadingHoldRewardConfig::default();
        let min_air = reward_cfg.min_airspeed;
        let mut env = HeadingHoldEnv::with_target_ranges(
            0.0..=0.0,
            1000.0..=1000.0,
            // The floor `DEFAULT_TARGET_AIRSPEED_MIN` is about to become (commit 4).
            90.0..=90.0,
            jet_cfg(),
            reward_cfg,
        );
        for _ in 0..200 {
            env.reset();
            let spawn = env.current_state().airspeed;
            assert!(
                spawn >= min_air + MIN_SPAWN_AIRSPEED_MARGIN,
                "spawn {spawn} m/s leaves under {MIN_SPAWN_AIRSPEED_MARGIN} m/s \
                 over the min_airspeed {min_air} instant-fail floor"
            );
        }
    }

    #[test]
    fn spawn_airspeed_is_clamped() {
        let mut env = HeadingHoldEnv::with_target_ranges(
            0.0..=0.0,
            1000.0..=1000.0,
            DEFAULT_TARGET_AIRSPEED_MIN..=DEFAULT_TARGET_AIRSPEED_MIN,
            jet_cfg(),
            HeadingHoldRewardConfig::default(),
        );
        env.airspeed_spawn_offset_range = -1000.0..=-1000.0;
        env.reset();
        let floor = HeadingHoldRewardConfig::default().min_airspeed + MIN_SPAWN_AIRSPEED_MARGIN;
        assert!(
            env.current_state().airspeed >= floor,
            "airspeed {} should be clamped above {}",
            env.current_state().airspeed,
            floor
        );
    }

    #[test]
    fn throttle_remapping_is_correct() {
        let inputs = HeadingHoldEnv::action_to_inputs(&[0.0, -1.0, 0.0, 0.0]);
        assert!(
            (inputs.throttle - 0.0).abs() < 1e-5,
            "throttle={}",
            inputs.throttle
        );
        let inputs = HeadingHoldEnv::action_to_inputs(&[0.0, 1.0, 0.0, 0.0]);
        assert!(
            (inputs.throttle - 1.0).abs() < 1e-5,
            "throttle={}",
            inputs.throttle
        );
    }

    #[test]
    fn expert_targets_track_resampled_episode_targets() {
        // Regression guard for the `make_expert` gotcha documented above: the
        // expert's setpoints must match the *episode's* targets, not the
        // spawn state, even though reset() spawns at target ± an offset.
        let mut env = HeadingHoldEnv::with_target_ranges(
            -std::f32::consts::PI..=std::f32::consts::PI,
            500.0..=5000.0,
            DEFAULT_TARGET_AIRSPEED_MIN..=DEFAULT_TARGET_AIRSPEED_MAX,
            jet_cfg(),
            HeadingHoldRewardConfig::default(),
        );
        for _ in 0..10 {
            env.reset();
            let expert = env.make_expert();
            match expert.targets() {
                ControllerTargets::HeadingHold {
                    heading,
                    altitude,
                    airspeed,
                } => {
                    assert!(
                        (heading - env.target_heading).abs() < 1e-5,
                        "expert heading {heading} != episode target {}",
                        env.target_heading
                    );
                    assert!(
                        (altitude - env.target_altitude).abs() < 1e-5,
                        "expert altitude {altitude} != episode target {}",
                        env.target_altitude
                    );
                    assert!(
                        (airspeed - env.target_airspeed).abs() < 1e-5,
                        "expert airspeed {airspeed} != episode target {}",
                        env.target_airspeed
                    );
                }
                other => panic!("expected ControllerTargets::HeadingHold, got {other:?}"),
            }
        }
    }
}
