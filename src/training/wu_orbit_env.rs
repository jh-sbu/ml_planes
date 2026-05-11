//! Wu et al. (2025) orbit training environment.
//!
//! Identical spawn / termination logic to `OrbitEnv`; replaces the weighted
//! linear reward with Wu et al.'s multiplicative Gaussian formulation
//! (R^TT × R^PS × R^RS) with an automatic 3-stage curriculum.

use bevy::math::{Mat3, Quat, Vec3};

use crate::controllers::orbit::{
    build_orbit_observation, build_orbit_observation_from_terms, orbit_observation_terms,
    OrbitDirection, OrbitObservationTerms, ORBIT_OBS_DIM,
};
use crate::plane::{FlightState, PlaneConfig};
use crate::training::flight_env::{
    direct_action_to_inputs, integrate_state, pitch_angle, roll_angle, Lcg,
};
use crate::training::reward_config::load_reward_config;
use crate::training::wu_orbit_reward::{r_ps, r_rs, r_tt, CurriculumStage, WuOrbitRewardConfig};
use crate::training::{CurriculumEnv, Observation, SpawnSpec, StepInfo, TrainingEnv};

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
pub struct WuOrbitEnv {
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

    reward_cfg: WuOrbitRewardConfig,
    /// Active heading bandwidth — switches between coarse and fine per curriculum.
    b_heading_active: f32,
    pub curriculum_stage: CurriculumStage,

    cfg: PlaneConfig,
    dt: f32,
    state: FlightState,
    direction: OrbitDirection,
    episode_step: u32,
    /// Previous altitude error used for vertical speed approximation.
    prev_alt_err: f32,
    rng: Lcg,
    rng_seed: u64,
}

impl WuOrbitEnv {
    pub fn new(
        target_altitude: f32,
        target_airspeed: f32,
        target_radius: f32,
        cfg: PlaneConfig,
    ) -> Self {
        let reward_cfg = WuOrbitRewardConfig::default();
        Self::with_reward_config(
            target_altitude,
            target_airspeed,
            target_radius,
            cfg,
            reward_cfg,
        )
    }

    pub fn with_reward_config(
        target_altitude: f32,
        target_airspeed: f32,
        target_radius: f32,
        cfg: PlaneConfig,
        reward_cfg: WuOrbitRewardConfig,
    ) -> Self {
        let b_heading_active = reward_cfg.b_heading_coarse;
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
            target_radius_range: 2500.0..=4000.0,
            b_heading_active,
            curriculum_stage: CurriculumStage::Coarse,
            reward_cfg,
            cfg,
            dt: 1.0 / 60.0,
            state: FlightState::default(),
            direction: OrbitDirection::CounterClockwise,
            episode_step: 0,
            prev_alt_err: 0.0,
            rng: Lcg::new(4242),
            rng_seed: 4242,
        }
    }

    /// Load reward config from file, falling back to defaults on error.
    pub fn load_reward_config_or_default(path: &str) -> WuOrbitRewardConfig {
        load_reward_config::<WuOrbitRewardConfig>(path).unwrap_or_else(|e| {
            eprintln!("wu_orbit reward config load failed ({e}); using defaults");
            WuOrbitRewardConfig::default()
        })
    }

    /// Advance curriculum to the next stage.  Idempotent at `Full`.
    pub fn advance_curriculum(&mut self) {
        self.curriculum_stage = match self.curriculum_stage {
            CurriculumStage::Coarse => {
                self.b_heading_active = self.reward_cfg.b_heading_fine;
                CurriculumStage::HeadingFine
            }
            CurriculumStage::HeadingFine => CurriculumStage::Full,
            CurriculumStage::Full => CurriculumStage::Full,
        };
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

    fn compute_reward(&self, terms: &OrbitObservationTerms) -> f32 {
        let cfg = &self.reward_cfg;
        let alt_err = self.state.altitude - self.target_altitude;
        let speed_err = self.state.airspeed - self.target_airspeed;
        let roll = roll_angle(self.state.attitude);
        let roll_vs_ff = roll - terms.bank_ff;
        let pitch = pitch_angle(self.state.attitude);
        let alt_dot = self.state.velocity.y;
        let roll_rate = self.state.angular_velocity.x;

        let rtt = r_tt(
            terms.radial_error,
            terms.heading_error,
            alt_err,
            speed_err,
            roll_vs_ff,
            cfg,
            self.b_heading_active,
        );

        match self.curriculum_stage {
            CurriculumStage::Coarse | CurriculumStage::HeadingFine => rtt,
            CurriculumStage::Full => {
                rtt * r_ps(pitch, alt_err, alt_dot, cfg) * r_rs(terms.heading_error, roll_rate, cfg)
            }
        }
    }

    fn termination_reason(&self, terms: &OrbitObservationTerms) -> Option<TerminationReason> {
        let c = &self.reward_cfg;
        if self.state.altitude < c.min_altitude
            || (self.state.altitude - self.target_altitude).abs() > c.max_altitude_error
            || terms.radial_error.abs() > c.max_radial_error
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

impl TrainingEnv for WuOrbitEnv {
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
        let (radial_x, radial_z) = (cos_a, sin_a);

        let (tang_x, tang_z) = match self.direction {
            OrbitDirection::CounterClockwise => (-radial_z, radial_x),
            OrbitDirection::Clockwise => (radial_z, -radial_x),
        };
        let hp = self
            .rng
            .next_f32(-HEADING_PERTURB_RANGE, HEADING_PERTURB_RANGE);
        let (sin_h, cos_h) = hp.sin_cos();
        let (head_x, head_z) = (
            cos_h * tang_x - sin_h * tang_z,
            sin_h * tang_x + cos_h * tang_z,
        );

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
        self.prev_alt_err = self.state.altitude - self.target_altitude;

        let obs = build_orbit_observation(
            &self.state,
            self.center_x,
            self.center_z,
            self.target_radius,
            self.target_altitude,
            self.target_airspeed,
            self.direction,
        );
        let spawn_spec = SpawnSpec {
            position: Some(self.state.position),
            velocity: Some(self.state.velocity),
            attitude: Some(attitude),
            angular_velocity: Some(angular_velocity),
        };
        (obs, spawn_spec)
    }

    fn step(&mut self, action: &[f32]) -> (Observation, f32, bool, StepInfo) {
        let inputs = direct_action_to_inputs(action);
        integrate_state(&mut self.state, &inputs, &self.cfg, self.dt);
        self.episode_step += 1;

        let terms = self.current_terms();
        let obs = build_orbit_observation_from_terms(
            &self.state,
            self.target_altitude,
            self.target_airspeed,
            &terms,
        );
        let termination = self.termination_reason(&terms);
        let mut reward = self.compute_reward(&terms);
        if termination == Some(TerminationReason::Failure) {
            reward += self.reward_cfg.terminal_failure_penalty;
        }
        self.prev_alt_err = self.state.altitude - self.target_altitude;
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

impl CurriculumEnv for WuOrbitEnv {
    fn advance_curriculum(&mut self) {
        self.advance_curriculum();
    }

    fn curriculum_stage_name(&self) -> &'static str {
        self.curriculum_stage.name()
    }

    fn next_stage_threshold(&self) -> f32 {
        match self.curriculum_stage {
            CurriculumStage::Coarse => self.reward_cfg.stage2_threshold,
            CurriculumStage::HeadingFine => self.reward_cfg.stage3_threshold,
            CurriculumStage::Full => f32::INFINITY,
        }
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

    fn on_orbit_state(radius: f32, heading: Vec3) -> FlightState {
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
    fn wu_orbit_dimensions() {
        let env = WuOrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        assert_eq!(env.observation_dim(), ORBIT_OBS_DIM);
        assert_eq!(env.action_dim(), 4);
    }

    #[test]
    fn wu_orbit_reset_and_step_finite() {
        let mut env = WuOrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        let (obs, _) = env.reset();
        assert_eq!(obs.len(), ORBIT_OBS_DIM);
        assert!(obs.iter().all(|v| v.is_finite()), "reset obs: {obs:?}");

        let (obs, reward, _done, info) = env.step(&[0.0, 0.0, 0.0, 0.0]);
        assert!(obs.iter().all(|v| v.is_finite()), "step obs: {obs:?}");
        assert!(reward.is_finite(), "reward not finite: {reward}");
        assert_eq!(info.episode_step, 1);
    }

    #[test]
    fn wu_orbit_reward_in_unit_range_on_orbit() {
        let mut env = WuOrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        env.direction = OrbitDirection::CounterClockwise;
        env.state = on_orbit_state(1000.0, Vec3::X);
        env.prev_alt_err = env.state.altitude - env.target_altitude;
        let terms = env.current_terms();
        let r = env.compute_reward(&terms);
        assert!(
            r >= 0.0 && r <= 1.0 + 1e-5,
            "reward outside [0,1] on orbit: {r}"
        );
    }

    #[test]
    fn wu_orbit_reward_prefers_lower_error() {
        let mut good = WuOrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        good.direction = OrbitDirection::CounterClockwise;
        good.state = on_orbit_state(1000.0, Vec3::X);
        good.prev_alt_err = 0.0;
        let good_terms = good.current_terms();
        let r_good = good.compute_reward(&good_terms);

        let mut bad = good.clone();
        bad.state = on_orbit_state(1400.0, Vec3::Z);
        let bad_terms = bad.current_terms();
        let r_bad = bad.compute_reward(&bad_terms);

        assert!(r_good > r_bad, "good={r_good} bad={r_bad}");
    }

    #[test]
    fn terminal_conditions_trigger() {
        let mut env = WuOrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        env.state = on_orbit_state(1000.0, Vec3::X);

        env.state.position.y = 5.0;
        env.state.altitude = 5.0;
        let terms = env.current_terms();
        assert!(
            env.termination_reason(&terms).is_some(),
            "low altitude should terminate"
        );

        env.state = on_orbit_state(1000.0, Vec3::X);
        env.episode_step = env.max_episode_steps;
        let terms = env.current_terms();
        assert!(
            env.termination_reason(&terms).is_some(),
            "max steps should terminate"
        );
    }

    #[test]
    fn failure_penalty_added_on_terminal_step() {
        let mut env = WuOrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        env.state = on_orbit_state(1000.0, Vec3::X);
        env.state.position.y = 5.0;
        env.state.altitude = 5.0;

        let (_obs, reward, done, _) = env.step(&[0.0, 0.0, 0.0, 0.0]);
        assert!(done, "should be done");
        assert!(
            reward < 0.0,
            "terminal failure reward should include penalty: {reward}"
        );
    }

    #[test]
    fn curriculum_advances_heading_band() {
        let mut env = WuOrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        assert_eq!(env.curriculum_stage, CurriculumStage::Coarse);
        let b_before = env.b_heading_active;

        env.advance_curriculum();
        assert_eq!(env.curriculum_stage, CurriculumStage::HeadingFine);
        assert!(
            env.b_heading_active < b_before,
            "heading band should narrow: {} < {}",
            env.b_heading_active,
            b_before
        );

        env.advance_curriculum();
        assert_eq!(env.curriculum_stage, CurriculumStage::Full);

        // Idempotent at Full.
        env.advance_curriculum();
        assert_eq!(env.curriculum_stage, CurriculumStage::Full);
    }

    #[test]
    fn stage3_reward_uses_smoothing_constraints() {
        let mut env = WuOrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        env.direction = OrbitDirection::CounterClockwise;
        env.state = on_orbit_state(1000.0, Vec3::X);
        env.prev_alt_err = 0.0;
        // Inject large pitch so R^PS < 1, which should reduce total reward in stage 3.
        env.state.attitude = {
            let mut a = env.state.attitude;
            // rotate 30° about forward axis ≈ large pitch
            a = Quat::from_rotation_z(0.5) * a;
            a
        };

        let terms = env.current_terms();

        // Stage 2: no smoothing constraints applied.
        env.advance_curriculum(); // → HeadingFine
        let r2 = env.compute_reward(&terms);

        // Stage 3: full reward.
        env.advance_curriculum(); // → Full
        let r3 = env.compute_reward(&terms);

        // r3 ≤ r2 because R^PS×R^RS are additional multiplicative factors ≤ 1.
        assert!(
            r3 <= r2 + 1e-5,
            "stage 3 reward should be ≤ stage 2 reward: r3={r3} r2={r2}"
        );
    }

    #[test]
    fn wu_orbit_ron_parses() {
        let src = r#"(
            b_radial: 500.0,
            b_heading_coarse: 0.034906585,
            b_heading_fine: 0.008726646,
            b_altitude: 30.0,
            b_speed: 10.0,
            b_roll_vs_ff: 0.34906585,
            pitch_target_denom: 0.17453293,
            roll_rate_target_denom: 0.17453293,
            min_altitude: 200.0,
            max_altitude_error: 500.0,
            max_radial_error: 1500.0,
            min_airspeed: 30.0,
            max_episode_steps: 3600,
            terminal_failure_penalty: -50.0,
            stage2_threshold: 0.3,
            stage3_threshold: 0.55,
        )"#;
        let cfg: WuOrbitRewardConfig = ron::de::from_str(src).expect("RON parse failed");
        assert_eq!(cfg.max_episode_steps, 3600);
        assert!((cfg.b_altitude - 30.0).abs() < 1e-5);
    }
}
