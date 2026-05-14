//! Wu et al. (2025) multiplicative Gaussian reward for orbit control.
//!
//! Reference: aerospace-12-00670-v2.pdf
//!
//! Three-component multiplicative reward:
//!   R = R^TT × R^PS × R^RS   (stage 3 only; stages 1–2 use R^TT alone)
//!
//! Three-stage curriculum:
//!   Stage 1 (Coarse):      R^TT with wide heading band (b_heading_coarse)
//!   Stage 2 (HeadingFine): R^TT with tight heading band (b_heading_fine)
//!   Stage 3 (Full):        R^TT × R^PS × R^RS

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Curriculum stage
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurriculumStage {
    Coarse,
    HeadingFine,
    Full,
}

impl CurriculumStage {
    pub fn name(self) -> &'static str {
        match self {
            CurriculumStage::Coarse => "Coarse",
            CurriculumStage::HeadingFine => "HeadingFine",
            CurriculumStage::Full => "Full",
        }
    }
}

// ---------------------------------------------------------------------------
// Reward config
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WuOrbitRewardConfig {
    // --- Gaussian bandwidth parameters ---
    /// Radial error bandwidth [m].
    pub b_radial: f32,
    /// Heading error bandwidth for stage 1 [rad] ≈ 2°.
    pub b_heading_coarse: f32,
    /// Heading error bandwidth for stages 2–3 [rad] ≈ 0.5°.
    pub b_heading_fine: f32,
    /// Altitude error bandwidth [m].
    pub b_altitude: f32,
    /// Speed error bandwidth [m/s].
    pub b_speed: f32,
    /// Roll-vs-feedforward bandwidth [rad] ≈ 20°.
    pub b_roll_vs_ff: f32,

    // --- Pitch smoothing (R^PS) ---
    /// Denominator of R^PS Gaussian [rad] ≈ 10°.
    pub pitch_target_denom: f32,

    // --- Roll smoothing (R^RS) ---
    /// Denominator of R^RS Gaussian [rad/s] ≈ 10°/s.
    pub roll_rate_target_denom: f32,

    // --- Termination ---
    pub min_altitude: f32,
    pub max_altitude_error: f32,
    pub max_radial_error: f32,
    pub min_airspeed: f32,
    pub max_episode_steps: u32,
    pub terminal_failure_penalty: f32,

    // --- Curriculum thresholds (mean episode return) ---
    /// Advance Coarse → HeadingFine when mean_return exceeds this.
    pub stage2_threshold: f32,
    /// Advance HeadingFine → Full when mean_return exceeds this.
    pub stage3_threshold: f32,
}

impl Default for WuOrbitRewardConfig {
    fn default() -> Self {
        Self {
            b_radial: 500.0,
            b_heading_coarse: 0.034906585, // 2° in rad
            b_heading_fine: 0.008726646,   // 0.5° in rad
            b_altitude: 30.0,
            b_speed: 10.0,
            b_roll_vs_ff: 0.34906585,           // 20° in rad
            pitch_target_denom: 0.17453293,     // 10° in rad
            roll_rate_target_denom: 0.17453293, // 10°/s in rad/s
            min_altitude: 200.0,
            max_altitude_error: 500.0,
            max_radial_error: 1500.0,
            min_airspeed: 30.0,
            max_episode_steps: 3_600,
            terminal_failure_penalty: -50.0,
            stage2_threshold: 0.3,
            stage3_threshold: 0.55,
        }
    }
}

// ---------------------------------------------------------------------------
// Reward computation helpers (Wu et al. formulae, translated to radians)
// ---------------------------------------------------------------------------

/// R^TT: trajectory-tracking multiplicative Gaussian.
///
/// `b_heading` is passed explicitly so the caller can switch coarse/fine.
pub fn r_tt(
    radial_err: f32,
    heading_err: f32,
    alt_err: f32,
    speed_err: f32,
    roll_vs_ff: f32, // φ − φ_ff
    cfg: &WuOrbitRewardConfig,
    b_heading: f32,
) -> f32 {
    gauss(radial_err, cfg.b_radial)
        * gauss(heading_err, b_heading)
        * gauss(alt_err, cfg.b_altitude)
        * gauss(speed_err, cfg.b_speed)
        * gauss(roll_vs_ff, cfg.b_roll_vs_ff)
}

/// R^PS: pitch-smoothing constraint (Wu eq. 7).
///
/// `pitch` is the current pitch angle [rad].
/// `alt_err` is (altitude − target_altitude) [m].
/// `alt_dot` is vertical speed [m/s].
pub fn r_ps(pitch: f32, alt_err: f32, alt_dot: f32, cfg: &WuOrbitRewardConfig) -> f32 {
    // Wu eq. 7 uses Δh = target − altitude (positive when below target).
    // This codebase passes alt_err = altitude − target, so negate both inputs
    // to recover the correct sign: below target → positive θ* → nose-up.
    let log2_m_to_rad = 3.0_f32.to_radians();
    let delta_h = -alt_err;
    let delta_h_dot = -alt_dot;
    let theta_star = delta_h.signum() * log2_m_to_rad * (delta_h.abs() + 1.0).log2()
        + delta_h_dot * (1.0_f32 / 3.0_f32).to_radians();
    gauss(pitch - theta_star, cfg.pitch_target_denom)
}

/// R^RS: roll-smoothing constraint (Wu eq. 8).
///
/// `heading_err`     is orbit guidance heading error [rad].
/// `heading_err_dot` is finite-difference derivative of heading_err [rad/s].
/// `roll_rate`       is p (body-frame roll angular velocity) [rad/s].
pub fn r_rs(
    heading_err: f32,
    heading_err_dot: f32,
    roll_rate: f32,
    cfg: &WuOrbitRewardConfig,
) -> f32 {
    use std::f32::consts::FRAC_PI_3;
    let phi_tgt_star = (0.35 * heading_err + 0.1 * heading_err_dot).clamp(-FRAC_PI_3, FRAC_PI_3);
    let omega_phi_star =
        (0.15 * phi_tgt_star).clamp(-cfg.roll_rate_target_denom, cfg.roll_rate_target_denom);
    gauss(roll_rate - omega_phi_star, cfg.roll_rate_target_denom)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

#[inline]
fn gauss(x: f32, b: f32) -> f32 {
    (-(x / b).powi(2)).exp()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gauss_peaks_at_zero() {
        let cfg = WuOrbitRewardConfig::default();
        // At zero error all Gaussian terms equal 1.0 → R^TT = 1.0.
        let rtt = r_tt(0.0, 0.0, 0.0, 0.0, 0.0, &cfg, cfg.b_heading_fine);
        assert!(
            (rtt - 1.0).abs() < 1e-5,
            "R^TT at zero error should be 1.0, got {rtt}"
        );
    }

    #[test]
    fn r_tt_decreases_with_error() {
        let cfg = WuOrbitRewardConfig::default();
        let b = cfg.b_heading_fine;
        let good = r_tt(0.0, 0.0, 0.0, 0.0, 0.0, &cfg, b);
        let bad = r_tt(200.0, 0.05, 50.0, 5.0, 0.2, &cfg, b);
        assert!(good > bad, "good={good} bad={bad}");
    }

    #[test]
    fn r_tt_in_unit_range() {
        let cfg = WuOrbitRewardConfig::default();
        let b = cfg.b_heading_coarse;
        for v in [-1000.0_f32, -1.0, 0.0, 1.0, 1000.0] {
            let r = r_tt(v, 0.0, 0.0, 0.0, 0.0, &cfg, b);
            assert!(
                (0.0..=1.0).contains(&r),
                "R^TT outside [0,1] at radial={v}: {r}"
            );
        }
    }

    #[test]
    fn r_ps_below_target_wants_nose_up() {
        let cfg = WuOrbitRewardConfig::default();
        // alt_err = altitude - target = -100 (plane is below target).
        // theta_star should be positive (nose-up) so the peak reward is at positive pitch.
        let r_nose_up = r_ps(0.1, -100.0, 0.0, &cfg);
        let r_nose_down = r_ps(-0.1, -100.0, 0.0, &cfg);
        assert!(
            r_nose_up > r_nose_down,
            "below target: nose-up pitch should score higher than nose-down (got up={r_nose_up} down={r_nose_down})"
        );
    }

    #[test]
    fn r_ps_above_target_wants_nose_down() {
        let cfg = WuOrbitRewardConfig::default();
        // alt_err = +100 (plane is above target). theta_star should be negative (nose-down).
        let r_nose_down = r_ps(-0.1, 100.0, 0.0, &cfg);
        let r_nose_up = r_ps(0.1, 100.0, 0.0, &cfg);
        assert!(
            r_nose_down > r_nose_up,
            "above target: nose-down pitch should score higher (got down={r_nose_down} up={r_nose_up})"
        );
    }

    #[test]
    fn r_ps_finite_for_large_errors() {
        let cfg = WuOrbitRewardConfig::default();
        for delta_h in [-300.0_f32, 0.0, 300.0] {
            let r = r_ps(0.0, delta_h, 0.0, &cfg);
            assert!(r.is_finite(), "R^PS not finite at delta_h={delta_h}: {r}");
            assert!((0.0..=1.0).contains(&r), "R^PS outside [0,1]: {r}");
        }
    }

    #[test]
    fn r_rs_finite() {
        let cfg = WuOrbitRewardConfig::default();
        let r = r_rs(0.1, -0.02, 0.05, &cfg);
        assert!(r.is_finite(), "R^RS not finite: {r}");
        assert!((0.0..=1.0).contains(&r), "R^RS outside [0,1]: {r}");
    }

    #[test]
    fn heading_coarse_wider_than_fine() {
        let cfg = WuOrbitRewardConfig::default();
        let err = 0.01; // rad — inside coarse band, near edge of fine band
        let coarse = gauss(err, cfg.b_heading_coarse);
        let fine = gauss(err, cfg.b_heading_fine);
        assert!(
            coarse > fine,
            "coarse band should score higher for small errors: coarse={coarse} fine={fine}"
        );
    }
}
