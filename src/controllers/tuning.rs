//! Per-controller tuning parameter sets and the [`ControllerTuning`] trait.
//!
//! Each [`ControllerTuning`] implementation encodes one controller's complete
//! tunable parameter set and knows how to construct a properly-tuned controller,
//! avoiding downcasting by letting the concrete type drive construction.

use std::collections::HashMap;

use bevy::reflect::Reflect;
use serde::{Deserialize, Serialize};

use crate::plane::FlightState;

use super::level_hold::LevelHoldController;
use super::FlightController;

// ---------------------------------------------------------------------------
// Trait
// ---------------------------------------------------------------------------

/// Builds a tuned [`FlightController`] from a flight state snapshot.
///
/// Implementors encode one controller's complete set of tunable parameters.
/// The factory method captures the current state for a bumpless mode handoff.
pub trait ControllerTuning: std::fmt::Debug + Send + Sync + 'static {
    /// Construct a tuned controller, using `state` to seed targets.
    fn build(&self, state: &FlightState) -> Box<dyn FlightController>;
}

// ---------------------------------------------------------------------------
// LevelHoldTuning
// ---------------------------------------------------------------------------

/// Tunable outer-loop gains for [`LevelHoldController`].
///
/// Structural parameters (integral clamp, output limits) are fixed constants
/// that rarely require per-plane adjustment and are not exposed here.
#[derive(Debug, Clone, Serialize, Deserialize, Reflect)]
pub struct LevelHoldTuning {
    /// Altitude outer loop proportional gain.
    pub alt_kp: f32,
    /// Altitude outer loop integral gain.
    pub alt_ki: f32,
    /// Altitude outer loop derivative gain.
    pub alt_kd: f32,
    /// Alpha inner loop proportional gain.
    pub alpha_kp: f32,
    /// Alpha inner loop derivative gain.
    pub alpha_kd: f32,
    /// Airspeed loop proportional gain.
    pub spd_kp: f32,
    /// Airspeed loop integral gain.
    pub spd_ki: f32,
    /// Throttle feedforward gain (scales α_target [rad] into a throttle increment).
    pub throttle_ff_gain: f32,
}

impl Default for LevelHoldTuning {
    fn default() -> Self {
        // Values match the hardcoded defaults in LevelHoldController::new().
        Self {
            alt_kp:           0.01,
            alt_ki:           0.12,
            alt_kd:           0.04,
            alpha_kp:         1.0,
            alpha_kd:         0.5,
            spd_kp:           0.01,
            spd_ki:           0.06,
            throttle_ff_gain: 0.7,
        }
    }
}

impl ControllerTuning for LevelHoldTuning {
    fn build(&self, state: &FlightState) -> Box<dyn FlightController> {
        Box::new(LevelHoldController::with_tuning(state, self))
    }
}

// ---------------------------------------------------------------------------
// PlaneTuning asset
// ---------------------------------------------------------------------------

/// Per-plane tuning asset loaded from `<plane>.tuning.ron`.
///
/// Each controller kind has its own map of named profiles. Controllers not
/// listed here fall back to their [`Default`] tuning.
///
/// Example file:
/// ```ron
/// PlaneTuning(
///   level_hold: {
///     "normal": LevelHoldTuning(alt_kp: 0.01, alt_ki: 0.12, ...),
///     "aggressive": LevelHoldTuning(alt_kp: 0.03, ...),
///   },
/// )
/// ```
#[derive(bevy::asset::Asset, Reflect, Serialize, Deserialize, Debug, Clone, Default)]
pub struct PlaneTuning {
    /// Named tuning profiles for [`LevelHoldController`].
    #[serde(default)]
    pub level_hold: HashMap<String, LevelHoldTuning>,
}

impl PlaneTuning {
    /// Return the named level-hold profile, or `None` if not present.
    pub fn get_level_hold(&self, profile: &str) -> Option<&LevelHoldTuning> {
        self.level_hold.get(profile)
    }
}

/// Tracks which named tuning profile is selected for this plane entity.
/// Changing this component triggers `apply_controller_switch` to rebuild the controller.
#[derive(bevy::prelude::Component, Clone, Debug, PartialEq, Eq)]
pub struct SelectedTuningProfile(pub String);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plane::FlightState;

    #[test]
    fn level_hold_tuning_default_matches_controller_defaults() {
        let t = LevelHoldTuning::default();
        assert!((t.alt_kp - 0.01).abs() < 1e-6);
        assert!((t.alt_ki - 0.12).abs() < 1e-6);
        assert!((t.alt_kd - 0.04).abs() < 1e-6);
        assert!((t.alpha_kp - 1.0).abs() < 1e-6);
        assert!((t.alpha_kd - 0.5).abs() < 1e-6);
        assert!((t.spd_kp - 0.01).abs() < 1e-6);
        assert!((t.spd_ki - 0.06).abs() < 1e-6);
        assert!((t.throttle_ff_gain - 0.7).abs() < 1e-6);
    }

    #[test]
    fn plan_tuning_deserializes_from_ron() {
        let src = r#"PlaneTuning(
            level_hold: {
                "normal": LevelHoldTuning(
                    alt_kp: 0.015,
                    alt_ki: 0.12,
                    alt_kd: 0.04,
                    alpha_kp: 2.0,
                    alpha_kd: 0.764,
                    spd_kp: 0.01,
                    spd_ki: 0.10,
                    throttle_ff_gain: 0.7,
                ),
            },
        )"#;
        let pt: PlaneTuning = ron::de::from_str(src).expect("PlaneTuning should parse");
        let t = pt.get_level_hold("normal").expect("'normal' profile should exist");
        assert!((t.alt_kp - 0.015).abs() < 1e-6);
        assert!((t.alpha_kp - 2.0).abs() < 1e-6);
        assert!(pt.get_level_hold("missing").is_none());
    }

    #[test]
    fn plan_tuning_missing_controller_returns_none() {
        let pt = PlaneTuning::default();
        assert!(pt.get_level_hold("normal").is_none());
    }

    #[test]
    fn controller_tuning_build_applies_gains() {
        use std::f32::consts::FRAC_PI_2;
        use bevy::math::{Quat, Vec3};

        let tuning = LevelHoldTuning {
            alt_kp: 0.03,
            alt_ki: 0.20,
            alt_kd: 0.08,
            alpha_kp: 3.0,
            alpha_kd: 1.0,
            spd_kp: 0.02,
            spd_ki: 0.15,
            throttle_ff_gain: 0.5,
        };
        let state = FlightState {
            position:         Vec3::new(0.0, 500.0, 0.0),
            velocity:         Vec3::new(100.0, 0.0, 0.0),
            attitude:         Quat::from_rotation_x(-FRAC_PI_2),
            angular_velocity: Vec3::ZERO,
            alpha:            0.0,
            beta:             0.0,
            airspeed:         100.0,
            altitude:         500.0,
        };
        let ctrl = LevelHoldController::with_tuning(&state, &tuning);
        assert!((ctrl.altitude_pid.kp - 0.03).abs() < 1e-6, "alt_kp");
        assert!((ctrl.altitude_pid.ki - 0.20).abs() < 1e-6, "alt_ki");
        assert!((ctrl.altitude_pid.kd - 0.08).abs() < 1e-6, "alt_kd");
        assert!((ctrl.alpha_pid.kp - 3.0).abs() < 1e-6, "alpha_kp");
        assert!((ctrl.alpha_pid.kd - 1.0).abs() < 1e-6, "alpha_kd");
        assert!((ctrl.airspeed_pid.kp - 0.02).abs() < 1e-6, "spd_kp");
        assert!((ctrl.airspeed_pid.ki - 0.15).abs() < 1e-6, "spd_ki");
        assert!((ctrl.throttle_ff_gain - 0.5).abs() < 1e-6, "ff_gain");
        assert!((ctrl.target_altitude - 500.0).abs() < 1e-3, "target_altitude");
        assert!((ctrl.target_airspeed - 100.0).abs() < 1e-3, "target_airspeed");
    }
}
