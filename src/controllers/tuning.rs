//! Per-controller tuning parameter sets and the [`ControllerTuning`] trait.
//!
//! Each [`ControllerTuning`] implementation encodes one controller's complete
//! tunable parameter set and knows how to construct a properly-tuned controller,
//! avoiding downcasting by letting the concrete type drive construction.

use std::collections::HashMap;

use bevy::reflect::Reflect;
use serde::{Deserialize, Serialize};

use crate::plane::{ControlInputs, FlightState};

use super::heading_hold::HeadingHoldController;
use super::level_hold::LevelHoldController;
use super::orbit::OrbitController;
use super::FlightController;

// ---------------------------------------------------------------------------
// Trait
// ---------------------------------------------------------------------------

/// Builds a tuned [`FlightController`] from a flight state snapshot.
///
/// Implementors encode one controller's complete set of tunable parameters.
/// The factory method captures the current state for a bumpless mode handoff.
pub trait ControllerTuning: std::fmt::Debug + Send + Sync + 'static {
    /// Construct a tuned controller, using `state` to seed targets and
    /// `prev_inputs` to pre-load integrators for a bumpless handoff.
    fn build(&self, state: &FlightState, prev_inputs: &ControlInputs) -> Box<dyn FlightController>;
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
    /// Pitch inner loop proportional gain.
    pub pitch_kp: f32,
    /// Pitch inner loop derivative gain.
    pub pitch_kd: f32,
    /// Airspeed loop proportional gain.
    pub spd_kp: f32,
    /// Airspeed loop integral gain.
    pub spd_ki: f32,
    /// Throttle feedforward gain (scales pitch error [rad] into a throttle increment).
    pub throttle_ff_gain: f32,
}

impl Default for LevelHoldTuning {
    fn default() -> Self {
        // Values match the hardcoded defaults in LevelHoldController::new().
        Self {
            alt_kp: 0.01,
            alt_ki: 0.12,
            alt_kd: 0.04,
            pitch_kp: 1.0,
            pitch_kd: 0.5,
            spd_kp: 0.01,
            spd_ki: 0.06,
            throttle_ff_gain: 0.7,
        }
    }
}

impl ControllerTuning for LevelHoldTuning {
    fn build(&self, state: &FlightState, prev_inputs: &ControlInputs) -> Box<dyn FlightController> {
        Box::new(LevelHoldController::with_tuning(state, self, prev_inputs))
    }
}

// ---------------------------------------------------------------------------
// OrbitTuning
// ---------------------------------------------------------------------------

/// Tunable outer-loop gains for [`OrbitController`].
#[derive(Debug, Clone, Serialize, Deserialize, Reflect)]
pub struct OrbitTuning {
    /// Radial error [m] → heading offset [rad] proportional gain.
    pub radial_kp: f32,
    /// Radial error [m] → heading offset [rad] derivative gain.
    pub radial_kd: f32,
    /// Heading error [rad] → bank correction [rad] proportional gain.
    pub heading_kp: f32,
    /// Heading error [rad] → bank correction [rad] derivative gain.
    pub heading_kd: f32,
    /// Inner level-hold gains (altitude, airspeed, roll, beta loops).
    pub inner: LevelHoldTuning,
}

impl Default for OrbitTuning {
    fn default() -> Self {
        Self {
            radial_kp: 0.002,
            radial_kd: 0.01,
            heading_kp: 0.7,
            heading_kd: 0.1,
            inner: LevelHoldTuning::default(),
        }
    }
}

impl ControllerTuning for OrbitTuning {
    fn build(&self, state: &FlightState, prev_inputs: &ControlInputs) -> Box<dyn FlightController> {
        Box::new(OrbitController::with_tuning(state, self, prev_inputs))
    }
}

// ---------------------------------------------------------------------------
// HeadingHoldTuning
// ---------------------------------------------------------------------------

/// Tunable outer-loop gains for [`HeadingHoldController`].
#[derive(Debug, Clone, Serialize, Deserialize, Reflect)]
pub struct HeadingHoldTuning {
    /// Heading error [rad] → bank command [rad] proportional gain.
    pub heading_kp: f32,
    /// Heading error [rad] → bank command [rad] derivative gain.
    pub heading_kd: f32,
    /// Inner level-hold gains (altitude, airspeed, roll, beta loops).
    pub inner: LevelHoldTuning,
}

impl Default for HeadingHoldTuning {
    fn default() -> Self {
        Self {
            heading_kp: 0.7,
            heading_kd: 0.1,
            inner: LevelHoldTuning::default(),
        }
    }
}

impl ControllerTuning for HeadingHoldTuning {
    fn build(&self, state: &FlightState, prev_inputs: &ControlInputs) -> Box<dyn FlightController> {
        Box::new(HeadingHoldController::with_tuning(state, self, prev_inputs))
    }
}

// ---------------------------------------------------------------------------
// PlaneTuning asset
// ---------------------------------------------------------------------------

/// Per-plane tuning asset loaded from `<plane>.tuning.ron`.
///
/// Each controller kind has its own map of named profiles. Controllers not
/// listed here fall back to their [`Default`] tuning.
#[derive(bevy::asset::Asset, Reflect, Serialize, Deserialize, Debug, Clone, Default)]
pub struct PlaneTuning {
    /// Named tuning profiles for [`LevelHoldController`].
    #[serde(default)]
    pub level_hold: HashMap<String, LevelHoldTuning>,
    /// Named tuning profiles for [`OrbitController`].
    #[serde(default)]
    pub orbit: HashMap<String, OrbitTuning>,
    /// Named tuning profiles for [`HeadingHoldController`].
    #[serde(default)]
    pub heading_hold: HashMap<String, HeadingHoldTuning>,
}

impl PlaneTuning {
    /// Return the named level-hold profile, or `None` if not present.
    pub fn get_level_hold(&self, profile: &str) -> Option<&LevelHoldTuning> {
        self.level_hold.get(profile)
    }

    /// Return the named orbit profile, or `None` if not present.
    pub fn get_orbit(&self, profile: &str) -> Option<&OrbitTuning> {
        self.orbit.get(profile)
    }

    /// Return the named heading-hold profile, or `None` if not present.
    pub fn get_heading_hold(&self, profile: &str) -> Option<&HeadingHoldTuning> {
        self.heading_hold.get(profile)
    }

    /// Merge all profiles from `other` into `self`, overwriting on name collision.
    pub fn merge(&mut self, other: PlaneTuning) {
        self.level_hold.extend(other.level_hold);
        self.orbit.extend(other.orbit);
        self.heading_hold.extend(other.heading_hold);
    }
}

/// Tracks which named tuning profile is selected for this plane entity.
/// Changing this component triggers `apply_controller_switch` to rebuild the controller.
#[derive(bevy::prelude::Component, Clone, Debug, PartialEq, Eq)]
pub struct SelectedTuningProfile(pub String);

/// Marker inserted by `apply_initial_tuning` once the PlaneTuning asset has loaded and the
/// controller has been rebuilt from the named profile. Prevents the initial-apply system from
/// running again on subsequent frames.
#[derive(bevy::prelude::Component, Default)]
pub struct TuningApplied;

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
        assert!((t.pitch_kp - 1.0).abs() < 1e-6);
        assert!((t.pitch_kd - 0.5).abs() < 1e-6);
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
                    pitch_kp: 2.0,
                    pitch_kd: 0.764,
                    spd_kp: 0.01,
                    spd_ki: 0.10,
                    throttle_ff_gain: 0.7,
                ),
            },
        )"#;
        let pt: PlaneTuning = ron::de::from_str(src).expect("PlaneTuning should parse");
        let t = pt
            .get_level_hold("normal")
            .expect("'normal' profile should exist");
        assert!((t.alt_kp - 0.015).abs() < 1e-6);
        assert!((t.pitch_kp - 2.0).abs() < 1e-6);
        assert!(pt.get_level_hold("missing").is_none());
    }

    #[test]
    fn plan_tuning_merge_adds_and_overwrites() {
        let mut base = PlaneTuning::default();
        base.level_hold.insert(
            "normal".into(),
            LevelHoldTuning {
                alt_kp: 0.01,
                ..Default::default()
            },
        );

        let mut incoming = PlaneTuning::default();
        incoming.level_hold.insert(
            "normal".into(),
            LevelHoldTuning {
                alt_kp: 0.99,
                ..Default::default()
            },
        );
        incoming.level_hold.insert(
            "custom".into(),
            LevelHoldTuning {
                alt_kp: 0.05,
                ..Default::default()
            },
        );
        incoming.orbit.insert(
            "custom".into(),
            OrbitTuning {
                radial_kp: 0.42,
                ..Default::default()
            },
        );

        base.merge(incoming);

        assert_eq!(base.level_hold.len(), 2);
        assert!(
            (base.level_hold["normal"].alt_kp - 0.99).abs() < 1e-6,
            "overwrite"
        );
        assert!(
            (base.level_hold["custom"].alt_kp - 0.05).abs() < 1e-6,
            "new profile"
        );
        assert!(
            (base.orbit["custom"].radial_kp - 0.42).abs() < 1e-6,
            "orbit profile"
        );
    }

    #[test]
    fn plan_tuning_merge_into_empty() {
        let mut base = PlaneTuning::default();
        let mut incoming = PlaneTuning::default();
        incoming.orbit.insert("a".into(), OrbitTuning::default());
        base.merge(incoming);
        assert!(base.orbit.contains_key("a"));
    }

    #[test]
    fn plan_tuning_missing_controller_returns_none() {
        let pt = PlaneTuning::default();
        assert!(pt.get_level_hold("normal").is_none());
    }

    #[test]
    fn controller_tuning_build_applies_gains() {
        use bevy::math::{Quat, Vec3};
        use std::f32::consts::FRAC_PI_2;

        let tuning = LevelHoldTuning {
            alt_kp: 0.03,
            alt_ki: 0.20,
            alt_kd: 0.08,
            pitch_kp: 3.0,
            pitch_kd: 1.0,
            spd_kp: 0.02,
            spd_ki: 0.15,
            throttle_ff_gain: 0.5,
        };
        let state = FlightState {
            position: Vec3::new(0.0, 500.0, 0.0),
            velocity: Vec3::new(100.0, 0.0, 0.0),
            attitude: Quat::from_rotation_x(-FRAC_PI_2),
            angular_velocity: Vec3::ZERO,
            alpha: 0.0,
            beta: 0.0,
            airspeed: 100.0,
            altitude: 500.0,
        };
        let ctrl = LevelHoldController::with_tuning(&state, &tuning, &ControlInputs::default());
        assert!((ctrl.altitude_pid.kp - 0.03).abs() < 1e-6, "alt_kp");
        assert!((ctrl.altitude_pid.ki - 0.20).abs() < 1e-6, "alt_ki");
        assert!((ctrl.altitude_pid.kd - 0.08).abs() < 1e-6, "alt_kd");
        assert!((ctrl.pitch_pid.kp - 3.0).abs() < 1e-6, "pitch_kp");
        assert!((ctrl.pitch_pid.kd - 1.0).abs() < 1e-6, "pitch_kd");
        assert!((ctrl.airspeed_pid.kp - 0.02).abs() < 1e-6, "spd_kp");
        assert!((ctrl.airspeed_pid.ki - 0.15).abs() < 1e-6, "spd_ki");
        assert!((ctrl.throttle_ff_gain - 0.5).abs() < 1e-6, "ff_gain");
        assert!(
            (ctrl.target_altitude - 500.0).abs() < 1e-3,
            "target_altitude"
        );
        assert!(
            (ctrl.target_airspeed - 100.0).abs() < 1e-3,
            "target_airspeed"
        );
    }
}
