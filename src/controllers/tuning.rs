//! Per-controller tuning parameter sets and the [`ControllerTuning`] trait.
//!
//! Each [`ControllerTuning`] implementation encodes one controller's complete
//! tunable parameter set and knows how to construct a properly-tuned controller,
//! avoiding downcasting by letting the concrete type drive construction.

use std::collections::HashMap;

use bevy::math::Vec3;
use bevy::reflect::Reflect;
use serde::{Deserialize, Serialize};

use crate::plane::{ControlInputs, FlightState};

use super::heading_hold::HeadingHoldController;
use super::level_hold::LevelHoldController;
use super::orbit::OrbitController;
use super::refueling::RefuelConfig;
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
// RefuelingTuning
// ---------------------------------------------------------------------------

/// Tunable parameters for [`RefuelController`](crate::controllers::RefuelController):
/// the three approach stations, the gates between them, the station-transit rates, and
/// the outer cascade gains — plus the inner level-hold block.
///
/// This is the serializable face of [`RefuelConfig`]; `config()` resolves one.
/// Stations are in the **tanker's** body frame (+X fwd, +Y right, +Z up).
///
/// Note that `build()` returns a plain [`LevelHoldController`], not a `RefuelController`
/// — see the impl below for why.
#[derive(Debug, Clone, Serialize, Deserialize, Reflect)]
pub struct RefuelingTuning {
    /// Safe-trail station `(x, y, z)` [m]. Also the recovery station.
    pub astern: (f32, f32, f32),
    /// Closed-up station [m].
    pub precontact: (f32, f32, f32),
    /// Docking station [m].
    pub contact: (f32, f32, f32),

    /// `Astern → Precontact` gate on distance to the phase station [m].
    pub capture_radius: f32,
    /// `Precontact → Contact` gate on distance to the phase station [m].
    pub contact_radius: f32,
    /// Both gates also require `|closure_rate|` below this [m/s].
    pub closure_tolerance: f32,
    /// How long a gate must hold continuously before the phase advances [s].
    pub dwell_secs: f32,
    /// Tracking error above this outside `Astern` triggers a breakaway [m].
    pub abort_radius: f32,

    /// Rate the commanded station moves toward a new phase's station [m/s].
    pub approach_rate: f32,
    /// Rate it retreats to `Astern` after a breakaway [m/s].
    pub breakaway_rate: f32,

    /// Range error [m] → Δairspeed [m/s].
    pub range_kp: f32,
    pub range_ki: f32,
    pub range_kd: f32,
    /// Cross-track [m] → demanded heading offset [rad]. See [`RefuelConfig::lateral_kp`]
    /// before raising `lateral_kp` — the cascade is bandwidth-inverted and 0.002 is
    /// already at the ceiling.
    pub lateral_kp: f32,
    pub lateral_kd: f32,
    /// Heading error [rad] → commanded bank [rad].
    pub heading_kp: f32,
    pub heading_kd: f32,

    /// Inner level-hold gains (altitude, airspeed, roll, beta loops).
    pub inner: LevelHoldTuning,
}

impl Default for RefuelingTuning {
    fn default() -> Self {
        // Mirrors `RefuelConfig::default()` rather than repeating its literals, so the
        // shipped profiles and the compiled defaults cannot drift apart.
        let c = RefuelConfig::default();
        Self {
            astern: (c.astern.x, c.astern.y, c.astern.z),
            precontact: (c.precontact.x, c.precontact.y, c.precontact.z),
            contact: (c.contact.x, c.contact.y, c.contact.z),
            capture_radius: c.capture_radius,
            contact_radius: c.contact_radius,
            closure_tolerance: c.closure_tolerance,
            dwell_secs: c.dwell_secs,
            abort_radius: c.abort_radius,
            approach_rate: c.approach_rate,
            breakaway_rate: c.breakaway_rate,
            range_kp: c.range_kp,
            range_ki: c.range_ki,
            range_kd: c.range_kd,
            lateral_kp: c.lateral_kp,
            lateral_kd: c.lateral_kd,
            heading_kp: c.heading_kp,
            heading_kd: c.heading_kd,
            inner: LevelHoldTuning::default(),
        }
    }
}

impl RefuelingTuning {
    /// Resolve the runtime [`RefuelConfig`] this profile describes.
    pub fn config(&self) -> RefuelConfig {
        RefuelConfig {
            astern: Vec3::new(self.astern.0, self.astern.1, self.astern.2),
            precontact: Vec3::new(self.precontact.0, self.precontact.1, self.precontact.2),
            contact: Vec3::new(self.contact.0, self.contact.1, self.contact.2),
            capture_radius: self.capture_radius,
            contact_radius: self.contact_radius,
            closure_tolerance: self.closure_tolerance,
            dwell_secs: self.dwell_secs,
            abort_radius: self.abort_radius,
            approach_rate: self.approach_rate,
            breakaway_rate: self.breakaway_rate,
            range_kp: self.range_kp,
            range_ki: self.range_ki,
            range_kd: self.range_kd,
            lateral_kp: self.lateral_kp,
            lateral_kd: self.lateral_kd,
            heading_kp: self.heading_kp,
            heading_kd: self.heading_kd,
        }
    }
}

impl ControllerTuning for RefuelingTuning {
    /// Returns a tuned [`LevelHoldController`] — **not** a `RefuelController`.
    ///
    /// The generic factory has no tanker reference, exactly as with
    /// `ControllerKind::Wingman`. `sim_control::restore_refueling` downcasts this
    /// result and re-wraps it via `RefuelController::from_inner`, so changing the
    /// returned type here silently breaks the tuning rebuild.
    fn build(&self, state: &FlightState, prev_inputs: &ControlInputs) -> Box<dyn FlightController> {
        Box::new(LevelHoldController::with_tuning(
            state,
            &self.inner,
            prev_inputs,
        ))
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
    /// Named tuning profiles for [`RefuelController`](crate::controllers::RefuelController).
    #[serde(default)]
    pub refueling: HashMap<String, RefuelingTuning>,
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

    /// Return the named refueling profile, or `None` if not present.
    pub fn get_refueling(&self, profile: &str) -> Option<&RefuelingTuning> {
        self.refueling.get(profile)
    }

    /// Merge all profiles from `other` into `self`, overwriting on name collision.
    pub fn merge(&mut self, other: PlaneTuning) {
        self.level_hold.extend(other.level_hold);
        self.orbit.extend(other.orbit);
        self.heading_hold.extend(other.heading_hold);
        self.refueling.extend(other.refueling);
    }
}

// ---------------------------------------------------------------------------
// PlaneTuning asset
// ---------------------------------------------------------------------------

/// Tracks which named tuning profile is selected for this plane entity.
/// Changing this component triggers `apply_controller_switch` to rebuild the controller.
#[derive(bevy::prelude::Component, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "net", derive(serde::Serialize, serde::Deserialize))]
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

            consumable_remaining: f32::INFINITY,
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
