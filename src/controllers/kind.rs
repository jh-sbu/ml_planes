use bevy::prelude::Component;

use crate::controllers::heading_hold::HeadingHoldController;
use crate::controllers::orbit::OrbitController;
use crate::controllers::tuning::ControllerTuning;
use crate::controllers::{
    AscentController, FlightController, LevelHoldController, ManualController,
};
use crate::plane::{ControlInputs, FlightState};

/// Identifies which controller implementation is active on a plane entity.
///
/// Stored as a `Component` so Bevy change-detection can trigger a controller
/// rebuild whenever the kind changes (via key press or HUD dropdown).
#[derive(Component, Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "net", derive(serde::Serialize, serde::Deserialize))]
pub enum ControllerKind {
    Manual,
    LevelHold,
    /// Formation wingman. Controller must be constructed explicitly via
    /// `WingmanController::new()`; use `ControllerKind::LevelHold` as the
    /// fallback when cycling through kinds interactively.
    Wingman,
    /// Climbs to a preset altitude then holds. Controller must be constructed
    /// explicitly via `AscentController::new()`; `build()` falls back to
    /// `LevelHold` (no target altitude is available in the generic factory).
    Ascent,
    /// ML-based level hold (PPO policy). Controller must be constructed
    /// explicitly via `RlLevelHoldController::load()`; `build()` falls back to
    /// `LevelHold` (generic factory cannot produce an RL controller without a
    /// model path).
    RlLevelHold,
    /// Circular orbit around a configurable world-frame point.
    Orbit,
    /// ML-based circular orbit (PPO policy). Controller must be constructed
    /// explicitly via `RlOrbitController::load()`; `build()` falls back to
    /// `Orbit` when no model is available.
    RlOrbit,
    /// Residual ML correction over PID orbit (PPO policy). Controller must be
    /// constructed explicitly via `RlOrbitResidualController::load()`; `build()`
    /// falls back to `Orbit` when no model is available.
    RlOrbitResidual,
    /// Wu et al. FC-LSTM-FC orbit controller (recurrent PPO policy). Must be
    /// constructed explicitly via `RlLstmOrbitController::load()`; `build()`
    /// falls back to `Orbit` when no model is available.
    RlLstmOrbit,
    /// Holds a configurable heading (yaw direction) while maintaining altitude
    /// and airspeed via an inner level-hold cascade.
    HeadingHold,
    /// Follows a preset multi-leg flight plan (waypoint sequences + orbit
    /// circles) via L1 nonlinear lateral guidance. The plan is supplied through
    /// a `FlightPlanHandle` asset and applied by the `apply_flight_plan` system;
    /// the generic factory cannot construct it without the plan, so `build()`
    /// falls back to `Orbit`.
    FlightPlan,
    /// ML-based heading hold (PPO policy). Controller must be constructed
    /// explicitly via `RlHeadingHoldController::load()`; `build()` falls back
    /// to `HeadingHold` (generic factory cannot produce an RL controller
    /// without a model path).
    RlHeadingHold,
    /// Staged aerial-refueling approach onto a tanker. Like `Wingman`, the controller
    /// must be constructed explicitly (`RefuelController::new()`) because the generic
    /// factory has no tanker reference; `build()` falls back to `LevelHold`. Declared
    /// last (append-only) so existing bincode discriminants for a stale net peer
    /// don't shift.
    Refueling,
}

impl ControllerKind {
    // `Wingman` and `Refueling` are deliberately absent from both `ALL` lists: their
    // `build()` cannot reconstruct them, so cycling to one interactively would install a
    // `LevelHoldController` under the wrong label and `cleanup_orphaned_followers` would
    // demote it again the next tick — a flickering kind. Pinned by
    // `peer_following_kinds_are_not_interactively_cyclable`.
    #[cfg(not(feature = "inference"))]
    pub const ALL: &'static [ControllerKind] = &[
        Self::Manual,
        Self::LevelHold,
        Self::HeadingHold,
        Self::Ascent,
        Self::Orbit,
        Self::FlightPlan,
    ];

    #[cfg(feature = "inference")]
    pub const ALL: &'static [ControllerKind] = &[
        Self::Manual,
        Self::LevelHold,
        Self::HeadingHold,
        Self::RlHeadingHold,
        Self::Ascent,
        Self::RlLevelHold,
        Self::Orbit,
        Self::RlOrbit,
        Self::RlOrbitResidual,
        Self::RlLstmOrbit,
        Self::FlightPlan,
    ];

    pub fn name(self) -> &'static str {
        match self {
            ControllerKind::Manual => "Manual",
            ControllerKind::LevelHold => "Level Hold",
            ControllerKind::HeadingHold => "Heading Hold",
            ControllerKind::Wingman => "Wingman",
            ControllerKind::Ascent => "Ascent",
            ControllerKind::RlLevelHold => "RL Level Hold",
            ControllerKind::Orbit => "Orbit",
            ControllerKind::RlOrbit => "RL Orbit",
            ControllerKind::RlOrbitResidual => "RL Orbit Residual",
            ControllerKind::RlLstmOrbit => "RL LSTM Orbit",
            ControllerKind::FlightPlan => "Flight Plan (L1)",
            ControllerKind::RlHeadingHold => "RL Heading Hold",
            ControllerKind::Refueling => "Refueling",
        }
    }

    /// Returns the `models/` subdirectory name for ML controller kinds, or
    /// `None` for non-ML controllers.
    pub fn model_dir(self) -> Option<&'static str> {
        match self {
            ControllerKind::RlLevelHold => Some("level_hold"),
            ControllerKind::RlOrbit => Some("orbit"),
            ControllerKind::RlOrbitResidual => Some("orbit_residual"),
            ControllerKind::RlLstmOrbit => Some("lstm_orbit"),
            ControllerKind::RlHeadingHold => Some("heading_hold"),
            _ => None,
        }
    }

    /// Whether this kind uses the `heading_hold` tuning pool.
    pub fn is_heading_hold(self) -> bool {
        matches!(
            self,
            ControllerKind::HeadingHold | ControllerKind::RlHeadingHold
        )
    }

    /// Return the next kind in the cycle (interactive UI; Wingman is excluded).
    pub fn next(self) -> Self {
        let idx = Self::ALL.iter().position(|&k| k == self).unwrap_or(0);
        Self::ALL[(idx + 1) % Self::ALL.len()]
    }

    /// Construct a fresh controller for this kind, capturing relevant state
    /// so the handoff is bumpless.
    ///
    /// `prev_inputs` is the last `ControlInputs` from the outgoing controller;
    /// it is used to pre-seed PID integrals for a true bumpless transfer.
    /// Pass `&ControlInputs::default()` for fresh spawns with no prior output.
    ///
    /// Pass `tuning` to apply per-plane gains; `None` falls back to the
    /// controller's built-in defaults.
    ///
    /// Note: `Wingman` falls back to `LevelHold` here because the wingman
    /// controller requires a leader entity reference that cannot be passed
    /// through this generic factory. Spawn wingmen explicitly with
    /// `WingmanController::new()`.
    ///
    /// `Ascent` targets `state.altitude + 1000 m` — a sensible default for
    /// an interactive climb from wherever the plane is when switching.
    pub fn build(
        self,
        state: &FlightState,
        tuning: Option<&dyn ControllerTuning>,
        prev_inputs: &ControlInputs,
    ) -> Box<dyn FlightController> {
        match self {
            ControllerKind::Manual => Box::new(ManualController::new()),
            ControllerKind::Ascent => {
                Box::new(AscentController::new(state, state.altitude + 1000.0))
            }
            // RlHeadingHold requires a model path — fall back to HeadingHold.
            ControllerKind::HeadingHold | ControllerKind::RlHeadingHold => match tuning {
                Some(t) => t.build(state, prev_inputs),
                None => Box::new(HeadingHoldController::from_state(state, prev_inputs)),
            },
            ControllerKind::Orbit
            | ControllerKind::RlOrbit
            | ControllerKind::RlOrbitResidual
            | ControllerKind::RlLstmOrbit => match tuning {
                Some(t) => t.build(state, prev_inputs),
                None => Box::new(OrbitController::from_state(state, prev_inputs)),
            },
            // RlLevelHold requires a model path, and Wingman/Refueling need a peer
            // reference the generic factory doesn't have — all fall back to LevelHold.
            // For Wingman and Refueling this is load-bearing rather than merely
            // graceful: `sim_control`'s restore_wingman / restore_refueling downcast
            // this exact result back to a `LevelHoldController` to re-wrap it.
            ControllerKind::LevelHold
            | ControllerKind::Wingman
            | ControllerKind::Refueling
            | ControllerKind::RlLevelHold => match tuning {
                Some(t) => t.build(state, prev_inputs),
                None => Box::new(LevelHoldController::from_state(state, prev_inputs)),
            },
            // FlightPlan needs the plan asset, which the generic factory cannot
            // access; the real controller is built by `apply_flight_plan`. Fall
            // back to a PID orbit so an entity without a loaded plan still flies.
            ControllerKind::FlightPlan => Box::new(OrbitController::from_state(state, prev_inputs)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::{Quat, Vec3};

    fn state() -> FlightState {
        FlightState {
            position: Vec3::new(0.0, 1000.0, 0.0),
            velocity: Vec3::new(100.0, 0.0, 0.0),
            attitude: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            airspeed: 100.0,
            altitude: 1000.0,
            ..Default::default()
        }
    }

    #[cfg(feature = "inference")]
    #[test]
    fn rl_orbit_uses_orbit_model_dir() {
        assert_eq!(ControllerKind::RlOrbit.model_dir(), Some("orbit"));
        assert!(ControllerKind::ALL.contains(&ControllerKind::RlOrbit));
    }

    #[test]
    fn rl_orbit_builds_pid_orbit_fallback() {
        let mut controller =
            ControllerKind::RlOrbit.build(&state(), None, &ControlInputs::default());
        assert!(controller
            .as_any_mut()
            .downcast_mut::<OrbitController>()
            .is_some());
    }

    #[cfg(feature = "inference")]
    #[test]
    fn rl_heading_hold_uses_heading_hold_model_dir() {
        assert_eq!(
            ControllerKind::RlHeadingHold.model_dir(),
            Some("heading_hold")
        );
        assert!(ControllerKind::ALL.contains(&ControllerKind::RlHeadingHold));
    }

    #[test]
    fn rl_heading_hold_builds_pid_heading_hold_fallback() {
        let mut controller =
            ControllerKind::RlHeadingHold.build(&state(), None, &ControlInputs::default());
        assert!(controller
            .as_any_mut()
            .downcast_mut::<HeadingHoldController>()
            .is_some());
    }

    /// Pins the deliberate omission documented above `ALL`. Both kinds need a peer
    /// reference `build()` cannot supply, so reaching them from the interactive cycle
    /// would install a mislabelled `LevelHoldController`. Nothing else fails if someone
    /// "helpfully" adds them, which is exactly why this test exists.
    #[test]
    fn peer_following_kinds_are_not_interactively_cyclable() {
        for kind in [ControllerKind::Wingman, ControllerKind::Refueling] {
            assert!(
                !ControllerKind::ALL.contains(&kind),
                "{:?} must stay out of the interactive cycle: build() cannot reconstruct it",
                kind
            );
        }
    }

    #[test]
    fn refueling_builds_level_hold_fallback() {
        // Load-bearing, not merely graceful: `sim_control::restore_refueling` downcasts
        // this exact result back to a `LevelHoldController` to re-wrap it.
        let mut controller =
            ControllerKind::Refueling.build(&state(), None, &ControlInputs::default());
        assert!(controller
            .as_any_mut()
            .downcast_mut::<LevelHoldController>()
            .is_some());
        assert_eq!(ControllerKind::Refueling.name(), "Refueling");
        assert_eq!(ControllerKind::Refueling.model_dir(), None);
        assert!(!ControllerKind::Refueling.is_heading_hold());
    }

    #[test]
    fn rl_heading_hold_uses_heading_hold_tuning_pool() {
        assert!(ControllerKind::RlHeadingHold.is_heading_hold());
        assert!(ControllerKind::HeadingHold.is_heading_hold());
        assert!(!ControllerKind::LevelHold.is_heading_hold());
    }
}
