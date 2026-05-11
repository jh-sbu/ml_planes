use bevy::prelude::Component;

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
}

impl ControllerKind {
    #[cfg(not(feature = "training"))]
    pub const ALL: &'static [ControllerKind] =
        &[Self::Manual, Self::LevelHold, Self::Ascent, Self::Orbit];

    #[cfg(feature = "training")]
    pub const ALL: &'static [ControllerKind] = &[
        Self::Manual,
        Self::LevelHold,
        Self::Ascent,
        Self::RlLevelHold,
        Self::Orbit,
        Self::RlOrbit,
        Self::RlOrbitResidual,
        Self::RlLstmOrbit,
    ];

    pub fn name(self) -> &'static str {
        match self {
            ControllerKind::Manual => "Manual",
            ControllerKind::LevelHold => "Level Hold",
            ControllerKind::Wingman => "Wingman",
            ControllerKind::Ascent => "Ascent",
            ControllerKind::RlLevelHold => "RL Level Hold",
            ControllerKind::Orbit => "Orbit",
            ControllerKind::RlOrbit => "RL Orbit",
            ControllerKind::RlOrbitResidual => "RL Orbit Residual",
            ControllerKind::RlLstmOrbit => "RL LSTM Orbit",
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
            _ => None,
        }
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
            ControllerKind::Orbit
            | ControllerKind::RlOrbit
            | ControllerKind::RlOrbitResidual
            | ControllerKind::RlLstmOrbit => match tuning {
                Some(t) => t.build(state, prev_inputs),
                None => Box::new(OrbitController::from_state(state, prev_inputs)),
            },
            // RlLevelHold requires a model path — fall back to LevelHold like Wingman.
            ControllerKind::LevelHold | ControllerKind::Wingman | ControllerKind::RlLevelHold => {
                match tuning {
                    Some(t) => t.build(state, prev_inputs),
                    None => Box::new(LevelHoldController::from_state(state, prev_inputs)),
                }
            }
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

    #[cfg(feature = "training")]
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
}
