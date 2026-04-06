use bevy::prelude::Component;

use crate::controllers::tuning::ControllerTuning;
use crate::controllers::{FlightController, LevelHoldController, ManualController};
use crate::plane::FlightState;

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
}

impl ControllerKind {
    pub const ALL: &'static [ControllerKind] = &[Self::Manual, Self::LevelHold];

    pub fn name(self) -> &'static str {
        match self {
            ControllerKind::Manual    => "Manual",
            ControllerKind::LevelHold => "Level Hold",
            ControllerKind::Wingman   => "Wingman",
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
    /// Pass `tuning` to apply per-plane gains; `None` falls back to the
    /// controller's built-in defaults.
    ///
    /// Note: `Wingman` falls back to `LevelHold` here because the wingman
    /// controller requires a leader entity reference that cannot be passed
    /// through this generic factory. Spawn wingmen explicitly with
    /// `WingmanController::new()`.
    pub fn build(self, state: &FlightState, tuning: Option<&dyn ControllerTuning>) -> Box<dyn FlightController> {
        match self {
            ControllerKind::Manual => Box::new(ManualController::new()),
            ControllerKind::LevelHold | ControllerKind::Wingman => match tuning {
                Some(t) => t.build(state),
                None    => Box::new(LevelHoldController::from_state(state)),
            },
        }
    }
}
