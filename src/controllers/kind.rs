use bevy::prelude::Component;

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
}

impl ControllerKind {
    pub const ALL: &'static [ControllerKind] = &[Self::Manual, Self::LevelHold];

    pub fn name(self) -> &'static str {
        match self {
            ControllerKind::Manual => "Manual",
            ControllerKind::LevelHold => "Level Hold",
        }
    }

    /// Return the next kind in the cycle.
    pub fn next(self) -> Self {
        let idx = Self::ALL.iter().position(|&k| k == self).unwrap_or(0);
        Self::ALL[(idx + 1) % Self::ALL.len()]
    }

    /// Construct a fresh controller for this kind, capturing relevant state
    /// so the handoff is bumpless.
    pub fn build(self, state: &FlightState) -> Box<dyn FlightController> {
        match self {
            ControllerKind::Manual => Box::new(ManualController::new()),
            ControllerKind::LevelHold => Box::new(LevelHoldController::from_state(state)),
        }
    }
}
