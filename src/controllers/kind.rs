use bevy::prelude::Component;

use crate::controllers::tuning::ControllerTuning;
use crate::controllers::{AscentController, FlightController, LevelHoldController, ManualController};
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
}

impl ControllerKind {
    pub const ALL: &'static [ControllerKind] = &[Self::Manual, Self::LevelHold, Self::Ascent];

    pub fn name(self) -> &'static str {
        match self {
            ControllerKind::Manual    => "Manual",
            ControllerKind::LevelHold => "Level Hold",
            ControllerKind::Wingman   => "Wingman",
            ControllerKind::Ascent    => "Ascent",
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
            ControllerKind::Ascent => Box::new(AscentController::new(state, state.altitude + 1000.0)),
            ControllerKind::LevelHold | ControllerKind::Wingman => match tuning {
                Some(t) => t.build(state, prev_inputs),
                None    => Box::new(LevelHoldController::from_state(state, prev_inputs)),
            },
        }
    }
}
