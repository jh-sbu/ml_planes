//! Replicated controller setpoints — the settable half of controller state.
//!
//! The active controller (`ActiveController`, a `Box<dyn FlightController>`) lives
//! server-side only, so its editable setpoints — target altitude/airspeed/heading, orbit
//! geometry, wingman leader — never reach a networked client on their own.
//! `ControllerTargets` is a small, serializable snapshot of those setpoints: the server
//! copies it off the controller each fixed tick
//! (`plane::systems::sync_controller_targets`), replicon replicates it to the client, and
//! the client HUD (`ui::hud`) seeds its editors from it. Edits flow back via
//! `SetControllerTargetsCommand` (`net::protocol`), applied through
//! [`FlightController::apply_targets`](crate::controllers::FlightController::apply_targets).
//!
//! Kept separate from [`ControllerTelemetry`](crate::controllers::ControllerTelemetry):
//! telemetry is *derived, read-only* status (radial error, leg/status) that changes every
//! tick; targets are *settable* and change only when edited. Folding them together would
//! re-broadcast (and, on the client, re-stomp mid-edit) unedited setpoints every tick.
//!
//! One variant per *widget set*, not per controller kind: `RlLevelHoldController` reuses
//! `LevelHold`, and all three RL orbit variants (`RlOrbitController`,
//! `RlOrbitResidualController`, `RlLstmOrbitController`) reuse `Orbit` — the HUD does not
//! need to know which concrete controller is currently installed.
//!
//! Mirrors the `ControllerTelemetry` pattern: `Component` always, serde gated behind `net`.

use bevy::prelude::Component;

use crate::controllers::orbit::OrbitParams;
use crate::plane::PlaneId;

/// Replication-friendly view of the active controller's editable setpoints. Built by
/// [`FlightController::targets`](crate::controllers::FlightController::targets) and
/// snapshotted server-side. The `None` default covers controllers with nothing settable
/// (manual, flight-plan — the `.plan.ron` asset owns those targets).
#[derive(Component, Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "net", derive(serde::Serialize, serde::Deserialize))]
pub enum ControllerTargets {
    /// No editable setpoints.
    #[default]
    None,
    /// `LevelHoldController` and `RlLevelHoldController` share this variant.
    LevelHold { altitude: f32, airspeed: f32 },
    /// `AscentController`'s climb target.
    Ascent { altitude: f32 },
    /// `HeadingHoldController`. `heading` is in radians.
    HeadingHold {
        heading: f32,
        altitude: f32,
        airspeed: f32,
    },
    /// `OrbitController`, `RlOrbitController`, `RlOrbitResidualController`, and
    /// `RlLstmOrbitController` all share this variant.
    Orbit(OrbitParams),
    /// `WingmanController`'s formation leader.
    Wingman { leader: PlaneId },
}
