//! Replicated controller telemetry.
//!
//! The active controller (`ActiveController`, a `Box<dyn FlightController>`) lives
//! server-side only and cannot be serialized, so the read-only status it publishes
//! — orbit radial error, L1 flight-plan leg/status, wingman formation errors, ascent
//! progress — never reaches a networked client. `ControllerTelemetry` is a small,
//! serializable snapshot of that status: the server copies it off the controller each
//! fixed tick (`plane::systems::sync_controller_telemetry`), replicon replicates it to
//! the client, and the client HUD (`ui::hud`) displays it directly.
//!
//! Mirrors the `PlaneTuningPath` pattern: `Component` always, serde gated behind `net`.

use bevy::prelude::Component;

use crate::controllers::l1::L1Status;
use crate::controllers::wingman::WingmanDiagnostics;

/// Read-only, replication-friendly view of the active controller's status. Built by
/// [`FlightController::telemetry`](crate::controllers::FlightController::telemetry) and
/// snapshotted server-side. The `None` default covers controllers that publish nothing
/// (manual, level/heading hold).
#[derive(Component, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "net", derive(serde::Serialize, serde::Deserialize))]
pub enum ControllerTelemetry {
    /// No controller-specific telemetry to display.
    #[default]
    None,
    /// Ascent progress — `complete` latches once at altitude.
    Ascent { complete: bool },
    /// Orbit radial error [m] (current radius minus target; positive = outside).
    Orbit { radial_error: f32 },
    /// Formation-flight diagnostics relative to the leader.
    Wingman(WingmanDiagnostics),
    /// L1 flight-plan progress: active leg index, total legs, and per-leg status.
    FlightPlan {
        leg_index: usize,
        leg_count: usize,
        status: L1Status,
    },
}
