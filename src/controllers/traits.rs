use crate::controllers::targets::ControllerTargets;
use crate::controllers::telemetry::ControllerTelemetry;
use crate::plane::{ControlInputs, ControllerContext, FlightState};

pub trait FlightController: Send + Sync + 'static {
    fn update(&mut self, own: &FlightState, ctx: &ControllerContext, dt: f32) -> ControlInputs;

    fn name(&self) -> &'static str {
        "Unknown"
    }

    /// Read-only status for display, snapshotted server-side into the replicated
    /// [`ControllerTelemetry`] component so a networked client HUD can show it
    /// (`OrbitController` uses `state` to compute its radial error). Controllers with
    /// nothing to publish keep the `None` default.
    fn telemetry(&self, _state: &FlightState) -> ControllerTelemetry {
        ControllerTelemetry::None
    }

    /// Editable setpoints for display + remote editing, snapshotted server-side into the
    /// replicated [`ControllerTargets`] component so a networked client HUD can seed its
    /// editors from it. Controllers with nothing settable keep the `None` default.
    fn targets(&self) -> ControllerTargets {
        ControllerTargets::None
    }

    /// Apply setpoints received from the HUD (local sim) or a
    /// `SetControllerTargetsCommand` (networked client). Implementations must **ignore a
    /// mismatched variant** — a stale command can arrive just after a kind switch. `state`
    /// is available for bumpless re-seeding (e.g. the orbit bank feedforward).
    fn apply_targets(&mut self, _targets: &ControllerTargets, _state: &FlightState) {}

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;

    #[cfg(feature = "visual")]
    fn poll_input(&mut self, _keys: &bevy::input::ButtonInput<bevy::prelude::KeyCode>, _dt: f32) {}
}
