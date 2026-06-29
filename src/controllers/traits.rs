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

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;

    #[cfg(feature = "visual")]
    fn poll_input(&mut self, _keys: &bevy::input::ButtonInput<bevy::prelude::KeyCode>, _dt: f32) {}
}
