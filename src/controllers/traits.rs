use crate::plane::{ControlInputs, FlightState};

pub trait FlightController: Send + Sync + 'static {
    fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs;
    fn name(&self) -> &'static str {
        "Unknown"
    }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;

    #[cfg(feature = "visual")]
    fn poll_input(&mut self, _keys: &bevy::input::ButtonInput<bevy::prelude::KeyCode>, _dt: f32) {}
}
