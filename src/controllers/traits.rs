use crate::plane::{FlightState, ControlInputs};

pub trait FlightController: Send + Sync + 'static {
    fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs;
    fn name(&self) -> &'static str { "Unknown" }

    #[cfg(feature = "visual")]
    fn poll_input(
        &mut self,
        _keys: &bevy::input::ButtonInput<bevy::prelude::KeyCode>,
        _dt: f32,
    ) {}
}
