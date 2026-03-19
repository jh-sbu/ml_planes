use crate::plane::{FlightState, ControlInputs};

pub trait FlightController: Send + Sync + 'static {
    fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs;
    fn name(&self) -> &'static str { "Unknown" }
}
