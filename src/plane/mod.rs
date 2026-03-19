pub mod config;
pub mod inputs;
pub mod plugin;
pub mod state;
pub mod systems;

pub use config::PlaneConfig;
pub use inputs::ControlInputs;
pub use plugin::{PlaneConfigHandle, PlanePlugin};
pub use state::FlightState;
