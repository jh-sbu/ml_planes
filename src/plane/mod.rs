pub mod config;
pub mod context;
pub mod inputs;
pub mod plugin;
pub mod state;
pub mod systems;

pub use config::{FuelProperties, FuelType, PlaneConfig, Powerplant};
pub use context::{ControllerContext, NextPlaneId, PlaneId, PlaneSnapshot, SpawnedPlane};
pub use inputs::ControlInputs;
pub use plugin::{FlightPlanHandle, PlaneConfigHandle, PlanePlugin, PlaneTuningHandle};
pub use state::{FlightState, FUEL_OBS_SCALE};

/// Explicit stable ordering for plane entities (1 = leader, 2 = wingman, …).
/// Used by the camera cycle and HUD to label planes consistently regardless of
/// Bevy entity-ID allocation order.
#[derive(bevy::prelude::Component, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
#[cfg_attr(feature = "net", derive(serde::Serialize, serde::Deserialize))]
pub struct PlaneIndex(pub u32);
