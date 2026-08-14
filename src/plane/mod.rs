pub mod config;
pub mod context;
pub mod inputs;
pub mod plugin;
pub mod state;
pub mod systems;
pub mod timestep;

pub use config::{FuelProperties, FuelType, PlaneConfig, Powerplant};
pub use context::{ControllerContext, NextPlaneId, PlaneId, PlaneSnapshot, SpawnedPlane};
pub use inputs::ControlInputs;
pub use plugin::{
    FlightPlanHandle, PlaneConfigHandle, PlanePlugin, PlaneTuningHandle, PlaneTuningPath,
};
pub use state::{FlightState, FUEL_OBS_SCALE};
pub use timestep::{PHYSICS_DT, PHYSICS_DT_F64, PHYSICS_HZ, PHYSICS_HZ_F64};

/// Ordering contract for the plane `Transform` a frame renders.
///
/// On the networked client `net::client::render_net_interpolation` establishes each
/// plane's rendered `Transform` during `Update`, while `camera::update_follow_camera` and
/// `environment::draw_plane_gizmos` consume it in the same schedule. All three touch
/// `Transform`, so their order must be explicit to keep readers on the current pose.
///
/// Writers declare [`PlaneRenderPose::Write`], readers [`PlaneRenderPose::Read`], and
/// [`PlanePlugin`] orders the former before the latter. The sets are deliberately
/// feature-agnostic: `CameraPlugin` / `EnvironmentPlugin` compile without `net` and so
/// must not name the networked writer directly.
#[derive(bevy::prelude::SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum PlaneRenderPose {
    /// Establishes the `Transform` this frame renders.
    Write,
    /// Consumes that pose. Ordered after [`PlaneRenderPose::Write`].
    Read,
}

/// Explicit stable ordering for plane entities (1 = leader, 2 = wingman, …).
/// Used by the camera cycle and HUD to label planes consistently regardless of
/// Bevy entity-ID allocation order.
#[derive(bevy::prelude::Component, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
#[cfg_attr(feature = "net", derive(serde::Serialize, serde::Deserialize))]
pub struct PlaneIndex(pub u32);
