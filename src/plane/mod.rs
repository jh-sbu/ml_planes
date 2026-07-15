pub mod config;
pub mod context;
pub mod inputs;
pub mod plugin;
pub mod state;
pub mod systems;

pub use config::{FuelProperties, FuelType, PlaneConfig, Powerplant};
pub use context::{ControllerContext, NextPlaneId, PlaneId, PlaneSnapshot, SpawnedPlane};
pub use inputs::ControlInputs;
pub use plugin::{
    FlightPlanHandle, PlaneConfigHandle, PlanePlugin, PlaneTuningHandle, PlaneTuningPath,
};
pub use state::{FlightState, FUEL_OBS_SCALE};

/// Ordering contract for the plane `Transform` a frame renders.
///
/// On the networked client `net::client::render_net_interpolation` establishes each
/// plane's rendered `Transform` during `Update`, while `camera::update_follow_camera` and
/// `environment::draw_plane_gizmos` consume it in the same schedule. All three touch
/// `Transform`, so Bevy serialises them — but with no ordering edge the order is
/// *unspecified* and flips between frames, so the follow camera intermittently smooths
/// toward the previous frame's pose. Its exponential smoother cannot oscillate on a
/// monotonic input, but it does on an alternating one; because every plane shares one
/// camera, the result was all planes pulsing in perfect sync while their world-space
/// motion stayed perfectly smooth.
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
