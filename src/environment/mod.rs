#[cfg(feature = "visual")]
mod grid_material;
mod ground;
mod lifecycle;
#[cfg(feature = "visual")]
mod plane_model;
mod plugin;
mod scenario_spawn;
mod spawner;
#[cfg(feature = "visual")]
mod visual;

pub use lifecycle::{LifecyclePlugin, RemovePlaneCommand, SpawnPlaneCommand};
#[cfg(feature = "visual")]
pub use plane_model::{model_child_transform, scene_asset_path, PlaneModel};
pub use plugin::EnvironmentPlugin;
pub use scenario_spawn::{spawn_resolved_scenario, ScenarioSpawnResult};
pub use spawner::{
    finalize_pending_spawns, initial_state_from_spec, sanitize_asset_path, spawn_plane,
    spawn_plane_with_id, PendingPlaneSpawn, PlaneGroundContactEvent,
};
#[cfg(feature = "visual")]
pub use visual::{rendered_pose, PhysicsInterp};
