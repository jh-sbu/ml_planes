#[cfg(feature = "visual")]
mod grid_material;
mod ground;
mod lifecycle;
mod plugin;
mod scenario_spawn;
mod spawner;
#[cfg(feature = "visual")]
mod visual;

pub use lifecycle::{LifecyclePlugin, RemovePlaneCommand, SpawnPlaneCommand};
pub use plugin::EnvironmentPlugin;
pub use scenario_spawn::{spawn_resolved_scenario, ScenarioSpawnResult};
pub use spawner::{
    finalize_pending_spawns, initial_state_from_spec, spawn_plane, spawn_plane_with_id,
    PendingPlaneSpawn, PlaneGroundContactEvent,
};
#[cfg(feature = "visual")]
pub use visual::PhysicsInterp;
