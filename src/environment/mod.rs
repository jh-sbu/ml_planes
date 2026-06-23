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
    generic_jet_spawn_config, initial_state_from_spec, load_spawn_config, spawn_plane,
    PlaneGroundContactEvent,
};
#[cfg(feature = "visual")]
pub use visual::PhysicsInterp;
