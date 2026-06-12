#[cfg(feature = "visual")]
mod grid_material;
mod ground;
mod lifecycle;
mod plugin;
mod spawner;
#[cfg(feature = "visual")]
mod visual;

pub use lifecycle::{LifecyclePlugin, RemovePlaneCommand, SpawnPlaneCommand};
pub use plugin::EnvironmentPlugin;
pub use spawner::{
    generic_jet_spawn_config, initial_state_from_spec, spawn_plane, PlaneGroundContactEvent,
};
#[cfg(feature = "visual")]
pub use visual::PhysicsInterp;
