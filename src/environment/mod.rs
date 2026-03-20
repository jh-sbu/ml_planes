mod ground;
mod plugin;
mod spawner;
#[cfg(feature = "visual")]
mod visual;

pub use plugin::EnvironmentPlugin;
pub use spawner::{spawn_plane, PlaneGroundContactEvent};
