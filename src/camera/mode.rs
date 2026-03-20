#[cfg(feature = "visual")]
use bevy::prelude::*;

#[cfg(feature = "visual")]
#[derive(Resource, Default)]
pub enum CameraMode {
    #[default]
    FreeLook,
    Follow(Entity),
}
