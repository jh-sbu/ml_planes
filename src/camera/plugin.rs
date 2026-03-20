use bevy::prelude::*;

use super::mode::CameraMode;
use super::systems::{spawn_camera, update_follow_camera, update_free_look_camera};

pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CameraMode>();
        app.add_systems(Startup, spawn_camera);
        app.add_systems(Update, (update_free_look_camera, update_follow_camera));
    }
}
