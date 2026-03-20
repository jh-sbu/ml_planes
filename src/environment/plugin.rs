use bevy::prelude::*;
use bevy_rapier3d::prelude::PhysicsSet;

use super::ground::spawn_ground;
use super::spawner::{detect_ground_contact, PlaneGroundContactEvent};

#[cfg(feature = "visual")]
use super::visual::spawn_visual_ground;

pub struct EnvironmentPlugin;

impl Plugin for EnvironmentPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_ground);
        #[cfg(feature = "visual")]
        app.add_systems(Startup, spawn_visual_ground);
        app.add_systems(
            FixedUpdate,
            detect_ground_contact.after(PhysicsSet::StepSimulation),
        );
    }
}
