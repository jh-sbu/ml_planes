use bevy::prelude::*;
use bevy_rapier3d::prelude::PhysicsSet;

use super::ground::spawn_ground;
use super::spawner::detect_ground_contact;

#[cfg(feature = "visual")]
use super::visual::{draw_plane_gizmos, spawn_visual_ground};

pub struct EnvironmentPlugin;

impl Plugin for EnvironmentPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_ground);
        #[cfg(feature = "visual")]
        app.add_systems(Startup, spawn_visual_ground);
        #[cfg(feature = "visual")]
        app.add_systems(Update, draw_plane_gizmos);
        app.add_systems(
            FixedUpdate,
            detect_ground_contact.after(PhysicsSet::StepSimulation),
        );
    }
}
