use bevy::prelude::*;
use bevy_rapier3d::prelude::PhysicsSet;

use super::ground::spawn_ground;
use super::spawner::detect_ground_contact;

#[cfg(feature = "visual")]
use super::grid_material::{follow_camera, GridMaterial};
#[cfg(feature = "visual")]
use super::visual::{
    draw_orbit_pin_gizmo, draw_plane_gizmos, save_curr_physics_pose, save_prev_physics_pose,
    spawn_visual_ground,
};
#[cfg(feature = "visual")]
use bevy::pbr::MaterialPlugin;

pub struct EnvironmentPlugin;

impl Plugin for EnvironmentPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_ground);

        #[cfg(feature = "visual")]
        app.add_plugins(MaterialPlugin::<GridMaterial>::default());
        #[cfg(feature = "visual")]
        app.add_systems(Startup, spawn_visual_ground);
        #[cfg(feature = "visual")]
        app.add_systems(Update, draw_plane_gizmos);
        #[cfg(feature = "visual")]
        app.add_systems(Update, draw_orbit_pin_gizmo);
        #[cfg(feature = "visual")]
        app.add_systems(Update, follow_camera);

        app.add_systems(
            FixedUpdate,
            detect_ground_contact.after(PhysicsSet::StepSimulation),
        );

        #[cfg(feature = "visual")]
        app.add_systems(
            FixedUpdate,
            save_prev_physics_pose.before(PhysicsSet::SyncBackend),
        );
        #[cfg(feature = "visual")]
        app.add_systems(
            FixedUpdate,
            save_curr_physics_pose.after(PhysicsSet::Writeback),
        );
    }
}
