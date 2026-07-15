use bevy::prelude::*;
// Physics-dependent pieces (ground collider, ground-contact detection, the
// physics-pose interpolation buffers) are compiled out of the pure networked
// client, which has no Rapier schedule — see `plans/client_server.md` Phase 4.
#[cfg(any(not(feature = "net"), feature = "server"))]
use bevy_rapier3d::prelude::PhysicsSet;

#[cfg(any(not(feature = "net"), feature = "server"))]
use super::ground::spawn_ground;
#[cfg(any(not(feature = "net"), feature = "server"))]
use super::spawner::detect_ground_contact;

#[cfg(feature = "visual")]
use super::grid_material::{follow_camera, GridMaterial};
#[cfg(feature = "visual")]
use super::visual::{draw_orbit_pin_gizmo, draw_plane_gizmos, spawn_visual_ground};
#[cfg(all(feature = "visual", any(not(feature = "net"), feature = "server")))]
use super::visual::{save_curr_physics_pose, save_prev_physics_pose};
#[cfg(feature = "visual")]
use bevy::pbr::MaterialPlugin;

pub struct EnvironmentPlugin;

impl Plugin for EnvironmentPlugin {
    fn build(&self, app: &mut App) {
        // Physics ground collider (death plane) — server / local-sim builds only.
        #[cfg(any(not(feature = "net"), feature = "server"))]
        app.add_systems(Startup, spawn_ground);

        #[cfg(feature = "visual")]
        app.add_plugins(MaterialPlugin::<GridMaterial>::default());
        #[cfg(feature = "visual")]
        app.add_systems(Startup, spawn_visual_ground);
        // Draws the plane at its rendered pose, so it must run after whatever
        // establishes that pose this frame (see `PlaneRenderPose`).
        #[cfg(feature = "visual")]
        app.add_systems(
            Update,
            draw_plane_gizmos.in_set(crate::plane::PlaneRenderPose::Read),
        );
        #[cfg(feature = "visual")]
        app.add_systems(Update, draw_orbit_pin_gizmo);
        #[cfg(feature = "visual")]
        app.add_systems(Update, follow_camera);

        // Ground-contact detection reads the Rapier context — not present on the
        // client, which renders replicated planes and never collides locally.
        #[cfg(any(not(feature = "net"), feature = "server"))]
        app.add_systems(
            FixedUpdate,
            detect_ground_contact.after(PhysicsSet::StepSimulation),
        );

        // Physics-pose interpolation buffers feed the visual local sim; the client
        // interpolates from replicated state instead (see `crate::net::client`).
        #[cfg(all(feature = "visual", any(not(feature = "net"), feature = "server")))]
        app.add_systems(
            FixedUpdate,
            save_prev_physics_pose.before(PhysicsSet::SyncBackend),
        );
        #[cfg(all(feature = "visual", any(not(feature = "net"), feature = "server")))]
        app.add_systems(
            FixedUpdate,
            save_curr_physics_pose.after(PhysicsSet::Writeback),
        );
    }
}
