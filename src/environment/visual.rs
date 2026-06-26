use bevy::pbr::{DistanceFog, FogFalloff};
use bevy::prelude::*;

use crate::camera::CameraMode;
use crate::controllers::{active_orbit_center, ActiveController};
use crate::environment::grid_material::{GridMaterial, GroundPlane};
use crate::plane::FlightState;

/// Stores the physics Transform from the previous and current fixed step so that
/// the render loop can interpolate between them each frame.
#[derive(Component)]
pub struct PhysicsInterp {
    pub prev_pos: Vec3,
    pub prev_rot: Quat,
    pub curr_pos: Vec3,
    pub curr_rot: Quat,
}

/// Run BEFORE PhysicsSet::SyncBackend — saves the last step's result as "prev".
/// Physics-pose interpolation feeds the local-sim renderer; the networked client
/// interpolates from replicated state instead (`crate::net::client`).
#[cfg(any(not(feature = "net"), feature = "server"))]
pub fn save_prev_physics_pose(mut query: Query<(&Transform, &mut PhysicsInterp)>) {
    for (transform, mut interp) in query.iter_mut() {
        interp.prev_pos = transform.translation;
        interp.prev_rot = transform.rotation;
    }
}

/// Run AFTER PhysicsSet::Writeback — saves the just-completed step's result as "curr".
#[cfg(any(not(feature = "net"), feature = "server"))]
pub fn save_curr_physics_pose(mut query: Query<(&Transform, &mut PhysicsInterp)>) {
    for (transform, mut interp) in query.iter_mut() {
        interp.curr_pos = transform.translation;
        interp.curr_rot = transform.rotation;
    }
}

pub fn spawn_visual_ground(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<GridMaterial>>,
) {
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(20000.0, 20000.0))),
        MeshMaterial3d(materials.add(GridMaterial {})),
        Transform::from_xyz(0.0, -0.01, 0.0),
        GroundPlane,
    ));
    commands.spawn(DistanceFog {
        color: Color::srgb(0.5, 0.5, 0.5),
        falloff: FogFalloff::Linear {
            start: 1000.0,
            end: 5000.0,
        },
        ..default()
    });
}

pub fn draw_plane_gizmos(
    // Local-sim planes carry `PhysicsInterp` and are interpolated here against the
    // fixed-step overstep. Networked-client planes have no local physics, so they
    // lack `PhysicsInterp`; their `Transform` is already interpolated from replicated
    // state by `crate::net::client::render_net_interpolation`, so fall back to it.
    query: Query<(&FlightState, Option<&PhysicsInterp>, &Transform)>,
    mut gizmos: Gizmos,
    time_fixed: Res<Time<Fixed>>,
) {
    let alpha = time_fixed.overstep_fraction();
    for (state, interp, transform) in &query {
        let (pos, rot) = match interp {
            Some(interp) => (
                interp.prev_pos.lerp(interp.curr_pos, alpha),
                interp.prev_rot.slerp(interp.curr_rot, alpha),
            ),
            None => (transform.translation, transform.rotation),
        };

        // Velocity arrow — world frame, scaled so 100 m/s → 20 m arrow
        let vel_tip = pos + state.velocity * 0.2;
        gizmos.arrow(pos, vel_tip, Color::srgb(0.2, 0.8, 1.0));

        // Nose direction — body +X at interpolated attitude, fixed 15 m length
        let nose = pos + rot * Vec3::X * 15.0;
        gizmos.arrow(pos, nose, Color::srgb(1.0, 0.4, 0.1));

        // Body wireframe — full extents are 2× the Collider::cuboid(3, 0.5, 1) half-extents
        gizmos.primitive_3d(
            &Cuboid::new(6.0, 1.0, 2.0),
            Isometry3d::new(pos, rot),
            Color::srgb(0.3, 1.0, 0.3),
        );
    }
}

/// Vertical "pin" planted in the ground at the active orbit center of the
/// currently selected (followed) plane. Hidden in FreeLook and for non-orbit
/// controllers. The orbit center is resolved uniformly across `OrbitController`,
/// the RL orbit variants, and `L1Controller` orbit legs by
/// [`active_orbit_center`].
pub fn draw_orbit_pin_gizmo(
    camera_mode: Res<CameraMode>,
    mut planes: Query<&mut ActiveController>,
    mut gizmos: Gizmos,
) {
    /// Pin height above the ground [m].
    const PIN_HEIGHT: f32 = 300.0;
    /// Radius of the pin's head sphere [m].
    const HEAD_RADIUS: f32 = 8.0;

    let CameraMode::Follow(entity) = *camera_mode else {
        return;
    };
    let Ok(mut ctrl) = planes.get_mut(entity) else {
        return;
    };
    let Some(marker) = active_orbit_center(ctrl.0.as_mut()) else {
        return;
    };

    let color = Color::srgb(1.0, 0.9, 0.2);
    let base = Vec3::new(marker.center.x, 0.0, marker.center.y);
    let top = base + Vec3::Y * PIN_HEIGHT;
    gizmos.line(base, top, color);
    gizmos.sphere(Isometry3d::from_translation(top), HEAD_RADIUS, color);
}
