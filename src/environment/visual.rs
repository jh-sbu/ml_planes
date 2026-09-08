use bevy::pbr::{DistanceFog, FogFalloff};
use bevy::prelude::*;

use crate::camera::CameraMode;
use crate::controllers::{active_orbit_center, ActiveController};
use crate::environment::grid_material::{GridMaterial, GroundPlane};
use crate::environment::plane_model::{draws_body_wireframe, PlaneModel};
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

/// The pose a frame should render a plane at.
///
/// Local-sim planes carry [`PhysicsInterp`] and are interpolated against the fixed-step
/// overstep — their `Transform` holds the raw 64 Hz pose. Networked-client planes have
/// no local physics and so no buffer; their `Transform` was already interpolated from
/// replicated state by `crate::net::client::render_net_interpolation`. Shared by the
/// gizmos and by the model-placement system so the two can never disagree on where a
/// plane is this frame.
pub fn rendered_pose(
    interp: Option<&PhysicsInterp>,
    transform: &Transform,
    alpha: f32,
) -> (Vec3, Quat) {
    match interp {
        Some(interp) => (
            interp.prev_pos.lerp(interp.curr_pos, alpha),
            interp.prev_rot.slerp(interp.curr_rot, alpha),
        ),
        None => (transform.translation, transform.rotation),
    }
}

/// A sun and a raised ambient fill, so PBR meshes loaded by
/// [`crate::environment::plane_model`] are actually lit. The ground's `GridMaterial` is
/// an unlit fragment shader and is unaffected either way.
///
/// Shadows are deliberately off: cascades over the 20 km ground plane cost more than
/// they currently buy.
pub fn spawn_scene_lighting(mut commands: Commands, mut ambient: ResMut<GlobalAmbientLight>) {
    commands.spawn((
        DirectionalLight {
            illuminance: light_consts::lux::OVERCAST_DAY,
            shadows_enabled: false,
            ..default()
        },
        // High sun, slightly off the nose, so the dihedral and fin read as solid.
        Transform::from_rotation(Quat::from_euler(EulerRot::YXZ, -0.9, -1.0, 0.0)),
    ));
    // Bevy's default (80 lux) leaves an unshadowed underside almost black against a
    // bright sky.
    ambient.color = Color::srgb(0.68, 0.76, 0.92);
    ambient.brightness = 900.0;
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
    // The pose comes from `rendered_pose`, which handles both the local-sim
    // (`PhysicsInterp`) and networked-client (already-interpolated `Transform`) cases.
    query: Query<(
        &FlightState,
        Option<&PhysicsInterp>,
        &Transform,
        Option<&PlaneModel>,
    )>,
    mut gizmos: Gizmos,
    time_fixed: Res<Time<Fixed>>,
) {
    let alpha = time_fixed.overstep_fraction();
    for (state, interp, transform, model) in &query {
        let (pos, rot) = rendered_pose(interp, transform, alpha);

        // Velocity arrow — world frame, scaled so 100 m/s → 20 m arrow
        let vel_tip = pos + state.velocity * 0.2;
        gizmos.arrow(pos, vel_tip, Color::srgb(0.2, 0.8, 1.0));

        // Nose direction — body +X at interpolated attitude, fixed 15 m length
        let nose = pos + rot * Vec3::X * 15.0;
        gizmos.arrow(pos, nose, Color::srgb(1.0, 0.4, 0.1));

        // Body wireframe — full extents are 2× the Collider::cuboid(3, 0.5, 1)
        // half-extents. It is the stand-in for a mesh, so a plane with a model
        // attached drops it; the two arrows stay, since they show the velocity vector
        // against the body axis (α and β at a glance) and the mesh cannot.
        if draws_body_wireframe(model) {
            gizmos.primitive_3d(
                &Cuboid::new(6.0, 1.0, 2.0),
                Isometry3d::new(pos, rot),
                Color::srgb(0.3, 1.0, 0.3),
            );
        }
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A local-sim plane's `Transform` holds the raw fixed-step pose, so the rendered
    /// pose must come from the interpolation buffer, not from it.
    #[test]
    fn interpolated_pose_wins_over_the_fixed_step_transform() {
        let interp = PhysicsInterp {
            prev_pos: Vec3::ZERO,
            prev_rot: Quat::IDENTITY,
            curr_pos: Vec3::new(10.0, 0.0, 0.0),
            curr_rot: Quat::from_rotation_y(std::f32::consts::FRAC_PI_2),
        };
        // Deliberately somewhere else entirely, so a fallback would be obvious.
        let stale = Transform::from_xyz(-999.0, -999.0, -999.0);

        let (pos, rot) = rendered_pose(Some(&interp), &stale, 0.5);
        assert!(
            (pos - Vec3::new(5.0, 0.0, 0.0)).length() < 1e-5,
            "pos={pos}"
        );
        let expected = Quat::IDENTITY.slerp(interp.curr_rot, 0.5);
        assert!(rot.abs_diff_eq(expected, 1e-5), "rot={rot}");
    }

    /// A networked-client plane has no buffer — its `Transform` was already
    /// interpolated from replicated state, so it is the pose.
    #[test]
    fn without_a_buffer_the_transform_is_the_rendered_pose() {
        let t = Transform::from_xyz(1.0, 2.0, 3.0)
            .with_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2));
        let (pos, rot) = rendered_pose(None, &t, 0.75);
        assert_eq!(pos, t.translation);
        assert_eq!(rot, t.rotation);
    }
}
