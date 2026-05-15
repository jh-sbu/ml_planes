use bevy::pbr::{DistanceFog, FogFalloff};
use bevy::prelude::*;

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
pub fn save_prev_physics_pose(mut query: Query<(&Transform, &mut PhysicsInterp)>) {
    for (transform, mut interp) in query.iter_mut() {
        interp.prev_pos = transform.translation;
        interp.prev_rot = transform.rotation;
    }
}

/// Run AFTER PhysicsSet::Writeback — saves the just-completed step's result as "curr".
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
    query: Query<(&FlightState, &PhysicsInterp)>,
    mut gizmos: Gizmos,
    time_fixed: Res<Time<Fixed>>,
) {
    let alpha = time_fixed.overstep_fraction();
    for (state, interp) in &query {
        let pos = interp.prev_pos.lerp(interp.curr_pos, alpha);
        let rot = interp.prev_rot.slerp(interp.curr_rot, alpha);

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
