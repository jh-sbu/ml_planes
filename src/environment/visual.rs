use bevy::pbr::{DistanceFog, FogFalloff};
use bevy::prelude::*;

use crate::plane::FlightState;

pub fn spawn_visual_ground(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(5000.0, 5000.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.1, 0.5, 0.1),
            ..default()
        })),
        Transform::from_xyz(0.0, -0.01, 0.0),
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

pub fn draw_plane_gizmos(query: Query<&FlightState>, mut gizmos: Gizmos) {
    for state in &query {
        let origin = state.position;

        // Velocity arrow — world frame, scaled so 100 m/s → 20 m arrow
        let vel_tip = origin + state.velocity * 0.2;
        gizmos.arrow(origin, vel_tip, Color::srgb(0.2, 0.8, 1.0));

        // Nose direction — body +X rotated to world frame, fixed 15 m length
        let nose = origin + state.attitude * Vec3::X * 15.0;
        gizmos.arrow(origin, nose, Color::srgb(1.0, 0.4, 0.1));
    }
}
