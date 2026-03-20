use bevy::pbr::{DistanceFog, FogFalloff};
use bevy::prelude::*;

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
