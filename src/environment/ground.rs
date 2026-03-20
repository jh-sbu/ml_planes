use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

pub fn spawn_ground(mut commands: Commands) {
    commands.spawn((
        RigidBody::Fixed,
        Collider::halfspace(Vec3::Y).unwrap(),
        Transform::default(),
    ));
}
