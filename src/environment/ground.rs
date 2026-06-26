// The physics ground collider is only spawned by builds that run the simulation
// locally (tests / local-sim / WASM, or the authoritative server). The thin
// networked client renders replicated planes and has no physics world, so the whole
// module is compiled out there.
#![cfg(any(not(feature = "net"), feature = "server"))]

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

pub fn spawn_ground(mut commands: Commands) {
    commands.spawn((
        RigidBody::Fixed,
        Collider::halfspace(Vec3::Y).unwrap(),
        Transform::default(),
    ));
}
