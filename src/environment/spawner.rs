use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::controllers::{ActiveController, FlightController};
use crate::plane::{ControlInputs, FlightState, PlaneConfig, PlaneConfigHandle};
use crate::training::SpawnSpec;

/// Fired (via `Commands::trigger`) when a plane entity contacts the ground collider.
/// In Bevy 0.18 events are observer-based: listen with `app.add_observer(|on: On<PlaneGroundContactEvent>| …)`.
#[derive(Event, Debug, Clone)]
pub struct PlaneGroundContactEvent(pub Entity);

/// Spawn a physics-ready plane entity.
///
/// `cfg` is used immediately for Rapier `AdditionalMassProperties`; the async-
/// loaded `PlaneConfigHandle` stored on the entity drives aerodynamic forces at
/// runtime once the asset is ready.
pub fn spawn_plane(
    commands: &mut Commands,
    asset_server: &AssetServer,
    spec: &SpawnSpec,
    controller: Box<dyn FlightController>,
    cfg: &PlaneConfig,
) -> Entity {
    let position = spec.position.unwrap_or(Vec3::new(0.0, 500.0, 0.0));
    let attitude = spec.attitude.unwrap_or(Quat::IDENTITY);
    let linvel = spec.velocity.unwrap_or(Vec3::new(100.0, 0.0, 0.0));
    let angvel_body = spec.angular_velocity.unwrap_or(Vec3::ZERO);
    // Body-frame angular velocity → world frame for Rapier.
    let angvel_world = attitude.mul_vec3(angvel_body);

    let handle: Handle<PlaneConfig> = asset_server.load("planes/generic_jet.plane.ron");

    commands
        .spawn((
            RigidBody::Dynamic,
            Collider::cuboid(1.0, 0.5, 3.0),
            Velocity {
                linvel,
                angvel: angvel_world,
            },
            ExternalForce::default(),
            AdditionalMassProperties::MassProperties(MassProperties {
                local_center_of_mass: Vec3::ZERO,
                mass: cfg.mass,
                principal_inertia: cfg.inertia,
                principal_inertia_local_frame: Quat::IDENTITY,
            }),
            FlightState::default(),
            ControlInputs::default(),
            ActiveController(controller),
            PlaneConfigHandle(handle),
            Transform::from_translation(position).with_rotation(attitude),
        ))
        .id()
}

pub fn detect_ground_contact(
    plane_query: Query<Entity, With<FlightState>>,
    rapier_context: ReadRapierContext,
    ground_query: Query<Entity, (With<Collider>, Without<FlightState>)>,
    mut commands: Commands,
) {
    let Ok(ctx) = rapier_context.single() else {
        return;
    };
    for ground in ground_query.iter() {
        for plane in plane_query.iter() {
            if ctx.contact_pair(plane, ground).is_some() {
                commands.trigger(PlaneGroundContactEvent(plane));
            }
        }
    }
}
