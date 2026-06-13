use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::controllers::{ActiveController, ControllerKind, FlightController};
use crate::plane::{
    ControlInputs, FlightState, NextPlaneId, PlaneConfig, PlaneConfigHandle, PlaneId, PlaneIndex,
    SpawnedPlane,
};
use crate::training::SpawnSpec;

#[cfg(feature = "visual")]
use super::visual::PhysicsInterp;

/// Fired (via `Commands::trigger`) when a plane entity contacts the ground collider.
/// In Bevy 0.18 events are observer-based: listen with `app.add_observer(|on: On<PlaneGroundContactEvent>| …)`.
#[derive(Event, Debug, Clone)]
pub struct PlaneGroundContactEvent(pub Entity);

/// Mass/inertia (and control limits) for the generic-jet airframe, used to seed
/// Rapier `AdditionalMassProperties` at spawn time. Aerodynamic coefficients are
/// left at zero — the async-loaded `.plane.ron` handle drives aero forces once
/// the asset is ready. Shared by `main.rs`'s startup spawns and the runtime
/// `SpawnPlaneCommand` so both seed identical body mass.
pub fn generic_jet_spawn_config() -> PlaneConfig {
    PlaneConfig {
        wing_area: 20.0,
        mean_chord: 2.0,
        wing_span: 10.0,
        mass: 5000.0,
        inertia: Vec3::new(10000.0, 40000.0, 45000.0),
        cl0: 0.0,
        cl_alpha: 0.0,
        cl_delta_e: 0.0,
        cl_max: 1.4,
        cd0: 0.0,
        cd_induced: 0.0,
        cm0: 0.0,
        cm_alpha: 0.0,
        cm_q: 0.0,
        cm_delta_e: 0.0,
        cl_beta: 0.0,
        cl_p: 0.0,
        cl_r: 0.0,
        cl_delta_a: 0.0,
        cn_beta: 0.0,
        cn_r: 0.0,
        cn_delta_r: 0.0,
        thrust_max: 0.0,
        powerplant: Default::default(),
        aileron_limit: 0.4363,
        elevator_limit: 0.3491,
        rudder_limit: 0.2618,
    }
}

/// Resolve a `SpawnSpec` into the concrete initial `FlightState` a spawned plane
/// starts with, applying the same defaults `spawn_plane` uses. Shared so callers
/// that need to construct a controller *before* spawning (e.g. the lifecycle
/// spawn command) build it from the exact state the plane will hold.
pub fn initial_state_from_spec(spec: &SpawnSpec) -> FlightState {
    let position = spec.position.unwrap_or(Vec3::new(0.0, 500.0, 0.0));
    // -π/2 around X maps body +Z (cockpit up) → world +Y (up).
    let attitude = spec
        .attitude
        .unwrap_or(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2));
    let linvel = spec.velocity.unwrap_or(Vec3::new(100.0, 0.0, 0.0));
    let angvel_body = spec.angular_velocity.unwrap_or(Vec3::ZERO);

    let mut s = FlightState {
        position,
        velocity: linvel,
        attitude,
        angular_velocity: angvel_body,
        ..Default::default()
    };
    s.update_air_data();
    s
}

/// Spawn a physics-ready plane entity and assign it a stable `PlaneId`.
///
/// `ids` allocates the next available id (auto-incremented).
/// `config_path` is the `.plane.ron` asset loaded asynchronously to drive
/// aerodynamic forces once ready; `cfg` is used immediately for Rapier
/// `AdditionalMassProperties` (mass/inertia) at spawn time. Pass a `cfg` whose
/// mass/inertia match `config_path` — they describe the same airframe.
pub fn spawn_plane(
    commands: &mut Commands,
    ids: &mut NextPlaneId,
    asset_server: &AssetServer,
    config_path: &str,
    spec: &SpawnSpec,
    controller: Box<dyn FlightController>,
    kind: ControllerKind,
    cfg: &PlaneConfig,
) -> SpawnedPlane {
    let plane_id = PlaneId(ids.0);
    ids.0 += 1;
    let mut state = initial_state_from_spec(spec);
    // Opt this plane into the fuel model: fill the tank to the requested fraction of
    // capacity (default full). The integrator / `consume_fuel` then burn it down.
    state.consumable_remaining = cfg.powerplant.capacity() * spec.fuel_fraction.unwrap_or(1.0);
    let position = state.position;
    let attitude = state.attitude;
    let linvel = state.velocity;
    // Body-frame angular velocity → world frame for Rapier.
    let angvel_world = attitude.mul_vec3(state.angular_velocity);

    let handle: Handle<PlaneConfig> = asset_server.load(config_path.to_string());

    let entity = commands
        .spawn((
            RigidBody::Dynamic,
            Collider::cuboid(3.0, 0.5, 1.0),
            Velocity {
                linvel,
                angvel: angvel_world,
            },
            ExternalForce::default(),
            AdditionalMassProperties::MassProperties(MassProperties {
                local_center_of_mass: Vec3::ZERO,
                // Empty mass + fuel load (jets); empty mass alone for electric.
                // `update_plane_mass` keeps this current as fuel burns.
                mass: cfg
                    .powerplant
                    .effective_mass(cfg.mass, state.consumable_remaining),
                principal_inertia: cfg.inertia,
                principal_inertia_local_frame: Quat::IDENTITY,
            }),
            state,
            ControlInputs::default(),
            ActiveController(controller),
            kind,
            // `PlaneIndex` is the display/cycle ordinal used by the camera, map,
            // and HUD. Deriving it from the already-unique, monotonic `PlaneId`
            // guarantees every spawned plane is indexed automatically — callers
            // never have to insert it by hand.
            (plane_id, PlaneIndex(plane_id.0)),
            PlaneConfigHandle(handle),
            Transform::from_translation(position).with_rotation(attitude),
        ))
        .id();

    #[cfg(feature = "visual")]
    commands.entity(entity).insert(PhysicsInterp {
        prev_pos: position,
        prev_rot: attitude,
        curr_pos: position,
        curr_rot: attitude,
    });

    SpawnedPlane {
        entity,
        id: plane_id,
    }
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
