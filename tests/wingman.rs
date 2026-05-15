mod common;

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::{
    ActiveController, FormationOffset, LeaderRef, LeaderState, LevelHoldController,
    WingmanController,
};
use ml_planes::plane::{ControlInputs, FlightState, PlaneConfig, PlaneConfigHandle};

/// Spawn a plane and return its Entity.
fn spawn_plane_entity(
    app: &mut App,
    pos: Vec3,
    velocity: Vec3,
    controller: impl ml_planes::controllers::FlightController + 'static,
) -> Entity {
    let cfg = common::generic_jet_config();
    let handle: Handle<PlaneConfig> = app
        .world_mut()
        .resource_mut::<Assets<PlaneConfig>>()
        .add(cfg.clone());

    let attitude = Quat::from_rotation_x(-FRAC_PI_2);

    app.world_mut()
        .spawn((
            RigidBody::Dynamic,
            Collider::cuboid(1.0, 0.5, 3.0),
            Velocity {
                linvel: velocity,
                angvel: Vec3::ZERO,
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
            ActiveController(Box::new(controller)),
            PlaneConfigHandle(handle),
            Transform::from_translation(pos).with_rotation(attitude),
        ))
        .id()
}

/// Read all FlightStates from the world.
fn read_states(app: &mut App) -> Vec<FlightState> {
    let mut q = app.world_mut().query::<&FlightState>();
    q.iter(app.world()).cloned().collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// Both planes (leader + wingman) should produce finite flight states and stay
/// airborne over 30 simulated seconds. This validates the full pipeline:
/// feed_leader_state injects the leader's FlightState into the wingman controller,
/// which then commands control surfaces that keep the wingman flying.
#[test]
fn wingman_and_leader_stay_airborne() {
    const TARGET_ALT: f32 = 1000.0;
    const TARGET_SPD: f32 = 100.0;

    let mut app = common::build_headless_app();

    let attitude = Quat::from_rotation_x(-FRAC_PI_2);
    let leader_pos = Vec3::new(0.0, TARGET_ALT, 0.0);
    let leader_vel = Vec3::new(TARGET_SPD, 0.0, 0.0);

    // Leader state approximation for WingmanController constructor.
    let leader_initial = FlightState {
        position: leader_pos,
        velocity: leader_vel,
        attitude,
        airspeed: TARGET_SPD,
        altitude: TARGET_ALT,
        ..Default::default()
    };

    // Spawn leader.
    let leader = spawn_plane_entity(
        &mut app,
        leader_pos,
        leader_vel,
        LevelHoldController::new(TARGET_ALT, TARGET_SPD),
    );

    // Compute wingman's initial position from the formation offset.
    let offset = FormationOffset::default(); // (-20, 15, 0) body frame
    let wingman_pos = leader_pos + attitude * offset.offset_body;
    let own_initial = FlightState {
        position: wingman_pos,
        velocity: leader_vel,
        attitude,
        airspeed: TARGET_SPD,
        altitude: wingman_pos.y,
        ..Default::default()
    };

    // Spawn wingman.
    let wingman = spawn_plane_entity(
        &mut app,
        wingman_pos,
        leader_vel,
        WingmanController::new(&leader_initial, &own_initial, offset.clone()),
    );

    // Attach wingman-specific components.
    app.world_mut()
        .entity_mut(wingman)
        .insert((LeaderRef(leader), LeaderState::default(), offset));

    // Run for 30 simulated seconds (1920 updates × 1/64 s).
    for _ in 0..1920 {
        app.update();
    }

    let states = read_states(&mut app);
    assert_eq!(states.len(), 2, "expected exactly 2 plane entities");

    for state in &states {
        assert!(
            state.altitude.is_finite(),
            "altitude not finite: {}",
            state.altitude
        );
        assert!(
            state.airspeed.is_finite(),
            "airspeed not finite: {}",
            state.airspeed
        );
        assert!(
            state.altitude > 0.0,
            "plane hit the ground: altitude={}",
            state.altitude,
        );
        assert!(
            state.airspeed > 10.0,
            "plane stalled: airspeed={}",
            state.airspeed,
        );
    }
}

/// Wingman tracks the leader's altitude over time: after the leader climbs,
/// the wingman's altitude should also increase.
#[test]
fn wingman_follows_leader_altitude_change() {
    const INITIAL_ALT: f32 = 1000.0;
    const TARGET_SPD: f32 = 100.0;

    let mut app = common::build_headless_app();

    let attitude = Quat::from_rotation_x(-FRAC_PI_2);
    let leader_pos = Vec3::new(0.0, INITIAL_ALT, 0.0);
    let leader_vel = Vec3::new(TARGET_SPD, 0.0, 0.0);

    let leader_initial = FlightState {
        position: leader_pos,
        velocity: leader_vel,
        attitude,
        airspeed: TARGET_SPD,
        altitude: INITIAL_ALT,
        ..Default::default()
    };

    let offset = FormationOffset::default();
    let wingman_pos = leader_pos + attitude * offset.offset_body;
    let own_initial = FlightState {
        position: wingman_pos,
        velocity: leader_vel,
        attitude,
        airspeed: TARGET_SPD,
        altitude: wingman_pos.y,
        ..Default::default()
    };

    let leader = spawn_plane_entity(
        &mut app,
        leader_pos,
        leader_vel,
        LevelHoldController::new(INITIAL_ALT, TARGET_SPD),
    );
    let wingman = spawn_plane_entity(
        &mut app,
        wingman_pos,
        leader_vel,
        WingmanController::new(&leader_initial, &own_initial, offset.clone()),
    );
    app.world_mut()
        .entity_mut(wingman)
        .insert((LeaderRef(leader), LeaderState::default(), offset));

    // Establish steady-state (10 s).
    for _ in 0..640 {
        app.update();
    }

    // Command the leader to climb 100 m.
    {
        let mut ctrl_q = app.world_mut().query::<&mut ActiveController>();
        let mut ctrl = ctrl_q.get_mut(app.world_mut(), leader).unwrap();
        if let Some(lh) = ctrl.0.as_any_mut().downcast_mut::<LevelHoldController>() {
            lh.target_altitude = INITIAL_ALT + 100.0;
        }
    }

    // Run another 20 s for wingman to begin tracking the climb.
    for _ in 0..1280 {
        app.update();
    }

    let states = read_states(&mut app);
    let wingman_state = app
        .world()
        .entity(wingman)
        .get::<FlightState>()
        .unwrap()
        .clone();

    // Wingman should have climbed meaningfully above initial altitude.
    assert!(
        wingman_state.altitude > INITIAL_ALT + 20.0,
        "wingman did not follow leader climb: altitude={}",
        wingman_state.altitude,
    );

    // All planes still airborne.
    for state in &states {
        assert!(
            state.altitude > 0.0,
            "plane crashed: altitude={}",
            state.altitude
        );
    }
}
