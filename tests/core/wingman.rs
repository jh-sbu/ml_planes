//! Requires the 6-DOF sim chain (`PlanePlugin` FixedUpdate systems), which is compiled out on a
//! `net`-without-`server` build (e.g. bare `--features mcp`). The `sim_enabled` cfg (see
//! `build.rs`) gates this module (from `tests/core/main.rs`) so it skips there instead of failing on a default
//! `FlightState`; test networked builds with `--no-default-features --features "…​ server"`.

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::{
    ActiveController, FormationOffset, LevelHoldController, WingmanController,
};
use ml_planes::plane::{ControlInputs, FlightState, PlaneConfig, PlaneConfigHandle, PlaneId};

/// Spawn a plane and return its Entity.
fn spawn_plane_entity(
    app: &mut App,
    id: PlaneId,
    pos: Vec3,
    velocity: Vec3,
    controller: impl ml_planes::controllers::FlightController + 'static,
) -> Entity {
    let cfg = crate::common::generic_jet_config();
    let handle: Handle<PlaneConfig> = app
        .world_mut()
        .resource_mut::<Assets<PlaneConfig>>()
        .add(cfg.clone());

    let attitude = Quat::from_rotation_x(-FRAC_PI_2);

    app.world_mut()
        .spawn((
            RigidBody::Dynamic,
            Collider::cuboid(1.0, 0.5, 3.0),
            ColliderMassProperties::Mass(0.0),
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
            id,
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
/// airborne over 30 simulated seconds.
#[test]
fn wingman_and_leader_stay_airborne() {
    const TARGET_ALT: f32 = 1000.0;
    const TARGET_SPD: f32 = 100.0;

    let mut app = crate::common::build_headless_app();

    let attitude = Quat::from_rotation_x(-FRAC_PI_2);
    let leader_pos = Vec3::new(0.0, TARGET_ALT, 0.0);
    let leader_vel = Vec3::new(TARGET_SPD, 0.0, 0.0);

    let leader_initial = FlightState {
        position: leader_pos,
        velocity: leader_vel,
        attitude,
        airspeed: TARGET_SPD,
        altitude: TARGET_ALT,
        ..Default::default()
    };

    spawn_plane_entity(
        &mut app,
        PlaneId(1),
        leader_pos,
        leader_vel,
        LevelHoldController::new(TARGET_ALT, TARGET_SPD),
    );

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

    spawn_plane_entity(
        &mut app,
        PlaneId(2),
        wingman_pos,
        leader_vel,
        WingmanController::new(PlaneId(1), &leader_initial, &own_initial, offset.clone()),
    );

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

/// Multi-minute formation hold: spawned on-slot, the wingman must STAY on-slot
/// for ~180 simulated seconds without the bank loop running away into a spiral.
/// This pins the heading/yaw-damping fix — the pre-fix pure position→bank loop
/// holds for ~60 s then diverges into a permanent circling turn.
#[test]
fn wingman_holds_slot_over_three_minutes() {
    const TARGET_ALT: f32 = 1000.0;
    const TARGET_SPD: f32 = 100.0;

    let mut app = crate::common::build_headless_app();

    let attitude = Quat::from_rotation_x(-FRAC_PI_2);
    let leader_pos = Vec3::new(0.0, TARGET_ALT, 0.0);
    let leader_vel = Vec3::new(TARGET_SPD, 0.0, 0.0);

    let leader_initial = FlightState {
        position: leader_pos,
        velocity: leader_vel,
        attitude,
        airspeed: TARGET_SPD,
        altitude: TARGET_ALT,
        ..Default::default()
    };

    spawn_plane_entity(
        &mut app,
        PlaneId(1),
        leader_pos,
        leader_vel,
        LevelHoldController::new(TARGET_ALT, TARGET_SPD),
    );

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

    spawn_plane_entity(
        &mut app,
        PlaneId(2),
        wingman_pos,
        leader_vel,
        WingmanController::new(PlaneId(1), &leader_initial, &own_initial, offset.clone()),
    );

    // 180 simulated seconds (11520 updates × 1/64 s).
    for _ in 0..11520 {
        app.update();
    }

    // Pull leader + wingman states by PlaneId so the slot error is unambiguous.
    let mut q = app
        .world_mut()
        .query::<(&PlaneId, &FlightState, &Transform)>();
    let mut leader_state = None;
    let mut wingman_state = None;
    for (id, fs, tf) in q.iter(app.world()) {
        match id.0 {
            1 => leader_state = Some((fs.clone(), *tf)),
            2 => wingman_state = Some((fs.clone(), *tf)),
            _ => {}
        }
    }
    let (leader, leader_tf) = leader_state.expect("leader not found");
    let (wingman, _) = wingman_state.expect("wingman not found");

    // Desired world slot from the leader's actual attitude (Transform rotation).
    let slot = leader.position + leader_tf.rotation * offset.offset_body;
    let err = wingman.position - slot;

    assert!(
        wingman.altitude.is_finite() && wingman.airspeed.is_finite(),
        "wingman state non-finite: alt={}, spd={}",
        wingman.altitude,
        wingman.airspeed
    );
    assert!(
        err.length() < 15.0,
        "wingman drifted out of slot after 180 s: error={:.1} m (slot={:?}, pos={:?})",
        err.length(),
        slot,
        wingman.position
    );
}

/// Wingman tracks the leader's altitude over time: after the leader climbs,
/// the wingman's altitude should also increase.
#[test]
fn wingman_follows_leader_altitude_change() {
    const INITIAL_ALT: f32 = 1000.0;
    const TARGET_SPD: f32 = 100.0;

    let mut app = crate::common::build_headless_app();

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
        PlaneId(1),
        leader_pos,
        leader_vel,
        LevelHoldController::new(INITIAL_ALT, TARGET_SPD),
    );
    spawn_plane_entity(
        &mut app,
        PlaneId(2),
        wingman_pos,
        leader_vel,
        WingmanController::new(PlaneId(1), &leader_initial, &own_initial, offset),
    );

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
    let wingman_entity = app
        .world_mut()
        .query::<(Entity, &PlaneId)>()
        .iter(app.world())
        .find(|(_, id)| id.0 == 2)
        .map(|(e, _)| e)
        .expect("wingman entity not found");
    let wingman_state = app
        .world()
        .entity(wingman_entity)
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

/// Context delivery behavioral test: spawn wingman 200 m below the leader
/// (clearly outside the formation slot). After two app.updates() (one FixedUpdate
/// tick), the wingman's ControlInputs must be non-default, proving that
/// `run_flight_controllers` delivered the leader's FlightState via ControllerContext
/// and the guidance law fired.
#[test]
fn wingman_ctx_delivers_leader_state() {
    const TARGET_ALT: f32 = 1000.0;
    const TARGET_SPD: f32 = 100.0;
    // Spawn wingman 200 m below the leader — creates a clear altitude error.
    const WINGMAN_ALT: f32 = 800.0;

    let mut app = crate::common::build_headless_app();

    let attitude = Quat::from_rotation_x(-FRAC_PI_2);
    let leader_pos = Vec3::new(0.0, TARGET_ALT, 0.0);
    let leader_vel = Vec3::new(TARGET_SPD, 0.0, 0.0);

    let leader_initial = FlightState {
        position: leader_pos,
        velocity: leader_vel,
        attitude,
        airspeed: TARGET_SPD,
        altitude: TARGET_ALT,
        ..Default::default()
    };

    spawn_plane_entity(
        &mut app,
        PlaneId(1),
        leader_pos,
        leader_vel,
        LevelHoldController::new(TARGET_ALT, TARGET_SPD),
    );

    // Wingman spawned 200 m below the leader, NOT at the formation slot.
    let wingman_pos = Vec3::new(0.0, WINGMAN_ALT, 0.0);
    let own_initial = FlightState {
        position: wingman_pos,
        velocity: leader_vel,
        attitude,
        airspeed: TARGET_SPD,
        altitude: WINGMAN_ALT,
        ..Default::default()
    };

    let wingman = spawn_plane_entity(
        &mut app,
        PlaneId(2),
        wingman_pos,
        leader_vel,
        WingmanController::new(
            PlaneId(1),
            &leader_initial,
            &own_initial,
            FormationOffset::default(),
        ),
    );

    // Two updates: frame 0 initializes timing, frame 1 fires FixedUpdate.
    app.update();
    app.update();

    let inputs = app
        .world()
        .entity(wingman)
        .get::<ControlInputs>()
        .unwrap()
        .clone();

    // With a 200 m altitude error the guidance law must produce non-trivial
    // elevator and/or throttle — proving context delivery worked.
    assert!(
        inputs.throttle.abs() > 0.01 || inputs.elevator.abs() > 0.01,
        "wingman ControlInputs near-zero despite 200 m altitude error — \
         context delivery may have failed: {inputs:?}"
    );
}
