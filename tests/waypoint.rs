mod common;

use std::f32::consts::FRAC_PI_2;

use bevy::math::{Quat, Vec3};
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::{
    ActiveController, ControllerKind, FlightController, WaypointController, WaypointPhase,
};
use ml_planes::plane::{
    ControlInputs, ControllerContext, FlightState, PlaneConfig, PlaneConfigHandle, PlaneId,
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Level-flight attitude: body +Z (cockpit up) → world +Y.
fn level_attitude() -> Quat {
    Quat::from_rotation_x(-FRAC_PI_2)
}

fn make_state(position: Vec3, velocity: Vec3) -> FlightState {
    let airspeed = velocity.length();
    FlightState {
        position,
        velocity,
        attitude: level_attitude(),
        angular_velocity: Vec3::ZERO,
        alpha: 0.0,
        beta: 0.0,
        airspeed,
        altitude: position.y,
    }
}

fn spawn_plane(app: &mut App, pos: Vec3, velocity: Vec3, controller: WaypointController) {
    let cfg = common::generic_jet_config();
    let handle: Handle<PlaneConfig> = app
        .world_mut()
        .resource_mut::<Assets<PlaneConfig>>()
        .add(cfg.clone());

    app.world_mut().spawn((
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
        PlaneId::TEST,
        FlightState::default(),
        ControlInputs::default(),
        ActiveController(Box::new(controller)),
        PlaneConfigHandle(handle),
        Transform::from_translation(pos).with_rotation(level_attitude()),
    ));
}

fn read_state(app: &mut App) -> FlightState {
    let mut q = app.world_mut().query::<&FlightState>();
    q.single(app.world())
        .expect("expected exactly one FlightState")
        .clone()
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

/// A plane heading along +Z with a waypoint in the +X direction should produce
/// a negative bank command (inner.target_roll < 0 = right bank = right turn).
///
/// Sign chain:
///   heading +Z, target_bearing = 0 (toward +X)
///   → heading_error = +π/2  (current heading is CCW / left of target)
///   → bank_cmd = -pid(+π/2, dt)  →  negative (right bank)
///   → inner.target_roll < 0
#[test]
fn bearing_toward_target_banks_right() {
    // Plane at origin, flying along +Z.
    let state = make_state(Vec3::new(0.0, 1000.0, 0.0), Vec3::new(0.0, 0.0, 100.0));
    // Waypoint at +X — directly to the plane's right.
    let mut ctrl = WaypointController::from_state(
        &state,
        1000.0, // target_x
        0.0,    // target_z
        1000.0, // target_altitude
        100.0,  // target_airspeed
        &ControlInputs::default(),
    );

    let ctx = ControllerContext::empty_for(PlaneId::TEST);
    ctrl.update(&state, &ctx, 1.0 / 64.0);

    // Bearing to +X from a +Z heading is a right turn → negative target_roll.
    assert!(
        ctrl.inner.target_roll < -0.1,
        "expected right-bank command (target_roll < -0.1), got {}",
        ctrl.inner.target_roll
    );
    // Must be in Approach phase (target is 1000 m away, well outside arrival_radius).
    assert_eq!(
        ctrl.phase,
        WaypointPhase::Approach,
        "should still be in Approach phase"
    );
}

/// Constructing a WaypointController with the plane already inside arrival_radius
/// transitions to Orbit on the first call to update().
#[test]
fn phase_transitions_to_orbit_on_arrival() {
    let state = make_state(
        Vec3::new(150.0, 1000.0, 50.0), // 158 m from (0, 0) in XZ — inside 300 m radius
        Vec3::new(80.0, 0.0, 0.0),
    );
    let mut ctrl = WaypointController::from_state(
        &state,
        0.0,    // target_x
        0.0,    // target_z
        1000.0, // target_altitude
        80.0,   // target_airspeed
        &ControlInputs::default(),
    );

    assert_eq!(ctrl.phase, WaypointPhase::Approach, "starts in Approach");

    let ctx = ControllerContext::empty_for(PlaneId::TEST);
    ctrl.update(&state, &ctx, 1.0 / 64.0);

    assert_eq!(
        ctrl.phase,
        WaypointPhase::Orbit,
        "should transition to Orbit when inside arrival_radius"
    );
}

/// ControllerKind::Waypoint.build() must produce a non-panicking controller.
#[test]
fn build_no_panic() {
    let state = make_state(Vec3::new(0.0, 1000.0, 0.0), Vec3::new(100.0, 0.0, 0.0));
    let mut ctrl = ControllerKind::Waypoint.build(&state, None, &ControlInputs::default());
    let ctx = ControllerContext::empty_for(PlaneId::TEST);
    let inputs = ctrl.update(&state, &ctx, 1.0 / 64.0);
    assert!(!ctrl.name().is_empty(), "name() must not be empty");
    assert!(
        inputs.elevator.is_finite()
            && inputs.aileron.is_finite()
            && inputs.rudder.is_finite()
            && inputs.throttle.is_finite(),
        "all ControlInputs must be finite: {inputs:?}"
    );
}

/// WaypointController with a 180° heading error stays finite and points the
/// correct bank direction on the first update.  This guards against NaN from
/// heading_error = ±π.
#[test]
fn large_initial_heading_error_stays_finite() {
    // Plane heading -X, waypoint directly in +X → 180° bearing error.
    let state = make_state(Vec3::new(0.0, 1000.0, 0.0), Vec3::new(-80.0, 0.0, 0.0));
    let mut ctrl = WaypointController::from_state(
        &state,
        2000.0, // target_x — far ahead in +X
        0.0,
        1000.0,
        80.0,
        &ControlInputs::default(),
    );
    let ctx = ControllerContext::empty_for(PlaneId::TEST);
    let inputs = ctrl.update(&state, &ctx, 1.0 / 64.0);
    assert!(
        inputs.aileron.is_finite() && inputs.elevator.is_finite(),
        "outputs must be finite at 180° heading error"
    );
    // Bank command should be non-zero (error is large).
    assert!(
        ctrl.inner.target_roll.abs() > 0.05,
        "expected non-zero bank at 180° error, got {}",
        ctrl.inner.target_roll
    );
}

// ---------------------------------------------------------------------------
// Integration test
// ---------------------------------------------------------------------------

/// A plane spawned 2 km from the waypoint should remain airborne and fly toward
/// the target over 30 simulated seconds without crashing.
#[test]
fn waypoint_controller_flies_toward_target() {
    let target_x = 0.0_f32;
    let target_z = 0.0_f32;
    let target_alt = 1200.0_f32;
    let target_spd = 80.0_f32;

    // Spawn 2 km in +X from target, heading toward it (-X direction).
    let spawn_pos = Vec3::new(2000.0, target_alt, 0.0);
    let spawn_vel = Vec3::new(-target_spd, 0.0, 0.0);

    let mut app = common::build_headless_app();
    let ctrl = WaypointController::from_state(
        &make_state(spawn_pos, spawn_vel),
        target_x,
        target_z,
        target_alt,
        target_spd,
        &ControlInputs::default(),
    );
    spawn_plane(&mut app, spawn_pos, spawn_vel, ctrl);

    // 30 simulated seconds (1920 steps at 1/64 s).
    for _ in 0..1920 {
        app.update();
    }

    let state = read_state(&mut app);

    assert!(
        state.altitude.is_finite() && state.altitude > 100.0,
        "plane crashed: altitude={}",
        state.altitude
    );
    assert!(
        state.airspeed.is_finite() && state.airspeed > 20.0,
        "plane stalled: airspeed={}",
        state.airspeed
    );

    // After 30 s the plane should be close to the target (or orbiting it).
    // Arrival_radius default = 300 m, orbit_radius default = 500 m.
    let dx = state.position.x - target_x;
    let dz = state.position.z - target_z;
    let horiz_dist = (dx * dx + dz * dz).sqrt();
    assert!(
        horiz_dist < 1800.0,
        "plane did not move toward target: horiz_dist={horiz_dist:.0} m (expected < 1800 m)"
    );
}
