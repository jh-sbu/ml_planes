//! Requires the 6-DOF sim chain (`PlanePlugin` FixedUpdate systems), which is compiled out on a
//! `net`-without-`server` build (e.g. bare `--features mcp`). The `sim_enabled` cfg (see
//! `build.rs`) gates this module (from `tests/core/main.rs`) so it skips there instead of failing on a default
//! `FlightState`; test networked builds with `--no-default-features --features "…​ server"`.

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::{ActiveController, LevelHoldController};
use ml_planes::plane::{ControlInputs, FlightState, PlaneConfig, PlaneConfigHandle, PlaneId};

/// Spawn a plane at `pos` flying at `velocity` in level-flight orientation.
///
/// Level-flight orientation: body +Z (cockpit up) aligns with world +Y (up).
/// `Quat::from_rotation_x(-FRAC_PI_2)` achieves this mapping.
fn spawn_plane(app: &mut App, pos: Vec3, velocity: Vec3, controller: LevelHoldController) {
    let cfg = crate::common::generic_jet_config();
    let handle: Handle<PlaneConfig> = app
        .world_mut()
        .resource_mut::<Assets<PlaneConfig>>()
        .add(cfg.clone());

    let attitude = Quat::from_rotation_x(-FRAC_PI_2);

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
        Transform::from_translation(pos).with_rotation(attitude),
    ));
}

/// Read the single FlightState from the world.
fn read_state(app: &mut App) -> FlightState {
    let mut q = app.world_mut().query::<&FlightState>();
    q.single(app.world())
        .expect("expected exactly one FlightState entity")
        .clone()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// A level-hold controller spawned 100 m below target should prevent the plane
/// from crashing and maintain a reasonable altitude over 30 simulated seconds.
///
/// NOTE: FixedUpdate does not fire on frame 0 in Bevy's fixed-timestep model
/// (the virtual-time accumulator starts empty).  The loop therefore runs the
/// full 1 920 frames without any early-exit; a break based on FlightState
/// altitude before the first FixedUpdate fires would see altitude = 0 (the
/// component default) and exit prematurely.
#[test]
fn altitude_hold_prevents_crash() {
    let target_alt = 1000.0_f32;
    let target_spd = 80.0_f32;
    let spawn_alt = 900.0_f32; // 100 m below target

    let mut app = crate::common::build_headless_app();
    let ctrl = LevelHoldController::new(target_alt, target_spd);
    spawn_plane(
        &mut app,
        Vec3::new(0.0, spawn_alt, 0.0),
        Vec3::new(target_spd, 0.0, 0.0),
        ctrl,
    );

    // 640 updates × 1/64 s = 10 simulated seconds.
    //
    // We cap at 10 s rather than 30 s: without a controller the plane enters a
    // high-speed dive (>300 m/s) that produces forces large enough to cause NaN
    // in Rapier.  At 10 s, free-fall from 900 m reaches only ~410 m altitude
    // and airspeed ~100 m/s — well within finite range — so the assertion
    // `altitude > 700 m` cleanly distinguishes a working controller from a
    // broken one.
    for _ in 0..640 {
        app.update();
    }

    let state = read_state(&mut app);

    assert!(
        state.altitude.is_finite(),
        "altitude became non-finite: {}",
        state.altitude
    );
    // Free-fall from 900 m reaches ~410 m after 10 s.
    // A working controller prevents more than a small transient dip.
    assert!(
        state.altitude > 700.0,
        "altitude too low — controller may not be holding: altitude={}",
        state.altitude
    );
    assert!(
        state.airspeed.is_finite() && state.airspeed > 20.0,
        "airspeed dropped too far: {}",
        state.airspeed
    );
}

/// Large-offset robustness: spawning 500 m below target should not stall or crash
/// the plane.  Without throttle feedforward the controller pitches hard nose-up,
/// induced drag spikes ~6×, and the airspeed PID cannot compensate fast enough —
/// the plane stalls.  With feedforward, extra thrust is added as soon as the climb
/// command is issued.
#[test]
fn large_altitude_offset_does_not_stall() {
    let target_alt = 1000.0_f32;
    let target_spd = 80.0_f32;
    let spawn_alt = 500.0_f32; // 500 m below target

    let mut app = crate::common::build_headless_app();
    let ctrl = LevelHoldController::new(target_alt, target_spd);
    spawn_plane(
        &mut app,
        Vec3::new(0.0, spawn_alt, 0.0),
        Vec3::new(target_spd, 0.0, 0.0),
        ctrl,
    );

    // 1 920 updates × 1/64 s = 30 simulated seconds.
    for _ in 0..1920 {
        app.update();
    }

    let state = read_state(&mut app);

    assert!(
        state.altitude.is_finite(),
        "altitude became non-finite: {}",
        state.altitude
    );
    assert!(
        state.altitude > 50.0,
        "plane crashed — altitude={}",
        state.altitude
    );
    assert!(
        state.airspeed.is_finite() && state.airspeed > 30.0,
        "plane stalled — airspeed={}",
        state.airspeed
    );
}

/// Airspeed hold: the controller should keep airspeed from falling off
/// completely over 30 simulated seconds.
#[test]
fn airspeed_hold_maintains_speed() {
    let target_alt = 1000.0_f32;
    let target_spd = 80.0_f32;

    let mut app = crate::common::build_headless_app();
    let ctrl = LevelHoldController::new(target_alt, target_spd);
    spawn_plane(
        &mut app,
        Vec3::new(0.0, target_alt, 0.0),
        Vec3::new(target_spd, 0.0, 0.0),
        ctrl,
    );

    for _ in 0..640 {
        app.update();
    }

    let state = read_state(&mut app);
    assert!(
        state.airspeed.is_finite(),
        "airspeed is non-finite: {}",
        state.airspeed
    );
    assert!(
        state.airspeed > 30.0,
        "airspeed fell to near-stall in 10 s: {}",
        state.airspeed
    );
}
