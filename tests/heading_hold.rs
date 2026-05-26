mod common;

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::{ActiveController, FlightController, HeadingHoldController};
use ml_planes::plane::{
    ControlInputs, ControllerContext, FlightState, PlaneConfig, PlaneConfigHandle, PlaneId,
};

fn level_attitude() -> Quat {
    Quat::from_rotation_x(-FRAC_PI_2)
}

fn spawn_plane(app: &mut App, pos: Vec3, velocity: Vec3, controller: HeadingHoldController) {
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
        .expect("expected exactly one FlightState entity")
        .clone()
}

fn current_heading(state: &FlightState) -> f32 {
    let speed_xz = (state.velocity.x.powi(2) + state.velocity.z.powi(2)).sqrt();
    if speed_xz > 1.0 {
        state.velocity.z.atan2(state.velocity.x)
    } else {
        0.0
    }
}

// ---------------------------------------------------------------------------
// Unit tests (no Bevy app required)
// ---------------------------------------------------------------------------

#[test]
fn zero_error_gives_near_zero_aileron() {
    let state = FlightState {
        position: Vec3::new(0.0, 1000.0, 0.0),
        velocity: Vec3::new(80.0, 0.0, 0.0),
        attitude: level_attitude(),
        angular_velocity: Vec3::ZERO,
        alpha: 0.0,
        beta: 0.0,
        airspeed: 80.0,
        altitude: 1000.0,
    };
    // Target heading = 0 rad (same as current velocity direction along +X).
    let mut ctrl = HeadingHoldController::from_state(&state, &ControlInputs::default());
    let ctx = ControllerContext::empty_for(PlaneId::TEST);
    let inputs = ctrl.update(&state, &ctx, 1.0 / 60.0);
    assert!(
        inputs.aileron.abs() < 0.1,
        "aileron={} expected ≈0 at zero heading error",
        inputs.aileron
    );
}

#[test]
fn heading_left_gives_right_bank() {
    // Current heading is 15° to the left of target (+Z component in XZ plane).
    let heading_offset = 15.0_f32.to_radians();
    let speed = 80.0_f32;
    let state = FlightState {
        position: Vec3::new(0.0, 1000.0, 0.0),
        velocity: Vec3::new(
            speed * heading_offset.cos(),
            0.0,
            speed * heading_offset.sin(),
        ),
        attitude: level_attitude(),
        angular_velocity: Vec3::ZERO,
        alpha: 0.0,
        beta: 0.0,
        airspeed: speed,
        altitude: 1000.0,
    };
    // Target heading = 0 rad (along +X). Current heading has drifted 15° toward +Z.
    let mut ctrl = HeadingHoldController::new(
        &state, 0.0, // target heading = 0 rad
    );
    let ctx = ControllerContext::empty_for(PlaneId::TEST);
    let inputs = ctrl.update(&state, &ctx, 1.0 / 60.0);
    // Positive roll_angle = left bank; to correct a left-drifted heading we bank right
    // (negative roll → negative aileron).
    assert!(
        inputs.aileron < 0.0,
        "aileron={} expected <0 (right bank) when current heading is left of target",
        inputs.aileron
    );
}

#[test]
fn heading_right_gives_left_bank() {
    // Current heading is 15° to the right of target (-Z component in XZ plane).
    let heading_offset = -15.0_f32.to_radians();
    let speed = 80.0_f32;
    let state = FlightState {
        position: Vec3::new(0.0, 1000.0, 0.0),
        velocity: Vec3::new(
            speed * heading_offset.cos(),
            0.0,
            speed * heading_offset.sin(),
        ),
        attitude: level_attitude(),
        angular_velocity: Vec3::ZERO,
        alpha: 0.0,
        beta: 0.0,
        airspeed: speed,
        altitude: 1000.0,
    };
    let mut ctrl = HeadingHoldController::new(&state, 0.0);
    let ctx = ControllerContext::empty_for(PlaneId::TEST);
    let inputs = ctrl.update(&state, &ctx, 1.0 / 60.0);
    assert!(
        inputs.aileron > 0.0,
        "aileron={} expected >0 (left bank) when current heading is right of target",
        inputs.aileron
    );
}

// ---------------------------------------------------------------------------
// Integration tests (headless Bevy app)
// ---------------------------------------------------------------------------

/// A heading-hold controller should keep the plane on its initial heading
/// (±15°) for 30 simulated seconds without crashing.
#[test]
fn heading_hold_maintains_heading() {
    let alt = 1000.0_f32;
    let spd = 80.0_f32;
    let velocity = Vec3::new(spd, 0.0, 0.0);

    let mut app = common::build_headless_app();
    let state_seed = FlightState {
        position: Vec3::new(0.0, alt, 0.0),
        velocity,
        attitude: level_attitude(),
        angular_velocity: Vec3::ZERO,
        alpha: 0.0,
        beta: 0.0,
        airspeed: spd,
        altitude: alt,
    };
    let ctrl = HeadingHoldController::from_state(&state_seed, &ControlInputs::default());
    spawn_plane(&mut app, Vec3::new(0.0, alt, 0.0), velocity, ctrl);

    // 1920 updates × 1/64 s = 30 simulated seconds.
    for _ in 0..1920 {
        app.update();
    }

    let state = read_state(&mut app);
    let heading = current_heading(&state);
    let tolerance = 15.0_f32.to_radians();

    assert!(
        state.altitude.is_finite() && state.altitude > 0.0,
        "plane crashed — altitude={}",
        state.altitude
    );
    assert!(
        heading.abs() < tolerance,
        "heading drifted too far: {:.1}° (tolerance ±15°)",
        heading.to_degrees()
    );
}

/// Changing target heading by 30° mid-flight should cause the plane to bank
/// and converge to the new heading within 20 simulated seconds.
#[test]
fn heading_hold_tracks_step_change() {
    let alt = 1000.0_f32;
    let spd = 80.0_f32;
    let velocity = Vec3::new(spd, 0.0, 0.0);
    let step_heading = 30.0_f32.to_radians();

    let mut app = common::build_headless_app();
    let state_seed = FlightState {
        position: Vec3::new(0.0, alt, 0.0),
        velocity,
        attitude: level_attitude(),
        angular_velocity: Vec3::ZERO,
        alpha: 0.0,
        beta: 0.0,
        airspeed: spd,
        altitude: alt,
    };
    let ctrl = HeadingHoldController::from_state(&state_seed, &ControlInputs::default());
    spawn_plane(&mut app, Vec3::new(0.0, alt, 0.0), velocity, ctrl);

    // Run 3 s before the step change (192 updates).
    for _ in 0..192 {
        app.update();
    }

    // Apply 30° heading step.
    {
        let mut q = app.world_mut().query::<&mut ActiveController>();
        let mut ctrl = q.single_mut(app.world_mut()).expect("single plane");
        if let Some(hh) = ctrl.0.as_any_mut().downcast_mut::<HeadingHoldController>() {
            hh.target_heading = step_heading;
        }
    }

    // Run from 3 s to 30 s (1728 more updates).
    for _ in 0..1728 {
        app.update();
    }

    let state = read_state(&mut app);
    let heading = current_heading(&state);
    let err = (heading - step_heading + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU)
        - std::f32::consts::PI;

    assert!(
        state.altitude.is_finite() && state.altitude > 0.0,
        "plane crashed — altitude={}",
        state.altitude
    );
    assert!(
        err.abs() < 8.0_f32.to_radians(),
        "heading did not converge: err={:.1}°, target=30°, current={:.1}°",
        err.to_degrees(),
        heading.to_degrees()
    );
}
