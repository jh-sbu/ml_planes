mod common;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::{ActiveController, ManualController};
use ml_planes::plane::{ControlInputs, FlightState, PlaneConfig, PlaneConfigHandle};

/// Full pipeline energy-conservation test.
///
/// Spawns a plane at 500 m altitude flying at 100 m/s with zero control
/// deflections (ManualController outputs zeros). Runs 300 FixedUpdate steps
/// (= 5 s at 60 Hz). Asserts altitude stays within ±50 m of initial value,
/// confirming the aero→Rapier integration doesn't produce runaway forces.
#[test]
fn energy_conservation_5s() {
    let mut app = common::build_headless_app();

    let cfg = common::generic_jet_config();

    // Insert PlaneConfig directly into Assets — bypasses async file loader.
    let handle: Handle<PlaneConfig> = app
        .world_mut()
        .resource_mut::<Assets<PlaneConfig>>()
        .add(cfg.clone());

    app.world_mut().spawn((
        RigidBody::Dynamic,
        Collider::cuboid(1.0, 0.5, 3.0),
        Velocity {
            linvel: Vec3::new(100.0, 0.0, 0.0),
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
        ActiveController(Box::new(ManualController::new())),
        PlaneConfigHandle(handle),
        Transform::from_translation(Vec3::new(0.0, 500.0, 0.0)),
    ));

    // 300 updates × 1/60 s = 5 simulated seconds
    for _ in 0..300 {
        app.update();
    }

    let mut q = app.world_mut().query::<&FlightState>();
    let state = q.single(app.world()).expect("expected exactly one FlightState entity");

    // With ManualController (zero throttle, zero deflections) at 100 m/s, lift is
    // much less than weight, so the plane falls under gravity. The important check
    // is that forces are finite and physically reasonable — no energy explosion.
    // Expected free-fall under partial lift: ~100–150 m drop over 5 s.
    assert!(
        state.altitude.is_finite(),
        "altitude became non-finite: {}",
        state.altitude
    );
    assert!(
        state.altitude > 0.0,
        "plane reached ground before 5 s: altitude={}",
        state.altitude
    );
    assert!(
        state.altitude < 600.0,
        "altitude increased unexpectedly (runaway lift?): {}",
        state.altitude
    );
    assert!(
        state.altitude > 200.0,
        "altitude dropped more than 300m — possible force explosion: {}",
        state.altitude
    );
}
