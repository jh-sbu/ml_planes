//! Integration tests for the L1 flight-plan controller and its `.plan.ron` asset.

mod common;

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::{
    ActiveController, FlightPlan, FlightPlanLeg, L1Controller, L1Phase, OrbitDirection,
};
use ml_planes::plane::{
    ControlInputs, FlightPlanHandle, FlightState, PlaneConfig, PlaneConfigHandle, PlaneId,
};

/// Spawn a plane at `pos` flying `velocity` (heading +X level orientation),
/// driven by an `L1Controller` following `plan`.
fn spawn_l1_plane(app: &mut App, pos: Vec3, velocity: Vec3, plan: FlightPlan) {
    let cfg = common::generic_jet_config();
    let handle: Handle<PlaneConfig> = app
        .world_mut()
        .resource_mut::<Assets<PlaneConfig>>()
        .add(cfg.clone());

    let attitude = Quat::from_rotation_x(-FRAC_PI_2);

    // Initial state for bumpless controller construction.
    let mut init = FlightState {
        position: pos,
        velocity,
        attitude,
        ..Default::default()
    };
    init.update_air_data();
    let controller = L1Controller::from_plan(&init, plan, &ControlInputs::default());

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

fn read_state(app: &mut App) -> FlightState {
    let mut q = app.world_mut().query::<&FlightState>();
    q.single(app.world()).expect("one FlightState").clone()
}

/// The Bevy asset loader can read `assets/plans/patrol.plan.ron` into a
/// populated `FlightPlan` asset through a headless app.
#[test]
fn loads_patrol_plan_asset() {
    let mut app = common::build_headless_app();

    let handle: Handle<FlightPlan> = app
        .world()
        .resource::<AssetServer>()
        .load("plans/patrol.plan.ron");
    app.world_mut().spawn(FlightPlanHandle(handle.clone()));

    // Pump the app until the async load completes (bounded so a missing/broken
    // asset fails fast rather than hanging).
    let mut loaded = None;
    for _ in 0..2000 {
        app.update();
        if let Some(plan) = app.world().resource::<Assets<FlightPlan>>().get(&handle) {
            loaded = Some(plan.clone());
            break;
        }
    }

    let plan = loaded.expect("patrol.plan.ron should load within 2000 updates");
    assert_eq!(plan.legs.len(), 5, "patrol plan should have 5 legs");
    assert!(plan.l1_period > 0.0 && plan.l1_damping > 0.0);
}

/// End-to-end: an L1 plane flies a 2-waypoint dogleg then settles into a
/// hold-forever orbit, visiting each waypoint and never crashing.
#[test]
fn l1_flies_waypoint_sequence_then_loiters() {
    let wp1 = Vec2::new(3000.0, 0.0);
    let wp2 = Vec2::new(5000.0, 2000.0);

    let plan = FlightPlan {
        l1_period: 20.0,
        l1_damping: 0.75,
        legs: vec![
            FlightPlanLeg::Waypoint {
                x: wp1.x,
                z: wp1.y,
                altitude: 1000.0,
                airspeed: 100.0,
                capture_radius: 300.0,
            },
            FlightPlanLeg::Waypoint {
                x: wp2.x,
                z: wp2.y,
                altitude: 1000.0,
                airspeed: 100.0,
                capture_radius: 350.0,
            },
            FlightPlanLeg::Orbit {
                center_x: wp2.x,
                center_z: wp2.y,
                radius: 1000.0,
                altitude: 1000.0,
                airspeed: 100.0,
                direction: OrbitDirection::CounterClockwise,
                turns: None,
            },
        ],
    };

    let mut app = common::build_headless_app();
    spawn_l1_plane(
        &mut app,
        Vec3::new(0.0, 1000.0, 0.0),
        Vec3::new(100.0, 0.0, 0.0),
        plan,
    );

    let mut min_wp1 = f32::INFINITY;
    let mut min_wp2 = f32::INFINITY;

    // ~110 s of flight (7000 fixed ticks at 1/64 s). The first ~64 frames are a
    // warmup: FixedUpdate has not yet synced FlightState off its zero default.
    for frame in 0..7000 {
        app.update();
        let s = read_state(&mut app);
        let p = Vec2::new(s.position.x, s.position.z);
        min_wp1 = min_wp1.min(p.distance(wp1));
        min_wp2 = min_wp2.min(p.distance(wp2));
        if frame >= 64 {
            assert!(
                s.altitude.is_finite() && s.altitude > 500.0,
                "plane lost altitude (crashed) at frame {frame}: {}",
                s.altitude
            );
        }
    }

    assert!(
        min_wp1 < 300.0,
        "should have captured WP1 (min dist {min_wp1:.0} m)"
    );
    assert!(
        min_wp2 < 350.0,
        "should have captured WP2 (min dist {min_wp2:.0} m)"
    );

    // The controller should have sequenced onto the final hold-forever orbit.
    let mut q = app.world_mut().query::<&mut ActiveController>();
    let mut ac = q.single_mut(app.world_mut()).expect("one controller");
    let l1 =
        ac.0.as_any_mut()
            .downcast_mut::<L1Controller>()
            .expect("controller should be an L1Controller");
    assert_eq!(l1.leg_index, 2, "should be on the final orbit leg");
    assert_eq!(
        l1.phase,
        L1Phase::Following,
        "hold-forever orbit never finishes"
    );

    // And it should be loitering near WP2 (within ~1.5× orbit radius).
    let s = read_state(&mut app);
    let final_dist = Vec2::new(s.position.x, s.position.z).distance(wp2);
    assert!(
        final_dist < 1500.0,
        "should be loitering near WP2, dist {final_dist:.0} m"
    );
}
