//! Controller targets accessor — the *settable* half of controller state, published via
//! `FlightController::targets()` and applied via `FlightController::apply_targets()`.
//! Snapshotted server-side into the replicated `ControllerTargets` component so the
//! networked client HUD can display and edit it (see `src/controllers/targets.rs`).

use std::f32::consts::FRAC_PI_2;

use bevy::math::{Quat, Vec3};

use ml_planes::controllers::{
    ActiveController, AscentController, ControllerTargets, FlightController, FlightPlan,
    FlightPlanLeg, FormationOffset, HeadingHoldController, L1Controller, LevelHoldController,
    ManualController, OrbitController, OrbitDirection, OrbitParams, RefuelConfig, RefuelController,
    WingmanController,
};
use ml_planes::plane::{ControlInputs, FlightState, PlaneId, PHYSICS_DT};

/// Level flight state heading +X at `altitude`/`airspeed`.
fn level_state(position: Vec3, airspeed: f32) -> FlightState {
    let mut s = FlightState {
        position,
        velocity: Vec3::new(airspeed, 0.0, 0.0),
        attitude: Quat::from_rotation_x(-FRAC_PI_2),
        ..Default::default()
    };
    s.update_air_data();
    s
}

// ---------------------------------------------------------------------------
// targets() read accessors
// ---------------------------------------------------------------------------

#[test]
fn default_targets_is_none() {
    assert_eq!(ControllerTargets::default(), ControllerTargets::None);
}

#[test]
fn level_hold_targets_reports_setpoints() {
    let ctrl = LevelHoldController::new(1200.0, 90.0);
    assert_eq!(
        ctrl.targets(),
        ControllerTargets::LevelHold {
            altitude: 1200.0,
            airspeed: 90.0,
        }
    );
}

#[test]
fn ascent_targets_reports_altitude() {
    let state = level_state(Vec3::new(0.0, 500.0, 0.0), 100.0);
    let ctrl = AscentController::new(&state, 1500.0);
    assert_eq!(
        ctrl.targets(),
        ControllerTargets::Ascent { altitude: 1500.0 }
    );
}

#[test]
fn heading_hold_targets_reports_heading_altitude_airspeed() {
    let state = level_state(Vec3::new(0.0, 1000.0, 0.0), 80.0);
    let ctrl = HeadingHoldController::new(&state, 0.5);
    assert_eq!(
        ctrl.targets(),
        ControllerTargets::HeadingHold {
            heading: 0.5,
            altitude: ctrl.inner.target_altitude,
            airspeed: ctrl.inner.target_airspeed,
        }
    );
}

#[test]
fn orbit_targets_reports_geometry() {
    let state = level_state(Vec3::new(0.0, 1000.0, 1000.0), 100.0);
    let mut ctrl = OrbitController::from_state(&state, &ControlInputs::default());
    ctrl.center_x = 10.0;
    ctrl.center_z = 20.0;
    ctrl.target_radius = 2000.0;
    ctrl.target_altitude = 1234.0;
    ctrl.target_airspeed = 88.0;
    ctrl.direction = OrbitDirection::Clockwise;

    assert_eq!(
        ctrl.targets(),
        ControllerTargets::Orbit(OrbitParams {
            center_x: 10.0,
            center_z: 20.0,
            target_radius: 2000.0,
            target_altitude: 1234.0,
            target_airspeed: 88.0,
            direction: OrbitDirection::Clockwise,
        })
    );
}

#[test]
fn wingman_targets_reports_leader() {
    let leader = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
    let own = level_state(Vec3::new(-20.0, 1000.0, 15.0), 100.0);
    let ctrl = WingmanController::new(PlaneId(7), &leader, &own, FormationOffset::default());
    assert_eq!(
        ctrl.targets(),
        ControllerTargets::Wingman { leader: PlaneId(7) }
    );
}

#[test]
fn manual_and_flight_plan_targets_are_none() {
    assert_eq!(ManualController::new().targets(), ControllerTargets::None);

    let plan = FlightPlan {
        legs: vec![FlightPlanLeg::Waypoint {
            x: 3000.0,
            z: 0.0,
            altitude: 1000.0,
            airspeed: 100.0,
            capture_radius: 200.0,
        }],
        ..Default::default()
    };
    let state = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
    let l1 = L1Controller::from_plan(&state, plan, &ControlInputs::default());
    assert_eq!(l1.targets(), ControllerTargets::None);
}

#[test]
fn targets_are_readable_through_boxed_controller() {
    let ctrl = LevelHoldController::new(900.0, 95.0);
    let boxed = ActiveController(Box::new(ctrl));
    assert_eq!(
        boxed.0.targets(),
        ControllerTargets::LevelHold {
            altitude: 900.0,
            airspeed: 95.0,
        }
    );
}

// ---------------------------------------------------------------------------
// apply_targets() write accessors
// ---------------------------------------------------------------------------

#[test]
fn apply_targets_sets_level_hold_setpoints() {
    let mut ctrl = LevelHoldController::new(1000.0, 80.0);
    let state = level_state(Vec3::new(0.0, 1000.0, 0.0), 80.0);
    ctrl.apply_targets(
        &ControllerTargets::LevelHold {
            altitude: 1600.0,
            airspeed: 110.0,
        },
        &state,
    );
    assert_eq!(ctrl.target_altitude, 1600.0);
    assert_eq!(ctrl.target_airspeed, 110.0);
}

#[test]
fn apply_targets_on_ascent_clears_complete_when_altitude_changes() {
    let at_target = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
    let mut ctrl = AscentController::new(&at_target, 1000.0);
    ctrl.update(
        &at_target,
        &ml_planes::plane::ControllerContext::empty_for(PlaneId::TEST),
        PHYSICS_DT,
    );
    assert!(ctrl.complete, "precondition: latched complete");

    ctrl.apply_targets(&ControllerTargets::Ascent { altitude: 2000.0 }, &at_target);
    assert_eq!(ctrl.target_altitude, 2000.0);
    assert!(
        !ctrl.complete,
        "changing target altitude should clear the complete latch"
    );
}

#[test]
fn apply_targets_on_ascent_keeps_complete_when_altitude_unchanged() {
    let at_target = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
    let mut ctrl = AscentController::new(&at_target, 1000.0);
    ctrl.update(
        &at_target,
        &ml_planes::plane::ControllerContext::empty_for(PlaneId::TEST),
        PHYSICS_DT,
    );
    assert!(ctrl.complete, "precondition: latched complete");

    ctrl.apply_targets(&ControllerTargets::Ascent { altitude: 1000.0 }, &at_target);
    assert!(
        ctrl.complete,
        "re-applying the same target altitude must not clear the latch"
    );
}

#[test]
fn apply_targets_sets_heading_hold_targets() {
    let state = level_state(Vec3::new(0.0, 1000.0, 0.0), 80.0);
    let mut ctrl = HeadingHoldController::new(&state, 0.0);
    ctrl.apply_targets(
        &ControllerTargets::HeadingHold {
            heading: 1.2,
            altitude: 1500.0,
            airspeed: 95.0,
        },
        &state,
    );
    assert_eq!(ctrl.target_heading, 1.2);
    assert_eq!(ctrl.inner.target_altitude, 1500.0);
    assert_eq!(ctrl.inner.target_airspeed, 95.0);
}

#[test]
fn apply_targets_orbit_sets_geometry_and_inner_targets() {
    let state = level_state(Vec3::new(0.0, 1000.0, 1000.0), 100.0);
    let mut ctrl = OrbitController::from_state(&state, &ControlInputs::default());
    let params = OrbitParams {
        center_x: 50.0,
        center_z: 60.0,
        target_radius: 3000.0,
        target_altitude: 1800.0,
        target_airspeed: 120.0,
        direction: OrbitDirection::CounterClockwise,
    };
    ctrl.apply_targets(&ControllerTargets::Orbit(params), &state);
    assert_eq!(ctrl.center_x, 50.0);
    assert_eq!(ctrl.center_z, 60.0);
    assert_eq!(ctrl.target_radius, 3000.0);
    assert_eq!(ctrl.target_altitude, 1800.0);
    assert_eq!(ctrl.target_airspeed, 120.0);
    assert_eq!(ctrl.inner.target_altitude, 1800.0);
    assert_eq!(ctrl.inner.target_airspeed, 120.0);
}

#[test]
fn apply_targets_orbit_resets_pids_on_direction_flip() {
    // Warm one controller's radial/heading PIDs with sustained error, then flip
    // direction via apply_targets. Its next update() output must match a controller
    // freshly built with the flipped direction from the same state (proving the
    // PIDs were actually reset, not just carrying stale integral/derivative state).
    let state = level_state(Vec3::new(0.0, 1000.0, 1300.0), 100.0);
    let mut warmed = OrbitController::from_state(&state, &ControlInputs::default());
    warmed.center_x = 0.0;
    warmed.center_z = 0.0;
    warmed.target_radius = 1000.0;
    warmed.target_altitude = 1000.0;
    warmed.target_airspeed = 100.0;
    warmed.direction = OrbitDirection::CounterClockwise;
    let ctx = ml_planes::plane::ControllerContext::empty_for(PlaneId::TEST);
    for _ in 0..30 {
        warmed.update(&state, &ctx, 1.0 / 60.0);
    }

    let flipped_params = OrbitParams {
        center_x: 0.0,
        center_z: 0.0,
        target_radius: 1000.0,
        target_altitude: 1000.0,
        target_airspeed: 100.0,
        direction: OrbitDirection::Clockwise,
    };
    warmed.apply_targets(&ControllerTargets::Orbit(flipped_params), &state);

    let mut fresh = OrbitController::from_state(&state, &ControlInputs::default());
    fresh.center_x = 0.0;
    fresh.center_z = 0.0;
    fresh.target_radius = 1000.0;
    fresh.target_altitude = 1000.0;
    fresh.target_airspeed = 100.0;
    fresh.direction = OrbitDirection::Clockwise;
    fresh.radial_pid.reset();
    fresh.heading_pid.reset();

    let warmed_out = warmed.update(&state, &ctx, 1.0 / 60.0);
    let fresh_out = fresh.update(&state, &ctx, 1.0 / 60.0);
    assert!(
        (warmed_out.aileron - fresh_out.aileron).abs() < 1e-4,
        "warmed={} fresh={} — PIDs were not reset on direction flip",
        warmed_out.aileron,
        fresh_out.aileron
    );
}

#[test]
fn apply_targets_sets_wingman_leader() {
    let leader = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
    let own = level_state(Vec3::new(-20.0, 1000.0, 15.0), 100.0);
    let mut ctrl = WingmanController::new(PlaneId(1), &leader, &own, FormationOffset::default());
    ctrl.apply_targets(&ControllerTargets::Wingman { leader: PlaneId(9) }, &own);
    assert_eq!(ctrl.leader_id, PlaneId(9));
}

#[test]
fn apply_targets_ignores_mismatched_variant() {
    let mut ctrl = LevelHoldController::new(1000.0, 80.0);
    let state = level_state(Vec3::new(0.0, 1000.0, 0.0), 80.0);
    ctrl.apply_targets(
        &ControllerTargets::Orbit(OrbitParams {
            center_x: 1.0,
            center_z: 2.0,
            target_radius: 3.0,
            target_altitude: 4.0,
            target_airspeed: 5.0,
            direction: OrbitDirection::Clockwise,
        }),
        &state,
    );
    assert_eq!(
        ctrl.target_altitude, 1000.0,
        "mismatched variant must be a no-op"
    );
    assert_eq!(
        ctrl.target_airspeed, 80.0,
        "mismatched variant must be a no-op"
    );
}

#[test]
fn apply_targets_roundtrips_through_targets() {
    let state = level_state(Vec3::new(0.0, 1000.0, 1000.0), 100.0);

    let mut lh = LevelHoldController::new(1000.0, 80.0);
    let t = ControllerTargets::LevelHold {
        altitude: 1234.0,
        airspeed: 77.0,
    };
    lh.apply_targets(&t, &state);
    assert_eq!(lh.targets(), t);

    let mut ascent = AscentController::new(&state, 1000.0);
    let t = ControllerTargets::Ascent { altitude: 2500.0 };
    ascent.apply_targets(&t, &state);
    assert_eq!(ascent.targets(), t);

    let mut hh = HeadingHoldController::new(&state, 0.0);
    let t = ControllerTargets::HeadingHold {
        heading: -0.7,
        altitude: 1300.0,
        airspeed: 91.0,
    };
    hh.apply_targets(&t, &state);
    assert_eq!(hh.targets(), t);

    let mut orbit = OrbitController::from_state(&state, &ControlInputs::default());
    let t = ControllerTargets::Orbit(OrbitParams {
        center_x: 5.0,
        center_z: 6.0,
        target_radius: 4000.0,
        target_altitude: 1900.0,
        target_airspeed: 130.0,
        direction: OrbitDirection::Clockwise,
    });
    orbit.apply_targets(&t, &state);
    assert_eq!(orbit.targets(), t);

    let leader = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
    let mut wingman =
        WingmanController::new(PlaneId(1), &leader, &state, FormationOffset::default());
    let t = ControllerTargets::Wingman { leader: PlaneId(3) };
    wingman.apply_targets(&t, &state);
    assert_eq!(wingman.targets(), t);
}

#[test]
fn apply_targets_through_boxed_controller() {
    let state = level_state(Vec3::new(0.0, 1000.0, 0.0), 80.0);
    let mut boxed = ActiveController(Box::new(LevelHoldController::new(1000.0, 80.0)));
    boxed.0.apply_targets(
        &ControllerTargets::LevelHold {
            altitude: 1700.0,
            airspeed: 105.0,
        },
        &state,
    );
    assert_eq!(
        boxed.0.targets(),
        ControllerTargets::LevelHold {
            altitude: 1700.0,
            airspeed: 105.0,
        }
    );
}

/// A refueler publishes its tanker through its own variant, not `Wingman`'s — the HUD
/// label and the MCP docs differ, and a shared variant would let a wingman-shaped edit
/// land on a refueler.
#[test]
fn refuel_targets_reports_tanker() {
    let tanker = level_state(Vec3::new(0.0, 2000.0, 0.0), 130.0);
    let own = level_state(Vec3::new(-150.0, 1970.0, 0.0), 130.0);
    let ctrl = RefuelController::new(PlaneId(7), &tanker, &own, RefuelConfig::default());
    assert_eq!(
        ctrl.targets(),
        ControllerTargets::Refueling { tanker: PlaneId(7) }
    );
}

#[test]
fn apply_targets_sets_refuel_tanker_and_ignores_mismatches() {
    let tanker = level_state(Vec3::new(0.0, 2000.0, 0.0), 130.0);
    let own = level_state(Vec3::new(-150.0, 1970.0, 0.0), 130.0);
    let mut ctrl = RefuelController::new(PlaneId(1), &tanker, &own, RefuelConfig::default());

    ctrl.apply_targets(&ControllerTargets::Refueling { tanker: PlaneId(9) }, &own);
    assert_eq!(ctrl.tanker_id, PlaneId(9));
    assert_eq!(
        ctrl.targets(),
        ControllerTargets::Refueling { tanker: PlaneId(9) }
    );

    // A stale wingman-shaped command must be a no-op, not a silent retarget.
    ctrl.apply_targets(&ControllerTargets::Wingman { leader: PlaneId(3) }, &own);
    assert_eq!(ctrl.tanker_id, PlaneId(9));
}
