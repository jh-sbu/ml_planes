//! Controller telemetry accessor — the read-only status each controller publishes
//! via `FlightController::telemetry()`, snapshotted server-side into the replicated
//! `ControllerTelemetry` component so the networked client HUD can display it.

use std::f32::consts::FRAC_PI_2;

use bevy::math::{Quat, Vec3};

use ml_planes::controllers::{
    AscentController, ControllerTelemetry, FlightController, FlightPlan, FlightPlanLeg,
    FormationOffset, L1Controller, L1Status, OrbitController, OrbitDirection, WingmanController,
};
use ml_planes::plane::{ControlInputs, FlightState, PlaneId};

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

#[test]
fn default_telemetry_is_none() {
    assert_eq!(ControllerTelemetry::default(), ControllerTelemetry::None);
}

#[test]
fn orbit_telemetry_reports_radial_error() {
    // Plane 1200 m east of an orbit centered at the origin with a 1000 m radius
    // ⇒ radial error = 1200 − 1000 = 200 m (outside).
    let state = level_state(Vec3::new(1200.0, 1000.0, 0.0), 100.0);
    let mut orbit = OrbitController::from_state(&state, &ControlInputs::default());
    orbit.center_x = 0.0;
    orbit.center_z = 0.0;
    orbit.target_radius = 1000.0;

    match orbit.telemetry(&state) {
        ControllerTelemetry::Orbit { radial_error } => {
            assert!(
                (radial_error - 200.0).abs() < 1e-3,
                "radial_error={radial_error}"
            );
        }
        other => panic!("expected Orbit telemetry, got {other:?}"),
    }
}

#[test]
fn flight_plan_telemetry_reports_leg_and_status() {
    let plan = FlightPlan {
        legs: vec![
            FlightPlanLeg::Waypoint {
                x: 3000.0,
                z: 0.0,
                altitude: 1000.0,
                airspeed: 100.0,
                capture_radius: 200.0,
            },
            FlightPlanLeg::Orbit {
                center_x: 0.0,
                center_z: 3000.0,
                radius: 1000.0,
                altitude: 1000.0,
                airspeed: 100.0,
                direction: OrbitDirection::CounterClockwise,
                turns: None,
            },
        ],
        ..Default::default()
    };
    let leg_count = plan.legs.len();

    let state = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
    let mut l1 = L1Controller::from_plan(&state, plan, &ControlInputs::default());
    // Drive one tick so `status` is populated for the active leg.
    l1.update(
        &state,
        &ml_planes::plane::ControllerContext::empty_for(PlaneId::TEST),
        1.0 / 64.0,
    );

    match l1.telemetry(&state) {
        ControllerTelemetry::FlightPlan {
            leg_index,
            leg_count: n,
            status,
        } => {
            assert_eq!(leg_index, 0);
            assert_eq!(n, leg_count);
            assert!(
                matches!(status, L1Status::Waypoint { .. }),
                "expected a Waypoint status on leg 0, got {status:?}"
            );
        }
        other => panic!("expected FlightPlan telemetry, got {other:?}"),
    }
}

#[test]
fn wingman_telemetry_reports_diagnostics() {
    let leader = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
    let own = level_state(Vec3::new(-20.0, 1000.0, 15.0), 100.0);
    let wingman = WingmanController::new(PlaneId(1), &leader, &own, FormationOffset::default());

    match wingman.telemetry(&own) {
        // Before any update with a populated context the leader is "not found".
        ControllerTelemetry::Wingman(diag) => {
            assert!(!diag.leader_found);
        }
        other => panic!("expected Wingman telemetry, got {other:?}"),
    }
}

#[test]
fn ascent_telemetry_reflects_complete_latch() {
    let below = level_state(Vec3::new(0.0, 500.0, 0.0), 100.0);
    let mut ascent = AscentController::new(&below, 1000.0);
    assert_eq!(
        ascent.telemetry(&below),
        ControllerTelemetry::Ascent { complete: false }
    );

    // Climb into the threshold band → latch complete.
    let at_target = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
    ascent.update(
        &at_target,
        &ml_planes::plane::ControllerContext::empty_for(PlaneId::TEST),
        1.0 / 64.0,
    );
    assert_eq!(
        ascent.telemetry(&at_target),
        ControllerTelemetry::Ascent { complete: true }
    );
}
