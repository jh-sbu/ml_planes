//! Serde round-trip coverage for the core
//! sim-state types replicated or carried in network commands.
//!
//! Gated on `net` (enabled by `--features server`) because the derives — and the
//! `Vec3`/`Quat` serde impls they rely on (`bevy/serialize`) — only exist there.

use bevy::math::{Quat, Vec3};
use ml_planes::controllers::{
    ControllerKind, ControllerTargets, ControllerTelemetry, L1Status, OrbitDirection, OrbitParams,
    RefuelDiagnostics, RefuelPhase, WingmanDiagnostics,
};
use ml_planes::plane::{ControlInputs, FlightState, PlaneId, PlaneIndex};
use ml_planes::training::SpawnSpec;

/// Round-trip via RON and assert byte-stability: serialize → deserialize →
/// serialize must reproduce the same encoding. RON is used (over JSON) because
/// it encodes `f32::INFINITY` as `inf` and parses it back — `FlightState`'s
/// `consumable_remaining` defaults to infinity. Re-serialization comparison avoids
/// needing `PartialEq` on the value types.
fn assert_ron_roundtrip<T>(value: &T)
where
    T: serde::Serialize + serde::de::DeserializeOwned,
{
    let s1 = ron::to_string(value).expect("serialize");
    let back: T = ron::from_str(&s1).expect("deserialize");
    let s2 = ron::to_string(&back).expect("re-serialize");
    assert_eq!(s1, s2, "round-trip changed the encoding");
}

#[test]
fn flight_state_roundtrips_finite() {
    let mut s = FlightState {
        position: Vec3::new(10.0, 1500.0, -30.0),
        velocity: Vec3::new(100.0, 1.0, -2.0),
        attitude: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
        angular_velocity: Vec3::new(0.1, -0.2, 0.05),
        consumable_remaining: 1234.5,
        ..Default::default()
    };
    s.update_air_data();
    assert_ron_roundtrip(&s);
}

#[test]
fn flight_state_roundtrips_infinite_default_fuel() {
    // The default `consumable_remaining` is `f32::INFINITY`; RON must preserve it.
    let s = FlightState::default();
    assert!(!s.consumable_remaining.is_finite());
    assert_ron_roundtrip(&s);
}

#[test]
fn control_inputs_roundtrips() {
    let inputs = ControlInputs {
        aileron: -0.25,
        elevator: 0.5,
        rudder: 0.1,
        throttle: 0.8,
    };
    assert_ron_roundtrip(&inputs);
}

#[test]
fn spawn_spec_roundtrips_all_some() {
    let spec = SpawnSpec {
        position: Some(Vec3::new(0.0, 500.0, 0.0)),
        velocity: Some(Vec3::new(100.0, 0.0, 0.0)),
        attitude: Some(Quat::from_rotation_y(0.3)),
        angular_velocity: Some(Vec3::new(0.0, 0.0, 0.1)),
        fuel_fraction: Some(0.5),
    };
    assert_ron_roundtrip(&spec);
}

#[test]
fn spawn_spec_roundtrips_all_none() {
    assert_ron_roundtrip(&SpawnSpec::default());
}

#[test]
fn plane_id_and_index_roundtrip() {
    assert_ron_roundtrip(&PlaneId(7));
    assert_ron_roundtrip(&PlaneIndex(3));
}

#[test]
fn controller_telemetry_roundtrips() {
    assert_ron_roundtrip(&ControllerTelemetry::None);
    assert_ron_roundtrip(&ControllerTelemetry::Ascent { complete: true });
    assert_ron_roundtrip(&ControllerTelemetry::Orbit {
        radial_error: -42.5,
    });
    assert_ron_roundtrip(&ControllerTelemetry::Wingman(WingmanDiagnostics {
        leader_found: true,
        pos_error: Vec3::new(1.0, -2.0, 3.0),
        pos_error_mag: 3.74,
        cross_track: 0.5,
        range_error: -1.5,
        altitude_error: -2.0,
    }));
    assert_ron_roundtrip(&ControllerTelemetry::Refueling(RefuelDiagnostics {
        tanker_found: true,
        phase: RefuelPhase::Precontact,
        pos_error: Vec3::new(0.5, -1.0, 0.0),
        pos_error_mag: 1.12,
        cross_track: 0.0,
        range_error: 0.5,
        altitude_error: -1.0,
        closure_rate: 1.8,
        phase_error: 24.0,
        breakaways: 2,
    }));
    assert_ron_roundtrip(&ControllerTelemetry::FlightPlan {
        leg_index: 1,
        leg_count: 3,
        status: L1Status::Orbit {
            center_x: 0.0,
            center_z: 3000.0,
            radius: 1000.0,
            radial_error: 12.5,
            direction: OrbitDirection::CounterClockwise,
            turns_done: 0.25,
            turns_total: Some(2.0),
        },
    });
    assert_ron_roundtrip(&ControllerTelemetry::FlightPlan {
        leg_index: 0,
        leg_count: 2,
        status: L1Status::Waypoint {
            x: 3000.0,
            z: 0.0,
            distance: 1200.0,
            capture_radius: 200.0,
            eta: 0.1,
            xtrack: -5.0,
        },
    });
}

/// The settable half of controller state — replicated separately from
/// `ControllerTelemetry` (read-only, derived) so unedited setpoints don't get
/// re-broadcast (and re-stomp a mid-drag client edit) every tick.
#[test]
fn controller_targets_roundtrips() {
    assert_ron_roundtrip(&ControllerTargets::None);
    assert_ron_roundtrip(&ControllerTargets::LevelHold {
        altitude: 1200.0,
        airspeed: 90.0,
    });
    assert_ron_roundtrip(&ControllerTargets::Ascent { altitude: 1500.0 });
    assert_ron_roundtrip(&ControllerTargets::HeadingHold {
        heading: 0.5,
        altitude: 1000.0,
        airspeed: 80.0,
    });
    assert_ron_roundtrip(&ControllerTargets::Orbit(OrbitParams {
        center_x: 10.0,
        center_z: -20.0,
        target_radius: 3000.0,
        target_altitude: 800.0,
        target_airspeed: 100.0,
        direction: OrbitDirection::Clockwise,
    }));
    assert_ron_roundtrip(&ControllerTargets::Wingman { leader: PlaneId(4) });
    assert_ron_roundtrip(&ControllerTargets::Refueling { tanker: PlaneId(5) });
}

#[test]
fn controller_kind_roundtrips() {
    for kind in [
        ControllerKind::Manual,
        ControllerKind::LevelHold,
        ControllerKind::Orbit,
        ControllerKind::FlightPlan,
        ControllerKind::Wingman,
        // Appended (protocol v4) — round-trip it like every other variant even
        // though the variant only *builds* on inference builds; Serialize/Deserialize
        // are unconditional on the enum.
        ControllerKind::RlHeadingHold,
        // Appended (protocol v5).
        ControllerKind::Refueling,
    ] {
        assert_ron_roundtrip(&kind);
    }
}
