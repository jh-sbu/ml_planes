//! Phase 0 (client/server foundations): serde round-trip coverage for the core
//! sim-state types that later phases replicate or carry in network commands.
//!
//! Gated on `net` (enabled by `--features server`) because the derives — and the
//! `Vec3`/`Quat` serde impls they rely on (`bevy/serialize`) — only exist there.
#![cfg(feature = "net")]

use bevy::math::{Quat, Vec3};
use ml_planes::controllers::ControllerKind;
use ml_planes::plane::{ControlInputs, FlightState, PlaneId, PlaneIndex};
use ml_planes::training::SpawnSpec;

/// Round-trip via RON and assert byte-stability: serialize → deserialize →
/// serialize must reproduce the original encoding. RON is used (over JSON) because
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
fn controller_kind_roundtrips() {
    for kind in [
        ControllerKind::Manual,
        ControllerKind::LevelHold,
        ControllerKind::Orbit,
        ControllerKind::FlightPlan,
        ControllerKind::Wingman,
    ] {
        assert_ron_roundtrip(&kind);
    }
}
