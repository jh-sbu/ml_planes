//! Phase 1 (client/server): the shared network protocol module.
//!
//! Covers Milestone 1 — every client→server command type round-trips through serde,
//! and `NetProtocolPlugin` builds in a headless app without panicking.
//!
//! Gated on `net` (enabled by `--features server`); the command types and the
//! replicon plugins only exist there.
#![cfg(feature = "net")]

use bevy::math::{Quat, Vec3};
use bevy::prelude::*;
use bevy::state::app::StatesPlugin;
use bevy_replicon::prelude::RepliconPlugins;

use ml_planes::controllers::ControllerKind;
use ml_planes::net::{
    ManualInputCommand, NetProtocolPlugin, RemovePlaneNetCommand, SetSimSpeedCommand,
    SetTuningProfileCommand, SpawnPlaneNetCommand, SwitchControllerCommand,
};
use ml_planes::plane::{ControlInputs, PlaneId};
use ml_planes::sim_speed::SimSpeed;
use ml_planes::training::SpawnSpec;

/// Round-trip via RON and assert byte-stability (serialize → deserialize →
/// serialize reproduces the original encoding), mirroring `tests/net_serde.rs`.
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
fn switch_controller_command_roundtrips() {
    assert_ron_roundtrip(&SwitchControllerCommand {
        plane: PlaneId(3),
        kind: ControllerKind::Orbit,
    });
}

#[test]
fn set_tuning_profile_command_roundtrips() {
    assert_ron_roundtrip(&SetTuningProfileCommand {
        plane: PlaneId(1),
        profile: "aggressive".to_string(),
    });
}

#[cfg(feature = "inference")]
#[test]
fn set_model_command_roundtrips() {
    use ml_planes::net::SetModelCommand;
    assert_ron_roundtrip(&SetModelCommand {
        plane: PlaneId(2),
        model_stem: "fuel_smoke_orbit".to_string(),
    });
}

#[test]
fn manual_input_command_roundtrips() {
    assert_ron_roundtrip(&ManualInputCommand {
        plane: PlaneId(5),
        inputs: ControlInputs {
            aileron: -0.25,
            elevator: 0.5,
            rudder: 0.1,
            throttle: 0.8,
        },
    });
}

#[test]
fn spawn_plane_net_command_roundtrips() {
    assert_ron_roundtrip(&SpawnPlaneNetCommand {
        config_path: "assets/planes/generic_jet.plane.ron".to_string(),
        kind: ControllerKind::LevelHold,
        spec: SpawnSpec {
            position: Some(Vec3::new(0.0, 500.0, 0.0)),
            velocity: Some(Vec3::new(100.0, 0.0, 0.0)),
            attitude: Some(Quat::from_rotation_y(0.3)),
            angular_velocity: Some(Vec3::ZERO),
            fuel_fraction: Some(0.5),
        },
    });
}

#[test]
fn remove_plane_net_command_roundtrips() {
    assert_ron_roundtrip(&RemovePlaneNetCommand { plane: PlaneId(9) });
}

#[test]
fn set_sim_speed_command_roundtrips() {
    for speed in [SimSpeed::Paused, SimSpeed::X1, SimSpeed::X5, SimSpeed::X10] {
        assert_ron_roundtrip(&SetSimSpeedCommand { speed });
    }
}

/// The protocol plugin must register its replication rules and command events into a
/// headless replicon app without panicking. `RepliconPlugins` must precede it so the
/// registries exist; `StatesPlugin` is required with `MinimalPlugins`.
#[test]
fn net_protocol_plugin_builds_headless() {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        StatesPlugin,
        RepliconPlugins,
        NetProtocolPlugin,
    ));
    app.finish();
    app.update();
}
