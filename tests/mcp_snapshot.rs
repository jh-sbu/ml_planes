//! MCP Phase 1: the snapshot mirror (read path).
//!
//! Transport-free headless test mirroring `tests/server_sim.rs`: register
//! `RepliconPlugins` + `NetProtocolPlugin` + `McpBridgePlugin` (no renet transport, no
//! `start_renet_client`), spawn a plane entity carrying the replicated components, run a
//! couple of updates, and assert the shared `SimSnapshot` mirrors it. The `collect_snapshot`
//! system reads whatever entities exist regardless of connection state, so no socket is
//! needed.
//!
//! Gated on `mcp` (`cargo test --features mcp`).
#![cfg(feature = "mcp")]

mod common;

use bevy::prelude::*;
use bevy::state::app::StatesPlugin;
use bevy_replicon::prelude::RepliconPlugins;

use common::build_headless_app_with;
use ml_planes::controllers::{ControllerKind, ControllerTelemetry, SelectedTuningProfile};
use ml_planes::mcp::{McpBridgePlugin, SnapshotHandle};
use ml_planes::net::{ConnectTarget, NetProtocolPlugin, PROTOCOL_ID};
use ml_planes::plane::{ControlInputs, FlightState, PlaneId, PlaneIndex};

/// Build a headless app with the MCP snapshot bridge installed (no transport). Returns the
/// app and a clone of the shared `SnapshotHandle` for assertions.
fn build_bridge_app() -> (App, SnapshotHandle) {
    let handle = SnapshotHandle::new();
    let handle_for_app = handle.clone();
    let app = build_headless_app_with(move |app| {
        app.add_plugins(StatesPlugin)
            .add_plugins(RepliconPlugins)
            .add_plugins(NetProtocolPlugin)
            .add_plugins(McpBridgePlugin::new(handle_for_app))
            .insert_resource(ConnectTarget("127.0.0.1:5555".parse().unwrap()));
    });
    (app, handle)
}

#[test]
fn snapshot_mirrors_a_spawned_plane() {
    let (mut app, handle) = build_bridge_app();

    app.world_mut().spawn((
        PlaneId(42),
        PlaneIndex(0),
        FlightState {
            position: Vec3::new(100.0, 1500.0, -50.0),
            airspeed: 120.0,
            altitude: 1500.0,
            consumable_remaining: 1800.0,
            ..Default::default()
        },
        ControlInputs {
            throttle: 0.7,
            ..Default::default()
        },
        ControllerKind::Orbit,
        SelectedTuningProfile("default".to_string()),
        ControllerTelemetry::Orbit { radial_error: 8.0 },
    ));

    // Two updates so the Update-schedule `collect_snapshot` runs against the spawned entity.
    app.update();
    app.update();

    let snap = handle.0.read().unwrap();
    assert_eq!(snap.protocol_id, PROTOCOL_ID);
    assert_eq!(snap.server_addr, "127.0.0.1:5555");
    // No renet transport, so the client never connects.
    assert!(!snap.connected, "no transport ⇒ not connected");
    // Phase 1 never calls set_sim_speed.
    assert_eq!(snap.requested_sim_speed, None);

    assert_eq!(snap.planes.len(), 1, "the spawned plane should be mirrored");
    let plane = &snap.planes[0];
    assert_eq!(plane.plane_id, 42);
    assert_eq!(plane.controller_kind, ControllerKind::Orbit);
    assert_eq!(plane.position, Vec3::new(100.0, 1500.0, -50.0));
    assert_eq!(plane.airspeed, 120.0);
    assert_eq!(plane.consumable_remaining, 1800.0);
    assert_eq!(plane.control_inputs.throttle, 0.7);
    assert_eq!(plane.tuning_profile.as_deref(), Some("default"));
    assert_eq!(
        plane.telemetry,
        ControllerTelemetry::Orbit { radial_error: 8.0 }
    );
}

#[test]
fn snapshot_is_empty_with_no_planes() {
    let (mut app, handle) = build_bridge_app();

    app.update();
    app.update();

    let snap = handle.0.read().unwrap();
    assert!(snap.planes.is_empty(), "no plane entities ⇒ empty mirror");
    assert!(!snap.connected);
}
