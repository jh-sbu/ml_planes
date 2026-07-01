//! MCP Phase 3: the command bridge (write path).
//!
//! Transport-free headless test mirroring `tests/mcp_snapshot.rs`: register
//! `RepliconPlugins` + `NetProtocolPlugin` + `McpBridgePlugin` (no renet transport), push
//! `ControlRequest`s onto the command channel, run an update, and assert
//! `drain_control_requests` consumed them (and did its `SetSimSpeed` bookkeeping). Genuine
//! client→server delivery of the resulting `client_trigger` is covered server-side by
//! `tests/server_sim.rs` and by the live-server Milestone 3 check.
//!
//! Gated on `mcp` (`cargo test --features mcp`).
#![cfg(feature = "mcp")]

mod common;

use bevy::prelude::*;
use bevy::state::app::StatesPlugin;
use bevy_replicon::prelude::RepliconPlugins;

use common::build_headless_app_with;
use ml_planes::mcp::{
    control_channel, ControlRequest, ControlSender, McpBridgePlugin, RequestedSimSpeed,
    SnapshotHandle,
};
use ml_planes::net::{ConnectTarget, NetProtocolPlugin};
use ml_planes::plane::PlaneId;
use ml_planes::sim_speed::SimSpeed;

/// Build the bridge app (no transport) and return it plus the sender end so the test can
/// enqueue requests and observe the channel depth.
fn build_bridge_app() -> (App, ControlSender) {
    let handle = SnapshotHandle::new();
    let (tx, rx) = control_channel();
    let app = build_headless_app_with(move |app| {
        app.add_plugins(StatesPlugin)
            .add_plugins(RepliconPlugins)
            .add_plugins(NetProtocolPlugin)
            .add_plugins(McpBridgePlugin::new(handle, rx))
            .insert_resource(ConnectTarget("127.0.0.1:5555".parse().unwrap()));
    });
    (app, tx)
}

#[test]
fn drain_consumes_queued_requests() {
    let (mut app, tx) = build_bridge_app();

    tx.0.send(ControlRequest::Remove { plane: PlaneId(1) })
        .unwrap();
    tx.0.send(ControlRequest::Remove { plane: PlaneId(2) })
        .unwrap();
    assert_eq!(tx.0.len(), 2, "two requests queued before the drain runs");

    app.update();

    assert_eq!(
        tx.0.len(),
        0,
        "drain_control_requests should have consumed both queued requests"
    );
}

#[test]
fn drain_records_requested_sim_speed() {
    let (mut app, tx) = build_bridge_app();

    tx.0.send(ControlRequest::SetSimSpeed {
        speed: SimSpeed::X5,
    })
    .unwrap();
    app.update();

    assert_eq!(tx.0.len(), 0, "the sim-speed request should be drained");
    assert_eq!(
        app.world().resource::<RequestedSimSpeed>().0,
        Some(SimSpeed::X5),
        "drain should echo the last-requested speed into RequestedSimSpeed"
    );
}
