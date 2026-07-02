//! MCP Phase 5: connection lifecycle (auto-reconnect + clean shutdown).
//!
//! Transport-free headless tests mirroring `tests/mcp_bridge.rs`: register `RepliconPlugins`
//! + `NetProtocolPlugin` + `McpBridgePlugin` (no renet driver plugin) and exercise the two
//! lifecycle systems the plugin adds. `poll_reconnect` rebuilds the client transport while
//! disconnected; `check_shutdown` turns the shared `ShutdownFlag` into an `AppExit`. The pure
//! backoff/predicate helpers are unit-tested in `src/mcp/lifecycle.rs`; genuine reconnection
//! over a real socket is covered by `tests/mcp_e2e.rs`.
//!
//! Gated on `mcp` (`cargo test --features mcp`).
#![cfg(feature = "mcp")]

mod common;

use std::time::Duration;

use bevy::prelude::*;
use bevy::state::app::StatesPlugin;
use bevy_replicon::prelude::RepliconPlugins;
use bevy_replicon_renet::RenetClient;

use common::build_headless_app_with;
use ml_planes::mcp::{
    check_shutdown, control_channel, McpBridgePlugin, ShutdownFlag, SnapshotHandle,
};
use ml_planes::net::{ConnectTarget, NetProtocolPlugin};

#[test]
fn poll_reconnect_rebuilds_transport_while_disconnected() {
    // A zero connect-timeout ⇒ zero backoff, so the very first update — Disconnected with no
    // transport yet — rebuilds the client. (No `RepliconRenetPlugins` here, so nothing advances
    // the handshake; the inserted `RenetClient` resource is the observable that a rebuild ran.)
    let handle = SnapshotHandle::new();
    let (_tx, rx) = control_channel();
    let mut app = build_headless_app_with(move |app| {
        app.add_plugins(StatesPlugin)
            .add_plugins(RepliconPlugins)
            .add_plugins(NetProtocolPlugin)
            .add_plugins(McpBridgePlugin::new(handle, rx).with_connect_timeout(Duration::ZERO))
            .insert_resource(ConnectTarget("127.0.0.1:5555".parse().unwrap()));
    });

    assert!(
        app.world().get_resource::<RenetClient>().is_none(),
        "no client transport before the first update"
    );
    app.update();
    assert!(
        app.world().get_resource::<RenetClient>().is_some(),
        "poll_reconnect should (re)build the client transport while disconnected"
    );
}

/// Probe resource + system so the test can observe an emitted `AppExit`.
#[derive(Resource, Default)]
struct ExitSeen(bool);

fn watch_exit(mut reader: MessageReader<AppExit>, mut seen: ResMut<ExitSeen>) {
    if reader.read().next().is_some() {
        seen.0 = true;
    }
}

#[test]
fn shutdown_flag_emits_app_exit() {
    let handle = SnapshotHandle::new();
    let (_tx, rx) = control_channel();
    let shutdown = ShutdownFlag::new();
    let flag = shutdown.clone();
    let mut app = build_headless_app_with(move |app| {
        app.add_plugins(StatesPlugin)
            .add_plugins(RepliconPlugins)
            .add_plugins(NetProtocolPlugin)
            .add_plugins(McpBridgePlugin::new(handle, rx).with_shutdown(shutdown))
            .insert_resource(ConnectTarget("127.0.0.1:5555".parse().unwrap()))
            .init_resource::<ExitSeen>()
            .add_systems(Update, watch_exit.after(check_shutdown));
    });

    app.update();
    assert!(
        !app.world().resource::<ExitSeen>().0,
        "no AppExit before the shutdown flag is set"
    );

    flag.request();
    app.update();
    assert!(
        app.world().resource::<ExitSeen>().0,
        "check_shutdown should emit AppExit once the flag is set"
    );
}
