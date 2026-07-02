//! MCP control client (`ml_planes_mcp`).
//!
//! A standalone, headless renet/replicon **client** that joins a running
//! `ml_planes_server` over the existing net protocol and exposes the live simulation
//! to an LLM agent over **MCP stdio** (the `rmcp` SDK). It reuses `net::NetProtocolPlugin`
//! verbatim — no server-side changes.
//!
//! The binary (`src/bin/mcp.rs`) runs two cooperating runtimes bridged by shared state:
//! a headless Bevy `App` on a dedicated thread (the replicon client) and an `rmcp` stdio
//! server on the tokio main thread. See `plans/mcp_server.md` for the full design.
//!
//! Layout:
//! - [`snapshot`] — the read path: [`collect_snapshot`] mirrors the replicated world into a
//!   shared [`SimSnapshot`] behind an `Arc<RwLock<…>>` ([`SnapshotHandle`]) each frame.
//! - [`bridge`] — the write path: [`drain_control_requests`] turns queued [`ControlRequest`]s
//!   into `client_trigger` commands.
//! - [`lifecycle`] — auto-reconnect ([`poll_reconnect`]) + clean shutdown ([`check_shutdown`]).
//! - [`service`] — the `rmcp` `ServerHandler` + `#[tool]` methods (all `rmcp` types quarantined
//!   here). Read tools clone the snapshot; write tools enqueue a [`ControlRequest`].
//! - [`args`] — `--connect` / `--connect-timeout` / `--quiet` parsing.
//!
//! [`McpBridgePlugin`] wires the four Bevy systems (`collect_snapshot`, `drain_control_requests`,
//! `poll_reconnect`, `check_shutdown`) into the headless replicon-client app.

pub mod args;
pub mod bridge;
pub mod lifecycle;
pub mod service;
pub mod snapshot;

use std::time::Duration;

use bevy::prelude::*;

pub use args::McpArgs;
pub use bridge::{
    control_channel, drain_control_requests, parse_sim_speed, parse_spawnable_controller_kind,
    ControlReceiver, ControlRequest, ControlSender,
};
pub use lifecycle::{
    check_shutdown, next_backoff, poll_reconnect, should_attempt, ReconnectState, ShutdownFlag,
    RECONNECT_BACKOFF_CAP,
};
pub use service::PlanesService;
pub use snapshot::{
    build_snapshot, collect_snapshot, plane_snapshot, PlaneSnapshot, RequestedSimSpeed,
    SimSnapshot, SnapshotHandle,
};

/// Bevy plugin (added to the headless replicon-client app) bridging the sim to the rmcp
/// service. It mirrors the replicated world into the shared [`SnapshotHandle`] each frame
/// ([`collect_snapshot`], read path), drains queued [`ControlRequest`]s into `client_trigger`
/// commands ([`drain_control_requests`], write path), keeps the client connected across server
/// restarts ([`poll_reconnect`]), and exits cleanly on a [`ShutdownFlag`] ([`check_shutdown`]).
///
/// Constructed with the same [`SnapshotHandle`] + [`ControlReceiver`] the binary pairs with the
/// rmcp [`PlanesService`]'s snapshot clone and [`ControlSender`]. The optional
/// [`ShutdownFlag`] and reconnect backoff are supplied by the binary via the builder methods;
/// tests that omit them get a defaulted [`ReconnectState`] and a fresh (never-signalled) flag.
pub struct McpBridgePlugin {
    handle: SnapshotHandle,
    receiver: ControlReceiver,
    reconnect: ReconnectState,
    shutdown: ShutdownFlag,
}

impl McpBridgePlugin {
    /// Build the plugin sharing `handle` (read path) and `receiver` (write path) with the
    /// rest of the process. Reconnect backoff defaults; the shutdown flag is a private,
    /// never-signalled one unless overridden with [`Self::with_shutdown`].
    pub fn new(handle: SnapshotHandle, receiver: ControlReceiver) -> Self {
        Self {
            handle,
            receiver,
            reconnect: ReconnectState::default(),
            shutdown: ShutdownFlag::default(),
        }
    }

    /// Seed the reconnect backoff from the MCP's `--connect-timeout` (also the handshake window).
    pub fn with_connect_timeout(mut self, timeout: Duration) -> Self {
        self.reconnect = ReconnectState::new(timeout);
        self
    }

    /// Share the shutdown flag `main` sets when the rmcp stdio stream closes.
    pub fn with_shutdown(mut self, shutdown: ShutdownFlag) -> Self {
        self.shutdown = shutdown;
        self
    }
}

impl Plugin for McpBridgePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(self.handle.clone())
            .insert_resource(self.receiver.clone())
            .insert_resource(self.reconnect.clone())
            .insert_resource(self.shutdown.clone())
            .init_resource::<RequestedSimSpeed>()
            .add_systems(
                Update,
                (
                    collect_snapshot,
                    drain_control_requests,
                    poll_reconnect,
                    check_shutdown,
                ),
            );
    }
}
