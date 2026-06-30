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
//! Phase 0 stood up the buildable skeleton (arg parsing, a stub `PlanesService::ping`,
//! the dual-runtime wiring). **Phase 1** (this revision) adds the read-path foundation:
//! [`McpBridgePlugin`] runs [`collect_snapshot`] every frame to mirror the replicated world
//! into a shared [`SimSnapshot`] behind an `Arc<RwLock<…>>` ([`SnapshotHandle`]). Read tools
//! (Phase 2) and the command bridge + write tools (Phases 3–4) consume this snapshot later.

pub mod args;
pub mod service;
pub mod snapshot;

use bevy::prelude::*;

pub use args::McpArgs;
pub use service::PlanesService;
pub use snapshot::{
    build_snapshot, collect_snapshot, plane_snapshot, PlaneSnapshot, RequestedSimSpeed,
    SimSnapshot, SnapshotHandle,
};

/// Bevy plugin (added to the headless replicon-client app) that mirrors the replicated world
/// into the shared [`SnapshotHandle`] each frame via [`collect_snapshot`].
///
/// Constructed with the same [`SnapshotHandle`] the binary keeps a clone of, so Phase 2 can
/// hand a second clone to the rmcp [`PlanesService`].
pub struct McpBridgePlugin {
    handle: SnapshotHandle,
}

impl McpBridgePlugin {
    /// Build the plugin sharing `handle` with the rest of the process.
    pub fn new(handle: SnapshotHandle) -> Self {
        Self { handle }
    }
}

impl Plugin for McpBridgePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(self.handle.clone())
            .init_resource::<RequestedSimSpeed>()
            .add_systems(Update, collect_snapshot);
    }
}
