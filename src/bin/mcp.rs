//! `ml_planes_mcp` — an MCP control client for a running `ml_planes_server`.
//!
//! Joins the authoritative sim as a headless renet/replicon **client** (reusing
//! `NetProtocolPlugin` verbatim) and exposes it to an LLM agent over **MCP stdio**
//! (the `rmcp` SDK). The binary runs two cooperating runtimes:
//!
//!   * a headless Bevy `App` on a dedicated thread — the replicon client transport
//!     (`MinimalPlugins` + `RepliconPlugins` + `RepliconRenetPlugins` + `NetProtocolPlugin`),
//!   * an `rmcp` stdio server on the tokio main thread (`PlanesService`).
//!
//! The read path is a shared snapshot: the Bevy thread runs `McpBridgePlugin`, which mirrors
//! the replicated world into a `SnapshotHandle` (`Arc<RwLock<SimSnapshot>>`) every frame, and
//! the rmcp `PlanesService` holds a clone of the same `Arc` to back its read tools
//! (`get_sim_status`, `list_planes`, `get_plane_state`). The command bridge (write path) lands
//! in Phases 3–4.
//!
//! **stdout is the MCP JSON-RPC channel** — all logging goes to **stderr** only. Run from
//! the project root:
//!
//!   cargo run --features mcp --bin ml_planes_mcp -- --connect 127.0.0.1:5555
//!
//! Options:
//!   --connect host:port      server to join (default: `127.0.0.1:DEFAULT_PORT`).
//!   --connect-timeout SECS    connect deadline in whole seconds (default: 5).

use std::error::Error;
use std::time::Duration;

use bevy::app::ScheduleRunnerPlugin;
use bevy::prelude::*;
use bevy::state::app::StatesPlugin;
use bevy_replicon::prelude::RepliconPlugins;
use bevy_replicon_renet::RepliconRenetPlugins;
use rmcp::{transport::stdio, ServiceExt};

use ml_planes::mcp::{McpArgs, McpBridgePlugin, PlanesService, SnapshotHandle};
use ml_planes::net::{start_renet_client, ConnectTarget, NetProtocolPlugin};

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // stdout is reserved for the MCP JSON-RPC stream — route every log line to stderr.
    // This MUST be installed before any `tracing` event (renet/replicon emit on connect).
    tracing_subscriber::fmt()
        .with_writer(std::io::stderr)
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    let args = McpArgs::parse();

    // Shared snapshot mirror: the Bevy thread rewrites it each frame; the rmcp service holds a
    // clone of the same `Arc` so its read tools serve the latest state.
    let snapshot = SnapshotHandle::new();

    // Headless replicon client on its own thread — it owns the `app.run()` loop.
    spawn_sim_client(args, snapshot.clone());

    // rmcp stdio server on the tokio main thread, backed by the shared snapshot.
    let service = PlanesService::new(snapshot).serve(stdio()).await?;
    service.waiting().await?;
    Ok(())
}

/// Build and run the headless Bevy replicon client on a dedicated thread.
///
/// Mirrors `src/bin/server.rs`'s minimal headless stack, swapping the server pieces for
/// the client: insert [`ConnectTarget`] and register [`start_renet_client`] at `Startup`.
/// No Rapier / sim plugins — the client never simulates, it only mirrors replicated state.
fn spawn_sim_client(args: McpArgs, snapshot: SnapshotHandle) {
    std::thread::spawn(move || {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(
            Duration::from_secs_f64(1.0 / 64.0),
        )))
        .add_plugins(bevy::transform::TransformPlugin)
        .add_plugins(bevy::asset::AssetPlugin::default())
        .add_plugins(StatesPlugin)
        .add_plugins(RepliconPlugins)
        .add_plugins(RepliconRenetPlugins)
        .add_plugins(NetProtocolPlugin)
        .add_plugins(McpBridgePlugin::new(snapshot))
        .insert_resource(ConnectTarget(args.connect))
        .add_systems(Startup, start_renet_client);
        app.run();
    });
}
