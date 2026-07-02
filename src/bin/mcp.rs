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
//! (`get_sim_status`, `list_planes`, `get_plane_state`). The write path is a command channel:
//! write tools (`spawn_plane`, `remove_plane`) push `ControlRequest`s that the Bevy thread's
//! `drain_control_requests` system fires as `client_trigger` commands.
//!
//! The client is resilient: `McpBridgePlugin`'s `poll_reconnect` rebuilds the transport when
//! the connection drops or the server restarts, and a shared `ShutdownFlag` lets `main` stop the
//! Bevy thread cleanly once the MCP stdio stream closes.
//!
//! **stdout is the MCP JSON-RPC channel** — all logging goes to **stderr** only. Run from
//! the project root:
//!
//!   cargo run --features mcp --bin ml_planes_mcp -- --connect 127.0.0.1:5555
//!
//! Options:
//!   --connect host:port      server to join (default: `127.0.0.1:DEFAULT_PORT`).
//!   --connect-timeout SECS    connect/handshake window + initial reconnect backoff (default: 5).
//!   --quiet                   silence non-error logging (unless `RUST_LOG` overrides).

use std::error::Error;
use std::thread::JoinHandle;
use std::time::Duration;

use bevy::app::ScheduleRunnerPlugin;
use bevy::prelude::*;
use bevy::state::app::StatesPlugin;
use bevy_replicon::prelude::RepliconPlugins;
use bevy_replicon_renet::RepliconRenetPlugins;
use rmcp::{transport::stdio, ServiceExt};

use ml_planes::mcp::{
    control_channel, ControlReceiver, McpArgs, McpBridgePlugin, PlanesService, ShutdownFlag,
    SnapshotHandle,
};
use ml_planes::net::{start_renet_client, ConnectTarget, NetProtocolPlugin};

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let args = McpArgs::parse();

    // stdout is reserved for the MCP JSON-RPC stream — route every log line to stderr.
    // This MUST be installed before any `tracing` event (renet/replicon emit on connect).
    // `RUST_LOG` always wins; otherwise `--quiet` drops the default level from info to error.
    let default_level = if args.quiet { "error" } else { "info" };
    tracing_subscriber::fmt()
        .with_writer(std::io::stderr)
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new(default_level)),
        )
        .init();

    // Shared snapshot mirror: the Bevy thread rewrites it each frame; the rmcp service holds a
    // clone of the same `Arc` so its read tools serve the latest state.
    let snapshot = SnapshotHandle::new();

    // Command channel (write path): rmcp write tools push `ControlRequest`s onto the sender; the
    // Bevy thread drains the receiver into `client_trigger` commands.
    let (control_tx, control_rx) = control_channel();

    // Cross-thread shutdown signal — set once the rmcp stdio stream closes below.
    let shutdown = ShutdownFlag::new();

    // Headless replicon client on its own thread — it owns the `app.run()` loop.
    let sim_thread = spawn_sim_client(args, snapshot.clone(), control_rx, shutdown.clone());

    // rmcp stdio server on the tokio main thread, backed by the shared snapshot + command sender.
    let service = PlanesService::new(snapshot, control_tx)
        .serve(stdio())
        .await?;
    // Blocks until the MCP client hangs up / stdin reaches EOF.
    service.waiting().await?;

    // Clean shutdown: ask the Bevy thread to exit (dropping its transport) and join it, with a
    // bounded fallback so a wedged sim thread never hangs process exit.
    shutdown.request();
    join_bounded(sim_thread, Duration::from_secs(2));
    Ok(())
}

/// Join `handle`, but give up after `timeout` so a stuck Bevy thread can't wedge process exit.
/// Polls the thread's liveness on a short interval rather than blocking indefinitely on `join`.
fn join_bounded(handle: JoinHandle<()>, timeout: Duration) {
    let deadline = std::time::Instant::now() + timeout;
    while !handle.is_finished() {
        if std::time::Instant::now() >= deadline {
            // The `run_loop` didn't observe the flag in time; the process is exiting anyway, so
            // let the OS reap the detached thread.
            return;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    let _ = handle.join();
}

/// Build and run the headless Bevy replicon client on a dedicated thread, returning its
/// [`JoinHandle`] so `main` can join it on shutdown.
///
/// Mirrors `src/bin/server.rs`'s minimal headless stack, swapping the server pieces for
/// the client: insert [`ConnectTarget`] and register [`start_renet_client`] at `Startup`.
/// No Rapier / sim plugins — the client never simulates, it only mirrors replicated state.
/// [`McpBridgePlugin`] carries the reconnect backoff (`--connect-timeout`) and the shared
/// [`ShutdownFlag`] so the loop reconnects on drops and exits when `main` signals it.
fn spawn_sim_client(
    args: McpArgs,
    snapshot: SnapshotHandle,
    control_rx: ControlReceiver,
    shutdown: ShutdownFlag,
) -> JoinHandle<()> {
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
        .add_plugins(
            McpBridgePlugin::new(snapshot, control_rx)
                .with_connect_timeout(args.connect_timeout)
                .with_shutdown(shutdown),
        )
        .insert_resource(ConnectTarget(args.connect))
        .add_systems(Startup, start_renet_client);
        app.run();
    })
}
