//! MCP Phase 5: end-to-end over a real renet transport.
//!
//! Unlike the other net/MCP tests (deliberately transport-free), this one boots a **real**
//! `ml_planes_server` child on an ephemeral UDP port, stands up the headless MCP replicon
//! client in-process (the same plugin stack `src/bin/mcp.rs` runs), and validates a full
//! inspect + spawn/remove round-trip over the wire. It exercises the whole chain: renet
//! handshake → replication → snapshot mirror (the read path), and `ControlSender` →
//! `drain_control_requests` → `client_trigger` → server spawn/remove → replication back →
//! snapshot (the write path).
//!
//! It asserts against the shared **snapshot `Arc`** and the **`ControlRequest` channel** — the
//! exact primitives the rmcp read/write tools wrap. Those tools' thin JSON shaping + arg parsing
//! + confirmation polling are unit-tested directly in `src/mcp/service.rs` (the tool methods are
//! private to that module); the gap this test closes is the real-transport round-trip. Driving
//! the rmcp **stdio** handshake as a subprocess is intentionally avoided — the in-process client
//! covers the same round-trip without a brittle JSON-RPC-over-pipes harness.
//!
//! `#[ignore]` by default (binds sockets, spawns a child, runs a Rapier sim) — run explicitly:
//!   cargo test --no-default-features --features "mcp server" --test net -- --ignored mcp_e2e
//!
//! Gated on both `mcp` (the client) and `server` (the `CARGO_BIN_EXE_ml_planes_server` bin +
//! the sim chain); this module is gated from `tests/net/main.rs`.

use std::collections::HashSet;
use std::net::{SocketAddr, UdpSocket};
use std::process::{Child, Command};
use std::time::{Duration, Instant};

use bevy::app::ScheduleRunnerPlugin;
use bevy::prelude::*;
use bevy::state::app::StatesPlugin;
use bevy_replicon::prelude::RepliconPlugins;
use bevy_replicon_renet::RepliconRenetPlugins;

use ml_planes::controllers::ControllerKind;
use ml_planes::mcp::{control_channel, ControlRequest, McpBridgePlugin, SnapshotHandle};
use ml_planes::net::{start_renet_client, ConnectTarget, NetProtocolPlugin};
use ml_planes::plane::PlaneId;
use ml_planes::training::SpawnSpec;

/// Kills the server child when the test scope ends (pass or panic).
struct ServerGuard(Child);

impl Drop for ServerGuard {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Grab a currently-free UDP port by binding `:0` and reading the assigned port back. Small
/// TOCTOU race before the server rebinds it, acceptable for an ignored, run-explicitly test.
fn free_udp_port() -> u16 {
    let sock = UdpSocket::bind("127.0.0.1:0").expect("bind ephemeral udp port");
    sock.local_addr().expect("local addr").port()
}

/// Launch the real `ml_planes_server` child on `port`, serving the default scenario. Cargo
/// builds + exposes the binary via `CARGO_BIN_EXE_ml_planes_server`; run it from the crate
/// root so its relative `assets/...` paths resolve.
fn spawn_server(port: u16) -> ServerGuard {
    let child = Command::new(env!("CARGO_BIN_EXE_ml_planes_server"))
        .current_dir(env!("CARGO_MANIFEST_DIR"))
        .arg("--scenario")
        .arg("assets/scenarios/default.scenario.ron")
        .arg("--port")
        .arg(port.to_string())
        .spawn()
        .expect("spawn ml_planes_server child");
    ServerGuard(child)
}

/// Stand up the headless MCP replicon client in-process on its own thread (the `src/bin/mcp.rs`
/// stack), returning the shared snapshot handle (read path) + the `ControlRequest` sender (write
/// path) — the same primitives the rmcp tools hold.
fn spawn_client(addr: SocketAddr) -> (SnapshotHandle, crossbeam_channel::Sender<ControlRequest>) {
    let handle = SnapshotHandle::new();
    let (tx, rx) = control_channel();
    let app_handle = handle.clone();
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
            McpBridgePlugin::new(app_handle, rx).with_connect_timeout(Duration::from_secs(2)),
        )
        .insert_resource(ConnectTarget(addr))
        .add_systems(Startup, start_renet_client);
        app.run();
    });
    (handle, tx.0)
}

/// Poll `predicate` against the shared snapshot until it holds or `deadline` elapses.
async fn wait_until(
    handle: &SnapshotHandle,
    deadline: Duration,
    what: &str,
    predicate: impl Fn(&ml_planes::mcp::SimSnapshot) -> bool,
) {
    let stop = Instant::now() + deadline;
    loop {
        if predicate(&handle.0.read().expect("snapshot lock")) {
            return;
        }
        assert!(Instant::now() < stop, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
}

fn plane_ids(handle: &SnapshotHandle) -> HashSet<u32> {
    handle
        .0
        .read()
        .expect("snapshot lock")
        .planes
        .iter()
        .map(|p| p.plane_id)
        .collect()
}

#[tokio::test]
#[ignore = "boots a real ml_planes_server child + binds UDP sockets; run explicitly"]
async fn mcp_e2e_inspect_spawn_remove() {
    let port = free_udp_port();
    let _server = spawn_server(port);
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let (handle, tx) = spawn_client(addr);

    // 1. Read path: connect + replicate the scenario's planes into the snapshot the read tools
    //    serve from.
    wait_until(&handle, Duration::from_secs(20), "client to connect", |s| {
        s.connected
    })
    .await;
    wait_until(
        &handle,
        Duration::from_secs(10),
        "planes to replicate",
        |s| !s.planes.is_empty(),
    )
    .await;

    // The replicated planes carry live per-plane state (what `get_plane_state` serializes).
    {
        let snap = handle.0.read().unwrap();
        let plane = &snap.planes[0];
        assert!(
            plane.airspeed > 0.0,
            "a replicated plane should carry live flight state, got airspeed {}",
            plane.airspeed
        );
    }

    // 2. Write round-trip: spawn a plane via the same channel the write tools feed.
    let before = plane_ids(&handle);
    tx.send(ControlRequest::Spawn {
        config_path: "planes/generic_jet.plane.ron".to_string(),
        kind: ControllerKind::LevelHold,
        spec: SpawnSpec {
            position: Some(Vec3::new(0.0, 1500.0, 0.0)),
            velocity: Some(Vec3::new(100.0, 0.0, 0.0)),
            attitude: None,
            angular_velocity: None,
            fuel_fraction: None,
        },
    })
    .expect("enqueue spawn");

    wait_until(
        &handle,
        Duration::from_secs(10),
        "the new plane to appear",
        {
            let before = before.clone();
            move |s| s.planes.iter().any(|p| !before.contains(&p.plane_id))
        },
    )
    .await;

    let new_id = plane_ids(&handle)
        .difference(&before)
        .copied()
        .max()
        .expect("a new plane id");

    // 3. Remove it and confirm it's gone.
    tx.send(ControlRequest::Remove {
        plane: PlaneId(new_id),
    })
    .expect("enqueue remove");
    wait_until(
        &handle,
        Duration::from_secs(10),
        "the plane to be removed",
        { move |s| !s.planes.iter().any(|p| p.plane_id == new_id) },
    )
    .await;
}
