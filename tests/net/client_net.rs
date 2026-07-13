//! Phase 4 client-side networking (`plans/client_server.md`): the client renders
//! replicated plane state. These tests cover the client-only logic headlessly
//! (no socket), mirroring the style of `tests/server_sim.rs`. The pure
//! interpolation math (`sample` / `push_snapshot`) is unit-tested in
//! `src/net/client.rs`; here we drive the decorate + snapshot-ingest systems through
//! a real (if local) world, simulating what replication produces by mutating the
//! replicated components and writing the `EntityReplicated` message directly.
//!
//! Runs under `cargo test --features server` (which enables `net`).

use bevy::prelude::*;
use bevy_replicon::client::confirm_history::EntityReplicated;
use bevy_replicon::prelude::RepliconTick;
use ml_planes::net::{ClientNetPlugin, NetClockOffset, NetInterpolation};
use ml_planes::plane::{FlightState, PlaneId};

/// A minimal headless app with just the client rendering systems. `MinimalPlugins`
/// supplies the `Time` the interpolation reads. The real client gets
/// `EntityReplicated` registered by `RepliconPlugins`; here we register it directly
/// (this app has no replicon transport).
fn build_client_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_message::<EntityReplicated>();
    app.add_plugins(ClientNetPlugin);
    app.finish();
    app
}

/// A replicated plane (PlaneId + FlightState, as the server sends) gets an (initially
/// empty) `NetInterpolation` ring buffer.
#[test]
fn replicated_plane_gets_interpolation_buffer() {
    let mut app = build_client_app();
    let pos = Vec3::new(1.0, 2.0, 3.0);
    let entity = app
        .world_mut()
        .spawn((
            PlaneId(0),
            FlightState {
                position: pos,
                ..Default::default()
            },
            Transform::default(),
        ))
        .id();

    app.update();

    let interp = app
        .world()
        .get::<NetInterpolation>(entity)
        .expect("decorate should attach a NetInterpolation buffer to the replicated plane");
    assert!(
        interp.snapshots.is_empty(),
        "buffer starts empty; snapshots arrive via EntityReplicated"
    );
}

/// An `EntityReplicated` message (with the FlightState mutated to the new pose) pushes
/// a snapshot stamped with the server-tick time, and seeds the global clock offset.
#[test]
fn entity_replicated_pushes_a_server_timed_snapshot() {
    let mut app = build_client_app();
    let a = Vec3::new(1.0, 2.0, 3.0);
    let b = Vec3::new(4.0, 5.0, 6.0);
    let entity = app
        .world_mut()
        .spawn((
            PlaneId(0),
            FlightState {
                position: a,
                ..Default::default()
            },
            Transform::default(),
        ))
        .id();
    app.update(); // decorate attaches the empty buffer

    // Simulate the server replicating a moved plane at tick 64 (= 1.0 s server time).
    app.world_mut()
        .get_mut::<FlightState>(entity)
        .unwrap()
        .position = b;
    app.world_mut().write_message(EntityReplicated {
        entity,
        tick: RepliconTick::new(64),
    });
    app.update(); // ingest_snapshots records the snapshot

    let interp = app.world().get::<NetInterpolation>(entity).unwrap();
    assert_eq!(interp.snapshots.len(), 1, "one snapshot recorded");
    let snap = interp.snapshots.back().unwrap();
    assert_eq!(snap.pos, b, "snapshot holds the replicated pose");
    assert!(
        (snap.server_time - 1.0).abs() < 1e-9,
        "server_time = tick / 64 Hz = 1.0, got {}",
        snap.server_time
    );

    let clock = app.world().resource::<NetClockOffset>();
    assert!(
        clock.initialized,
        "the first snapshot seeds the clock offset"
    );
}

/// Only the newest tick of a same-frame batch contributes a snapshot (we only hold the
/// latest FlightState value), so two messages for one entity yield a single snapshot at
/// the newer tick.
#[test]
fn batched_ticks_collapse_to_the_newest() {
    let mut app = build_client_app();
    let entity = app
        .world_mut()
        .spawn((PlaneId(0), FlightState::default(), Transform::default()))
        .id();
    app.update();

    app.world_mut()
        .get_mut::<FlightState>(entity)
        .unwrap()
        .position = Vec3::new(0.0, 100.0, 0.0);
    app.world_mut().write_message(EntityReplicated {
        entity,
        tick: RepliconTick::new(10),
    });
    app.world_mut().write_message(EntityReplicated {
        entity,
        tick: RepliconTick::new(12),
    });
    app.update();

    let interp = app.world().get::<NetInterpolation>(entity).unwrap();
    assert_eq!(interp.snapshots.len(), 1, "batch collapses to one snapshot");
    let snap = interp.snapshots.back().unwrap();
    assert!(
        (snap.server_time - 12.0 / 64.0).abs() < 1e-9,
        "snapshot uses the newest tick (12), got server_time {}",
        snap.server_time
    );
}
