#![cfg(feature = "net")]
//! Phase 4 client-side networking (`plans/client_server.md`): the client renders
//! replicated plane state. These tests cover the client-only logic headlessly
//! (no socket), mirroring the style of `tests/server_sim.rs`. The pure
//! interpolation math is unit-tested in `src/net/client.rs`; here we drive the
//! decorate + pose-buffer systems through a real (if local) world, simulating what
//! replication produces by spawning/mutating the replicated components directly.
//!
//! Runs under `cargo test --features server` (which enables `net`).

use bevy::prelude::*;
use ml_planes::net::{ClientNetPlugin, NetInterpolation};
use ml_planes::plane::{FlightState, PlaneId};

/// A minimal headless app with just the client rendering systems. `MinimalPlugins`
/// supplies the `Time` the interpolation buffer reads.
fn build_client_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(ClientNetPlugin);
    app.finish();
    app
}

/// A replicated plane (PlaneId + FlightState, as the server sends) gets a
/// `NetInterpolation` buffer seeded from its current pose.
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
    assert_eq!(
        interp.curr_pos, pos,
        "buffer seeds curr from the spawn pose"
    );
    assert_eq!(interp.prev_pos, pos, "and seeds prev to the same pose");
}

/// A replication update to `FlightState` shifts the old curr into prev and records
/// the new pose as curr — the two snapshots the renderer blends between.
#[test]
fn flight_state_update_shifts_the_pose_buffer() {
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
    app.update(); // decorate seeds prev = curr = a

    // Simulate the server replicating a moved plane.
    app.world_mut()
        .get_mut::<FlightState>(entity)
        .unwrap()
        .position = b;
    app.update(); // buffer_net_pose shifts: prev = a, curr = b

    let interp = app.world().get::<NetInterpolation>(entity).unwrap();
    assert_eq!(interp.prev_pos, a, "previous snapshot holds the old pose");
    assert_eq!(
        interp.curr_pos, b,
        "current snapshot holds the replicated pose"
    );
    assert!(
        interp.curr_time >= interp.prev_time,
        "curr snapshot is no older than prev"
    );
}
