//! Client-side networking: the renet client transport plus rendering of
//! replicated plane state.
//!
//! Phase 4 of the client/server split (`plans/client_server.md`). The visual app
//! (`src/main.rs`) runs no physics of its own; planes arrive purely via replication
//! ([`crate::net::protocol`]). This module turns those replicated entities into
//! smoothly-rendered planes:
//!
//! - [`ClientNetPlugin`] — decorates each replicated plane with a
//!   [`NetInterpolation`] buffer and renders an interpolated [`Transform`].
//! - [`connect_to_server`] / [`start_renet_client`] — build the renet/netcode
//!   client transport (mirrors [`crate::net::server::start_renet_server`]); added by
//!   the binary when a connection is requested, not by the plugin, so tests never
//!   bind a socket.
//!
//! Interpolation is driven from the replicated [`FlightState`] (which carries
//! `position` + `attitude`) and *writes* [`Transform`]. Reading one component and
//! writing another avoids any replication/self-write feedback. Each snapshot is
//! stamped with the **server tick** it represents (via [`EntityReplicated`]), so the
//! render timeline is uniform and independent of the client frame rate; poses are
//! buffered per plane ([`NetInterpolation`]) and sampled at a playback point that lags
//! the newest snapshot by [`RENDER_DELAY`], keeping a straddling pair available to
//! blend. Stamping on client frame-arrival time instead made the rendered lag track
//! frame-time jitter, which showed up as plane gizmos pulsing back and forth.

use std::collections::{HashMap, VecDeque};
use std::net::{Ipv4Addr, SocketAddr, UdpSocket};
use std::time::SystemTime;

use bevy::prelude::*;
use bevy_replicon::client::confirm_history::EntityReplicated;
use bevy_replicon::prelude::*;
use bevy_replicon_renet::netcode::{ClientAuthentication, NetcodeClientTransport};
use bevy_replicon_renet::renet::ConnectionConfig;
use bevy_replicon_renet::{RenetChannelsExt, RenetClient};

use crate::net::protocol::PROTOCOL_ID;
use crate::plane::{FlightState, PlaneId, PlaneTuningHandle, PlaneTuningPath};

/// A child `ml_planes_server` process that is killed when this handle is dropped,
/// so a server the client launched (Start New Server) dies with the client on every
/// exit path — window close or Quit, which skip the `OnExit(InGame)` teardown.
/// `std::process::Child::drop` does *not* kill the process, so orphaning is the
/// default without this.
pub struct ServerProcess(pub std::process::Child);

impl Drop for ServerProcess {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait(); // reap the zombie
    }
}

/// Server replication tick rate. Snapshots are stamped with `tick / SERVER_TICK_HZ`
/// to reconstruct a uniform server-time timeline, independent of the client's frame
/// rate. Must match the server's `Time<Fixed>` rate (bevy_replicon increments its tick
/// in `FixedPostUpdate`, Bevy's default 64 Hz) and the Rapier `TimestepMode::Fixed`
/// `dt = 1/64` used by `ml_planes_server`.
const SERVER_TICK_HZ: f64 = 64.0;

/// How far behind the newest received snapshot the render pose is sampled, so a pair
/// of snapshots always straddles the playback point. ~2.5 server ticks — enough
/// cushion for client frame-time / packet jitter while staying visually tight.
const RENDER_DELAY: f64 = 2.5 / SERVER_TICK_HZ;

/// Ring-buffer depth per plane (≈125 ms at 64 Hz) — comfortably covers `RENDER_DELAY`
/// plus jitter at any client frame rate.
const SNAPSHOT_CAP: usize = 8;

/// Low-pass factor for the server↔client clock-offset estimate. Small = smoother
/// (rejects per-frame jitter) at the cost of slower convergence to clock drift.
const OFFSET_SMOOTHING: f64 = 0.1;

/// One received pose, stamped with the server time it represents.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Snapshot {
    /// `replicon_tick / SERVER_TICK_HZ`, in seconds on the server's timeline.
    pub server_time: f64,
    pub pos: Vec3,
    pub rot: Quat,
}

/// Per-plane ring buffer of recent server-timed poses. Filled from [`FlightState`] as
/// replication updates arrive ([`ingest_snapshots`]); sampled into the entity's
/// [`Transform`] at the playback time ([`render_net_interpolation`]).
#[derive(Component, Debug, Clone, Default, PartialEq)]
pub struct NetInterpolation {
    pub snapshots: VecDeque<Snapshot>,
}

/// Smoothed estimate of `client_now − server_time_of_latest_snapshot`, shared by all
/// planes (they ride the same server tick timeline). Maps client wall-clock into the
/// server-time space the snapshots live in; low-passing it is what removes the
/// frame-time jitter that made plane gizmos pulse.
#[derive(Resource, Debug, Clone, Copy)]
pub struct NetClockOffset {
    pub offset: f64,
    pub initialized: bool,
}

impl Default for NetClockOffset {
    fn default() -> Self {
        Self {
            offset: 0.0,
            initialized: false,
        }
    }
}

/// Append `snap` to the ring buffer, capping at `cap`. Replaces the newest entry when
/// `snap` shares its `server_time` (a re-push of the same tick) and drops strictly
/// older ticks (out-of-order). Pure so it is unit-testable without an app.
pub fn push_snapshot(buf: &mut VecDeque<Snapshot>, snap: Snapshot, cap: usize) {
    if let Some(last) = buf.back() {
        if snap.server_time < last.server_time {
            return; // stale / out-of-order — ignore
        }
        if snap.server_time == last.server_time {
            *buf.back_mut().unwrap() = snap; // same tick — supersede in place
            return;
        }
    }
    buf.push_back(snap);
    while buf.len() > cap {
        buf.pop_front();
    }
}

/// Blend the two buffered snapshots straddling `render_time`. Clamps to the oldest
/// snapshot if `render_time` precedes the buffer and to the newest if it follows (no
/// extrapolation). Returns `None` for an empty buffer. Pure so it is unit-testable.
pub fn sample(buf: &VecDeque<Snapshot>, render_time: f64) -> Option<(Vec3, Quat)> {
    let first = buf.front()?;
    if render_time <= first.server_time {
        return Some((first.pos, first.rot));
    }
    let last = buf.back().unwrap();
    if render_time >= last.server_time {
        return Some((last.pos, last.rot));
    }
    for (a, b) in buf.iter().zip(buf.iter().skip(1)) {
        if a.server_time <= render_time && render_time <= b.server_time {
            let span = b.server_time - a.server_time;
            let alpha = if span > 1e-9 {
                ((render_time - a.server_time) / span) as f32
            } else {
                1.0
            };
            return Some((a.pos.lerp(b.pos, alpha), a.rot.slerp(b.rot, alpha)));
        }
    }
    Some((last.pos, last.rot)) // unreachable given the clamps above
}

/// Fold a fresh `client_now − latest_server_time` observation into the smoothed
/// offset, seeding exactly on the first sample.
fn update_offset(clock: &mut NetClockOffset, observed: f64) {
    if clock.initialized {
        clock.offset += (observed - clock.offset) * OFFSET_SMOOTHING;
    } else {
        clock.offset = observed;
        clock.initialized = true;
    }
}

/// Attach an (empty) [`NetInterpolation`] ring buffer to each replicated plane once
/// it carries both a [`PlaneId`] and a [`FlightState`]. Not keyed on `Added<…>` so it
/// is robust to the two components arriving in different replication ticks. Planes
/// render as gizmos (`draw_plane_gizmos`), so no mesh/material is needed here. The
/// buffer stays empty until [`ingest_snapshots`] records the first server-timed pose.
fn decorate_replicated_plane(
    mut commands: Commands,
    planes: Query<Entity, (With<PlaneId>, With<FlightState>, Without<NetInterpolation>)>,
) {
    for entity in &planes {
        commands.entity(entity).insert(NetInterpolation::default());
    }
}

/// Record a server-timed [`Snapshot`] for each plane that received a replication
/// update this frame, and fold the arrival into the shared [`NetClockOffset`]. Runs
/// after [`ClientSystems::Receive`] so it sees the values replicon just applied.
///
/// [`EntityReplicated`] carries the exact [`RepliconTick`] of the applied data (it
/// fires on mutations, unlike the global `ServerUpdateTick`). Multiple ticks can be
/// applied in one client frame; since only the latest [`FlightState`] value survives,
/// we collapse a batch to its newest tick and push a single snapshot at that
/// server time.
fn ingest_snapshots(
    time: Res<Time>,
    mut replicated: MessageReader<EntityReplicated>,
    mut planes: Query<(&FlightState, &mut NetInterpolation)>,
    mut clock: ResMut<NetClockOffset>,
) {
    // Newest tick per entity this frame.
    let mut newest: HashMap<Entity, u32> = HashMap::new();
    for msg in replicated.read() {
        let slot = newest.entry(msg.entity).or_insert(0);
        *slot = (*slot).max(msg.tick.get());
    }
    if newest.is_empty() {
        return;
    }

    let now = time.elapsed_secs_f64();
    let mut latest_server_time = f64::NEG_INFINITY;
    for (entity, tick) in newest {
        let Ok((state, mut interp)) = planes.get_mut(entity) else {
            continue; // buffer not attached yet (decorate runs next); catch it next frame
        };
        let server_time = tick as f64 / SERVER_TICK_HZ;
        push_snapshot(
            &mut interp.snapshots,
            Snapshot {
                server_time,
                pos: state.position,
                rot: state.attitude,
            },
            SNAPSHOT_CAP,
        );
        latest_server_time = latest_server_time.max(server_time);
    }

    if latest_server_time.is_finite() {
        update_offset(&mut clock, now - latest_server_time);
    }
}

/// Rebuild a [`PlaneTuningHandle`] on the client from the replicated
/// [`PlaneTuningPath`] (a `Handle<T>` cannot be replicated). Once the handle is
/// present the asset loads via the usual `.tuning.ron` loader, so the HUD's tuning
/// dropdown / `T`-key cycler enumerate profiles exactly as in the local-sim build.
fn reconstruct_tuning_handle(
    mut commands: Commands,
    // `Option` so the system is a no-op in a headless test app that registers the
    // protocol without an `AssetPlugin`; the real client always has one.
    asset_server: Option<Res<AssetServer>>,
    planes: Query<(Entity, &PlaneTuningPath), Without<PlaneTuningHandle>>,
) {
    let Some(asset_server) = asset_server else {
        return;
    };
    for (entity, path) in &planes {
        let handle: Handle<crate::controllers::PlaneTuning> = asset_server.load(path.0.clone());
        commands.entity(entity).insert(PlaneTuningHandle(handle));
    }
}

/// Write the interpolated render pose into each plane's [`Transform`] every frame.
/// The playback point `render_time = client_now − offset − RENDER_DELAY` maps client
/// wall-clock into server-time space, where it lands between two uniformly-spaced
/// buffered snapshots — so the rendered motion is smooth regardless of client frame
/// jitter. Planes with an empty buffer (pre-warmup) keep their replicated `Transform`.
fn render_net_interpolation(
    time: Res<Time>,
    clock: Res<NetClockOffset>,
    mut planes: Query<(&NetInterpolation, &mut Transform)>,
) {
    if !clock.initialized {
        return;
    }
    let render_time = time.elapsed_secs_f64() - clock.offset - RENDER_DELAY;
    for (interp, mut transform) in &mut planes {
        if let Some((pos, rot)) = sample(&interp.snapshots, render_time) {
            transform.translation = pos;
            transform.rotation = rot;
        }
    }
}

/// The server address a [`start_renet_client`] run connects to. Inserted by the
/// binary (e.g. the `--connect` bridge / Phase 5 menu) only when a connection is
/// requested.
#[derive(Resource, Clone, Copy, Debug)]
pub struct ConnectTarget(pub SocketAddr);

/// Build a renet [`RenetClient`] + netcode transport for `server_addr`, using the
/// replicon channel configs. Mirrors [`crate::net::server::start_renet_server`].
pub fn connect_to_server(
    server_addr: SocketAddr,
    channels: &RepliconChannels,
) -> Result<(RenetClient, NetcodeClientTransport)> {
    let client = RenetClient::new(ConnectionConfig {
        server_channels_config: channels.server_configs(),
        client_channels_config: channels.client_configs(),
        ..Default::default()
    });

    let current_time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH)?;
    // A locally-unique client id; the server uses unsecure auth over LAN/localhost.
    let client_id = current_time.as_millis() as u64;
    let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0))?;
    let authentication = ClientAuthentication::Unsecure {
        client_id,
        protocol_id: PROTOCOL_ID,
        server_addr,
        user_data: None,
    };
    let transport = NetcodeClientTransport::new(current_time, authentication, socket)?;
    Ok((client, transport))
}

/// `Startup` system that opens the client transport to [`ConnectTarget`]. Added by
/// the binary when `--connect` is passed; kept out of [`ClientNetPlugin`] so plain
/// builds/tests never bind a socket.
pub fn start_renet_client(
    mut commands: Commands,
    channels: Res<RepliconChannels>,
    target: Res<ConnectTarget>,
) -> Result {
    let (client, transport) = connect_to_server(target.0, &channels)?;
    commands.insert_resource(client);
    commands.insert_resource(transport);
    info!("client: connecting to {}", target.0);
    Ok(())
}

/// Client-side replicated-state rendering: decorate replicated planes and blend
/// their poses into `Transform`. Add after `RepliconPlugins` +
/// [`crate::net::NetProtocolPlugin`]. The transport itself is opened separately
/// (see [`start_renet_client`]).
pub struct ClientNetPlugin;

impl Plugin for ClientNetPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<NetClockOffset>();
        app.add_systems(PreUpdate, ingest_snapshots.after(ClientSystems::Receive));
        app.add_systems(
            Update,
            (
                decorate_replicated_plane,
                reconstruct_tuning_handle,
                render_net_interpolation,
            )
                .chain(),
        );

        // Populate `ModelLibrary` for the HUD model dropdown / `T`-key cycler. On
        // the client `SimControlPlugin` (which normally scans) is compiled out, so
        // run the scan here. Only relevant in an inference-enabled client build.
        #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
        app.add_systems(Startup, crate::controllers::sim_control::scan_models);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn snap(t: f64, pos: Vec3) -> Snapshot {
        Snapshot {
            server_time: t,
            pos,
            rot: Quat::IDENTITY,
        }
    }

    fn buf(snaps: impl IntoIterator<Item = Snapshot>) -> VecDeque<Snapshot> {
        snaps.into_iter().collect()
    }

    #[test]
    fn sample_blends_between_straddling_snapshots() {
        let b = buf([
            snap(1.0, Vec3::ZERO),
            snap(2.0, Vec3::new(10.0, 20.0, -30.0)),
        ]);
        let (pos, _) = sample(&b, 1.5).unwrap();
        assert!(
            (pos - Vec3::new(5.0, 10.0, -15.0)).length() < 1e-4,
            "got {pos:?}"
        );
    }

    #[test]
    fn sample_uses_the_correct_pair_from_a_longer_buffer() {
        let b = buf([
            snap(1.0, Vec3::new(0.0, 0.0, 0.0)),
            snap(2.0, Vec3::new(10.0, 0.0, 0.0)),
            snap(3.0, Vec3::new(10.0, 10.0, 0.0)),
        ]);
        // render_time 2.5 straddles the 2.0/3.0 pair → halfway up in Y.
        let (pos, _) = sample(&b, 2.5).unwrap();
        assert!(
            (pos - Vec3::new(10.0, 5.0, 0.0)).length() < 1e-4,
            "got {pos:?}"
        );
    }

    #[test]
    fn sample_clamps_before_and_after_the_buffer() {
        let b = buf([snap(1.0, Vec3::ZERO), snap(2.0, Vec3::new(1.0, 2.0, 3.0))]);
        // Before the oldest → oldest pose.
        assert!((sample(&b, 0.0).unwrap().0 - Vec3::ZERO).length() < 1e-4);
        // After the newest → newest pose (no extrapolation).
        assert!((sample(&b, 9.0).unwrap().0 - Vec3::new(1.0, 2.0, 3.0)).length() < 1e-4);
    }

    #[test]
    fn sample_returns_none_for_empty_buffer() {
        let b: VecDeque<Snapshot> = VecDeque::new();
        assert!(sample(&b, 1.0).is_none());
    }

    #[test]
    fn push_snapshot_caps_and_drops_oldest() {
        let mut b = VecDeque::new();
        for i in 0..10 {
            push_snapshot(&mut b, snap(i as f64, Vec3::splat(i as f32)), 8);
        }
        assert_eq!(b.len(), 8, "buffer capped at 8");
        assert_eq!(b.front().unwrap().server_time, 2.0, "oldest two dropped");
        assert_eq!(b.back().unwrap().server_time, 9.0, "newest retained");
    }

    #[test]
    fn push_snapshot_supersedes_same_tick_and_ignores_stale() {
        let mut b = VecDeque::new();
        push_snapshot(&mut b, snap(1.0, Vec3::ZERO), 8);
        // Same server_time → replace in place (a re-push of the same tick).
        push_snapshot(&mut b, snap(1.0, Vec3::new(5.0, 5.0, 5.0)), 8);
        assert_eq!(b.len(), 1);
        assert_eq!(b.back().unwrap().pos, Vec3::new(5.0, 5.0, 5.0));
        // Strictly older → dropped.
        push_snapshot(&mut b, snap(0.5, Vec3::new(9.0, 9.0, 9.0)), 8);
        assert_eq!(b.len(), 1, "stale snapshot ignored");
        assert_eq!(b.back().unwrap().pos, Vec3::new(5.0, 5.0, 5.0));
    }

    #[test]
    fn update_offset_seeds_then_low_passes() {
        let mut clock = NetClockOffset::default();
        update_offset(&mut clock, 10.0);
        assert!(clock.initialized);
        assert!((clock.offset - 10.0).abs() < 1e-9, "seeded exactly");
        update_offset(&mut clock, 20.0);
        // Low-pass toward 20 by OFFSET_SMOOTHING (0.1).
        assert!((clock.offset - 11.0).abs() < 1e-9, "got {}", clock.offset);
    }
}
