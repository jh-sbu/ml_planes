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
const TICK: f64 = 1.0 / SERVER_TICK_HZ;

/// Target lag of the playback clock behind the newest received snapshot. Must exceed
/// the client's snapshot inter-arrival interval *including jitter* so the playback
/// point always sits between two buffered snapshots (never past the newest, which
/// would clamp-and-stall). ~4 ticks (≈62 ms) covers a jittery ~35 fps client where
/// arrivals span 1–3 ticks.
const RENDER_DELAY: f64 = 4.0 * TICK;

/// Healthy band for the playback clock's lag behind the newest snapshot. The clock
/// free-runs on real `dt` (smooth by construction) and only hard-resyncs to
/// `newest − RENDER_DELAY` when it leaves this band — i.e. on genuine starvation
/// (`< MIN_LEAD`, about to outrun the buffer) or a large hitch / time-accel jump
/// (`> MAX_LEAD`). At 1× the client and server both run real-time, so after warmup the
/// clock stays in-band and never resyncs → no periodic jump.
const MIN_LEAD: f64 = 1.0 * TICK;
const MAX_LEAD: f64 = 10.0 * TICK;

/// Ring-buffer depth per plane (≈250 ms at 64 Hz) — comfortably covers `MAX_LEAD`
/// plus history behind the playback point, at any client frame rate.
const SNAPSHOT_CAP: usize = 16;

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

/// Shared playback clock, in the server's tick-time space. All planes ride the same
/// server tick timeline, so one clock drives them all. `playback` advances purely by
/// real `dt` between resyncs, so the rendered motion is smooth regardless of client
/// frame-time or snapshot-arrival jitter — the fix for the gizmo pulse.
#[derive(Resource, Debug, Clone, Copy, Default)]
pub struct NetRenderClock {
    /// Server time we currently render at.
    pub playback: f64,
    /// Newest snapshot `server_time` seen so far.
    pub latest: f64,
    pub initialized: bool,
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

/// One frame's worth of playback-clock movement, reported by [`advance_clock`] for
/// diagnostics.
#[derive(Debug, Clone, Copy, PartialEq)]
struct ClockStep {
    /// `latest − playback` after advancing by `dt` but *before* any resync, so a clock
    /// grinding down toward [`MIN_LEAD`] stays visible instead of being masked by the
    /// snap (which always lands the lead exactly on [`RENDER_DELAY`]).
    lead: f64,
    /// Playback displacement if a resync fired: `new − old`. **Negative means a backward
    /// teleport** — the starvation branch snapping from near `latest` back to
    /// `latest − RENDER_DELAY`. Every plane samples the one shared clock, so a backward
    /// snap moves them all back together.
    resync_delta: Option<f64>,
}

/// Advance the free-running playback clock by real `dt`, resyncing to
/// `latest − RENDER_DELAY` only when it leaves the healthy lead band. Pure (takes the
/// clock + `dt`) so the resync policy is unit-testable without an app. Returns `None`
/// before the clock is seeded, else the [`ClockStep`] taken.
fn advance_clock(clock: &mut NetRenderClock, dt: f64) -> Option<ClockStep> {
    if !clock.initialized {
        return None; // seeded by ingest on the first snapshot
    }
    clock.playback += dt;
    let lead = clock.latest - clock.playback;
    let resync_delta = if !(MIN_LEAD..=MAX_LEAD).contains(&lead) {
        let before = clock.playback;
        clock.playback = clock.latest - RENDER_DELAY;
        Some(clock.playback - before)
    } else {
        None
    };
    Some(ClockStep { lead, resync_delta })
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
/// update this frame, and advance the shared [`NetRenderClock`]'s `latest`. Runs after
/// [`ClientSystems::Receive`] so it sees the values replicon just applied.
///
/// [`EntityReplicated`] carries the exact [`RepliconTick`] of the applied data (it
/// fires on mutations, unlike the global `ServerUpdateTick`). Multiple ticks can be
/// applied in one client frame; since only the latest [`FlightState`] value survives,
/// we collapse a batch to its newest tick and push a single snapshot at that
/// server time.
fn ingest_snapshots(
    mut replicated: MessageReader<EntityReplicated>,
    mut planes: Query<(&FlightState, &mut NetInterpolation)>,
    mut clock: ResMut<NetRenderClock>,
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
        clock.latest = clock.latest.max(latest_server_time);
        if !clock.initialized {
            // Seed the playback clock one RENDER_DELAY behind the first snapshot.
            clock.playback = clock.latest - RENDER_DELAY;
            clock.initialized = true;
        }
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

/// Advance the shared [`NetRenderClock`] once per frame, before rendering. Kept
/// separate from [`render_net_interpolation`] (which only reads the clock) so the
/// resync policy runs exactly once regardless of plane count — which also makes this
/// the one honest place to count resyncs into [`ClockDiag`].
fn advance_render_clock(
    time: Res<Time>,
    mut clock: ResMut<NetRenderClock>,
    mut diag: ResMut<ClockDiag>,
) {
    let Some(step) = advance_clock(&mut clock, time.delta_secs_f64()) else {
        return; // clock not seeded yet — nothing meaningful to record
    };
    if let Some(delta) = step.resync_delta {
        diag.resyncs += 1;
        if delta < 0.0 {
            diag.max_back_jump = diag.max_back_jump.max(-delta);
        }
    }
    diag.lead_min = diag.lead_min.min(step.lead);
    diag.lead_max = diag.lead_max.max(step.lead);
}

/// Temporary per-second aggregate of the shared [`NetRenderClock`]'s health, filled by
/// [`advance_render_clock`] and reported alongside the sampled-output stats by
/// [`render_net_interpolation`], which also resets it each window.
///
/// Exists to test one hypothesis: the planes' *synchronized* back-and-forth pulse is the
/// starvation branch of [`advance_clock`] teleporting the shared playback clock backward.
/// `resyncs` should then match the visible pulse rate, `max_back_jump` should be a few
/// ticks, and `lead_min` should graze [`MIN_LEAD`] just before each snap.
#[derive(Resource, Debug)]
struct ClockDiag {
    resyncs: u32,
    /// Largest backward playback teleport this window, in seconds (0 if none).
    max_back_jump: f64,
    lead_min: f64,
    lead_max: f64,
}

impl Default for ClockDiag {
    fn default() -> Self {
        Self {
            resyncs: 0,
            max_back_jump: 0.0,
            // Sentinels so the first sample of the window wins either comparison.
            lead_min: f64::INFINITY,
            lead_max: f64::NEG_INFINITY,
        }
    }
}

/// Temporary per-second aggregate of the *sampled output* smoothness (first plane):
/// counts direction reversals in the rendered position. A smooth trajectory reverses
/// ~never; a back-and-forth pulse reverses several times per second.
#[derive(Default)]
struct OutputDiag {
    window_start: f64,
    frames: u32,
    reversals: u32,
    max_step: f32,
    prev_pos: Option<Vec3>,
    prev_delta: Option<Vec3>,
}

/// Write the interpolated render pose into each plane's [`Transform`] every frame,
/// sampling every plane at the shared playback time. The playback clock lives in
/// server-time space and advances smoothly (see [`advance_render_clock`]), so the
/// rendered motion is decoupled from client frame / arrival jitter. Planes with an
/// empty buffer (pre-warmup) keep their replicated `Transform`.
fn render_net_interpolation(
    time: Res<Time>,
    clock: Res<NetRenderClock>,
    mut planes: Query<(&NetInterpolation, &mut Transform)>,
    mut diag: Local<OutputDiag>,
    mut clock_diag: ResMut<ClockDiag>,
) {
    if !clock.initialized {
        return;
    }
    let render_time = clock.playback;
    let mut first: Option<Vec3> = None;
    for (interp, mut transform) in &mut planes {
        if let Some((pos, rot)) = sample(&interp.snapshots, render_time) {
            transform.translation = pos;
            transform.rotation = rot;
            if first.is_none() {
                first = Some(pos);
            }
        }
    }

    // Temporary diagnostic: characterise the sampled output of the first plane.
    if let Some(pos) = first {
        diag.frames += 1;
        if let Some(prev) = diag.prev_pos {
            let delta = pos - prev;
            diag.max_step = diag.max_step.max(delta.length());
            if let Some(pd) = diag.prev_delta {
                if delta.dot(pd) < 0.0 && delta.length_squared() > 1e-8 {
                    diag.reversals += 1;
                }
            }
            diag.prev_delta = Some(delta);
        }
        diag.prev_pos = Some(pos);

        let now = time.elapsed_secs_f64();
        if now - diag.window_start >= 1.0 {
            // Leads in ticks, to read directly against MIN_LEAD / RENDER_DELAY / MAX_LEAD.
            // A window with no clock samples leaves the ±INF sentinels — print 0 instead.
            let (lead_min, lead_max) = if clock_diag.lead_min.is_finite() {
                (clock_diag.lead_min / TICK, clock_diag.lead_max / TICK)
            } else {
                (0.0, 0.0)
            };
            info!(
                "net-diag output: {:>3} frames/s  reversals={}  max_step={:.3}m  |  \
                 clock: resyncs={} max_back_jump={:.1}ms lead={:.2}..{:.2} ticks",
                diag.frames,
                diag.reversals,
                diag.max_step,
                clock_diag.resyncs,
                clock_diag.max_back_jump * 1000.0,
                lead_min,
                lead_max,
            );
            *diag = OutputDiag {
                window_start: now,
                prev_pos: diag.prev_pos,
                prev_delta: diag.prev_delta,
                ..Default::default()
            };
            *clock_diag = ClockDiag::default();
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
        app.init_resource::<NetRenderClock>();
        app.init_resource::<ClockDiag>();
        app.add_systems(PreUpdate, ingest_snapshots.after(ClientSystems::Receive));
        app.add_systems(
            Update,
            (
                decorate_replicated_plane,
                reconstruct_tuning_handle,
                advance_render_clock,
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
    fn advance_clock_free_runs_in_band() {
        // Seeded one RENDER_DELAY behind the newest snapshot (as ingest does).
        let latest = 10.0;
        let mut clock = NetRenderClock {
            playback: latest - RENDER_DELAY,
            latest,
            initialized: true,
        };
        // A frame's worth of dt keeps it in-band → no resync, playback += dt.
        let before = clock.playback;
        let step = advance_clock(&mut clock, TICK).expect("initialised clock steps");
        assert!(
            step.resync_delta.is_none(),
            "in-band advance must not resync"
        );
        assert!((clock.playback - (before + TICK)).abs() < 1e-12);
    }

    #[test]
    fn advance_clock_resyncs_when_starved() {
        // Playback has caught up to within < MIN_LEAD of latest (data starvation):
        // lead = 0 < MIN_LEAD → resync back to latest - RENDER_DELAY.
        let latest = 10.0;
        let mut clock = NetRenderClock {
            playback: latest, // lead 0
            latest,
            initialized: true,
        };
        let step = advance_clock(&mut clock, TICK).expect("initialised clock steps");
        assert!(step.resync_delta.is_some(), "starved clock must resync");
        assert!((clock.playback - (latest - RENDER_DELAY)).abs() < 1e-12);
    }

    /// The starvation resync is a *backward* teleport: playback sits near `latest` and is
    /// snapped back to `latest − RENDER_DELAY`. Every plane samples the one shared clock,
    /// so this is the suspected cause of all planes hopping backward in perfect sync.
    /// Pins both the sign and the magnitude the net-diag `max_back_jump` should report.
    #[test]
    fn advance_clock_starvation_resync_jumps_backward() {
        let latest = 10.0;
        let mut clock = NetRenderClock {
            playback: latest,
            latest,
            initialized: true,
        };
        let step = advance_clock(&mut clock, TICK).expect("initialised clock steps");
        let delta = step.resync_delta.expect("starved clock must resync");
        assert!(
            delta < 0.0,
            "starvation resync teleports backward, got {delta}"
        );
        // playback reached latest + TICK after the advance, then snapped to
        // latest - RENDER_DELAY ⇒ a jump of -(RENDER_DELAY + TICK).
        assert!(
            (delta - -(RENDER_DELAY + TICK)).abs() < 1e-12,
            "expected {} got {delta}",
            -(RENDER_DELAY + TICK)
        );
        assert!(
            step.lead < MIN_LEAD,
            "reported lead is the pre-resync excursion, got {}",
            step.lead
        );
    }

    #[test]
    fn advance_clock_resyncs_when_far_behind() {
        // A large jump in latest (hitch / time-accel) pushes lead > MAX_LEAD → resync.
        let latest = 10.0;
        let mut clock = NetRenderClock {
            playback: latest - (MAX_LEAD + 5.0 * TICK),
            latest,
            initialized: true,
        };
        let step = advance_clock(&mut clock, TICK).expect("initialised clock steps");
        let delta = step.resync_delta.expect("clock too far behind must resync");
        assert!(delta > 0.0, "catch-up resync moves forward, got {delta}");
        assert!((clock.playback - (latest - RENDER_DELAY)).abs() < 1e-12);
    }

    #[test]
    fn advance_clock_noop_before_init() {
        let mut clock = NetRenderClock::default();
        assert!(advance_clock(&mut clock, TICK).is_none());
        assert_eq!(clock.playback, 0.0, "uninitialised clock does not advance");
    }
}
