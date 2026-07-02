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
//! writing another avoids any replication/self-write feedback: replicon keeps
//! overwriting `FlightState` from the server, and we keep deriving a render pose
//! that lags ~2 server ticks so there is always a prev/curr pair to blend.

use std::net::{Ipv4Addr, SocketAddr, UdpSocket};
use std::time::SystemTime;

use bevy::prelude::*;
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

/// How far behind the latest received snapshot the rendered pose is held, so a
/// prev/curr pair is always available to interpolate between. ~2 server ticks at
/// 64 Hz.
const RENDER_DELAY: f32 = 2.0 / 64.0;

/// Two-snapshot pose buffer for a replicated plane. Filled from [`FlightState`]
/// each time replication updates it ([`buffer_net_pose`]); blended into the
/// entity's [`Transform`] at `now - RENDER_DELAY` ([`render_net_interpolation`]).
#[derive(Component, Debug, Clone, PartialEq)]
pub struct NetInterpolation {
    pub prev_pos: Vec3,
    pub prev_rot: Quat,
    pub prev_time: f32,
    pub curr_pos: Vec3,
    pub curr_rot: Quat,
    pub curr_time: f32,
}

impl NetInterpolation {
    /// Both snapshots seeded to the same pose at `time` (no motion until the next
    /// replication update arrives).
    fn at(pos: Vec3, rot: Quat, time: f32) -> Self {
        Self {
            prev_pos: pos,
            prev_rot: rot,
            prev_time: time,
            curr_pos: pos,
            curr_rot: rot,
            curr_time: time,
        }
    }
}

/// Blend the buffered prev/curr poses at `render_time`. Clamps to the curr pose
/// once `render_time` reaches/exceeds `curr_time` (and holds the latest pose if the
/// two snapshots share a timestamp). Pure so it is unit-testable without an app.
pub fn interpolate(interp: &NetInterpolation, render_time: f32) -> (Vec3, Quat) {
    let span = interp.curr_time - interp.prev_time;
    let alpha = if span > 1e-6 {
        ((render_time - interp.prev_time) / span).clamp(0.0, 1.0)
    } else {
        1.0
    };
    (
        interp.prev_pos.lerp(interp.curr_pos, alpha),
        interp.prev_rot.slerp(interp.curr_rot, alpha),
    )
}

/// Attach a [`NetInterpolation`] buffer to each replicated plane once it carries
/// both a [`PlaneId`] and a [`FlightState`]. Not keyed on `Added<…>` so it is
/// robust to the two components arriving in different replication ticks. Planes
/// render as gizmos (`draw_plane_gizmos`), so no mesh/material is needed here.
fn decorate_replicated_plane(
    mut commands: Commands,
    time: Res<Time>,
    planes: Query<(Entity, &FlightState), (With<PlaneId>, Without<NetInterpolation>)>,
) {
    let now = time.elapsed_secs();
    for (entity, state) in &planes {
        commands
            .entity(entity)
            .insert(NetInterpolation::at(state.position, state.attitude, now));
    }
}

/// Push a fresh snapshot whenever replication updates a plane's [`FlightState`],
/// shifting the previous curr into prev. Runs after [`ClientSystems::Receive`] so
/// it sees the values replicon just applied.
fn buffer_net_pose(
    time: Res<Time>,
    mut planes: Query<(&FlightState, &mut NetInterpolation), Changed<FlightState>>,
) {
    let now = time.elapsed_secs();
    for (state, mut interp) in &mut planes {
        interp.prev_pos = interp.curr_pos;
        interp.prev_rot = interp.curr_rot;
        interp.prev_time = interp.curr_time;
        interp.curr_pos = state.position;
        interp.curr_rot = state.attitude;
        interp.curr_time = now;
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
fn render_net_interpolation(
    time: Res<Time>,
    mut planes: Query<(&NetInterpolation, &mut Transform)>,
) {
    let render_time = time.elapsed_secs() - RENDER_DELAY;
    for (interp, mut transform) in &mut planes {
        let (pos, rot) = interpolate(interp, render_time);
        transform.translation = pos;
        transform.rotation = rot;
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
        app.add_systems(PreUpdate, buffer_net_pose.after(ClientSystems::Receive));
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

    #[test]
    fn interpolate_blends_at_midpoint() {
        let interp = NetInterpolation {
            prev_pos: Vec3::new(0.0, 0.0, 0.0),
            prev_rot: Quat::IDENTITY,
            prev_time: 1.0,
            curr_pos: Vec3::new(10.0, 20.0, -30.0),
            curr_rot: Quat::from_rotation_y(std::f32::consts::FRAC_PI_2),
            curr_time: 2.0,
        };
        let (pos, rot) = interpolate(&interp, 1.5);
        assert!(
            (pos - Vec3::new(5.0, 10.0, -15.0)).length() < 1e-4,
            "got {pos:?}"
        );
        let expected = Quat::IDENTITY.slerp(interp.curr_rot, 0.5);
        assert!(rot.angle_between(expected) < 1e-4);
    }

    #[test]
    fn interpolate_clamps_past_curr_time() {
        let interp = NetInterpolation::at(Vec3::ZERO, Quat::IDENTITY, 0.0);
        let mut moved = interp.clone();
        moved.curr_pos = Vec3::new(1.0, 2.0, 3.0);
        moved.prev_time = 0.0;
        moved.curr_time = 1.0;
        // render_time well past curr_time → fully at curr.
        let (pos, _) = interpolate(&moved, 5.0);
        assert!(
            (pos - Vec3::new(1.0, 2.0, 3.0)).length() < 1e-4,
            "got {pos:?}"
        );
    }

    #[test]
    fn interpolate_holds_when_snapshots_share_a_timestamp() {
        let interp = NetInterpolation::at(Vec3::new(4.0, 5.0, 6.0), Quat::IDENTITY, 3.0);
        let (pos, _) = interpolate(&interp, 0.0);
        assert!(
            (pos - Vec3::new(4.0, 5.0, 6.0)).length() < 1e-4,
            "got {pos:?}"
        );
    }
}
