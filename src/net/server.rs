//! Server-side networking: the authoritative-sim plugin and renet transport setup.
//!
//! Phase 3 of the client/server split (`plans/client_server.md`). The dedicated
//! server binary (`src/bin/server.rs`) runs the headless 64 Hz sim
//! (`PlanePlugin` + Rapier + `SimControlPlugin` + `LifecyclePlugin`) and adds the
//! pieces here:
//!
//! - [`ServerSimPlugin`] — scenario spawn, the `Replicated` marker, sim-speed
//!   application, and the client→server command handlers. Deliberately
//!   transport-free so `tests/server_sim.rs` can drive it without binding a UDP
//!   socket.
//! - [`start_renet_server`] — the renet/netcode transport startup system, added
//!   only by the binary.
//!
//! Commands ([`crate::net::protocol`]) are registered with `add_client_event`, so
//! the server receives each as an `On<FromClient<…>>` observer. Most handlers just
//! mutate the component that `SimControlPlugin` (controller rebuilds) or the
//! `LifecyclePlugin` observers already react to, keeping a single authoritative
//! application path shared with the visual app.

use std::net::{Ipv4Addr, UdpSocket};
use std::path::PathBuf;
use std::time::SystemTime;

use bevy::prelude::*;
use bevy_replicon::prelude::*;
use bevy_replicon_renet::netcode::{NetcodeServerTransport, ServerAuthentication, ServerConfig};
use bevy_replicon_renet::renet::ConnectionConfig;
use bevy_replicon_renet::{RenetChannelsExt, RenetServer};

use crate::controllers::{
    ActiveController, ControllerKind, ManualController, SelectedTuningProfile,
};
use crate::environment::{spawn_resolved_scenario, RemovePlaneCommand, SpawnPlaneCommand};
use crate::net::protocol::{
    ManualInputCommand, RemovePlaneNetCommand, SetSimSpeedCommand, SetTuningProfileCommand,
    SpawnPlaneNetCommand, SwitchControllerCommand, DEFAULT_PORT, PROTOCOL_ID,
};
use crate::plane::{NextPlaneId, PlaneId};
use crate::scenario::Scenario;
use crate::sim_speed::SimSpeed;

#[cfg(feature = "inference")]
use crate::controllers::SelectedModel;
#[cfg(feature = "inference")]
use crate::net::protocol::SetModelCommand;

/// Scenario the server spawns on startup (the observe_state `assets/...`
/// convention). Defaults to the bundled default scenario.
#[derive(Resource, Clone, Debug)]
pub struct ServerScenario(pub PathBuf);

impl Default for ServerScenario {
    fn default() -> Self {
        Self(PathBuf::from("assets/scenarios/default.scenario.ron"))
    }
}

/// UDP port the renet transport binds to.
#[derive(Resource, Clone, Copy, Debug)]
pub struct ServerPort(pub u16);

impl Default for ServerPort {
    fn default() -> Self {
        Self(DEFAULT_PORT)
    }
}

/// Authoritative-sim systems and command handlers, minus the renet transport.
///
/// Add this after `RepliconPlugins` + [`crate::net::NetProtocolPlugin`] (so the
/// command events are registered) and alongside the headless sim plugins
/// (`PlanePlugin`, `SimControlPlugin`, `LifecyclePlugin`).
pub struct ServerSimPlugin;

impl Plugin for ServerSimPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ServerScenario>()
            .init_resource::<SimSpeed>();

        app.add_systems(Startup, spawn_startup_scenario);
        app.add_systems(Update, (mark_planes_replicated, apply_sim_speed));

        app.add_observer(on_switch_controller)
            .add_observer(on_set_tuning_profile)
            .add_observer(on_manual_input)
            .add_observer(on_spawn_plane)
            .add_observer(on_remove_plane)
            .add_observer(on_set_sim_speed);

        #[cfg(feature = "inference")]
        app.add_observer(on_set_model);
    }
}

/// Load + resolve the configured scenario and spawn its planes. Mirrors the visual
/// menu's Start Scenario flow (both call [`spawn_resolved_scenario`]).
fn spawn_startup_scenario(
    mut commands: Commands,
    mut ids: ResMut<NextPlaneId>,
    asset_server: Res<AssetServer>,
    scenario: Res<ServerScenario>,
) {
    let path = scenario.0.as_path();
    let resolved = match Scenario::from_path(path).and_then(|s| s.resolve()) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("server: cannot load scenario '{}': {e}", path.display());
            return;
        }
    };

    let result = spawn_resolved_scenario(&mut commands, &mut ids, &asset_server, &resolved);
    for skipped in &result.skipped {
        eprintln!("server: {skipped}");
    }
    println!(
        "server: spawned {} plane(s) from '{}'",
        result.spawned.len(),
        path.display()
    );
}

/// Tag every freshly-spawned plane for replication. Keyed on `Added<PlaneId>` so it
/// covers both the startup scenario and runtime spawns, without feature-gating the
/// shared `spawn_plane` (which the no-net tests and visual client also use).
fn mark_planes_replicated(
    mut commands: Commands,
    planes: Query<Entity, (Added<PlaneId>, Without<Replicated>)>,
) {
    for entity in planes.iter() {
        commands.entity(entity).insert(Replicated);
    }
}

/// Push the authoritative [`SimSpeed`] onto the server's `Time<Virtual>` clock
/// whenever it changes (mirrors the visual HUD's `draw_time_control`).
fn apply_sim_speed(sim_speed: Res<SimSpeed>, mut virtual_time: ResMut<Time<Virtual>>) {
    if !sim_speed.is_changed() {
        return;
    }
    if sim_speed.is_paused() {
        virtual_time.pause();
    } else {
        virtual_time.unpause();
        virtual_time.set_relative_speed(sim_speed.relative_speed());
    }
}

/// Resolve a `PlaneId` to its entity, if a plane with that id is live.
fn entity_for_plane(planes: &Query<(Entity, &PlaneId)>, plane: PlaneId) -> Option<Entity> {
    planes
        .iter()
        .find_map(|(e, id)| (*id == plane).then_some(e))
}

fn on_switch_controller(
    on: On<FromClient<SwitchControllerCommand>>,
    planes: Query<(Entity, &PlaneId)>,
    mut kinds: Query<&mut ControllerKind>,
) {
    let cmd = &on.message;
    if let Some(entity) = entity_for_plane(&planes, cmd.plane) {
        if let Ok(mut kind) = kinds.get_mut(entity) {
            // SimControlPlugin rebuilds ActiveController on the change.
            kind.set_if_neq(cmd.kind);
        }
    }
}

fn on_set_tuning_profile(
    on: On<FromClient<SetTuningProfileCommand>>,
    planes: Query<(Entity, &PlaneId)>,
    mut commands: Commands,
) {
    let cmd = &on.message;
    if let Some(entity) = entity_for_plane(&planes, cmd.plane) {
        // Overwrites any existing profile; SimControlPlugin rebuilds on the change.
        commands
            .entity(entity)
            .insert(SelectedTuningProfile(cmd.profile.clone()));
    }
}

#[cfg(feature = "inference")]
fn on_set_model(
    on: On<FromClient<SetModelCommand>>,
    planes: Query<(Entity, &PlaneId)>,
    mut commands: Commands,
) {
    let cmd = &on.message;
    if let Some(entity) = entity_for_plane(&planes, cmd.plane) {
        commands
            .entity(entity)
            .insert(SelectedModel(cmd.model_stem.clone()));
    }
}

fn on_manual_input(
    on: On<FromClient<ManualInputCommand>>,
    planes: Query<(Entity, &PlaneId)>,
    mut controllers: Query<&mut ActiveController>,
) {
    let cmd = &on.message;
    let Some(entity) = entity_for_plane(&planes, cmd.plane) else {
        return;
    };
    let Ok(mut controller) = controllers.get_mut(entity) else {
        return;
    };
    // Latest-wins: feed the inputs into the plane's ManualController so the next
    // run_flight_controllers tick emits them. No-op if the plane isn't manual.
    if let Some(manual) = controller.0.as_any_mut().downcast_mut::<ManualController>() {
        manual.inputs = cmd.inputs.clone();
    }
}

fn on_spawn_plane(on: On<FromClient<SpawnPlaneNetCommand>>, mut commands: Commands) {
    let cmd = &on.message;
    commands.trigger(SpawnPlaneCommand {
        spec: cmd.spec.clone(),
        kind: cmd.kind,
        config_path: cmd.config_path.clone(),
    });
}

fn on_remove_plane(
    on: On<FromClient<RemovePlaneNetCommand>>,
    planes: Query<(Entity, &PlaneId)>,
    mut commands: Commands,
) {
    if let Some(entity) = entity_for_plane(&planes, on.message.plane) {
        commands.trigger(RemovePlaneCommand(entity));
    }
}

fn on_set_sim_speed(on: On<FromClient<SetSimSpeedCommand>>, mut sim_speed: ResMut<SimSpeed>) {
    // apply_sim_speed propagates this to Time<Virtual> on the next tick.
    sim_speed.set_if_neq(on.message.speed);
}

/// Create the renet [`RenetServer`] + netcode transport bound to [`ServerPort`] and
/// insert both as resources. Runs at `Startup`, after plugin build so the replicon
/// channels are registered. Added only by the server binary — kept out of
/// [`ServerSimPlugin`] so tests never bind a socket.
pub fn start_renet_server(
    mut commands: Commands,
    channels: Res<RepliconChannels>,
    port: Res<ServerPort>,
) -> Result {
    let server = RenetServer::new(ConnectionConfig {
        server_channels_config: channels.server_configs(),
        client_channels_config: channels.client_configs(),
        ..Default::default()
    });

    let current_time = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH)?;
    let socket = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, port.0))?;
    let server_config = ServerConfig {
        current_time,
        max_clients: 64,
        protocol_id: PROTOCOL_ID,
        authentication: ServerAuthentication::Unsecure,
        public_addresses: Default::default(),
    };
    let transport = NetcodeServerTransport::new(server_config, socket)?;

    commands.insert_resource(server);
    commands.insert_resource(transport);
    println!("server: listening on 0.0.0.0:{}", port.0);
    Ok(())
}
