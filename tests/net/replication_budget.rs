//! Replication through the real renet transport, in memory.
//!
//! Every other net test drives replicon without a transport, which cannot see the
//! failure this module exists for: renet spends a per-tick byte budget
//! (`ConnectionConfig::available_bytes_per_tick`) and **drops** any unreliable
//! message that does not fit. Replicon sends per-tick mutations unreliably and in a
//! stable order, so a fleet that overflows the budget starves the *same* planes on
//! every tick. They arrive in the initial snapshot and then freeze on the client,
//! while the server flies them normally and nothing logs an error.
//!
//! No socket is bound: [`step`] moves packets between a `RenetServer` and a
//! `RenetClient` by hand, through the same budgeted `get_packets_to_send` a real
//! transport calls once per frame. The server uses the production
//! [`server_connection_config`], so a change to the shipped budget is what this
//! measures.
//!
//! Gated on `server`, like `server_sim`.

use std::collections::HashMap;
use std::time::Duration;

use bevy::prelude::*;
use bevy::state::app::StatesPlugin;
use bevy::time::TimeUpdateStrategy;
use bevy_replicon::prelude::*;
use bevy_replicon_renet::{RenetClient, RenetServer, RepliconRenetPlugins};

use crate::common::build_headless_app_with;
use ml_planes::controllers::{ControllerKind, SimControlPlugin};
use ml_planes::environment::{EnvironmentPlugin, LifecyclePlugin, PendingPlaneSpawn};
use ml_planes::net::{
    server_connection_config, NetProtocolPlugin, ServerScenario, ServerSimPlugin,
};
use ml_planes::plane::{FlightState, PlaneId, PHYSICS_DT};

const CLIENT_ID: u64 = 1;

/// Ticks between the two position samples. Every shipped airframe flies at
/// >= 75 m/s here, so a plane that is being updated moves tens of metres.
const SAMPLE_WINDOW_TICKS: usize = 32;

/// A server-sim app on the production connection config, with the renet server
/// resource in place but no transport.
fn build_server(scenario: &str) -> App {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(StatesPlugin)
            .add_plugins(RepliconPlugins)
            .add_plugins(RepliconRenetPlugins)
            .add_plugins(NetProtocolPlugin)
            .add_plugins(EnvironmentPlugin)
            .add_plugins(LifecyclePlugin)
            .add_plugins(SimControlPlugin)
            .add_plugins(ServerSimPlugin)
            .insert_resource(ServerScenario(scenario.into()));
    });
    let config = server_connection_config(app.world().resource::<RepliconChannels>());
    app.insert_resource(RenetServer::new(config));
    app
}

/// A bare replicon client: the protocol and renet, nothing that renders.
fn build_client() -> App {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        StatesPlugin,
        RepliconPlugins,
        RepliconRenetPlugins,
        NetProtocolPlugin,
    ))
    .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
        PHYSICS_DT,
    )));
    app.finish();
    app
}

/// One frame on each side, then one exchange of packets — the one budgeted send
/// per server frame that the production transport makes.
fn step(server: &mut App, client: &mut App) {
    server.update();
    client.update();
    let mut renet_server = server.world_mut().resource_mut::<RenetServer>();
    let mut renet_client = client.world_mut().resource_mut::<RenetClient>();
    for packet in renet_server
        .get_packets_to_send(CLIENT_ID)
        .expect("the in-memory client stays connected")
    {
        renet_client.process_packet(&packet);
    }
    for packet in renet_client.get_packets_to_send() {
        renet_server
            .process_packet_from(&packet, CLIENT_ID)
            .expect("the in-memory client stays connected");
    }
}

fn positions(app: &mut App) -> HashMap<PlaneId, Vec3> {
    let world = app.world_mut();
    world
        .query::<(&PlaneId, &FlightState)>()
        .iter(world)
        .map(|(id, state)| (*id, state.position))
        .collect()
}

fn pending_spawns(app: &mut App) -> usize {
    let world = app.world_mut();
    world.query::<&PendingPlaneSpawn>().iter(world).count()
}

/// Every plane the server is flying must keep moving on the client too.
///
/// Uses the shipped 500-plane stress scenario (466 planes in this non-`inference`
/// binary, where its RL planes are skipped), because that scenario is the load the
/// server is claimed to carry.
#[test]
fn every_plane_in_the_stress_scenario_keeps_updating_on_the_client() {
    let mut server = build_server("assets/scenarios/stress_500.scenario.ron");

    // Let the scenario spawn and every `.plane.ron` load before connecting.
    for _ in 0..200 {
        server.update();
        if !positions(&mut server).is_empty() && pending_spawns(&mut server) == 0 {
            break;
        }
    }
    assert_eq!(
        pending_spawns(&mut server),
        0,
        "scenario planes never finished spawning"
    );
    let fleet = positions(&mut server).len();
    assert!(fleet > 400, "expected the stress fleet, got {fleet} planes");

    // Not `RenetServer::new_local_client`: it builds the client with the *server's*
    // channel orientation (send on the server channels), which replicon's asymmetric
    // channel sets cannot decode — the client would never see a plane. Build a real
    // client end instead, as `RenetClient::new` does for the netcode transport.
    let mut client = build_client();
    let config = server_connection_config(server.world().resource::<RepliconChannels>());
    let mut renet_client = RenetClient::new(config);
    renet_client.set_connected();
    client.insert_resource(renet_client);
    server
        .world_mut()
        .resource_mut::<RenetServer>()
        .add_connection(CLIENT_ID);

    // Connect and take the initial snapshot. It rides the reliable channel, which
    // re-sends what does not fit, so every plane does arrive eventually.
    for _ in 0..400 {
        step(&mut server, &mut client);
        if positions(&mut client).len() == fleet {
            break;
        }
    }
    assert_eq!(
        positions(&mut client).len(),
        fleet,
        "the client never received the whole fleet"
    );

    let server_before = positions(&mut server);
    let client_before = positions(&mut client);
    for _ in 0..SAMPLE_WINDOW_TICKS {
        step(&mut server, &mut client);
    }
    let server_after = positions(&mut server);
    let client_after = positions(&mut client);

    // Frozen = the server moved it and the client never heard.
    let mut frozen: Vec<PlaneId> = client_before
        .iter()
        .filter(|(id, before)| {
            let moved_on_server = server_before.get(id) != server_after.get(id);
            let moved_on_client = client_after.get(id) != Some(before);
            moved_on_server && !moved_on_client
        })
        .map(|(id, _)| *id)
        .collect();
    frozen.sort_by_key(|id| id.0);

    if !frozen.is_empty() {
        let world = client.world_mut();
        let mut kinds: HashMap<String, usize> = HashMap::new();
        for (id, kind) in world.query::<(&PlaneId, &ControllerKind)>().iter(world) {
            if frozen.contains(id) {
                *kinds.entry(format!("{kind:?}")).or_default() += 1;
            }
        }
        panic!(
            "{} of {fleet} planes moved on the server but froze on the client over \
             {SAMPLE_WINDOW_TICKS} ticks — their per-tick mutations overflow the renet \
             send budget (`available_bytes_per_tick`) and are dropped. Frozen by kind: \
             {kinds:?}. Ids: {:?}",
            frozen.len(),
            frozen.iter().map(|id| id.0).collect::<Vec<_>>(),
        );
    }
}
