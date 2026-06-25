//! `ml_planes_server` — the dedicated authoritative simulation server.
//!
//! Runs the headless 64 Hz Rapier sim (the same `PlanePlugin` + `EnvironmentPlugin`
//! + `LifecyclePlugin` + `SimControlPlugin` stack the visual app uses) with no
//! rendering, plus the replicon/renet networking layer (`NetProtocolPlugin` +
//! `ServerSimPlugin` + the renet transport). It loads a `.scenario.ron`, spawns its
//! planes with the `Replicated` marker, broadcasts their state to connected clients,
//! and applies the client→server commands defined in `ml_planes::net::protocol`.
//!
//! Run from the project root so the relative `assets/` paths resolve:
//!
//!   cargo run --features server --bin ml_planes_server -- \
//!     --scenario assets/scenarios/default.scenario.ron --port 5555
//!
//! Options:
//!   --scenario PATH   `.scenario.ron` to load (default: the bundled default scene).
//!   --port N          UDP port to bind (default: `ml_planes::net::DEFAULT_PORT`).

use std::path::PathBuf;
use std::time::Duration;

use bevy::app::ScheduleRunnerPlugin;
use bevy::prelude::*;
use bevy::state::app::StatesPlugin;
use bevy_rapier3d::prelude::*;
use bevy_replicon::prelude::RepliconPlugins;
use bevy_replicon_renet::RepliconRenetPlugins;

use ml_planes::controllers::SimControlPlugin;
use ml_planes::environment::{EnvironmentPlugin, LifecyclePlugin};
use ml_planes::net::{
    start_renet_server, NetProtocolPlugin, ServerPort, ServerScenario, ServerSimPlugin,
    DEFAULT_PORT,
};
use ml_planes::plane::PlanePlugin;

fn main() {
    let args = parse_args();

    let mut app = App::new();

    // Headless core, mirroring examples/observe_state.rs and tests/common/mod.rs.
    // ScheduleRunnerPlugin loops the app; cap it at ~240 Hz so the box isn't pegged
    // while the 64 Hz fixed schedule advances on the real-time clock.
    app.add_plugins(
        MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::from_secs_f64(
            1.0 / 240.0,
        ))),
    )
    .add_plugins(bevy::transform::TransformPlugin)
    .add_plugins(bevy::asset::AssetPlugin::default())
    .add_plugins(StatesPlugin)
    .insert_resource(TimestepMode::Fixed {
        dt: 1.0 / 64.0,
        substeps: 1,
    })
    .add_plugins(RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule())
    .add_plugins(PlanePlugin)
    .add_plugins(EnvironmentPlugin)
    .add_plugins(LifecyclePlugin)
    .add_plugins(SimControlPlugin);

    // Networking: shared protocol + renet transport + server-side sim/command handlers.
    app.add_plugins(RepliconPlugins)
        .add_plugins(RepliconRenetPlugins)
        .add_plugins(NetProtocolPlugin)
        .add_plugins(ServerSimPlugin);

    app.insert_resource(ServerScenario(args.scenario))
        .insert_resource(ServerPort(args.port))
        .add_systems(Startup, start_renet_server);

    app.run();
}

// ---------------------------------------------------------------------------
// CLI helpers (mirrors examples/observe_state.rs)

struct Args {
    scenario: PathBuf,
    port: u16,
}

fn parse_args() -> Args {
    let args: Vec<String> = std::env::args().collect();
    let scenario = get_arg(&args, "--scenario")
        .map(PathBuf::from)
        .unwrap_or_else(|| ServerScenario::default().0);
    let port = get_arg(&args, "--port")
        .map(|raw| {
            raw.parse().unwrap_or_else(|_| {
                eprintln!("--port expects a u16, got '{raw}'");
                std::process::exit(2);
            })
        })
        .unwrap_or(DEFAULT_PORT);
    Args { scenario, port }
}

fn get_arg(args: &[String], flag: &str) -> Option<String> {
    args.windows(2)
        .find(|pair| pair[0] == flag)
        .map(|pair| pair[1].clone())
}
