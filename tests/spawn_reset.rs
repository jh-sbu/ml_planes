mod common;

use bevy::ecs::system::RunSystemOnce;
use bevy::prelude::*;

use ml_planes::controllers::ManualController;
use ml_planes::environment::spawn_plane;
use ml_planes::plane::FlightState;
use ml_planes::training::SpawnSpec;

/// Verify spawn_plane → sync_flight_state correctly populates FlightState.
///
/// Spawns a plane with known non-default position/velocity, runs one app
/// update (which triggers sync_flight_state via FixedUpdate), then checks
/// that FlightState fields match the spawn spec exactly.
///
/// `run_system_once` is used to spawn synchronously (commands are applied
/// immediately) so the entity is present when `app.update()` runs FixedUpdate.
#[test]
fn spawn_then_sync_flight_state() {
    let mut app = common::build_headless_app();

    let spec = SpawnSpec {
        position: Some(Vec3::new(100.0, 200.0, 300.0)),
        velocity: Some(Vec3::new(50.0, 0.0, 0.0)),
        attitude: Some(Quat::IDENTITY),
        angular_velocity: Some(Vec3::ZERO),
    };
    let cfg = common::generic_jet_config();

    // run_system_once executes the closure as a one-off system and immediately
    // applies deferred Commands — the entity exists in the world before update().
    app.world_mut()
        .run_system_once(
            move |mut commands: Commands, asset_server: Res<AssetServer>| {
                spawn_plane(
                    &mut commands,
                    &asset_server,
                    &spec,
                    Box::new(ManualController::new()),
                    &cfg,
                );
            },
        )
        .expect("spawn_plane system failed");

    // First update: timing infrastructure initializes; FixedUpdate does not fire on
    // frame 0 in Bevy's fixed-timestep model (virtual time accumulator starts empty).
    app.update();
    // Second update: FixedUpdate fires → sync_flight_state reads Transform + Velocity
    // into FlightState before the physics step overwrites them.
    app.update();

    let mut q = app.world_mut().query::<&FlightState>();
    let state = q
        .single(app.world())
        .expect("expected exactly one FlightState entity");

    assert_eq!(state.position, Vec3::new(100.0, 200.0, 300.0));
    assert_eq!(state.velocity, Vec3::new(50.0, 0.0, 0.0));
    assert_eq!(state.attitude, Quat::IDENTITY);
    assert_eq!(state.angular_velocity, Vec3::ZERO);
    assert!(
        (state.airspeed - 50.0).abs() < 0.01,
        "airspeed={} expected ~50.0",
        state.airspeed
    );
    assert!(
        (state.altitude - 200.0).abs() < 0.01,
        "altitude={} expected ~200.0",
        state.altitude
    );
}
