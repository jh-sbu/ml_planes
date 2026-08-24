use std::time::Duration;

use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use bevy_rapier3d::prelude::*;

use ml_planes::environment::PendingPlaneSpawn;
use ml_planes::plane::{PlaneConfig, PlanePlugin, PHYSICS_DT};

/// Build a headless Bevy app with Rapier physics and PlanePlugin.
///
/// **Requires the sim chain.** `PlanePlugin` only adds the 6-DOF FixedUpdate systems (including
/// `sync_flight_state`) under `any(not(feature = "net"), feature = "server")` — see
/// `src/plane/plugin.rs`. On a `net`-without-`server` build the plane never simulates and
/// `FlightState` stays at its `Default`, so tests using this helper gate on `#[cfg(sim_enabled)]`
/// (see `build.rs`) and compile out there. Don't "fix" a spurious failure by touching physics —
/// build with the server feature instead, e.g. `--no-default-features --features "mcp server"`.
///
/// Each `app.update()` advances virtual time by exactly 1/64 s,
/// making FixedUpdate steps deterministic regardless of wall-clock speed.
/// One update = one fixed tick = one Rapier step (no accumulator overflow).
///
/// `app.finish()` is called before returning so that plugins which initialize
/// resources in their `finish()` impl are properly set up (required when
/// driving the app manually via `app.update()` instead of `app.run()`).
#[allow(dead_code)]
pub fn build_headless_app() -> App {
    build_headless_app_with(|_| {})
}

/// Like [`build_headless_app`], but runs `configure` to add extra plugins or
/// systems *before* `app.finish()` (plugins cannot be added after finish).
#[allow(dead_code)]
pub fn build_headless_app_with(configure: impl FnOnce(&mut App)) -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins)
        .add_plugins(bevy::transform::TransformPlugin)
        .add_plugins(bevy::asset::AssetPlugin::default())
        .insert_resource(TimestepMode::Fixed {
            dt: PHYSICS_DT,
            substeps: 1,
        })
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule())
        .add_plugins(PlanePlugin);

    // 1/64 s per update = exactly one fixed tick per update (no accumulator overflow).
    // Tests are deterministic: 1 update = 1 fixed tick = 1 Rapier step.
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
        PHYSICS_DT,
    )));

    configure(&mut app);

    // Finalize plugin setup — equivalent to what app.run() does internally.
    // Required when driving the app manually via app.update().
    app.finish();

    app
}

/// The frozen generic-jet airframe every integration test flies.
///
/// Reads `fixtures/generic_jet.plane.ron` via `include_str!` — a **snapshot**, not a
/// mirror of `assets/planes/generic_jet.plane.ron`. Keeping them separate means
/// retuning a shipped airframe cannot silently move a test's expected numbers; the
/// shipped assets are covered on their own by `tests/core/plane_assets.rs`. The
/// `src/` unit tests get the same values from `plane::config::fixture_jet_config()`.
///
/// Inserted into `Assets<PlaneConfig>` synchronously by callers, bypassing async
/// file loading.
#[allow(dead_code)]
pub fn generic_jet_config() -> PlaneConfig {
    const SRC: &str = include_str!("../../fixtures/generic_jet.plane.ron");
    ron::de::from_str(SRC).expect("fixtures/generic_jet.plane.ron is valid RON")
}

/// Finalize the outstanding plane spawns **without advancing the app**.
///
/// Hands `cfg` over as the `.plane.ron` asset and runs `finalize_pending_spawns` as a
/// one-off system, so the plane exists with no physics tick having run. Use this instead
/// of [`resolve_pending_spawns`] when a test asserts on the exact spawn pose — an
/// `app.update()` would step Rapier and move the plane before the assertion.
///
/// Requires the spawn request to already be in the world (e.g. issued through
/// `run_system_once`, which applies its deferred `Commands` immediately), since nothing
/// here flushes a queued command.
#[allow(dead_code)]
pub fn finalize_pending_spawns_now(app: &mut App, cfg: &PlaneConfig) {
    use bevy::ecs::system::RunSystemOnce;

    let ids: Vec<_> = {
        let world = app.world_mut();
        world
            .query::<&PendingPlaneSpawn>()
            .iter(world)
            .map(|pending| pending.config.id())
            .collect()
    };
    assert!(!ids.is_empty(), "no pending plane spawn to finalize");
    let mut assets = app.world_mut().resource_mut::<Assets<PlaneConfig>>();
    for id in ids {
        assets
            .insert(id, cfg.clone())
            .expect("insert PlaneConfig asset synchronously");
    }
    app.world_mut()
        .run_system_once(ml_planes::environment::finalize_pending_spawns)
        .expect("finalize_pending_spawns system failed");
}

/// Drive every outstanding plane spawn to completion, handing over `cfg` as the
/// `.plane.ron` asset instead of waiting on a real filesystem load.
///
/// Spawning is asset-driven and therefore deferred: `spawn_plane` parks a
/// [`PendingPlaneSpawn`] until its config is available. How long a genuine load takes
/// is not something to assert on — under `--no-default-features` it can land inside the
/// same frame, while `--features visual` pulls `bevy/default` (hence `multi_threaded`)
/// and it does not. Handing the asset over directly makes the test read the same either
/// way; it is the same trick `tests/core/scenario.rs` uses for `Assets<PlaneTuning>`.
///
/// Loops because the spawn may still be sitting in an unflushed command queue on entry
/// (`World::trigger` runs the observer immediately, but its `Commands` apply at the next
/// flush), so the first pass often has nothing to resolve yet. Stops as soon as nothing
/// is pending, to keep the number of simulated ticks — and so any fuel burn or physics
/// motion a caller then measures — to the minimum.
#[allow(dead_code)]
pub fn resolve_pending_spawns(app: &mut App, cfg: &PlaneConfig) {
    for _ in 0..8 {
        let ids: Vec<_> = {
            let world = app.world_mut();
            world
                .query::<&PendingPlaneSpawn>()
                .iter(world)
                .map(|pending| pending.config.id())
                .collect()
        };
        let mut assets = app.world_mut().resource_mut::<Assets<PlaneConfig>>();
        for id in ids {
            assets
                .insert(id, cfg.clone())
                .expect("insert PlaneConfig asset synchronously");
        }
        app.update();

        let world = app.world_mut();
        if world
            .query::<&PendingPlaneSpawn>()
            .iter(world)
            .next()
            .is_none()
        {
            return;
        }
    }
    panic!("pending plane spawns never finalized");
}
