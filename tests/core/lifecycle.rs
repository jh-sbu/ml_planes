//! Runtime plane lifecycle: per-plane config path, spawn/remove commands,
//! automatic indexing, and removal cleanup.

use crate::common::{
    build_headless_app, build_headless_app_with, generic_jet_config, resolve_pending_spawns,
};
use bevy::prelude::*;
use bevy_rapier3d::prelude::AdditionalMassProperties;
#[cfg(feature = "visual")]
use ml_planes::camera::{systems::recover_camera_on_target_loss, CameraMode};
use ml_planes::controllers::{
    ActiveController, ControllerKind, FormationOffset, LevelHoldController, PlaneTuning,
    SelectedTuningProfile, WingmanController,
};
use ml_planes::environment::PendingPlaneSpawn;
use ml_planes::environment::{spawn_plane, LifecyclePlugin, RemovePlaneCommand, SpawnPlaneCommand};
use ml_planes::plane::{
    FlightState, NextPlaneId, PlaneConfig, PlaneConfigHandle, PlaneId, PlaneIndex,
    PlaneTuningHandle, PlaneTuningPath,
};
use ml_planes::training::SpawnSpec;

/// Number of finalized planes (entities carrying a `PlaneId`).
fn count_planes(app: &mut App) -> usize {
    let world = app.world_mut();
    world.query::<&PlaneId>().iter(world).count()
}

/// Number of spawn requests still waiting on their `.plane.ron` asset.
fn count_pending(app: &mut App) -> usize {
    let world = app.world_mut();
    world.query::<&PendingPlaneSpawn>().iter(world).count()
}

/// Collect the `PlaneIndex` ordinal of every live plane.
fn plane_indices(app: &mut App) -> Vec<u32> {
    let world = app.world_mut();
    let mut q = world.query::<&PlaneIndex>();
    let mut v: Vec<u32> = q.iter(world).map(|p| p.0).collect();
    v.sort();
    v
}

/// A `SpawnPlaneCommand` adds a fully-indexed plane; two commands yield
/// distinct increasing indices; `RemovePlaneCommand` despawns it.
#[test]
fn spawn_commands_index_planes_and_remove_despawns() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(LifecyclePlugin);
    });
    // Run once so the app is live before issuing commands.
    app.update();

    app.world_mut().trigger(SpawnPlaneCommand::at(
        Vec3::new(0.0, 1000.0, 0.0),
        Vec3::new(100.0, 0.0, 0.0),
        ControllerKind::LevelHold,
    ));
    app.world_mut().trigger(SpawnPlaneCommand::at(
        Vec3::new(0.0, 1200.0, 0.0),
        Vec3::new(100.0, 0.0, 0.0),
        ControllerKind::Orbit,
    ));
    resolve_pending_spawns(&mut app, &generic_jet_config());

    let indices = plane_indices(&mut app);
    assert_eq!(indices.len(), 2, "two planes spawned via command");
    assert_ne!(
        indices[0], indices[1],
        "spawned planes get distinct indices"
    );

    // Remove one and confirm it despawns; the other survives.
    let entity = {
        let world = app.world_mut();
        let mut q = world.query_filtered::<Entity, With<PlaneIndex>>();
        q.iter(world).next().expect("a plane exists")
    };
    app.world_mut().trigger(RemovePlaneCommand(entity));
    app.update();

    assert_eq!(plane_indices(&mut app).len(), 1, "one plane removed");
}

#[derive(Resource)]
struct SpawnedEntity(Entity);

/// `spawn_plane` must load the aero config from the supplied path,
/// not a hardcoded one.
#[test]
fn spawn_plane_uses_supplied_config_path() {
    let mut app = build_headless_app();
    app.add_systems(Startup, spawn_with_custom_path);
    app.update();

    let world = app.world_mut();
    let spawned = world.resource::<SpawnedEntity>().0;
    let handle = world
        .entity(spawned)
        .get::<PlaneConfigHandle>()
        .expect("spawned plane has PlaneConfigHandle")
        .0
        .clone();
    let expected: Handle<PlaneConfig> = world
        .resource::<AssetServer>()
        .load("planes/cargo_jet.plane.ron");
    assert_eq!(
        handle, expected,
        "spawned plane should carry the config handle for the supplied path"
    );
}

fn spawn_with_custom_path(
    mut commands: Commands,
    mut ids: ResMut<NextPlaneId>,
    asset_server: Res<AssetServer>,
) {
    let spawned = spawn_plane(
        &mut commands,
        &mut ids,
        &asset_server,
        "planes/cargo_jet.plane.ron",
        &SpawnSpec::default(),
        Box::new(LevelHoldController::new(1000.0, 100.0)),
        ControllerKind::LevelHold,
    );
    commands.insert_resource(SpawnedEntity(spawned.entity));
}

/// A runtime-spawned plane whose config ships a `.tuning.ron` sibling must carry
/// the `PlaneTuningHandle` + `SelectedTuningProfile` so the visual
/// `apply_initial_tuning` system can apply the per-config gains — otherwise it
/// silently flies on `LevelHoldController` defaults.
#[test]
fn spawn_command_attaches_tuning_for_config_with_sibling() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(LifecyclePlugin);
    });
    app.update();

    app.world_mut().trigger(SpawnPlaneCommand {
        spec: SpawnSpec {
            position: Some(Vec3::new(0.0, 1000.0, 0.0)),
            velocity: Some(Vec3::new(120.0, 0.0, 0.0)),
            ..Default::default()
        },
        kind: ControllerKind::LevelHold,
        config_path: "planes/cargo_jet.plane.ron".to_string(),
    });
    resolve_pending_spawns(&mut app, &generic_jet_config());

    let world = app.world_mut();
    let entity = {
        let mut q = world.query_filtered::<Entity, With<PlaneIndex>>();
        q.iter(world).next().expect("a plane was spawned")
    };
    let handle = world
        .entity(entity)
        .get::<PlaneTuningHandle>()
        .expect("cargo jet should carry a tuning handle")
        .0
        .clone();
    let expected: Handle<PlaneTuning> = world
        .resource::<AssetServer>()
        .load("planes/cargo_jet.tuning.ron");
    assert_eq!(
        handle, expected,
        "tuning handle should point at the cargo jet's .tuning.ron"
    );
    assert_eq!(
        world
            .entity(entity)
            .get::<SelectedTuningProfile>()
            .expect("spawned plane should select a tuning profile")
            .0,
        "normal",
        "runtime spawns default to the \"normal\" profile"
    );
}

/// A plane is not spawned when its config cannot be loaded.
#[test]
fn spawn_command_with_unloadable_config_spawns_no_plane() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(LifecyclePlugin);
    });
    app.update();

    app.world_mut().trigger(SpawnPlaneCommand {
        spec: SpawnSpec::default(),
        kind: ControllerKind::LevelHold,
        config_path: "planes/no_such_airframe.plane.ron".to_string(),
    });

    // Give the asset server room to reach a terminal load-failure state.
    for _ in 0..64 {
        app.update();
        if count_pending(&mut app) == 0 {
            break;
        }
    }

    assert_eq!(
        count_planes(&mut app),
        0,
        "an unloadable config must not produce a plane"
    );
    assert_eq!(
        count_pending(&mut app),
        0,
        "the pending request must be dropped, not leaked"
    );
}

/// A traversal `config_path` must not escape `assets/`. The existing contract is
/// that a bad path still spawns (never panics) on the generic-jet fallback, so
/// this asserts the fallback *and* that no tuning components were derived from
/// the attacker-supplied path (`PlaneTuningPath` is replicated to clients).
#[test]
fn spawn_command_rejects_traversal_config_path() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(LifecyclePlugin);
    });
    app.update();

    app.world_mut().trigger(SpawnPlaneCommand {
        spec: SpawnSpec {
            position: Some(Vec3::new(0.0, 1000.0, 0.0)),
            velocity: Some(Vec3::new(120.0, 0.0, 0.0)),
            ..Default::default()
        },
        kind: ControllerKind::LevelHold,
        // Resolves to a real airframe if traversal is not rejected.
        config_path: "planes/../planes/cargo_jet.plane.ron".to_string(),
    });
    resolve_pending_spawns(&mut app, &generic_jet_config());

    let world = app.world_mut();
    let entity = {
        let mut q = world.query_filtered::<Entity, With<PlaneIndex>>();
        q.iter(world).next().expect("a plane still spawns")
    };
    // The plane falls back to the generic jet, so it legitimately carries the
    // *default's* tuning. What must never happen is tuning derived from the
    // caller-supplied path — `PlaneTuningPath` is replicated, so that would be
    // a file-existence oracle that crosses the wire.
    let tuning = world.entity(entity).get::<PlaneTuningPath>();
    assert_ne!(
        tuning.map(|p| p.0.as_str()),
        Some("planes/../planes/cargo_jet.tuning.ron"),
        "tuning must not be derived from a traversal path"
    );
    assert_ne!(
        tuning.map(|p| p.0.as_str()),
        Some("planes/cargo_jet.tuning.ron"),
        "`..` must not be resolved into the traversal's target airframe"
    );
}

#[derive(Resource)]
struct LeaderWingman {
    leader: Entity,
    wingman: Entity,
}

/// Removing a wingman's leader drops the wingman to `LevelHold` instead
/// of leaving it silently masquerading as formation flight against a dead leader.
#[test]
fn removing_leader_drops_wingman_to_level_hold() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(LifecyclePlugin);
    });
    app.add_systems(Startup, spawn_leader_and_wingman);
    resolve_pending_spawns(&mut app, &generic_jet_config());

    let (leader, wingman) = {
        let pair = app.world().resource::<LeaderWingman>();
        (pair.leader, pair.wingman)
    };
    assert_eq!(
        *app.world().entity(wingman).get::<ControllerKind>().unwrap(),
        ControllerKind::Wingman,
        "wingman starts as Wingman kind"
    );

    app.world_mut().trigger(RemovePlaneCommand(leader));
    app.update();

    assert_eq!(
        *app.world().entity(wingman).get::<ControllerKind>().unwrap(),
        ControllerKind::LevelHold,
        "orphaned wingman should fall back to LevelHold once its leader is gone"
    );
}

/// A plane can carry `ControllerKind::Wingman` without an actual
/// `WingmanController` because the generic factory falls back to `LevelHold`.
/// It must demote to `LevelHold` like an orphaned wingman.
#[test]
fn wingman_kind_without_wingman_controller_falls_back_to_level_hold() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(LifecyclePlugin);
    });

    let entity = app
        .world_mut()
        .spawn((
            ml_planes::plane::PlaneId(1),
            ControllerKind::Wingman,
            ActiveController(Box::new(LevelHoldController::new(1000.0, 100.0))),
        ))
        .id();

    app.update();

    assert_eq!(
        *app.world().entity(entity).get::<ControllerKind>().unwrap(),
        ControllerKind::LevelHold,
        "a Wingman kind with no WingmanController installed must be demoted to LevelHold"
    );
}

/// A leader still waiting on its `.plane.ron` is live for orphan detection. A wingman
/// whose config lands first must not be demoted because the demotion is one-way.
///
/// The leader here holds a *reserved* handle whose asset is never inserted, which is
/// what a real load-in-flight looks like to `finalize_pending_spawns` (`configs.get`
/// is `None`, `load_state` is not failed) — no dependence on filesystem timing.
#[test]
fn wingman_survives_a_leader_still_waiting_on_its_config() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(LifecyclePlugin);
    });

    let leader_id = PlaneId(1);
    let never_loads = app
        .world()
        .resource::<Assets<PlaneConfig>>()
        .reserve_handle();
    app.world_mut().spawn(PendingPlaneSpawn {
        plane_id: leader_id,
        spec: SpawnSpec::default(),
        kind: ControllerKind::LevelHold,
        controller: Some(Box::new(LevelHoldController::new(1000.0, 100.0))),
        config: never_loads,
        config_path: "planes/generic_jet.plane.ron".to_string(),
    });

    let leader_state = FlightState {
        position: Vec3::new(0.0, 1000.0, 0.0),
        velocity: Vec3::new(100.0, 0.0, 0.0),
        airspeed: 100.0,
        altitude: 1000.0,
        ..Default::default()
    };
    let offset = FormationOffset::default();
    let own_state = FlightState {
        position: leader_state.position + offset.offset_body,
        ..leader_state.clone()
    };
    let wingman = app
        .world_mut()
        .spawn((
            PlaneId(2),
            ControllerKind::Wingman,
            ActiveController(Box::new(WingmanController::new(
                leader_id,
                &leader_state,
                &own_state,
                offset,
            ))),
        ))
        .id();

    app.update();

    assert_eq!(
        count_pending(&mut app),
        1,
        "the leader must still be pending — otherwise this test proves nothing"
    );
    assert_eq!(
        *app.world().entity(wingman).get::<ControllerKind>().unwrap(),
        ControllerKind::Wingman,
        "a wingman must not be demoted while its leader is still loading its config"
    );
}

fn spawn_leader_and_wingman(
    mut commands: Commands,
    mut ids: ResMut<NextPlaneId>,
    asset_server: Res<AssetServer>,
) {
    let leader_pos = Vec3::new(0.0, 1000.0, 0.0);
    let vel = Vec3::new(100.0, 0.0, 0.0);
    let leader_state = FlightState {
        position: leader_pos,
        velocity: vel,
        airspeed: 100.0,
        altitude: 1000.0,
        ..Default::default()
    };
    let leader = spawn_plane(
        &mut commands,
        &mut ids,
        &asset_server,
        "planes/generic_jet.plane.ron",
        &SpawnSpec {
            position: Some(leader_pos),
            velocity: Some(vel),
            ..Default::default()
        },
        Box::new(LevelHoldController::new(1000.0, 100.0)),
        ControllerKind::LevelHold,
    );

    let offset = FormationOffset::default();
    let own_pos = leader_pos + offset.offset_body;
    let own_state = FlightState {
        position: own_pos,
        velocity: vel,
        airspeed: 100.0,
        altitude: own_pos.y,
        ..Default::default()
    };
    let wingman_ctrl = WingmanController::new(leader.id, &leader_state, &own_state, offset);
    let wingman = spawn_plane(
        &mut commands,
        &mut ids,
        &asset_server,
        "planes/generic_jet.plane.ron",
        &SpawnSpec {
            position: Some(own_pos),
            velocity: Some(vel),
            ..Default::default()
        },
        Box::new(wingman_ctrl),
        ControllerKind::Wingman,
    );

    commands.insert_resource(LeaderWingman {
        leader: leader.entity,
        wingman: wingman.entity,
    });
}

/// Removing the followed plane drops the camera back to free-look so it
/// (and the HUD keyed off the followed entity) doesn't freeze on a dead entity.
/// `camera` is gated behind `visual`, so this runs only in visual builds.
#[cfg(feature = "visual")]
#[test]
fn camera_recovers_to_free_look_when_followed_plane_removed() {
    let mut app = App::new();
    app.add_systems(Update, recover_camera_on_target_loss);

    let target = app.world_mut().spawn(FlightState::default()).id();
    app.insert_resource(CameraMode::Follow(target));
    app.update();
    assert!(
        matches!(*app.world().resource::<CameraMode>(), CameraMode::Follow(_)),
        "camera keeps following a live plane"
    );

    app.world_mut().entity_mut(target).despawn();
    app.update();
    assert!(
        matches!(*app.world().resource::<CameraMode>(), CameraMode::FreeLook),
        "camera falls back to free-look when its target is gone"
    );
}

// ---------------------------------------------------------------------------
// Deferred, asset-driven spawn

/// A spawn request is parked until its `.plane.ron` is available, then the whole
/// physics body is built from the real config in one step.
///
/// The assertion is on the *contract* (no plane without its config; a plane once it
/// has one), not on how many frames the asset takes — this crate does not enable
/// bevy's `multi_threaded` feature, so IO-pool timing is not something to pin.
#[test]
fn spawn_defers_until_the_plane_config_asset_is_available() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(LifecyclePlugin);
    });

    app.world_mut().commands().trigger(SpawnPlaneCommand {
        spec: SpawnSpec::default(),
        kind: ControllerKind::LevelHold,
        config_path: "planes/generic_jet.plane.ron".to_string(),
    });

    // Satisfy the asset by hand (no filesystem/async timing) and step once.
    resolve_pending_spawns(&mut app, &generic_jet_config());

    assert_eq!(count_planes(&mut app), 1, "the plane should now exist");
    assert_eq!(
        count_pending(&mut app),
        0,
        "the pending marker is consumed, never left behind"
    );

    // A finalized plane is whole: everything a plane-facing system queries on arrives
    // in the same insert, so nothing can observe a half-built plane.
    let world = app.world_mut();
    let mut q = world.query::<(&PlaneId, &FlightState, &ActiveController, &ControllerKind)>();
    assert_eq!(q.iter(world).count(), 1);
}

/// The finalized body's mass and inertia come from the loaded config.
#[test]
fn finalized_plane_takes_mass_from_the_loaded_config() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(LifecyclePlugin);
    });

    app.world_mut().commands().trigger(SpawnPlaneCommand {
        spec: SpawnSpec::default(),
        kind: ControllerKind::LevelHold,
        config_path: "planes/cargo_jet.plane.ron".to_string(),
    });

    // The real cargo jet, so the assertion holds whether the hand-inserted asset or
    // the genuine disk load gets there first.
    let cargo = ml_planes::training::load_plane_config("assets/planes/cargo_jet.plane.ron")
        .expect("cargo jet asset loads");
    resolve_pending_spawns(&mut app, &cargo);

    let world = app.world_mut();
    let mut q = world.query::<(&PlaneId, &AdditionalMassProperties)>();
    let (_, mass_props) = q.iter(world).next().expect("plane should be spawned");
    let AdditionalMassProperties::MassProperties(props) = mass_props else {
        panic!("spawn should seed explicit MassProperties");
    };
    // Dry 128000 kg + a full 90000 kg tank — not the 5000 kg generic-jet default.
    assert_eq!(props.mass, 218000.0);
    assert!((props.principal_inertia.y - 2.46e7).abs() < 1.0);
}
