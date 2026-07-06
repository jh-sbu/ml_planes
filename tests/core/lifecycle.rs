//! Runtime plane lifecycle: per-plane config path, spawn/remove commands,
//! automatic indexing, and removal cleanup.

use crate::common::{build_headless_app, build_headless_app_with, generic_jet_config};
use bevy::prelude::*;
#[cfg(feature = "visual")]
use ml_planes::camera::{systems::recover_camera_on_target_loss, CameraMode};
use ml_planes::controllers::{
    ControllerKind, FormationOffset, LevelHoldController, PlaneTuning, SelectedTuningProfile,
    WingmanController,
};
use ml_planes::environment::{spawn_plane, LifecyclePlugin, RemovePlaneCommand, SpawnPlaneCommand};
use ml_planes::plane::{
    FlightState, NextPlaneId, PlaneConfig, PlaneConfigHandle, PlaneIndex, PlaneTuningHandle,
};
use ml_planes::training::SpawnSpec;

/// Collect the `PlaneIndex` ordinal of every live plane.
fn plane_indices(app: &mut App) -> Vec<u32> {
    let world = app.world_mut();
    let mut q = world.query::<&PlaneIndex>();
    let mut v: Vec<u32> = q.iter(world).map(|p| p.0).collect();
    v.sort();
    v
}

/// Phase 2: a `SpawnPlaneCommand` adds a fully-indexed plane; two commands yield
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
    app.update();

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

/// Phase 1: `spawn_plane` must load the aero config from the supplied path,
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
    let expected: Handle<PlaneConfig> =
        world.resource::<AssetServer>().load("planes/f16.plane.ron");
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
    let cfg = generic_jet_config();
    let spawned = spawn_plane(
        &mut commands,
        &mut ids,
        &asset_server,
        "planes/f16.plane.ron",
        &SpawnSpec::default(),
        Box::new(LevelHoldController::new(1000.0, 100.0)),
        ControllerKind::LevelHold,
        &cfg,
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
    app.update();

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

/// A config with no `.tuning.ron` sibling must NOT attach tuning components, so
/// the plane keeps its `build()`-default controller without a dangling handle.
#[test]
fn spawn_command_skips_tuning_without_sibling() {
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
        config_path: "planes/no_such_airframe.plane.ron".to_string(),
    });
    app.update();

    let world = app.world_mut();
    let entity = {
        let mut q = world.query_filtered::<Entity, With<PlaneIndex>>();
        q.iter(world).next().expect("a plane was spawned")
    };
    assert!(
        world.entity(entity).get::<PlaneTuningHandle>().is_none(),
        "no tuning sibling → no tuning handle"
    );
    assert!(
        world
            .entity(entity)
            .get::<SelectedTuningProfile>()
            .is_none(),
        "no tuning sibling → no selected profile"
    );
}

#[derive(Resource)]
struct LeaderWingman {
    leader: Entity,
    wingman: Entity,
}

/// Phase 3: removing a wingman's leader drops the wingman to `LevelHold` instead
/// of leaving it silently masquerading as formation flight against a dead leader.
#[test]
fn removing_leader_drops_wingman_to_level_hold() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(LifecyclePlugin);
    });
    app.add_systems(Startup, spawn_leader_and_wingman);
    app.update();

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

fn spawn_leader_and_wingman(
    mut commands: Commands,
    mut ids: ResMut<NextPlaneId>,
    asset_server: Res<AssetServer>,
) {
    let cfg = generic_jet_config();
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
        &cfg,
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
        &cfg,
    );

    commands.insert_resource(LeaderWingman {
        leader: leader.entity,
        wingman: wingman.entity,
    });
}

/// Phase 3: removing the followed plane drops the camera back to free-look so it
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
