//! Runtime plane lifecycle: spawn / remove planes by command.
//!
//! Bevy 0.18 events are observer-based (see CLAUDE.md §5): fire a command with
//! `commands.trigger(SpawnPlaneCommand { .. })` and the registered observer
//! handles it. `spawn_plane` assigns the stable `PlaneId` and the derived
//! `PlaneIndex` automatically, so a commanded plane is immediately visible to
//! camera cycling, the map, and the HUD with no manual indexing.
//!
//! This module is headless-safe — it has no rendering dependencies, so the
//! commands work in tests and (eventually) any non-visual driver.

use bevy::prelude::*;

use std::collections::HashSet;

use crate::controllers::{ActiveController, ControllerKind, RefuelController, WingmanController};
use crate::plane::{ControlInputs, NextPlaneId, PlaneId};
use crate::training::SpawnSpec;

use super::spawner::{initial_state_from_spec, spawn_plane, PendingPlaneSpawn};

/// Spawn a new plane at runtime. Fire with `Commands::trigger`.
#[derive(Event, Debug, Clone)]
pub struct SpawnPlaneCommand {
    pub spec: SpawnSpec,
    pub kind: ControllerKind,
    /// `.plane.ron` asset path driving the new plane's aerodynamics.
    pub config_path: String,
}

impl SpawnPlaneCommand {
    /// Spawn at a position with a velocity, using the default generic-jet config.
    pub fn at(position: Vec3, velocity: Vec3, kind: ControllerKind) -> Self {
        Self {
            spec: SpawnSpec {
                position: Some(position),
                velocity: Some(velocity),
                ..Default::default()
            },
            kind,
            config_path: "planes/generic_jet.plane.ron".to_string(),
        }
    }
}

/// Remove (despawn) a plane entity at runtime. Fire with `Commands::trigger`.
#[derive(Event, Debug, Clone, Copy)]
pub struct RemovePlaneCommand(pub Entity);

/// Registers the spawn/remove observers. Headless-safe.
pub struct LifecyclePlugin;

impl Plugin for LifecyclePlugin {
    fn build(&self, app: &mut App) {
        app.add_observer(on_spawn_plane_command);
        app.add_observer(on_remove_plane_command);
        app.add_systems(Update, cleanup_orphaned_followers);
    }
}

/// The peer `PlaneId` a follower-kind controller depends on, or `None` when the kind
/// claims to be a follower but no follower law is actually installed.
fn follower_peer_id(ctrl: &mut ActiveController, kind: ControllerKind) -> Option<PlaneId> {
    match kind {
        ControllerKind::Wingman => ctrl
            .0
            .as_any_mut()
            .downcast_mut::<WingmanController>()
            .map(|w| w.leader_id),
        ControllerKind::Refueling => ctrl
            .0
            .as_any_mut()
            .downcast_mut::<RefuelController>()
            .map(|r| r.tanker_id),
        _ => None,
    }
}

/// A follower whose peer has been removed — a wingman with no leader, a receiver with no
/// tanker — has nothing to hold station on. Drop it to `LevelHold` so it flies honestly
/// (and the HUD/map reflect reality) instead of silently masquerading against a dead
/// `PlaneId`. Flipping `ControllerKind` lets the visual `apply_controller_switch` rebuild
/// it into a real `LevelHoldController`; headless drivers see the kind change directly.
///
/// Also demotes a follower-kind plane whose active controller isn't actually the matching
/// follower law; the kind must not claim behaviour the plane is not doing.
///
/// A peer still parked on its `.plane.ron` counts as **live**. A [`PendingPlaneSpawn`]
/// carries no `PlaneId`, but its id is already reserved; otherwise a follower whose config
/// loads first would be demoted. The demotion is one-way: it flips `ControllerKind`,
/// `apply_controller_switch` rebuilds a real `LevelHoldController`, and nothing ever
/// promotes back.
///
/// Kept as **one** system over both follower kinds rather than a per-kind sibling: the
/// load-bearing subtlety here is the `live` set including pending spawns, and a second
/// copy is exactly how that gets fixed in one place and silently missed in the other —
/// the `apply_initial_tuning`/`apply_controller_switch` bug class. Adding a third
/// follower kind means one arm in `follower_peer_id`, nothing else.
fn cleanup_orphaned_followers(
    mut planes: Query<(&mut ActiveController, &mut ControllerKind, &PlaneId)>,
    pending: Query<&PendingPlaneSpawn>,
) {
    let live: HashSet<PlaneId> = planes
        .iter()
        .map(|(_, _, id)| *id)
        .chain(pending.iter().map(|p| p.plane_id))
        .collect();
    for (mut ctrl, mut kind, _) in planes.iter_mut() {
        if !matches!(*kind, ControllerKind::Wingman | ControllerKind::Refueling) {
            continue;
        }
        match follower_peer_id(&mut ctrl, *kind) {
            Some(peer) if live.contains(&peer) => {}
            _ => {
                kind.set_if_neq(ControllerKind::LevelHold);
            }
        }
    }
}

fn on_spawn_plane_command(
    on: On<SpawnPlaneCommand>,
    mut commands: Commands,
    mut ids: ResMut<NextPlaneId>,
    asset_server: Res<AssetServer>,
) {
    let cmd = on.event();
    // Build the controller from the exact state the plane will spawn at so the
    // first tick is bumpless. `build()` returns a valid PID fallback for kinds
    // (Wingman/FlightPlan/RL) that need extra context the factory can't provide.
    let state = initial_state_from_spec(&cmd.spec);
    let controller = cmd.kind.build(&state, None, &ControlInputs::default());
    spawn_plane(
        &mut commands,
        &mut ids,
        &asset_server,
        &cmd.config_path,
        &cmd.spec,
        controller,
        cmd.kind,
    );
}

fn on_remove_plane_command(on: On<RemovePlaneCommand>, mut commands: Commands) {
    // Tolerate a stale entity (already despawned / never existed).
    if let Ok(mut entity) = commands.get_entity(on.event().0) {
        entity.despawn();
    }
}
