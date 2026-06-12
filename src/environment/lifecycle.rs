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

use crate::controllers::{ActiveController, ControllerKind, WingmanController};
use crate::plane::{ControlInputs, NextPlaneId, PlaneId};
use crate::training::SpawnSpec;

use super::spawner::{generic_jet_spawn_config, initial_state_from_spec, spawn_plane};

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
        app.add_systems(Update, cleanup_orphaned_wingmen);
    }
}

/// A wingman whose leader has been removed has no formation to hold. Drop it to
/// `LevelHold` so it flies honestly (and the HUD/map reflect reality) instead of
/// silently masquerading as formation flight against a dead `leader_id`. Flipping
/// `ControllerKind` lets the visual `apply_controller_switch` rebuild it into a
/// real `LevelHoldController`; headless drivers see the kind change directly.
fn cleanup_orphaned_wingmen(
    mut planes: Query<(&mut ActiveController, &mut ControllerKind, &PlaneId)>,
) {
    let live: HashSet<PlaneId> = planes.iter().map(|(_, _, id)| *id).collect();
    for (mut ctrl, mut kind, _) in planes.iter_mut() {
        if *kind != ControllerKind::Wingman {
            continue;
        }
        if let Some(wing) = ctrl.0.as_any_mut().downcast_mut::<WingmanController>() {
            if !live.contains(&wing.leader_id) {
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
    let cfg = generic_jet_spawn_config();
    spawn_plane(
        &mut commands,
        &mut ids,
        &asset_server,
        &cmd.config_path,
        &cmd.spec,
        controller,
        cmd.kind,
        &cfg,
    );
}

fn on_remove_plane_command(on: On<RemovePlaneCommand>, mut commands: Commands) {
    // Tolerate a stale entity (already despawned / never existed).
    if let Ok(mut entity) = commands.get_entity(on.event().0) {
        entity.despawn();
    }
}
