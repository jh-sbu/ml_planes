use bevy::prelude::*;
use bevy::tasks::{futures_lite::future, AsyncComputeTaskPool, Task};

use crate::controllers::{PlaneTuning, SelectedTuningProfile};
use crate::plane::PlaneTuningHandle;

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
use crate::controllers::SelectedModel;

// ---------------------------------------------------------------------------
// Pending load descriptors
// ---------------------------------------------------------------------------

pub struct PendingTuningLoad {
    pub entity: Entity,
    /// When true, auto-select from the orbit map; otherwise from level_hold.
    pub pick_orbit: bool,
    pub task: Task<Option<PlaneTuning>>,
}

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
pub struct PendingModelLoad {
    pub entity: Entity,
    pub task: Task<Option<String>>,
}

// ---------------------------------------------------------------------------
// Resource
// ---------------------------------------------------------------------------

#[derive(Resource, Default)]
pub struct PendingLoads {
    pub tuning: Vec<PendingTuningLoad>,
    #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
    pub model: Vec<PendingModelLoad>,
}

// ---------------------------------------------------------------------------
// Spawn helpers (called from the HUD button handlers)
// ---------------------------------------------------------------------------

pub fn spawn_tuning_load(entity: Entity, pick_orbit: bool, pending: &mut PendingLoads) {
    let task = AsyncComputeTaskPool::get().spawn(async {
        let handle = rfd::AsyncFileDialog::new()
            .add_filter("RON tuning", &["ron"])
            .pick_file()
            .await?;
        let bytes = handle.read().await;
        let text = std::str::from_utf8(&bytes).ok()?;
        ron::de::from_str::<PlaneTuning>(text).ok()
    });
    pending.tuning.push(PendingTuningLoad {
        entity,
        pick_orbit,
        task,
    });
}

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
pub fn spawn_model_load(entity: Entity, pending: &mut PendingLoads) {
    let task = AsyncComputeTaskPool::get().spawn(async {
        let handle = rfd::AsyncFileDialog::new()
            .add_filter("Model pack", &["mpk"])
            .pick_file()
            .await?;
        Some(
            handle
                .path()
                .with_extension("")
                .to_string_lossy()
                .into_owned(),
        )
    });
    pending.model.push(PendingModelLoad { entity, task });
}

// ---------------------------------------------------------------------------
// Polling system (PostUpdate)
// ---------------------------------------------------------------------------

pub fn poll_pending_loads(
    mut pending: ResMut<PendingLoads>,
    mut tuning_assets: ResMut<Assets<PlaneTuning>>,
    mut profiles: Query<(&PlaneTuningHandle, &mut SelectedTuningProfile)>,
    #[cfg(all(feature = "inference", not(target_arch = "wasm32")))] mut models: Query<
        &mut SelectedModel,
    >,
) {
    pending.tuning.retain_mut(
        |load| match future::block_on(future::poll_once(&mut load.task)) {
            None => true,
            Some(None) => false,
            Some(Some(new_tuning)) => {
                let first = if load.pick_orbit {
                    let mut keys: Vec<_> = new_tuning.orbit.keys().cloned().collect();
                    keys.sort();
                    keys.into_iter().next()
                } else {
                    let mut keys: Vec<_> = new_tuning.level_hold.keys().cloned().collect();
                    keys.sort();
                    keys.into_iter().next()
                };
                if let Ok((handle, mut profile)) = profiles.get_mut(load.entity) {
                    if let Some(asset) = tuning_assets.get_mut(&handle.0) {
                        asset.merge(new_tuning);
                        if let Some(name) = first {
                            profile.0 = name;
                        }
                    }
                }
                false
            }
        },
    );

    #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
    pending.model.retain_mut(
        |load| match future::block_on(future::poll_once(&mut load.task)) {
            None => true,
            Some(None) => false,
            Some(Some(stem)) => {
                if let Ok(mut sel) = models.get_mut(load.entity) {
                    sel.0 = stem;
                }
                false
            }
        },
    );
}
