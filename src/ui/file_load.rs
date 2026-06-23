use bevy::prelude::*;
use bevy::tasks::{futures_lite::future, AsyncComputeTaskPool, Task};

use std::path::Path;

use crate::controllers::{PlaneTuning, SelectedTuningProfile};
use crate::plane::PlaneTuningHandle;
use crate::ui::lifecycle_panel::PlanePanelState;

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

pub struct PendingConfigLoad {
    /// Resolves to the picked `.plane.ron` path, made relative to `assets/`.
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
    pub config: Vec<PendingConfigLoad>,
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

pub fn spawn_plane_config_load(pending: &mut PendingLoads) {
    let task = AsyncComputeTaskPool::get().spawn(async {
        let mut dialog = rfd::AsyncFileDialog::new().add_filter("Plane config", &["ron"]);
        // Start in the asset root so the common pick is a clean prefix strip.
        if let Ok(cwd) = std::env::current_dir() {
            dialog = dialog.set_directory(cwd.join("assets"));
        }
        let handle = dialog.pick_file().await?;
        Some(to_asset_relative(handle.path()))
    });
    pending.config.push(PendingConfigLoad { task });
}

/// Convert an absolute picked path into a Bevy asset path (relative to the
/// `assets/` dir): everything after the **last** `assets` path component. Falls
/// back to the file name when no `assets` component is present, so the user can
/// still hand-edit the result.
fn to_asset_relative(path: &Path) -> String {
    use std::path::Component;

    let parts: Vec<&std::ffi::OsStr> = path
        .components()
        .filter_map(|c| match c {
            Component::Normal(s) => Some(s),
            _ => None,
        })
        .collect();

    if let Some(idx) = parts.iter().rposition(|s| *s == "assets") {
        let rel: Vec<String> = parts[idx + 1..]
            .iter()
            .map(|s| s.to_string_lossy().into_owned())
            .collect();
        if !rel.is_empty() {
            return rel.join("/");
        }
    }

    path.file_name()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.to_string_lossy().into_owned())
}

// ---------------------------------------------------------------------------
// Polling system (PostUpdate)
// ---------------------------------------------------------------------------

pub fn poll_pending_loads(
    mut pending: ResMut<PendingLoads>,
    mut tuning_assets: ResMut<Assets<PlaneTuning>>,
    mut profiles: Query<(&PlaneTuningHandle, &mut SelectedTuningProfile)>,
    mut panel: ResMut<PlanePanelState>,
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

    pending.config.retain_mut(
        |load| match future::block_on(future::poll_once(&mut load.task)) {
            None => true,
            Some(None) => false,
            Some(Some(path)) => {
                panel.config_path = path;
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

#[cfg(test)]
mod tests {
    use super::to_asset_relative;
    use std::path::Path;

    #[test]
    fn to_asset_relative_strips_assets_prefix() {
        let p = Path::new("/home/jeanpierre/ml_planes/assets/planes/generic_jet.plane.ron");
        assert_eq!(to_asset_relative(p), "planes/generic_jet.plane.ron");
    }

    #[test]
    fn to_asset_relative_uses_last_assets_component() {
        // A duplicated `assets` segment must strip at the deepest one.
        let p = Path::new("/srv/assets/project/assets/planes/electric_trainer.plane.ron");
        assert_eq!(to_asset_relative(p), "planes/electric_trainer.plane.ron");
    }

    #[test]
    fn to_asset_relative_falls_back_without_assets_dir() {
        let p = Path::new("/tmp/somewhere/custom.plane.ron");
        assert_eq!(to_asset_relative(p), "custom.plane.ron");
    }
}
