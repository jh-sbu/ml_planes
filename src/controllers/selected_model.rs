use bevy::prelude::{Component, Resource};
use std::collections::HashMap;

/// Path stem of the active RL model (without `.mpk` extension).
/// Example: `"models/level_hold/ppo_level_hold"`
#[derive(Component, Clone, PartialEq)]
#[cfg_attr(feature = "net", derive(serde::Serialize, serde::Deserialize))]
pub struct SelectedModel(pub String);

/// Models discovered at startup, keyed by the `models/` subdirectory name.
///
/// Key = directory name matching `ControllerKind::model_dir()` (e.g. `"level_hold"`).
/// Value = sorted list of path stems (e.g. `["models/level_hold/ppo_level_hold"]`).
///
/// Populated by the `scan_models` startup system (requires `inference` feature).
/// Always inserted as an empty library in non-training builds.
#[derive(Resource, Default)]
pub struct ModelLibrary(pub HashMap<String, Vec<String>>);

/// Where the checkpoints behind `SelectedModel`'s logical `models/<dir>/<name>` ids live.
///
/// The ids themselves stay `models/...` everywhere — they are replicated, validated by
/// `model_path_matches_dir` (which only accepts that prefix, as path-traversal defence
/// for client-supplied ids), and listed in `ModelLibrary`. Only the filesystem lookup
/// goes through this root. It defaults to `models`, i.e. relative to the working
/// directory, which is exactly the old behaviour; tests point it at the frozen
/// `fixtures/models/` so they never depend on what a training run left in `models/`.
#[derive(Resource, Clone, Debug)]
pub struct ModelRoot(pub std::path::PathBuf);

impl Default for ModelRoot {
    fn default() -> Self {
        Self("models".into())
    }
}

impl ModelRoot {
    /// The on-disk stem (no `.mpk`) for a logical id. Ids outside `models/` — e.g. an
    /// explicit path a test put in `ModelLibrary` — are used as given.
    pub fn resolve(&self, id: &str) -> String {
        match id.strip_prefix("models/") {
            Some(rest) => self.0.join(rest).to_string_lossy().into_owned(),
            None => id.to_string(),
        }
    }
}
