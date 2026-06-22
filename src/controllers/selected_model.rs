use bevy::prelude::{Component, Resource};
use std::collections::HashMap;

/// Path stem of the active RL model (without `.mpk` extension).
/// Example: `"models/level_hold/ppo_level_hold"`
#[derive(Component, Clone)]
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
