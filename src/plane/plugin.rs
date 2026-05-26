use bevy::asset::io::Reader;
use bevy::asset::{AssetApp, AssetLoader, LoadContext};
use bevy::prelude::*;
use bevy_rapier3d::prelude::PhysicsSet;

use super::systems::{apply_aerodynamic_forces, run_flight_controllers, sync_flight_state};
use crate::controllers::tuning::{LevelHoldTuning, OrbitTuning, PlaneTuning};
use crate::plane::config::PlaneConfig;
use crate::plane::context::{NextPlaneId, PlaneId};

// --- PlaneConfigHandle component ---

/// Component that stores a handle to a plane's [`PlaneConfig`] asset.
/// In Bevy 0.18 `Handle<T>` is not itself a `Component`; this newtype bridges that.
#[derive(Component, Clone, Debug, Reflect)]
#[reflect(Component)]
pub struct PlaneConfigHandle(pub Handle<PlaneConfig>);

// --- PlaneTuningHandle component ---

/// Component that stores a handle to a plane's [`PlaneTuning`] asset.
/// Optional — planes without a `.tuning.ron` file omit this component and
/// controllers fall back to their built-in default gains.
#[derive(Component, Clone, Debug, Reflect)]
#[reflect(Component)]
pub struct PlaneTuningHandle(pub Handle<PlaneTuning>);

// --- Error type for the RON loader ---

#[derive(Debug)]
pub enum PlaneConfigLoaderError {
    Io(std::io::Error),
    Ron(ron::error::SpannedError),
}

impl std::fmt::Display for PlaneConfigLoaderError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "PlaneConfig I/O error: {e}"),
            Self::Ron(e) => write!(f, "PlaneConfig RON parse error: {e}"),
        }
    }
}
impl std::error::Error for PlaneConfigLoaderError {}

impl From<std::io::Error> for PlaneConfigLoaderError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}
impl From<ron::error::SpannedError> for PlaneConfigLoaderError {
    fn from(e: ron::error::SpannedError) -> Self {
        Self::Ron(e)
    }
}

// --- PlaneTuning loader ---

#[derive(Debug)]
pub enum PlaneTuningLoaderError {
    Io(std::io::Error),
    Ron(ron::error::SpannedError),
}

impl std::fmt::Display for PlaneTuningLoaderError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "PlaneTuning I/O error: {e}"),
            Self::Ron(e) => write!(f, "PlaneTuning RON parse error: {e}"),
        }
    }
}
impl std::error::Error for PlaneTuningLoaderError {}

impl From<std::io::Error> for PlaneTuningLoaderError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}
impl From<ron::error::SpannedError> for PlaneTuningLoaderError {
    fn from(e: ron::error::SpannedError) -> Self {
        Self::Ron(e)
    }
}

/// Loads `.tuning.ron` files as [`PlaneTuning`] assets.
#[derive(Default, bevy::reflect::TypePath)]
pub struct PlaneTuningLoader;

impl AssetLoader for PlaneTuningLoader {
    type Asset = PlaneTuning;
    type Settings = ();
    type Error = PlaneTuningLoaderError;

    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &(),
        _ctx: &mut LoadContext<'_>,
    ) -> Result<PlaneTuning, PlaneTuningLoaderError> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        Ok(ron::de::from_bytes(&bytes)?)
    }

    fn extensions(&self) -> &[&str] {
        &["tuning.ron"]
    }
}

// --- PlaneConfig loader ---

/// Loads `.plane.ron` files as [`PlaneConfig`] assets.
#[derive(Default, bevy::reflect::TypePath)]
pub struct PlaneConfigLoader;

impl AssetLoader for PlaneConfigLoader {
    type Asset = PlaneConfig;
    type Settings = ();
    type Error = PlaneConfigLoaderError;

    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &(),
        _ctx: &mut LoadContext<'_>,
    ) -> Result<PlaneConfig, PlaneConfigLoaderError> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        Ok(ron::de::from_bytes(&bytes)?)
    }

    fn extensions(&self) -> &[&str] {
        &["plane.ron"]
    }
}

// --- Plugin ---

pub struct PlanePlugin;

impl Plugin for PlanePlugin {
    fn build(&self, app: &mut App) {
        app.init_asset::<PlaneConfig>();
        app.register_type::<PlaneConfig>();
        app.register_type::<PlaneConfigHandle>();
        app.init_asset_loader::<PlaneConfigLoader>();

        app.init_asset::<PlaneTuning>();
        app.register_type::<PlaneTuning>();
        app.register_type::<LevelHoldTuning>();
        app.register_type::<OrbitTuning>();
        app.register_type::<PlaneTuningHandle>();
        app.init_asset_loader::<PlaneTuningLoader>();

        app.init_resource::<NextPlaneId>();
        app.register_type::<PlaneId>();

        app.add_systems(
            FixedUpdate,
            (
                sync_flight_state,
                run_flight_controllers,
                apply_aerodynamic_forces,
            )
                .into_configs()
                .chain()
                .before(PhysicsSet::StepSimulation),
        );
    }
}
