use bevy::asset::io::Reader;
use bevy::asset::{AssetApp, AssetLoader, Assets, LoadContext};
use bevy::prelude::*;
use bevy_rapier3d::prelude::PhysicsSet;

use crate::plane::config::PlaneConfig;
use super::systems::{apply_aerodynamic_forces, run_flight_controllers, sync_flight_state};

// --- PlaneConfigHandle component ---

/// Component that stores a handle to a plane's [`PlaneConfig`] asset.
/// In Bevy 0.18 `Handle<T>` is not itself a `Component`; this newtype bridges that.
#[derive(Component, Clone, Debug, Reflect)]
#[reflect(Component)]
pub struct PlaneConfigHandle(pub Handle<PlaneConfig>);

// --- Error type for the RON loader ---

#[derive(Debug)]
pub enum PlaneConfigLoaderError {
    Io(std::io::Error),
    Ron(ron::error::SpannedError),
}

impl std::fmt::Display for PlaneConfigLoaderError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e)  => write!(f, "PlaneConfig I/O error: {e}"),
            Self::Ron(e) => write!(f, "PlaneConfig RON parse error: {e}"),
        }
    }
}
impl std::error::Error for PlaneConfigLoaderError {}

impl From<std::io::Error>           for PlaneConfigLoaderError { fn from(e: std::io::Error)           -> Self { Self::Io(e) } }
impl From<ron::error::SpannedError> for PlaneConfigLoaderError { fn from(e: ron::error::SpannedError) -> Self { Self::Ron(e) } }

// --- Asset loader ---

/// Loads `.plane.ron` files as [`PlaneConfig`] assets.
#[derive(Default, bevy::reflect::TypePath)]
pub struct PlaneConfigLoader;

impl AssetLoader for PlaneConfigLoader {
    type Asset    = PlaneConfig;
    type Settings = ();
    type Error    = PlaneConfigLoaderError;

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

    fn extensions(&self) -> &[&str] { &["plane.ron"] }
}

// --- Plugin ---

pub struct PlanePlugin;

impl Plugin for PlanePlugin {
    fn build(&self, app: &mut App) {
        app.init_asset::<PlaneConfig>();
        app.register_type::<PlaneConfig>();
        app.register_type::<PlaneConfigHandle>();
        app.init_asset_loader::<PlaneConfigLoader>();

        app.add_systems(
            FixedUpdate,
            (sync_flight_state, run_flight_controllers, apply_aerodynamic_forces)
                .into_configs()
                .chain()
                .before(PhysicsSet::StepSimulation),
        );
    }
}
