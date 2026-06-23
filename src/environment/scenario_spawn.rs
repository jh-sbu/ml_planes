//! Spawn a resolved `.scenario.ron` into a live Bevy world.
//!
//! `examples/observe_state.rs` spawns scenario planes by hand for headless CSV;
//! this module is the live-app counterpart used by the visual menu's
//! "Start Scenario" flow. It reuses [`ResolvedScenario::build_controller`] (peer
//! references, RL load, synchronous L1 flight-plan load) and [`spawn_plane`] (the
//! same sync-mass spawn path as the runtime `SpawnPlaneCommand`), so a scenario
//! drives both the headless and live paths identically.
//!
//! A plane whose controller fails to build (e.g. a missing RL `.mpk`) is skipped
//! with a recorded warning rather than aborting the whole scenario — mirroring the
//! silent fallback the old hardcoded `main.rs::setup` used.

use bevy::prelude::*;

use crate::controllers::{FlightPlan, FormationOffset};
use crate::plane::{FlightPlanHandle, NextPlaneId};
use crate::scenario::{ControllerSpec, ResolvedScenario};
use crate::training::SpawnSpec;

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
use crate::controllers::SelectedModel;

use super::spawner::{load_spawn_config, spawn_plane};

/// Outcome of [`spawn_resolved_scenario`].
#[derive(Debug, Default)]
pub struct ScenarioSpawnResult {
    /// Entities spawned, in scenario order.
    pub spawned: Vec<Entity>,
    /// One human-readable message per plane skipped because its controller
    /// failed to build (e.g. a missing RL model file).
    pub skipped: Vec<String>,
}

/// Default `.plane.ron` (Bevy asset-relative) used when a scenario plane omits
/// `config`. Matches the embedded generic-jet the scenario model assumes.
const DEFAULT_CONFIG: &str = "planes/generic_jet.plane.ron";

/// Convert a scenario `config` path (the observe_state convention, e.g.
/// `assets/planes/cargo_jet.plane.ron`, or `None` → generic jet) into the
/// Bevy asset-relative path `spawn_plane`/`load_spawn_config` expect (relative to
/// `assets/`, e.g. `planes/cargo_jet.plane.ron`).
fn asset_relative_config(config: &Option<String>) -> String {
    match config {
        Some(p) => p.strip_prefix("assets/").unwrap_or(p).to_string(),
        None => DEFAULT_CONFIG.to_string(),
    }
}

/// Spawn every plane in `scenario` into the live world. Returns the spawned
/// entities and any skipped-plane warnings (see [`ScenarioSpawnResult`]).
pub fn spawn_resolved_scenario(
    commands: &mut Commands,
    ids: &mut NextPlaneId,
    asset_server: &AssetServer,
    scenario: &ResolvedScenario,
) -> ScenarioSpawnResult {
    let mut result = ScenarioSpawnResult::default();

    for idx in 0..scenario.planes.len() {
        let plane = &scenario.planes[idx];

        let controller = match scenario.build_controller(idx) {
            Ok(c) => c,
            Err(e) => {
                result
                    .skipped
                    .push(format!("'{}' skipped: {e}", plane.name));
                continue;
            }
        };

        // Seed the Rapier body from the chosen config synchronously (mass /
        // inertia / fuel) — a heavy airframe on generic inertia diverges to a
        // non-finite state. Aero is driven by the async-loaded handle once ready.
        let config_path = asset_relative_config(&plane.config);
        let cfg = load_spawn_config(&config_path);

        let spec = SpawnSpec {
            position: Some(plane.position),
            velocity: Some(plane.velocity),
            attitude: Some(plane.attitude),
            angular_velocity: Some(plane.angular_velocity),
            fuel_fraction: plane.fuel_fraction,
        };

        let spawned = spawn_plane(
            commands,
            ids,
            asset_server,
            &config_path,
            &spec,
            controller,
            plane.spec.kind(),
            &cfg,
        );

        // Per-kind extras the generic spawn path can't infer.
        if let Some(offset_body) = plane.spec.formation_offset() {
            commands
                .entity(spawned.entity)
                .insert(FormationOffset { offset_body });
        }
        // A FlightPlan plane's controller is rebuilt to a PID-orbit fallback by
        // the visual tuning systems (`kind.build()` can't make an L1Controller);
        // the `apply_flight_plan` system re-installs the real L1Controller from
        // this handle once the `.plan.ron` asset loads. The plan path uses the
        // observe_state convention (`assets/...`); make it Bevy asset-relative.
        if let ControllerSpec::FlightPlan { plan } = &plane.spec {
            let asset_path = plan.strip_prefix("assets/").unwrap_or(plan).to_string();
            let handle: Handle<FlightPlan> = asset_server.load(asset_path);
            commands
                .entity(spawned.entity)
                .insert(FlightPlanHandle(handle));
        }
        #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
        if let Some(stem) = plane.spec.rl_model_stem() {
            commands.entity(spawned.entity).insert(SelectedModel(stem));
        }

        result.spawned.push(spawned.entity);
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn asset_relative_config_strips_assets_prefix() {
        assert_eq!(
            asset_relative_config(&Some("assets/planes/cargo_jet.plane.ron".to_string())),
            "planes/cargo_jet.plane.ron"
        );
    }

    #[test]
    fn asset_relative_config_passes_through_relative_path() {
        assert_eq!(
            asset_relative_config(&Some("planes/business_jet.plane.ron".to_string())),
            "planes/business_jet.plane.ron"
        );
    }

    #[test]
    fn asset_relative_config_defaults_to_generic_jet() {
        assert_eq!(asset_relative_config(&None), DEFAULT_CONFIG);
    }
}
