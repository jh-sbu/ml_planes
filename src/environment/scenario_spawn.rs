//! Spawn a resolved `.scenario.ron` into a live Bevy world.
//!
//! `examples/observe_state.rs` spawns scenario planes by hand for headless CSV;
//! this module is the live-app counterpart used by the visual menu's
//! "Start Scenario" flow. It reuses [`ResolvedScenario::build_controller`] (peer
//! references, RL load, synchronous L1 flight-plan load) and [`spawn_plane_with_id`]
//! (the same deferred, asset-driven spawn path as the runtime `SpawnPlaneCommand`), so
//! a scenario drives both the headless and live paths identically.
//!
//! A plane whose controller fails to build (e.g. a missing RL `.mpk`) is skipped
//! with a recorded warning rather than aborting the whole scenario. A wingman whose leader
//! was skipped is transitively skipped too (checked to a fixed point, since a
//! wingman may itself lead another wingman).
//!
//! [`ResolvedScenario::resolve`] assigns **scenario-local** `PlaneId`s (`idx + 1`),
//! baked into a wingman's `leader_id` by `build_controller`. Those ids are *not*
//! the runtime ids planes actually spawn with — `NextPlaneId` is a persistent,
//! never-reset allocator, and a skipped plane shifts every later index. This
//! module reserves a contiguous block of runtime ids for the surviving planes and
//! remaps each surviving wingman's `leader_id` from resolved to runtime before
//! spawning.

use std::collections::HashMap;

use bevy::prelude::*;

use crate::controllers::{FlightController, FlightPlan, FormationOffset, WingmanController};
use crate::plane::{FlightPlanHandle, NextPlaneId, PlaneId};
use crate::scenario::{ControllerSpec, ResolvedScenario};
use crate::training::SpawnSpec;

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
use crate::controllers::SelectedModel;

use super::spawner::{sanitize_asset_path, spawn_plane_with_id};

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
/// Bevy asset-relative path `spawn_plane` expects (relative to `assets/`, e.g.
/// `planes/cargo_jet.plane.ron`).
///
/// The `assets/` strip is cosmetic, not validating — without the
/// `sanitize_asset_path` check a scenario's `assets/../../etc/x` would become
/// `../../etc/x` and reach `AssetServer::load`, whose bare `root_path.join` walks
/// straight out of the asset root. Scenario files are local, so this is a config
/// footgun rather than a remote hole, but it reaches the same sink; a rejected path
/// falls back to the generic jet.
fn asset_relative_config(config: &Option<String>) -> String {
    let stripped = match config {
        Some(p) => p.strip_prefix("assets/").unwrap_or(p),
        None => return DEFAULT_CONFIG.to_string(),
    };
    sanitize_asset_path(stripped).unwrap_or_else(|| DEFAULT_CONFIG.to_string())
}

/// Spawn every plane in `scenario` into the live world. Returns the spawned
/// entities and any skipped-plane warnings (see [`ScenarioSpawnResult`]).
///
/// Four passes, because a wingman's controller bakes in its leader's *runtime*
/// `PlaneId` — which isn't known until after skips are resolved and the id
/// block is reserved:
/// 1. build every plane's controller, recording failures;
/// 2. transitively skip any wingman whose leader failed (fixed point);
/// 3. reserve a contiguous runtime-id block for the survivors and remap each
///    surviving wingman's `leader_id` from resolved to runtime;
/// 4. spawn the survivors under their reserved ids.
pub fn spawn_resolved_scenario(
    commands: &mut Commands,
    ids: &mut NextPlaneId,
    asset_server: &AssetServer,
    scenario: &ResolvedScenario,
) -> ScenarioSpawnResult {
    let mut result = ScenarioSpawnResult::default();
    let n = scenario.planes.len();

    // Pass 1: build every controller.
    let mut controllers: Vec<Option<Box<dyn FlightController>>> = Vec::with_capacity(n);
    let mut skip_reason: Vec<Option<String>> = vec![None; n];
    for idx in 0..n {
        match scenario.build_controller(idx) {
            Ok(c) => controllers.push(Some(c)),
            Err(e) => {
                skip_reason[idx] = Some(format!("'{}' skipped: {e}", scenario.planes[idx].name));
                controllers.push(None);
            }
        }
    }

    // Pass 2: a wingman whose leader was skipped has nothing to follow — skip
    // it too. Looped to a fixed point since a wingman may itself lead another
    // wingman.
    loop {
        let mut changed = false;
        for idx in 0..n {
            if skip_reason[idx].is_some() {
                continue;
            }
            let ControllerSpec::Wingman { leader, .. } = &scenario.planes[idx].spec else {
                continue;
            };
            // Resolve by name, not by resolved PlaneId — the runtime remap
            // in pass 3 hasn't happened yet, and names are what the scenario
            // format itself uses to reference peers.
            let leader_skipped = scenario
                .planes
                .iter()
                .position(|p| &p.name == leader)
                .is_some_and(|li| skip_reason[li].is_some());
            if leader_skipped {
                skip_reason[idx] = Some(format!(
                    "'{}' skipped: leader '{leader}' was not spawned",
                    scenario.planes[idx].name
                ));
                controllers[idx] = None;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    result.skipped.extend(skip_reason.iter().flatten().cloned());

    // Pass 3: reserve a contiguous runtime-id block for the survivors (in
    // scenario order) and remap each surviving wingman's `leader_id` from the
    // scenario-local resolved id to the runtime id its leader actually got.
    let mut runtime_id: Vec<Option<PlaneId>> = vec![None; n];
    for idx in 0..n {
        if skip_reason[idx].is_none() {
            runtime_id[idx] = Some(PlaneId(ids.0));
            ids.0 += 1;
        }
    }
    let resolved_to_runtime: HashMap<PlaneId, PlaneId> = (0..n)
        .filter_map(|idx| runtime_id[idx].map(|rid| (scenario.planes[idx].id, rid)))
        .collect();
    for controller in controllers.iter_mut().flatten() {
        if let Some(wc) = controller.as_any_mut().downcast_mut::<WingmanController>() {
            if let Some(&mapped) = resolved_to_runtime.get(&wc.leader_id) {
                wc.leader_id = mapped;
            }
        }
    }

    // Pass 4: spawn the survivors under their reserved ids.
    for idx in 0..n {
        let Some(controller) = controllers[idx].take() else {
            continue;
        };
        let plane_id = runtime_id[idx].expect("a surviving plane must have a reserved runtime id");
        let plane = &scenario.planes[idx];

        // Mass, inertia, fuel, and aero all come from this one `.plane.ron` once it
        // loads (see `finalize_pending_spawns`), so they cannot disagree.
        let config_path = asset_relative_config(&plane.config);

        let spec = SpawnSpec {
            position: Some(plane.position),
            velocity: Some(plane.velocity),
            attitude: Some(plane.attitude),
            angular_velocity: Some(plane.angular_velocity),
            fuel_fraction: plane.fuel_fraction,
        };

        let spawned = spawn_plane_with_id(
            commands,
            plane_id,
            asset_server,
            &config_path,
            &spec,
            controller,
            plane.spec.kind(),
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
        // Validated for the same reason as the config path above: this reaches
        // `AssetServer::load`, whose bare `root_path.join` lets an absolute path
        // leave the asset root. A rejected plan simply attaches no handle, so
        // the plane keeps its PID-orbit fallback rather than flying a plan the
        // scenario could not name safely.
        if let ControllerSpec::FlightPlan { plan } = &plane.spec {
            let stripped = plan.strip_prefix("assets/").unwrap_or(plan);
            match sanitize_asset_path(stripped) {
                Some(asset_path) => {
                    let handle: Handle<FlightPlan> = asset_server.load(asset_path);
                    commands
                        .entity(spawned.entity)
                        .insert(FlightPlanHandle(handle));
                }
                None => {
                    result.skipped.push(format!(
                        "'{}' flight plan rejected: not an assets-relative path",
                        plane.name
                    ));
                }
            }
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

    #[test]
    fn asset_relative_config_rejects_traversal() {
        // The `assets/` strip is cosmetic: without validation this yields
        // `../../etc/x`, which `AssetServer::load`'s bare `root_path.join` would
        // happily walk out of the asset root to reach.
        for path in [
            "assets/../../etc/x.plane.ron",
            "../../etc/x.plane.ron",
            "/etc/passwd",
        ] {
            assert_eq!(
                asset_relative_config(&Some(path.to_string())),
                DEFAULT_CONFIG,
                "a scenario must not point outside assets/: {path}"
            );
        }
    }
}
