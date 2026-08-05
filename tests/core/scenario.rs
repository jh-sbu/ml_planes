//! Integration tests for the multi-plane scenario format.

use crate::common::{build_headless_app, build_headless_app_with};
use bevy::prelude::*;
use ml_planes::controllers::{
    ActiveController, ControllerKind, FormationOffset, LevelHoldTuning, OrbitDirection,
    PlaneTuning, SimControlPlugin, WingmanController,
};
use ml_planes::environment::spawn_resolved_scenario;
use ml_planes::plane::{NextPlaneId, PlaneId, PlaneTuningHandle};
use ml_planes::scenario::{ControllerSpec, ResolvedScenario, Scenario, CSV_HEADER};

use std::path::Path;

/// Each `ControllerSpec` variant maps to the matching `ControllerKind` so a
/// scenario-spawned plane displays/cycles correctly in the HUD.
#[test]
fn controller_spec_kind_maps_each_variant() {
    assert_eq!(
        ControllerSpec::LevelHold {
            altitude: 500.0,
            airspeed: 100.0,
            tuning: None,
        }
        .kind(),
        ControllerKind::LevelHold
    );
    assert_eq!(
        ControllerSpec::Orbit {
            center_x: None,
            center_z: None,
            radius: 1000.0,
            direction: OrbitDirection::CounterClockwise,
            altitude: 500.0,
            airspeed: 100.0,
            tuning: None,
        }
        .kind(),
        ControllerKind::Orbit
    );
    assert_eq!(
        ControllerSpec::HeadingHold {
            heading_deg: 90.0,
            altitude: 500.0,
            airspeed: 100.0,
            tuning: None,
        }
        .kind(),
        ControllerKind::HeadingHold
    );
    assert_eq!(
        ControllerSpec::Ascent {
            target_altitude: 2000.0,
        }
        .kind(),
        ControllerKind::Ascent
    );
    assert_eq!(
        ControllerSpec::Wingman {
            leader: "leader".into(),
            offset: None,
        }
        .kind(),
        ControllerKind::Wingman
    );
    assert_eq!(
        ControllerSpec::FlightPlan { plan: "p".into() }.kind(),
        ControllerKind::FlightPlan
    );
    assert_eq!(ControllerSpec::Manual.kind(), ControllerKind::Manual);
    assert_eq!(
        ControllerSpec::RlHeadingHold {
            model: "m".into(),
            heading_deg: 90.0,
            altitude: 500.0,
            airspeed: 100.0,
        }
        .kind(),
        ControllerKind::RlHeadingHold
    );
}

const LEADER_WINGMAN: &str = r#"(
    steps: 10,
    interval: 10,
    planes: [
        (
            name: "leader",
            position: (0.0, 1000.0, 0.0),
            velocity: (100.0, 0.0, 0.0),
            controller: LevelHold(altitude: 1000.0, airspeed: 100.0),
        ),
        (
            name: "wingman",
            position: (-20.0, 1000.0, -15.0),
            velocity: (100.0, 0.0, 0.0),
            controller: Wingman(leader: "leader", offset: (-20.0, 15.0, 0.0)),
        ),
    ],
)"#;

#[derive(Resource)]
struct ScenarioRes(ResolvedScenario);

fn spawn_scenario_system(
    mut commands: Commands,
    mut ids: ResMut<NextPlaneId>,
    asset_server: Res<AssetServer>,
    scenario: Res<ScenarioRes>,
) {
    spawn_resolved_scenario(&mut commands, &mut ids, &asset_server, &scenario.0);
}

/// `spawn_resolved_scenario` turns a resolved scenario into live plane entities:
/// each plane gets its `PlaneId` + the `ControllerKind` matching its spec, and a
/// wingman carries the `FormationOffset` component.
#[test]
fn spawn_resolved_scenario_spawns_all_planes() {
    let resolved = Scenario::from_ron_str(LEADER_WINGMAN)
        .expect("parse fixture")
        .resolve()
        .expect("resolve fixture");

    let mut app = build_headless_app();
    app.insert_resource(ScenarioRes(resolved));
    app.add_systems(Startup, spawn_scenario_system);
    app.update();
    app.update();

    let world = app.world_mut();
    let mut q = world.query::<(&PlaneId, &ControllerKind)>();
    let mut kinds: Vec<(u32, ControllerKind)> = q.iter(world).map(|(id, k)| (id.0, *k)).collect();
    kinds.sort_by_key(|(id, _)| *id);
    assert_eq!(kinds.len(), 2, "both planes spawned");
    assert_eq!(kinds[0].1, ControllerKind::LevelHold);
    assert_eq!(kinds[1].1, ControllerKind::Wingman);

    let mut offsets = world.query::<&FormationOffset>();
    assert_eq!(
        offsets.iter(world).count(),
        1,
        "the wingman carries a FormationOffset"
    );

    // Anchor: the wingman's controller must be the real WingmanController at
    // spawn time (pre-tuning) — guards the spawn path itself, independent of
    // the tuning-rebuild fix pinned by `scenario_wingman_survives_tuning_rebuild`.
    let mut wingmen = world.query::<(&ControllerKind, &mut ActiveController)>();
    let mut found = false;
    for (kind, mut ctrl) in wingmen.iter_mut(world) {
        if *kind == ControllerKind::Wingman {
            found = true;
            assert!(
                ctrl.0
                    .as_any_mut()
                    .downcast_mut::<WingmanController>()
                    .is_some(),
                "wingman must spawn with a real WingmanController"
            );
        }
    }
    assert!(found, "expected a Wingman-kind plane in the fixture");
}

/// `apply_initial_tuning` used to rebuild every controller unconditionally once
/// the `.tuning.ron` asset loaded, replacing a scenario-spawned wingman's real
/// `WingmanController` with the `LevelHold` fallback `ControllerKind::Wingman
/// .build()` produces. This pins the fix end-to-end through the live scenario
/// spawn path (`spawn_resolved_scenario` + `SimControlPlugin`), and — since it
/// reads the leader's *runtime* `PlaneId` rather than assuming `PlaneId(1)` —
/// doubles as a regression guard for the resolved-vs-runtime id remap (Bug B).
#[test]
fn scenario_wingman_survives_tuning_rebuild() {
    let resolved = Scenario::from_ron_str(LEADER_WINGMAN)
        .expect("parse fixture")
        .resolve()
        .expect("resolve fixture");

    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });
    app.insert_resource(ScenarioRes(resolved));
    app.add_systems(Startup, spawn_scenario_system);
    app.update();

    // Locate the spawned wingman and leader.
    let (wingman_entity, tuning_handle, leader_runtime_id) = {
        let world = app.world_mut();
        let mut leader_id = None;
        let mut q = world.query::<(&PlaneId, &ControllerKind)>();
        for (id, kind) in q.iter(world) {
            if *kind == ControllerKind::LevelHold {
                leader_id = Some(*id);
            }
        }
        let leader_id = leader_id.expect("leader plane must be spawned");

        let mut wq = world.query::<(Entity, &ControllerKind, &PlaneTuningHandle)>();
        let wingman = wq
            .iter(world)
            .find(|(_, kind, _)| **kind == ControllerKind::Wingman)
            .map(|(e, _, h)| (e, h.0.clone()));
        let (entity, handle) = wingman
            .expect("wingman must be spawned with a PlaneTuningHandle (generic_jet.tuning.ron)");
        (entity, handle, leader_id)
    };

    // Force the rebuild frame deterministically: insert a tuning profile
    // synchronously (no filesystem/async timing) so `apply_initial_tuning`
    // sees the asset as loaded on the very next update.
    let mut tuning = PlaneTuning::default();
    tuning.level_hold.insert(
        "normal".to_string(),
        LevelHoldTuning {
            alt_kp: 9.87,
            ..LevelHoldTuning::default()
        },
    );
    app.world_mut()
        .resource_mut::<Assets<PlaneTuning>>()
        .insert(tuning_handle.id(), tuning)
        .expect("insert tuning asset synchronously");

    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(wingman_entity).unwrap();
    let wc = ctrl
        .0
        .as_any_mut()
        .downcast_mut::<WingmanController>()
        .expect("wingman must still be a WingmanController after the tuning rebuild");
    assert_eq!(
        wc.leader_id, leader_runtime_id,
        "leader_id must reference the leader's actual runtime PlaneId"
    );
}

#[test]
fn wingman_formation_asset_resolves_to_two_planes() {
    let path = Path::new("assets/scenarios/wingman_formation.scenario.ron");
    let scenario = Scenario::from_path(path).expect("load wingman scenario");
    let resolved = scenario.resolve().expect("resolve");

    assert_eq!(resolved.planes.len(), 2);
    assert_eq!(resolved.planes[0].name, "leader");
    assert_eq!(resolved.planes[1].name, "wingman");
    assert_eq!(resolved.planes[0].id, PlaneId(1));
    assert_eq!(resolved.planes[1].id, PlaneId(2));

    // The wingman's controller must reference the leader's assigned id.
    let mut wingman = resolved.build_controller(1).expect("build wingman");
    let wc = wingman
        .as_any_mut()
        .downcast_mut::<WingmanController>()
        .expect("controller is a WingmanController");
    assert_eq!(wc.leader_id, resolved.planes[0].id);
}

const SELF_LEADING_WINGMAN: &str = r#"(
    steps: 10,
    interval: 10,
    planes: [
        (
            name: "solo",
            position: (0.0, 1000.0, 0.0),
            velocity: (100.0, 0.0, 0.0),
            controller: Wingman(leader: "solo"),
        ),
    ],
)"#;

/// A wingman naming itself as leader passes the "leader name exists" check
/// (it's scanning `planes`, which includes itself) but is nonsensical: it
/// would chase its own formation slot with an uncorrectable error. `resolve()`
/// must reject it explicitly.
#[test]
fn wingman_cannot_reference_itself_as_leader() {
    let scenario = Scenario::from_ron_str(SELF_LEADING_WINGMAN).expect("parse fixture");
    let err = scenario
        .resolve()
        .expect_err("a wingman naming itself as leader must fail to resolve");
    assert!(
        err.contains("solo") && err.contains("itself"),
        "error should name the plane and explain the self-reference: {err}"
    );
}

// ---------------------------------------------------------------------------
// Resolved-vs-runtime PlaneId remap
//
// `resolve()` assigns scenario-local ids (`idx + 1`), and `build_controller`
// bakes a wingman's *resolved* leader id into `WingmanController::leader_id`.
// `spawn_resolved_scenario` allocates *runtime* ids from the (never-reset)
// `NextPlaneId` allocator, which diverges from the resolved numbering whenever
// planes already exist or a plane is skipped. These tests pin the remap that
// keeps `leader_id` pointing at the plane's actual runtime id.

/// When `NextPlaneId` doesn't start at 1 (e.g. other planes already exist),
/// the live ids must still be internally consistent: the wingman's `leader_id`
/// must reference the leader's *actual* runtime id, not the resolved `idx+1`
/// baked in by `resolve()`.
#[test]
fn scenario_wingman_leader_id_uses_runtime_plane_ids() {
    let resolved = Scenario::from_ron_str(LEADER_WINGMAN)
        .expect("parse fixture")
        .resolve()
        .expect("resolve fixture");

    // Deliberately no SimControlPlugin — isolates this from the Bug A fix.
    let mut app = build_headless_app();
    app.insert_resource(NextPlaneId(5));
    app.insert_resource(ScenarioRes(resolved));
    app.add_systems(Startup, spawn_scenario_system);
    app.update();
    app.update();

    let world = app.world_mut();
    let mut q = world.query::<&PlaneId>();
    let mut ids: Vec<u32> = q.iter(world).map(|id| id.0).collect();
    ids.sort();
    assert_eq!(
        ids,
        vec![5, 6],
        "runtime ids must come from NextPlaneId, not the resolved idx+1 numbering"
    );

    let mut wq = world.query::<(&ControllerKind, &mut ActiveController)>();
    let mut leader_id = None;
    for (kind, mut ctrl) in wq.iter_mut(world) {
        if *kind == ControllerKind::Wingman {
            let wc = ctrl
                .0
                .as_any_mut()
                .downcast_mut::<WingmanController>()
                .expect("wingman controller");
            leader_id = Some(wc.leader_id);
        }
    }
    assert_eq!(
        leader_id,
        Some(PlaneId(5)),
        "leader_id must reference the leader's runtime PlaneId, not PlaneId(1)"
    );
}

#[derive(Resource, Default)]
struct CapturedSkips(Vec<String>);

fn spawn_scenario_capture_system(
    mut commands: Commands,
    mut ids: ResMut<NextPlaneId>,
    asset_server: Res<AssetServer>,
    scenario: Res<ScenarioRes>,
    mut out: ResMut<CapturedSkips>,
) {
    let result = spawn_resolved_scenario(&mut commands, &mut ids, &asset_server, &scenario.0);
    out.0 = result.skipped;
}

const BAD_LEADER_WINGMAN: &str = r#"(
    steps: 10,
    interval: 10,
    planes: [
        (
            name: "leader",
            position: (0.0, 1000.0, 0.0),
            velocity: (100.0, 0.0, 0.0),
            controller: FlightPlan(plan: "assets/plans/does_not_exist.plan.ron"),
        ),
        (
            name: "wingman",
            position: (-20.0, 1000.0, -15.0),
            velocity: (100.0, 0.0, 0.0),
            controller: Wingman(leader: "leader"),
        ),
    ],
)"#;

/// A leader whose controller fails to build (missing flight-plan asset — fails
/// in every feature config, unlike an RL model) must transitively skip its
/// wingman too, rather than spawning a wingman with a `leader_id` that
/// references a plane that was never spawned.
#[test]
fn scenario_skips_wingman_whose_leader_failed_to_build() {
    let resolved = Scenario::from_ron_str(BAD_LEADER_WINGMAN)
        .expect("parse fixture")
        .resolve()
        .expect("resolve fixture");

    let mut app = build_headless_app();
    app.insert_resource(ScenarioRes(resolved));
    app.insert_resource(CapturedSkips::default());
    app.add_systems(Startup, spawn_scenario_capture_system);
    app.update();
    app.update();

    let skips = app.world().resource::<CapturedSkips>().0.clone();
    assert_eq!(
        skips.len(),
        2,
        "both the leader and the wingman that depends on it must be skipped: {skips:?}"
    );
    assert!(
        skips.iter().any(|s| s.contains("leader")),
        "one skip message should name the leader: {skips:?}"
    );
    assert!(
        skips.iter().any(|s| s.contains("wingman")),
        "one skip message should name the wingman: {skips:?}"
    );

    let world = app.world_mut();
    let mut q = world.query::<&PlaneId>();
    assert_eq!(
        q.iter(world).count(),
        0,
        "no plane should have spawned — the leader failed and the wingman follows it"
    );
}

const SKIP_THEN_WINGMAN: &str = r#"(
    steps: 10,
    interval: 10,
    planes: [
        (
            name: "bad",
            position: (0.0, 500.0, 0.0),
            velocity: (100.0, 0.0, 0.0),
            controller: FlightPlan(plan: "assets/plans/does_not_exist.plan.ron"),
        ),
        (
            name: "leader",
            position: (0.0, 1000.0, 0.0),
            velocity: (100.0, 0.0, 0.0),
            controller: LevelHold(altitude: 1000.0, airspeed: 100.0),
        ),
        (
            name: "wingman",
            position: (-20.0, 1000.0, -15.0),
            velocity: (100.0, 0.0, 0.0),
            controller: Wingman(leader: "leader"),
        ),
    ],
)"#;

/// A plane skipped early in scenario order must not burn a runtime id: the
/// surviving planes get contiguous ids starting at 1 (or wherever `NextPlaneId`
/// starts), and `NextPlaneId` itself must not gain a gap for the skip.
#[test]
fn scenario_ids_are_contiguous_after_a_skip() {
    let resolved = Scenario::from_ron_str(SKIP_THEN_WINGMAN)
        .expect("parse fixture")
        .resolve()
        .expect("resolve fixture");

    let mut app = build_headless_app();
    app.insert_resource(ScenarioRes(resolved));
    app.add_systems(Startup, spawn_scenario_system);
    app.update();
    app.update();

    let world = app.world_mut();
    let mut q = world.query::<(&PlaneId, &ControllerKind)>();
    let mut kinds: Vec<(u32, ControllerKind)> = q.iter(world).map(|(id, k)| (id.0, *k)).collect();
    kinds.sort_by_key(|(id, _)| *id);
    assert_eq!(
        kinds,
        vec![(1, ControllerKind::LevelHold), (2, ControllerKind::Wingman)],
        "ids must be contiguous starting at 1 — no id burned for the skipped plane"
    );

    let mut wq = world.query::<(&ControllerKind, &mut ActiveController)>();
    for (kind, mut ctrl) in wq.iter_mut(world) {
        if *kind == ControllerKind::Wingman {
            let wc = ctrl
                .0
                .as_any_mut()
                .downcast_mut::<WingmanController>()
                .expect("wingman controller");
            assert_eq!(
                wc.leader_id,
                PlaneId(1),
                "leader_id must reference the leader's runtime id (1), not its resolved id (2)"
            );
        }
    }

    assert_eq!(
        world.resource::<NextPlaneId>().0,
        3,
        "NextPlaneId must not have a gap for the skipped plane"
    );
}

#[test]
fn shipped_scenarios_parse_and_resolve() {
    for name in [
        "level_hold",
        "heading_hold",
        "orbit",
        "wingman_formation",
        "mixed_powerplant",
        "fleet_demo",
    ] {
        let path = format!("assets/scenarios/{name}.scenario.ron");
        let scenario = Scenario::from_path(Path::new(&path)).unwrap_or_else(|e| {
            panic!("load {path}: {e}");
        });
        let resolved = scenario
            .resolve()
            .unwrap_or_else(|e| panic!("resolve {path}: {e}"));
        // Every plane must build a controller without error.
        for idx in 0..resolved.planes.len() {
            resolved
                .build_controller(idx)
                .unwrap_or_else(|e| panic!("build {path} plane {idx}: {e}"));
        }
    }
}

/// The refactored live-app default scene is a valid scenario with the six demo
/// planes in order. Non-RL planes must build a controller; the RL planes resolve
/// to their RL kinds (they only *build* under a native inference build, and are
/// otherwise skipped by the live spawner).
#[test]
fn default_scenario_resolves_to_full_demo() {
    let path = Path::new("assets/scenarios/default.scenario.ron");
    let scenario = Scenario::from_path(path).expect("load default scenario");
    let resolved = scenario.resolve().expect("resolve default scenario");

    let names: Vec<&str> = resolved.planes.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(
        names,
        [
            "leader",
            "wingman",
            "pid_orbit",
            "rl_orbit",
            "rl_level_hold",
            "flight_plan"
        ]
    );

    assert_eq!(resolved.planes[3].spec.kind(), ControllerKind::RlOrbit);
    assert_eq!(resolved.planes[4].spec.kind(), ControllerKind::RlLevelHold);

    // The non-RL planes must build cleanly in every feature config.
    for idx in [0usize, 1, 2, 5] {
        resolved
            .build_controller(idx)
            .unwrap_or_else(|e| panic!("build default plane {idx}: {e}"));
    }
}

/// The 100-plane stress scenario stays well-formed: it is easy to break the
/// count or duplicate a name when hand-editing 100 entries, and nothing else in
/// the suite loads this file. Like the default scenario it mixes RL and non-RL
/// planes, so only the non-RL ones are built here (RL specs resolve everywhere
/// but build only under a native inference build).
#[test]
fn stress_100_scenario_resolves_to_100_planes() {
    let path = Path::new("assets/scenarios/stress_100.scenario.ron");
    let scenario = Scenario::from_path(path).expect("load stress_100 scenario");
    let resolved = scenario.resolve().expect("resolve stress_100 scenario");

    assert_eq!(
        resolved.planes.len(),
        100,
        "stress scenario must be 100 planes"
    );

    // At least 10 planes must fly an RL controller — the point of the scenario.
    let rl_count = resolved
        .planes
        .iter()
        .filter(|p| {
            matches!(
                p.spec.kind(),
                ControllerKind::RlLevelHold
                    | ControllerKind::RlOrbit
                    | ControllerKind::RlOrbitResidual
                    | ControllerKind::RlLstmOrbit
            )
        })
        .count();
    assert!(rl_count >= 10, "expected >=10 RL planes, got {rl_count}");

    // Every non-RL plane must build cleanly in every feature config.
    for (idx, plane) in resolved.planes.iter().enumerate() {
        let is_rl = matches!(
            plane.spec.kind(),
            ControllerKind::RlLevelHold
                | ControllerKind::RlOrbit
                | ControllerKind::RlOrbitResidual
                | ControllerKind::RlLstmOrbit
        );
        if !is_rl {
            resolved
                .build_controller(idx)
                .unwrap_or_else(|e| panic!("build stress_100 plane {idx} ({}): {e}", plane.name));
        }
    }
}

/// `RlHeadingHold` specs always parse (per the `ControllerSpec` enum's doc), regardless
/// of whether `inference` is compiled in — mirrors the other RL variants.
#[test]
fn rl_heading_hold_spec_parses_and_resolves_to_rl_heading_hold_kind() {
    let src = r#"(
        steps: 10,
        interval: 10,
        planes: [
            (
                name: "rl_hh",
                position: (0.0, 1000.0, 0.0),
                velocity: (120.0, 0.0, 0.0),
                controller: RlHeadingHold(
                    model: "models/heading_hold/m",
                    heading_deg: 45.0,
                    altitude: 1000.0,
                    airspeed: 120.0,
                ),
            ),
        ],
    )"#;
    let resolved = Scenario::from_ron_str(src)
        .expect("parse")
        .resolve()
        .expect("resolve");
    assert_eq!(
        resolved.planes[0].spec.kind(),
        ControllerKind::RlHeadingHold
    );
}

#[test]
fn csv_header_is_pinned() {
    // Skills parse this exact column layout; guard against silent drift.
    // `fuel_remaining` is appended last so existing positional column indices
    // stay stable for skills that parse by position.
    assert_eq!(
        CSV_HEADER,
        "step,time_s,plane,pos_x,altitude_m,pos_z,airspeed_ms,alpha_deg,beta_deg,\
         roll_deg,pitch_deg,yaw_deg,pitch_rate,roll_rate,yaw_rate,\
         elevator,throttle,aileron,rudder,radial_error_m,heading_error_rad,bank_ff_rad,\
         fuel_remaining"
    );
}

#[test]
fn fuel_fraction_is_carried_through_resolve() {
    // A per-plane `fuel_fraction` in the scenario RON must survive resolution so
    // observe_state can load the tank to that fraction of capacity. Omitting it
    // resolves to None (observe_state then defaults to a full tank).
    let src = r#"(
        steps: 10,
        interval: 1,
        planes: [
            (
                name: "partial",
                position: (0.0, 500.0, 0.0),
                velocity: (100.0, 0.0, 0.0),
                fuel_fraction: 0.25,
                controller: LevelHold(altitude: 500.0, airspeed: 100.0),
            ),
            (
                name: "default_tank",
                position: (0.0, 500.0, 0.0),
                velocity: (100.0, 0.0, 0.0),
                controller: LevelHold(altitude: 500.0, airspeed: 100.0),
            ),
        ],
    )"#;
    let scenario = Scenario::from_ron_str(src).expect("parse fuel scenario");
    let resolved = scenario.resolve().expect("resolve");
    assert_eq!(resolved.planes[0].fuel_fraction, Some(0.25));
    assert_eq!(resolved.planes[1].fuel_fraction, None);
}
