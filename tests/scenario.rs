//! Integration tests for the multi-plane scenario format.

mod common;

use bevy::prelude::*;
use common::build_headless_app;
use ml_planes::controllers::{ControllerKind, FormationOffset, OrbitDirection, WingmanController};
use ml_planes::environment::spawn_resolved_scenario;
use ml_planes::plane::{NextPlaneId, PlaneId};
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

#[test]
fn shipped_scenarios_parse_and_resolve() {
    for name in [
        "level_hold",
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
