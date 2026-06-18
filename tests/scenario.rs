//! Integration tests for the multi-plane scenario format.

use ml_planes::controllers::WingmanController;
use ml_planes::plane::PlaneId;
use ml_planes::scenario::{Scenario, CSV_HEADER};

use std::path::Path;

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
