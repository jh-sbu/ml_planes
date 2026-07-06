//! Live-sim (Bevy/Rapier) fuel system: spawn-time consumable load, the per-tick
//! `consume_fuel` burn, and `update_plane_mass` keeping the Rapier body mass in
//! sync with remaining fuel. Headless — no rendering.
//!
//! Requires the 6-DOF sim chain (`PlanePlugin` FixedUpdate systems), which is compiled out on a
//! `net`-without-`server` build (e.g. bare `--features mcp`). The `sim_enabled` cfg (see
//! `build.rs`) gates this module (from `tests/core/main.rs`) so it skips there instead of failing on a default
//! `FlightState`; test networked builds with `--no-default-features --features "…​ server"`.

use crate::common::{build_headless_app, build_headless_app_with, generic_jet_config};
use bevy::prelude::*;
use bevy_rapier3d::prelude::{AdditionalMassProperties, MassProperties};
use ml_planes::controllers::{ControllerKind, LevelHoldController};
use ml_planes::environment::spawn_plane;
use ml_planes::plane::{
    ControlInputs, FlightState, FuelType, NextPlaneId, PlaneConfig, PlaneConfigHandle, Powerplant,
};
use ml_planes::training::SpawnSpec;

#[derive(Resource)]
struct SpawnParams {
    fuel_fraction: Option<f32>,
}

#[derive(Resource)]
struct Spawned(Entity);

fn spawn_sys(
    mut commands: Commands,
    mut ids: ResMut<NextPlaneId>,
    asset_server: Res<AssetServer>,
    params: Res<SpawnParams>,
) {
    let cfg = generic_jet_config();
    let spec = SpawnSpec {
        position: Some(Vec3::new(0.0, 1000.0, 0.0)),
        velocity: Some(Vec3::new(100.0, 0.0, 0.0)),
        fuel_fraction: params.fuel_fraction,
        ..Default::default()
    };
    let s = spawn_plane(
        &mut commands,
        &mut ids,
        &asset_server,
        "planes/generic_jet.plane.ron",
        &spec,
        Box::new(LevelHoldController::new(1000.0, 100.0)),
        ControllerKind::LevelHold,
        &cfg,
    );
    commands.insert_resource(Spawned(s.entity));
}

/// Spawn one plane with the given fuel fraction and return its initial
/// `consumable_remaining`.
fn spawn_and_get_fuel(fuel_fraction: Option<f32>) -> f32 {
    let mut app = build_headless_app_with(|app| {
        app.insert_resource(SpawnParams { fuel_fraction });
        app.add_systems(Startup, spawn_sys);
    });
    app.update();
    let e = app.world().resource::<Spawned>().0;
    app.world()
        .entity(e)
        .get::<FlightState>()
        .expect("spawned plane has FlightState")
        .consumable_remaining
}

#[test]
fn shipped_plane_assets_parse_with_expected_powerplants() {
    let jet: PlaneConfig =
        ron::de::from_str(&std::fs::read_to_string("assets/planes/generic_jet.plane.ron").unwrap())
            .expect("generic_jet parses");
    assert!(
        matches!(jet.powerplant, Powerplant::JetFuel { .. }),
        "generic jet should be jet-fuel"
    );
    assert!(jet.powerplant.contributes_mass());

    let electric: PlaneConfig = ron::de::from_str(
        &std::fs::read_to_string("assets/planes/electric_trainer.plane.ron").unwrap(),
    )
    .expect("electric_trainer parses");
    assert!(
        matches!(electric.powerplant, Powerplant::Electric { .. }),
        "electric trainer should be electric"
    );
    assert!(
        !electric.powerplant.contributes_mass(),
        "electric mass is constant"
    );

    // The transport-class jets (cargo, business, tanker) all burn jet fuel and so
    // contribute mass — they must parse and report a JetFuel powerplant.
    for stem in ["cargo_jet", "business_jet", "tanker"] {
        let cfg: PlaneConfig = ron::de::from_str(
            &std::fs::read_to_string(format!("assets/planes/{stem}.plane.ron")).unwrap(),
        )
        .unwrap_or_else(|e| panic!("{stem} parses: {e}"));
        assert!(
            matches!(cfg.powerplant, Powerplant::JetFuel { .. }),
            "{stem} should be jet-fuel"
        );
        assert!(
            cfg.powerplant.contributes_mass(),
            "{stem} jet fuel contributes mass"
        );
    }
}

#[test]
fn spawn_fills_tank_to_capacity_by_default() {
    let capacity = generic_jet_config().powerplant.capacity();
    assert_eq!(spawn_and_get_fuel(None), capacity);
}

#[test]
fn spawn_fuel_fraction_yields_partial_tank() {
    let capacity = generic_jet_config().powerplant.capacity();
    assert_eq!(spawn_and_get_fuel(Some(0.25)), capacity * 0.25);
}

#[test]
fn consume_fuel_system_burns_and_drops_body_mass() {
    let mut app = build_headless_app();

    let mut cfg = generic_jet_config();
    cfg.mass = 3500.0;
    cfg.thrust_max = 60000.0;
    cfg.powerplant = Powerplant::JetFuel {
        capacity_kg: 2000.0,
        tsfc: 2.0e-5,
        fuel_type: FuelType::JetA,
    };
    let initial_fuel = 2000.0;
    let initial_mass = cfg.powerplant.effective_mass(cfg.mass, initial_fuel);

    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneConfig>>()
        .add(cfg.clone());

    let entity = app
        .world_mut()
        .spawn((
            FlightState {
                position: Vec3::new(0.0, 1000.0, 0.0),
                velocity: Vec3::new(100.0, 0.0, 0.0),
                attitude: Quat::IDENTITY,
                consumable_remaining: initial_fuel,
                ..Default::default()
            },
            ControlInputs {
                throttle: 1.0,
                ..Default::default()
            },
            PlaneConfigHandle(handle),
            AdditionalMassProperties::MassProperties(MassProperties {
                local_center_of_mass: Vec3::ZERO,
                mass: initial_mass,
                principal_inertia: cfg.inertia,
                principal_inertia_local_frame: Quat::IDENTITY,
            }),
        ))
        .id();

    for _ in 0..10 {
        app.update();
    }

    let fuel = app
        .world()
        .entity(entity)
        .get::<FlightState>()
        .unwrap()
        .consumable_remaining;
    assert!(
        fuel < initial_fuel,
        "fuel should burn at positive throttle: {fuel} !< {initial_fuel}"
    );

    let mp = app
        .world()
        .entity(entity)
        .get::<AdditionalMassProperties>()
        .unwrap();
    let AdditionalMassProperties::MassProperties(m) = mp else {
        panic!("expected explicit MassProperties");
    };
    assert!(
        m.mass < initial_mass,
        "body mass should drop as fuel burns: {} !< {initial_mass}",
        m.mass
    );
}
