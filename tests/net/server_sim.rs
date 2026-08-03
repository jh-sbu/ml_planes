//! Phase 3 (client/server): the dedicated-server authoritative sim.
//!
//! Covers Milestone 3 — the headless server boots a scenario, marks its planes
//! `Replicated`, and applies the client→server commands (controller switch,
//! spawn/remove, sim-speed) by mutating the components `SimControlPlugin` /
//! `LifecyclePlugin` already react to. Runs `ServerSimPlugin` **without** the renet
//! transport, so no UDP socket is bound; client commands are injected directly as
//! `FromClient` events (exactly what the real transport produces server-side).
//!
//! Gated on `server` (`cargo test --features server`).

use bevy::prelude::*;
use bevy::state::app::StatesPlugin;
use bevy_replicon::prelude::{ClientId, FromClient, Replicated, RepliconPlugins};

use crate::common::build_headless_app_with;
use ml_planes::controllers::{
    ActiveController, ControllerKind, ControllerTargets, ControllerTelemetry, ManualController,
    OrbitController, SimControlPlugin,
};
use ml_planes::environment::{EnvironmentPlugin, LifecyclePlugin};
use ml_planes::net::{
    ManualInputCommand, NetProtocolPlugin, RemovePlaneNetCommand, ServerScenario, ServerSimPlugin,
    SetControllerTargetsCommand, SetSimSpeedCommand, SpawnPlaneNetCommand, SwitchControllerCommand,
};
use ml_planes::plane::{ControlInputs, PlaneId};
use ml_planes::sim_speed::SimSpeed;
use ml_planes::training::SpawnSpec;

/// Build a headless server-sim app (no transport) loading the default scenario.
fn build_server_app() -> App {
    build_headless_app_with(|app| {
        app.add_plugins(StatesPlugin)
            .add_plugins(RepliconPlugins)
            .add_plugins(NetProtocolPlugin)
            .add_plugins(EnvironmentPlugin)
            .add_plugins(LifecyclePlugin)
            .add_plugins(SimControlPlugin)
            .add_plugins(ServerSimPlugin)
            .insert_resource(ServerScenario(
                "assets/scenarios/default.scenario.ron".into(),
            ));
    })
}

/// Step the app enough times for the Startup scenario spawn + the
/// `mark_planes_replicated` / tuning systems to settle.
fn settle(app: &mut App) {
    for _ in 0..4 {
        app.update();
    }
}

/// Collect live planes as `(entity, PlaneId)`, sorted by id.
fn planes(app: &mut App) -> Vec<(Entity, PlaneId)> {
    let mut q = app.world_mut().query::<(Entity, &PlaneId)>();
    let mut v: Vec<(Entity, PlaneId)> = q.iter(app.world()).map(|(e, id)| (e, *id)).collect();
    v.sort_by_key(|(_, id)| id.0);
    v
}

#[test]
fn scenario_boots_and_planes_are_replicated() {
    let mut app = build_server_app();
    settle(&mut app);

    let live = planes(&mut app);
    assert!(
        !live.is_empty(),
        "the default scenario should spawn at least one plane"
    );

    // Every spawned plane must carry the replication marker so replicon broadcasts it.
    for (entity, id) in &live {
        assert!(
            app.world().get::<Replicated>(*entity).is_some(),
            "plane {id:?} should have been marked Replicated"
        );
    }
}

#[test]
fn switch_controller_command_rebuilds_active_controller() {
    let mut app = build_server_app();
    settle(&mut app);

    // Target the first plane and switch it to Manual (no default-scenario plane is
    // Manual, so the kind genuinely changes and the rebuild fires).
    let (entity, plane) = planes(&mut app)[0];

    app.world_mut().trigger(FromClient {
        client_id: ClientId::Server,
        message: SwitchControllerCommand {
            plane,
            kind: ControllerKind::Manual,
        },
    });
    // One update applies the command + lets SimControlPlugin (PostUpdate) rebuild.
    app.update();
    app.update();

    assert_eq!(
        app.world().get::<ControllerKind>(entity).copied(),
        Some(ControllerKind::Manual),
        "the command should have set the target's ControllerKind"
    );
    let binding = app.world_mut();
    let mut ctrl = binding.get_mut::<ActiveController>(entity).unwrap();
    assert!(
        ctrl.0
            .as_any_mut()
            .downcast_mut::<ManualController>()
            .is_some(),
        "SimControlPlugin should have rebuilt a ManualController"
    );
}

#[test]
fn server_populates_replicated_controller_telemetry() {
    let mut app = build_server_app();
    settle(&mut app);

    // Switch the first plane to Orbit so the server steps an OrbitController, which
    // publishes radial-error telemetry the copy system snapshots each fixed tick.
    let (entity, plane) = planes(&mut app)[0];
    app.world_mut().trigger(FromClient {
        client_id: ClientId::Server,
        message: SwitchControllerCommand {
            plane,
            kind: ControllerKind::Orbit,
        },
    });
    settle(&mut app);

    // The telemetry component exists, is registered for replication (the plane is
    // marked Replicated), and carries Orbit telemetry once the controller has run.
    assert!(
        app.world().get::<Replicated>(entity).is_some(),
        "telemetry rides on the replicated plane entity"
    );
    let tel = app
        .world()
        .get::<ControllerTelemetry>(entity)
        .expect("plane carries a ControllerTelemetry component");
    assert!(
        matches!(tel, ControllerTelemetry::Orbit { .. }),
        "expected Orbit telemetry after the server stepped the controller, got {tel:?}"
    );
}

#[test]
fn server_populates_replicated_controller_targets() {
    let mut app = build_server_app();
    settle(&mut app);

    // Switch the first plane to Orbit so the server steps an OrbitController, whose
    // geometry the copy system snapshots each fixed tick into `ControllerTargets`.
    let (entity, plane) = planes(&mut app)[0];
    app.world_mut().trigger(FromClient {
        client_id: ClientId::Server,
        message: SwitchControllerCommand {
            plane,
            kind: ControllerKind::Orbit,
        },
    });
    settle(&mut app);

    assert!(
        app.world().get::<Replicated>(entity).is_some(),
        "targets ride on the replicated plane entity"
    );
    let targets = app
        .world()
        .get::<ControllerTargets>(entity)
        .copied()
        .expect("plane carries a ControllerTargets component");
    let ControllerTargets::Orbit(replicated_params) = targets else {
        panic!("expected Orbit targets after the server stepped the controller, got {targets:?}")
    };

    let binding = app.world_mut();
    let mut ctrl = binding.get_mut::<ActiveController>(entity).unwrap();
    let live = ctrl
        .0
        .as_any_mut()
        .downcast_mut::<OrbitController>()
        .expect("plane should be an OrbitController");
    assert_eq!(replicated_params.target_radius, live.target_radius);
    assert_eq!(replicated_params.target_altitude, live.target_altitude);
    assert_eq!(replicated_params.target_airspeed, live.target_airspeed);
}

#[test]
fn set_controller_targets_command_updates_live_controller() {
    let mut app = build_server_app();
    settle(&mut app);

    // Switch the first plane to LevelHold so there's a scalar target to edit.
    let (entity, plane) = planes(&mut app)[0];
    app.world_mut().trigger(FromClient {
        client_id: ClientId::Server,
        message: SwitchControllerCommand {
            plane,
            kind: ControllerKind::LevelHold,
        },
    });
    app.update();
    app.update();

    app.world_mut().trigger(FromClient {
        client_id: ClientId::Server,
        message: SetControllerTargetsCommand {
            plane,
            targets: ControllerTargets::LevelHold {
                altitude: 1600.0,
                airspeed: 110.0,
            },
        },
    });
    settle(&mut app);

    let binding = app.world_mut();
    let mut ctrl = binding.get_mut::<ActiveController>(entity).unwrap();
    let live = ctrl
        .0
        .as_any_mut()
        .downcast_mut::<ml_planes::controllers::LevelHoldController>()
        .expect("plane should be LevelHold");
    assert_eq!(
        live.target_altitude, 1600.0,
        "live controller should apply the command"
    );
    assert_eq!(
        live.target_airspeed, 110.0,
        "live controller should apply the command"
    );

    // The read path echoes it back: the replicated component reflects the edit too.
    let targets = app
        .world()
        .get::<ControllerTargets>(entity)
        .copied()
        .expect("plane carries a ControllerTargets component");
    assert_eq!(
        targets,
        ControllerTargets::LevelHold {
            altitude: 1600.0,
            airspeed: 110.0,
        },
        "replicated targets should echo the applied command"
    );
}

#[test]
fn set_controller_targets_ignores_variant_mismatch() {
    let mut app = build_server_app();
    settle(&mut app);

    let (entity, plane) = planes(&mut app)[0];
    app.world_mut().trigger(FromClient {
        client_id: ClientId::Server,
        message: SwitchControllerCommand {
            plane,
            kind: ControllerKind::LevelHold,
        },
    });
    app.update();
    app.update();

    let before = {
        let binding = app.world_mut();
        let mut ctrl = binding.get_mut::<ActiveController>(entity).unwrap();
        let live = ctrl
            .0
            .as_any_mut()
            .downcast_mut::<ml_planes::controllers::LevelHoldController>()
            .expect("plane should be LevelHold");
        (live.target_altitude, live.target_airspeed)
    };

    // Send Orbit targets to a LevelHold plane — must be a no-op.
    app.world_mut().trigger(FromClient {
        client_id: ClientId::Server,
        message: SetControllerTargetsCommand {
            plane,
            targets: ControllerTargets::Orbit(ml_planes::controllers::orbit::OrbitParams {
                center_x: 1.0,
                center_z: 2.0,
                target_radius: 3.0,
                target_altitude: 4.0,
                target_airspeed: 5.0,
                direction: ml_planes::controllers::OrbitDirection::Clockwise,
            }),
        },
    });
    settle(&mut app);

    let binding = app.world_mut();
    let mut ctrl = binding.get_mut::<ActiveController>(entity).unwrap();
    let live = ctrl
        .0
        .as_any_mut()
        .downcast_mut::<ml_planes::controllers::LevelHoldController>()
        .expect("plane should still be LevelHold");
    assert_eq!(
        (live.target_altitude, live.target_airspeed),
        before,
        "a mismatched-variant command must be a no-op"
    );
}

#[test]
fn set_controller_targets_for_unknown_plane_is_a_noop() {
    let mut app = build_server_app();
    settle(&mut app);

    // Must not panic when the plane doesn't exist.
    app.world_mut().trigger(FromClient {
        client_id: ClientId::Server,
        message: SetControllerTargetsCommand {
            plane: PlaneId(9999),
            targets: ControllerTargets::LevelHold {
                altitude: 1000.0,
                airspeed: 80.0,
            },
        },
    });
    settle(&mut app);
}

#[test]
fn spawn_and_remove_commands_take_effect() {
    let mut app = build_server_app();
    settle(&mut app);

    let before = planes(&mut app).len();

    app.world_mut().trigger(FromClient {
        client_id: ClientId::Server,
        message: SpawnPlaneNetCommand {
            config_path: "planes/generic_jet.plane.ron".to_string(),
            kind: ControllerKind::LevelHold,
            spec: SpawnSpec {
                position: Some(Vec3::new(0.0, 1000.0, 0.0)),
                velocity: Some(Vec3::new(100.0, 0.0, 0.0)),
                ..Default::default()
            },
        },
    });
    settle(&mut app);

    let after_spawn = planes(&mut app);
    assert_eq!(
        after_spawn.len(),
        before + 1,
        "SpawnPlaneNetCommand should add exactly one plane"
    );
    // The newest plane is the one with the largest id.
    let (new_entity, new_id) = *after_spawn.last().unwrap();
    assert!(
        app.world().get::<Replicated>(new_entity).is_some(),
        "a runtime-spawned plane is replicated too"
    );

    app.world_mut().trigger(FromClient {
        client_id: ClientId::Server,
        message: RemovePlaneNetCommand { plane: new_id },
    });
    settle(&mut app);

    assert_eq!(
        planes(&mut app).len(),
        before,
        "RemovePlaneNetCommand should despawn the plane again"
    );
}

#[test]
fn set_sim_speed_command_pauses_the_clock() {
    let mut app = build_server_app();
    settle(&mut app);

    app.world_mut().trigger(FromClient {
        client_id: ClientId::Server,
        message: SetSimSpeedCommand {
            speed: SimSpeed::Paused,
        },
    });
    app.update();

    assert_eq!(
        *app.world().resource::<SimSpeed>(),
        SimSpeed::Paused,
        "the command should set the authoritative SimSpeed"
    );
    assert!(
        app.world().resource::<Time<Virtual>>().is_paused(),
        "apply_sim_speed should have paused Time<Virtual>"
    );
}

#[test]
fn manual_input_command_feeds_manual_controller() {
    let mut app = build_server_app();
    settle(&mut app);

    let (entity, plane) = planes(&mut app)[0];

    // Make the plane Manual first.
    app.world_mut().trigger(FromClient {
        client_id: ClientId::Server,
        message: SwitchControllerCommand {
            plane,
            kind: ControllerKind::Manual,
        },
    });
    app.update();
    app.update();

    let inputs = ControlInputs {
        aileron: 0.2,
        elevator: -0.3,
        rudder: 0.1,
        throttle: 0.7,
    };
    app.world_mut().trigger(FromClient {
        client_id: ClientId::Server,
        message: ManualInputCommand {
            plane,
            inputs: inputs.clone(),
        },
    });
    app.update();

    let binding = app.world_mut();
    let mut ctrl = binding.get_mut::<ActiveController>(entity).unwrap();
    let manual = ctrl
        .0
        .as_any_mut()
        .downcast_mut::<ManualController>()
        .expect("plane should be Manual");
    assert_eq!(
        manual.inputs.throttle, inputs.throttle,
        "ManualInputCommand should have stored the throttle input"
    );
}
