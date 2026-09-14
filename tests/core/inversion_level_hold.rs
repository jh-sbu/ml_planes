use ml_planes::controllers::{
    ControllerKind, ControllerTargets, FlightController, InversionLevelHoldController,
};
use ml_planes::plane::{ControlInputs, ControllerContext, FlightState, PlaneId, PHYSICS_DT};
use ml_planes::training::eval_metrics::MetricFamily;
use ml_planes::training::task::{self, EnvSpec, Task};
use ml_planes::training::{DemonstrationEnv, EvalRun, TrainingEnv};

fn env() -> ml_planes::training::LevelHoldEnv {
    let cfg = ml_planes::training::load_plane_config_or_exit("assets/planes/generic_jet.plane.ron");
    let spec = EnvSpec::defaults_for(Task::LevelHold, cfg);
    task::level_hold_env(
        &spec,
        ml_planes::training::reward_config::load_reward_config(
            "assets/training/level_hold.reward.ron",
        )
        .unwrap(),
    )
}

#[test]
fn inversion_level_hold_beats_incumbent_on_original_64_episodes() {
    let mut env = env();
    let ctx = ControllerContext::empty_for(PlaneId::TEST);
    let mut run = EvalRun::new(MetricFamily::LevelHold, 64, 3200, 1);
    for _ in 0..64 {
        env.reset();
        let mut ctrl = InversionLevelHoldController::new(env.target_altitude, env.target_airspeed);
        for step in 0..3200 {
            let u = ctrl.update(&env.current_state(), &ctx, PHYSICS_DT);
            let o = env.step(&[u.elevator, u.throttle * 2.0 - 1.0, u.aileron, u.rudder]);
            run.record(0, &o.obs, o.reward).unwrap();
            assert!(!o.done() || step == 3199, "early termination at {step}");
            if step == 3199 {
                run.finish(0, &o.obs).unwrap();
            }
        }
    }
    let report = run.report();
    assert_eq!(report.success_rate, 1.0);
    assert_eq!(report.mean_length_steps, 3200.0);
    assert!(
        (report.mean_return - -93.2572).abs() < 0.2,
        "return {}",
        report.mean_return
    );
    let metric = |key| report.rows.iter().find(|r| r.key == key).unwrap().value;
    assert!(metric("mean_tail_abs_altitude_m") < 0.015);
    assert!(metric("mean_tail_abs_speed_mps") < 0.17);
}

#[test]
fn inversion_factory_and_targets_survive_pid_tuning() {
    let state = FlightState {
        altitude: 1000.,
        airspeed: 110.,
        ..Default::default()
    };
    let tuning = ml_planes::controllers::LevelHoldTuning::default();
    let mut ctrl =
        ControllerKind::InversionLevelHold.build(&state, Some(&tuning), &ControlInputs::default());
    assert!(ctrl.as_any_mut().is::<InversionLevelHoldController>());
    assert!(ControllerKind::ALL.contains(&ControllerKind::InversionLevelHold));
    let targets = ControllerTargets::LevelHold {
        altitude: 1200.,
        airspeed: 125.,
    };
    ctrl.apply_targets(&targets, &state);
    assert_eq!(ctrl.targets(), targets);
    ctrl.apply_targets(&ControllerTargets::None, &state);
    assert_eq!(ctrl.targets(), targets);
}

#[test]
fn inversion_scenario_builds_without_inference() {
    let scenario: ml_planes::scenario::Scenario = ron::from_str(r#"(
        planes: [(name: "inversion", controller: InversionLevelHold(altitude: 1500.0, airspeed: 115.0))]
    )"#).unwrap();
    let resolved = scenario.resolve().unwrap();
    let mut ctrl = resolved.build_controller(0).unwrap();
    assert_eq!(
        resolved.planes[0].spec.kind(),
        ControllerKind::InversionLevelHold
    );
    assert!(ctrl.as_any_mut().is::<InversionLevelHoldController>());
    assert_eq!(
        ctrl.targets(),
        ControllerTargets::LevelHold {
            altitude: 1500.,
            airspeed: 115.
        }
    );
}

#[test]
fn inversion_survives_asset_load_and_profile_change_and_can_be_switched_to() {
    use bevy::prelude::*;
    use ml_planes::controllers::{
        ActiveController, PlaneTuning, SelectedTuningProfile, SimControlPlugin, TuningApplied,
    };
    use ml_planes::plane::PlaneTuningHandle;

    let mut app = crate::common::build_headless_app_with(|a| {
        a.add_plugins(SimControlPlugin);
    });
    let mut env = env();
    env.reset();
    let state = env.current_state();
    let mut controller = InversionLevelHoldController::new(2000., 120.);
    // A sentinel gain makes an unintended rebuild detectable.
    controller.gains.hp = 3.;
    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneTuning>>()
        .add(PlaneTuning::default());
    let entity = app
        .world_mut()
        .spawn((
            state.clone(),
            ControlInputs::default(),
            ActiveController(Box::new(controller)),
            ControllerKind::InversionLevelHold,
            PlaneTuningHandle(handle),
            SelectedTuningProfile("normal".into()),
        ))
        .id();
    app.update();
    assert!(app.world().get::<TuningApplied>(entity).is_some());
    app.world_mut()
        .get_mut::<SelectedTuningProfile>(entity)
        .unwrap()
        .0 = "other".into();
    app.update();
    {
        let mut active = app.world_mut().get_mut::<ActiveController>(entity).unwrap();
        let c = active
            .0
            .as_any_mut()
            .downcast_mut::<InversionLevelHoldController>()
            .unwrap();
        assert_eq!(c.gains.hp, 3.);
        assert_eq!(
            c.targets(),
            ControllerTargets::LevelHold {
                altitude: 2000.,
                airspeed: 120.
            }
        );
    }
    app.world_mut()
        .get_mut::<ControllerKind>(entity)
        .unwrap()
        .set_if_neq(ControllerKind::Manual);
    app.update();
    app.world_mut()
        .get_mut::<ControllerKind>(entity)
        .unwrap()
        .set_if_neq(ControllerKind::InversionLevelHold);
    app.update();
    let mut active = app.world_mut().get_mut::<ActiveController>(entity).unwrap();
    assert!(active.0.as_any_mut().is::<InversionLevelHoldController>());
    assert_eq!(
        active.0.targets(),
        ControllerTargets::LevelHold {
            altitude: state.altitude,
            airspeed: state.airspeed
        }
    );
}
