//! Phase 2: the controller-rebuild systems run headlessly via `SimControlPlugin`.
//!
//! These systems used to live in `main.rs` behind `#[cfg(feature = "visual")]` and
//! `run_if(in_state(AppState::InGame))`, so they could not run on a headless server.
//! This test pins that a `MinimalPlugins` app with only `SimControlPlugin` rebuilds a
//! plane's `ActiveController` when its `ControllerKind` is mutated — the mechanism the
//! server uses to apply a client's `SwitchControllerCommand`.

use crate::common::build_headless_app_with;
use bevy::prelude::*;
use ml_planes::controllers::{
    ActiveController, ControllerKind, FormationOffset, HeadingHoldController, HeadingHoldTuning,
    LevelHoldController, LevelHoldTuning, ManualController, PlaneTuning, SelectedTuningProfile,
    SimControlPlugin, TuningApplied, WingmanController,
};
use ml_planes::plane::{ControlInputs, FlightState, PlaneId, PlaneTuningHandle};

/// Spawn a plane carrying a `ManualController`, then flip its `ControllerKind` to
/// `LevelHold` and confirm `SimControlPlugin`'s rebuild system swapped in a
/// `LevelHoldController` — no rendering, no `AppState`.
#[test]
fn switching_controller_kind_rebuilds_active_controller() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    let mut state = FlightState {
        position: Vec3::new(0.0, 1000.0, 0.0),
        velocity: Vec3::new(100.0, 0.0, 0.0),
        ..Default::default()
    };
    state.update_air_data();

    let entity = app
        .world_mut()
        .spawn((
            state,
            ControlInputs::default(),
            ActiveController(Box::new(ManualController::new())),
            ControllerKind::Manual,
        ))
        .id();

    // First tick consumes the spawn-frame insertion: the rebuild system skips
    // entities whose `ControllerKind` was only just added.
    app.update();
    {
        let binding = app.world_mut();
        let mut ctrl = binding.get_mut::<ActiveController>(entity).unwrap();
        assert!(
            ctrl.0
                .as_any_mut()
                .downcast_mut::<ManualController>()
                .is_some(),
            "controller unchanged before the kind is mutated"
        );
    }

    // Mutate the kind — exactly what a server command handler does.
    app.world_mut()
        .get_mut::<ControllerKind>(entity)
        .unwrap()
        .set_if_neq(ControllerKind::LevelHold);
    app.update();

    let binding = app.world_mut();
    let mut ctrl = binding.get_mut::<ActiveController>(entity).unwrap();
    assert!(
        ctrl.0
            .as_any_mut()
            .downcast_mut::<LevelHoldController>()
            .is_some(),
        "rebuild system swapped in a LevelHoldController after the kind change"
    );
}

/// Regression guard for a bug fixed alongside `RlHeadingHold`: `apply_initial_tuning`'s
/// tuning-family match had no `HeadingHold` arm at all, so on the tuning-asset-load frame
/// a heading-hold plane got `level_hold` gains instead of `heading_hold` gains — while
/// `apply_controller_switch` (a later profile switch) correctly used the `heading_hold`
/// pool. This pins that the *initial* rebuild now reaches the same `heading_hold` profile.
#[test]
fn initial_tuning_applies_heading_hold_pool_to_heading_hold_controller() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    let state = level_state(1000.0, 100.0);
    let heading_hold = HeadingHoldController::from_state(&state, &ControlInputs::default());

    let mut tuning = PlaneTuning::default();
    // A distinctive level_hold gain that must NOT reach the controller — if the bug
    // regresses, apply_initial_tuning falls through to this pool instead.
    tuning.level_hold.insert(
        "normal".to_string(),
        LevelHoldTuning {
            alt_kp: 9.87,
            ..LevelHoldTuning::default()
        },
    );
    tuning.heading_hold.insert(
        "normal".to_string(),
        HeadingHoldTuning {
            heading_kp: 4.56,
            ..HeadingHoldTuning::default()
        },
    );
    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneTuning>>()
        .add(tuning);

    let entity = app
        .world_mut()
        .spawn((
            state,
            ControlInputs::default(),
            ActiveController(Box::new(heading_hold)),
            ControllerKind::HeadingHold,
            PlaneTuningHandle(handle),
            SelectedTuningProfile("normal".to_string()),
        ))
        .id();

    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(entity).unwrap();
    let hh = ctrl
        .0
        .as_any_mut()
        .downcast_mut::<HeadingHoldController>()
        .expect("controller must still be a HeadingHoldController after the tuning rebuild");
    assert!(
        (hh.heading_pid.kp - 4.56).abs() < 1e-6,
        "apply_initial_tuning must apply the heading_hold pool, not level_hold; got kp={}",
        hh.heading_pid.kp
    );
}

fn level_state(altitude: f32, airspeed: f32) -> FlightState {
    let mut state = FlightState {
        position: Vec3::new(0.0, altitude, 0.0),
        velocity: Vec3::new(airspeed, 0.0, 0.0),
        ..Default::default()
    };
    state.update_air_data();
    state
}

/// `ControllerKind::Wingman.build()` cannot construct a real `WingmanController`
/// (the generic factory has no leader reference) — it falls back to a plain
/// `LevelHoldController`. `apply_initial_tuning` used to rebuild *every*
/// controller unconditionally once the `.tuning.ron` asset loaded, silently
/// replacing a live wingman with that fallback. This pins that the rebuild now
/// preserves the wingman law (leader id + offset) while still applying the
/// loaded tuning profile to the inner `LevelHoldController`.
#[test]
fn initial_tuning_preserves_wingman_controller() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    let leader_state = level_state(1000.0, 100.0);
    let own_state = level_state(1000.0, 100.0);
    let offset = FormationOffset::default();
    let wingman = WingmanController::new(PlaneId(1), &leader_state, &own_state, offset.clone());

    let mut tuning = PlaneTuning::default();
    tuning.level_hold.insert(
        "normal".to_string(),
        LevelHoldTuning {
            alt_kp: 9.87,
            ..LevelHoldTuning::default()
        },
    );
    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneTuning>>()
        .add(tuning);

    let entity = app
        .world_mut()
        .spawn((
            own_state,
            ControlInputs::default(),
            ActiveController(Box::new(wingman)),
            ControllerKind::Wingman,
            PlaneTuningHandle(handle),
            SelectedTuningProfile("normal".to_string()),
        ))
        .id();

    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(entity).unwrap();
    let wc = ctrl
        .0
        .as_any_mut()
        .downcast_mut::<WingmanController>()
        .expect("controller must still be a WingmanController after the tuning rebuild");
    assert_eq!(
        wc.leader_id,
        PlaneId(1),
        "leader id must survive the rebuild"
    );
    assert_eq!(
        wc.offset.offset_body, offset.offset_body,
        "formation offset must survive the rebuild"
    );
    assert!(
        (wc.inner.altitude_pid.kp - 9.87).abs() < 1e-6,
        "the loaded tuning profile must reach the inner LevelHoldController, got kp={}",
        wc.inner.altitude_pid.kp
    );
    assert!(
        world.get::<TuningApplied>(entity).is_some(),
        "TuningApplied must be inserted so the rebuild doesn't repeat every frame"
    );
}

/// A later profile switch (`apply_controller_switch`) has the identical clobber
/// as the initial-tuning path. This pins that switching profiles on an
/// already-tuned wingman keeps the wingman law and re-applies the new profile.
#[test]
fn profile_switch_preserves_wingman_controller() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    let leader_state = level_state(1000.0, 100.0);
    let own_state = level_state(1000.0, 100.0);
    let offset = FormationOffset::default();
    let wingman = WingmanController::new(PlaneId(1), &leader_state, &own_state, offset);

    let mut tuning = PlaneTuning::default();
    tuning.level_hold.insert(
        "normal".to_string(),
        LevelHoldTuning {
            alt_kp: 9.87,
            ..LevelHoldTuning::default()
        },
    );
    tuning.level_hold.insert(
        "aggressive".to_string(),
        LevelHoldTuning {
            alt_kp: 1.23,
            ..LevelHoldTuning::default()
        },
    );
    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneTuning>>()
        .add(tuning);

    let entity = app
        .world_mut()
        .spawn((
            own_state,
            ControlInputs::default(),
            ActiveController(Box::new(wingman)),
            ControllerKind::Wingman,
            PlaneTuningHandle(handle),
            SelectedTuningProfile("normal".to_string()),
        ))
        .id();

    // First update: apply_initial_tuning fires and picks up "normal".
    app.update();

    // Switch profiles — exactly what a server command handler / HUD does.
    app.world_mut()
        .get_mut::<SelectedTuningProfile>(entity)
        .unwrap()
        .set_if_neq(SelectedTuningProfile("aggressive".to_string()));
    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(entity).unwrap();
    let wc = ctrl
        .0
        .as_any_mut()
        .downcast_mut::<WingmanController>()
        .expect("controller must still be a WingmanController after the profile switch");
    assert_eq!(
        wc.leader_id,
        PlaneId(1),
        "leader id must survive the switch"
    );
    assert!(
        (wc.inner.altitude_pid.kp - 1.23).abs() < 1e-6,
        "the new profile must reach the inner LevelHoldController, got kp={}",
        wc.inner.altitude_pid.kp
    );
}
