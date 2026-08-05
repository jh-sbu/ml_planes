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
    ActiveController, AscentController, ControllerKind, FormationOffset, HeadingHoldController,
    HeadingHoldTuning, LevelHoldController, LevelHoldTuning, ManualController, OrbitController,
    OrbitParams, OrbitTuning, PlaneTuning, SelectedTuningProfile, SimControlPlugin, TuningApplied,
    WingmanController,
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

/// `HeadingHoldController::from_state` (which `HeadingHoldTuning::build` starts from)
/// re-seeds `target_heading`/`inner.target_altitude`/`inner.target_airspeed` from the
/// *current* `FlightState`. Before the `ControllerTargets`-snapshot fix, a scenario- or
/// pilot-commanded heading target that differed from the plane's current ground track
/// (e.g. `assets/scenarios/heading_hold.scenario.ron`'s 90°/120 m/s target on a plane
/// spawned flying heading 0 at `DEFAULT_SPEED`) was silently cancelled the moment the
/// tuning asset finished loading. This pins that the rebuild now preserves all three.
#[test]
fn initial_tuning_preserves_heading_hold_targets() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    // State flies heading 0 at 1000 m / 100 m/s — deliberately different from the
    // commanded target below, so a reset-to-state is unmistakable.
    let state = level_state(1000.0, 100.0);
    let mut heading_hold = HeadingHoldController::from_state(&state, &ControlInputs::default());
    heading_hold.target_heading = std::f32::consts::FRAC_PI_2; // 90 deg
    heading_hold.inner.target_altitude = 2000.0;
    heading_hold.inner.target_airspeed = 130.0;

    let mut tuning = PlaneTuning::default();
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
        (hh.target_heading - std::f32::consts::FRAC_PI_2).abs() < 1e-6,
        "target_heading must survive the rebuild, got {}",
        hh.target_heading
    );
    assert!(
        (hh.inner.target_altitude - 2000.0).abs() < 1e-6,
        "target_altitude must survive the rebuild, got {}",
        hh.inner.target_altitude
    );
    assert!(
        (hh.inner.target_airspeed - 130.0).abs() < 1e-6,
        "target_airspeed must survive the rebuild, got {}",
        hh.inner.target_airspeed
    );
    assert!(
        (hh.heading_pid.kp - 4.56).abs() < 1e-6,
        "the loaded tuning profile must still reach the controller, got kp={}",
        hh.heading_pid.kp
    );
}

/// The identical clobber exists in `apply_controller_switch` (a later profile switch).
/// This pins that switching profiles on an already-turning heading-hold plane keeps
/// the commanded heading/altitude/airspeed while still re-applying the new gains.
#[test]
fn controller_switch_preserves_heading_hold_targets() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    let state = level_state(1000.0, 100.0);
    let mut heading_hold = HeadingHoldController::from_state(&state, &ControlInputs::default());
    heading_hold.target_heading = std::f32::consts::FRAC_PI_2;
    heading_hold.inner.target_altitude = 2000.0;
    heading_hold.inner.target_airspeed = 130.0;

    let mut tuning = PlaneTuning::default();
    tuning
        .heading_hold
        .insert("normal".to_string(), HeadingHoldTuning::default());
    tuning.heading_hold.insert(
        "aggressive".to_string(),
        HeadingHoldTuning {
            heading_kp: 9.99,
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

    // First update: apply_initial_tuning fires and picks up "normal".
    app.update();

    // Switch profiles mid-turn — exactly what a server command handler / HUD does.
    app.world_mut()
        .get_mut::<SelectedTuningProfile>(entity)
        .unwrap()
        .set_if_neq(SelectedTuningProfile("aggressive".to_string()));
    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(entity).unwrap();
    let hh = ctrl
        .0
        .as_any_mut()
        .downcast_mut::<HeadingHoldController>()
        .expect("controller must still be a HeadingHoldController after the profile switch");
    assert!(
        (hh.target_heading - std::f32::consts::FRAC_PI_2).abs() < 1e-6,
        "target_heading must survive the profile switch, got {}",
        hh.target_heading
    );
    assert!(
        (hh.inner.target_altitude - 2000.0).abs() < 1e-6,
        "target_altitude must survive the profile switch, got {}",
        hh.inner.target_altitude
    );
    assert!(
        (hh.inner.target_airspeed - 130.0).abs() < 1e-6,
        "target_airspeed must survive the profile switch, got {}",
        hh.inner.target_airspeed
    );
    assert!(
        (hh.heading_pid.kp - 9.99).abs() < 1e-6,
        "the new profile must reach the controller, got kp={}",
        hh.heading_pid.kp
    );
}

/// `LevelHoldController::with_tuning` (which `ControllerKind::LevelHold.build()` calls)
/// starts from `Self::new(state.altitude, state.airspeed)`, re-seeding both setpoints
/// from live state — the same class of bug as heading hold, just less visually obvious.
/// This pins that `apply_initial_tuning` now preserves a level-hold target that differs
/// from the plane's current altitude/airspeed (e.g. right after an ascent hands off, or
/// a scenario's `LevelHold { altitude, airspeed }` spec on a plane spawned below target).
#[test]
fn initial_tuning_preserves_level_hold_targets() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    let state = level_state(1000.0, 100.0);
    let mut level_hold = LevelHoldController::from_state(&state, &ControlInputs::default());
    level_hold.target_altitude = 3000.0;
    level_hold.target_airspeed = 140.0;

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
            state,
            ControlInputs::default(),
            ActiveController(Box::new(level_hold)),
            ControllerKind::LevelHold,
            PlaneTuningHandle(handle),
            SelectedTuningProfile("normal".to_string()),
        ))
        .id();

    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(entity).unwrap();
    let lh = ctrl
        .0
        .as_any_mut()
        .downcast_mut::<LevelHoldController>()
        .expect("controller must still be a LevelHoldController after the tuning rebuild");
    assert!(
        (lh.target_altitude - 3000.0).abs() < 1e-6,
        "target_altitude must survive the rebuild, got {}",
        lh.target_altitude
    );
    assert!(
        (lh.target_airspeed - 140.0).abs() < 1e-6,
        "target_airspeed must survive the rebuild, got {}",
        lh.target_airspeed
    );
    assert!(
        (lh.altitude_pid.kp - 9.87).abs() < 1e-6,
        "the loaded tuning profile must still reach the controller, got kp={}",
        lh.altitude_pid.kp
    );
}

/// `ControllerKind::Ascent.build()` always re-targets `state.altitude + 1000.0`
/// (`kind.rs`'s own doc comment says so) — a tuning rebuild mid-climb used to reset
/// the climb target to whatever the plane's current altitude happened to be plus
/// 1000 m, discarding the originally commanded target. This pins that it now survives.
#[test]
fn initial_tuning_preserves_ascent_target_altitude() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    // Mid-climb: below target, so state.altitude + 1000.0 (2100.0) would be a very
    // different — and wrong — target than the one actually commanded (5000.0).
    let state = level_state(1100.0, 100.0);
    let ascent = AscentController::new(&state, 5000.0);

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
            state,
            ControlInputs::default(),
            ActiveController(Box::new(ascent)),
            ControllerKind::Ascent,
            PlaneTuningHandle(handle),
            SelectedTuningProfile("normal".to_string()),
        ))
        .id();

    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(entity).unwrap();
    let ac = ctrl
        .0
        .as_any_mut()
        .downcast_mut::<AscentController>()
        .expect("controller must still be an AscentController after the tuning rebuild");
    assert!(
        (ac.target_altitude - 5000.0).abs() < 1e-6,
        "target_altitude must survive the rebuild instead of resetting to state.altitude + 1000, got {}",
        ac.target_altitude
    );
    // Note: `ControllerKind::Ascent.build()` (`kind.rs`) never forwards `tuning` into
    // `AscentController::new`, so the inner LevelHoldController always uses default
    // gains regardless of profile — a separate, pre-existing gap out of scope here.
}

/// Regression guard: orbit geometry preservation (`extract_orbit_params` /
/// `OrbitController::apply_params`) must be unaffected by the new
/// LevelHold/HeadingHold/Ascent targets-snapshot logic added alongside it —
/// `Orbit` is deliberately excluded from `rebuild_preserves_targets`.
#[test]
fn initial_tuning_still_rebuilds_orbit_geometry() {
    let mut app = build_headless_app_with(|app| {
        app.add_plugins(SimControlPlugin);
    });

    let state = level_state(1000.0, 100.0);
    let mut orbit = OrbitController::from_state(&state, &ControlInputs::default());
    orbit.apply_params(
        &OrbitParams {
            center_x: 500.0,
            center_z: -250.0,
            target_radius: 800.0,
            target_altitude: 1200.0,
            target_airspeed: 110.0,
            direction: ml_planes::controllers::OrbitDirection::Clockwise,
        },
        state.airspeed,
    );

    let mut tuning = PlaneTuning::default();
    tuning.orbit.insert(
        "normal".to_string(),
        OrbitTuning {
            radial_kp: 4.56,
            ..OrbitTuning::default()
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
            ActiveController(Box::new(orbit)),
            ControllerKind::Orbit,
            PlaneTuningHandle(handle),
            SelectedTuningProfile("normal".to_string()),
        ))
        .id();

    app.update();

    let world = app.world_mut();
    let mut ctrl = world.get_mut::<ActiveController>(entity).unwrap();
    let oc = ctrl
        .0
        .as_any_mut()
        .downcast_mut::<OrbitController>()
        .expect("controller must still be an OrbitController after the tuning rebuild");
    assert!(
        (oc.center_x - 500.0).abs() < 1e-6 && (oc.center_z - (-250.0)).abs() < 1e-6,
        "orbit geometry must still survive the rebuild"
    );
    assert!(
        (oc.radial_pid.kp - 4.56).abs() < 1e-6,
        "the loaded orbit tuning profile must still reach the controller, got kp={}",
        oc.radial_pid.kp
    );
}
