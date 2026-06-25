//! Phase 2: the controller-rebuild systems run headlessly via `SimControlPlugin`.
//!
//! These systems used to live in `main.rs` behind `#[cfg(feature = "visual")]` and
//! `run_if(in_state(AppState::InGame))`, so they could not run on a headless server.
//! This test pins that a `MinimalPlugins` app with only `SimControlPlugin` rebuilds a
//! plane's `ActiveController` when its `ControllerKind` is mutated — the mechanism the
//! server uses to apply a client's `SwitchControllerCommand`.

mod common;

use bevy::prelude::*;
use common::build_headless_app_with;
use ml_planes::controllers::{
    ActiveController, ControllerKind, LevelHoldController, ManualController, SimControlPlugin,
};
use ml_planes::plane::{ControlInputs, FlightState};

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
