use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

// The binary wires the core plugins; the controller-rebuild systems live in
// `SimControlPlugin` (shared with the headless server). Only the client-side
// input/hotkey systems remain here, gated to the visual build.
use ml_planes::controllers::SimControlPlugin;
use ml_planes::environment::{EnvironmentPlugin, LifecyclePlugin};
use ml_planes::plane::PlanePlugin;

#[cfg(feature = "visual")]
use ml_planes::controllers::{
    ActiveController, ControllerKind, PlaneTuning, SelectedTuningProfile,
};
#[cfg(feature = "visual")]
use ml_planes::plane::{FlightState, PlaneTuningHandle};

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
use ml_planes::controllers::{ModelLibrary, SelectedModel};

#[cfg(feature = "visual")]
use bevy::asset::{AssetMetaCheck, AssetPlugin};
#[cfg(feature = "visual")]
use bevy_egui::EguiPlugin;
#[cfg(feature = "visual")]
use ml_planes::camera::{CameraMode, CameraPlugin};
#[cfg(feature = "visual")]
use ml_planes::ui::{AppState, UiPlugin};

fn main() {
    let mut app = App::new();

    // This project does not use Bevy .meta sidecar files; skipping the check
    // eliminates spurious HTTP 404 errors in the WASM build and is safe because
    // no asset requires custom loader settings or processing overrides.
    #[cfg(feature = "visual")]
    app.add_plugins(DefaultPlugins.set(AssetPlugin {
        meta_check: AssetMetaCheck::Never,
        ..default()
    }));
    #[cfg(not(feature = "visual"))]
    app.add_plugins((
        MinimalPlugins,
        bevy::asset::AssetPlugin::default(),
        bevy::transform::TransformPlugin,
    ));

    app.insert_resource(TimestepMode::Fixed {
        dt: 1.0 / 64.0,
        substeps: 1,
    });
    app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule());
    app.add_plugins(PlanePlugin);
    app.add_plugins(EnvironmentPlugin);
    app.add_plugins(LifecyclePlugin);
    // The controller-rebuild systems (apply_initial_tuning / apply_controller_switch /
    // apply_flight_plan + the RL load arms) live here so the headless server runs them
    // too. The visual app boots into the main menu; the scenario it picks is spawned on
    // `OnEnter(AppState::InGame)` by the `MenuPlugin` (added via `UiPlugin`). The headless
    // build has no menu, so it spawns nothing automatically.
    app.add_plugins(SimControlPlugin);

    #[cfg(feature = "visual")]
    {
        app.add_plugins(EguiPlugin::default());
        app.add_plugins(CameraPlugin);
        app.add_plugins(UiPlugin);
    }

    // Client-side input/hotkey systems only run while a scenario is flying, so the menu
    // screens don't drive controller switching, input polling, etc. They mutate the
    // components that `SimControlPlugin` reacts to (and become network command senders in
    // a later client/server phase).
    #[cfg(feature = "visual")]
    app.add_systems(
        Update,
        (
            poll_controller_inputs,
            switch_controller,
            cycle_tune_profile,
        )
            .run_if(in_state(AppState::InGame)),
    );
    #[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
    app.add_systems(Update, cycle_rl_model.run_if(in_state(AppState::InGame)));

    app.run();
}

#[cfg(feature = "visual")]
fn poll_controller_inputs(
    mut query: Query<&mut ActiveController, With<FlightState>>,
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
) {
    let dt = time.delta_secs();
    for mut controller in query.iter_mut() {
        controller.0.poll_input(&keys, dt);
    }
}

/// C key: cycle through available controller kinds for every plane.
#[cfg(feature = "visual")]
fn switch_controller(
    mut query: Query<&mut ControllerKind, With<FlightState>>,
    keys: Res<ButtonInput<KeyCode>>,
) {
    if keys.just_pressed(KeyCode::KeyC) {
        for mut kind in query.iter_mut() {
            let next = kind.next();
            kind.set_if_neq(next);
        }
    }
}

/// T / Shift+T: cycle tune profile forward/backward for the followed plane.
#[cfg(feature = "visual")]
fn cycle_tune_profile(
    mode: Res<CameraMode>,
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<(
        &ControllerKind,
        Option<&mut SelectedTuningProfile>,
        Option<&PlaneTuningHandle>,
    )>,
    tuning_assets: Res<Assets<PlaneTuning>>,
) {
    if !keys.just_pressed(KeyCode::KeyT) {
        return;
    }
    let CameraMode::Follow(entity) = *mode else {
        return;
    };
    let Ok((kind, Some(mut profile), Some(handle))) = query.get_mut(entity) else {
        return;
    };
    let Some(pt) = tuning_assets.get(&handle.0) else {
        return;
    };
    let mut names: Vec<&str> = match *kind {
        ControllerKind::LevelHold | ControllerKind::RlLevelHold => {
            pt.level_hold.keys().map(|s| s.as_str()).collect()
        }
        ControllerKind::Orbit | ControllerKind::RlOrbit => {
            pt.orbit.keys().map(|s| s.as_str()).collect()
        }
        ControllerKind::HeadingHold => pt.heading_hold.keys().map(|s| s.as_str()).collect(),
        _ => return,
    };
    names.sort();
    if names.is_empty() {
        return;
    }
    let n = names.len();
    let cur = names.iter().position(|&nm| nm == profile.0).unwrap_or(0);
    let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    let next = if shift {
        (cur + n - 1) % n
    } else {
        (cur + 1) % n
    };
    profile.set_if_neq(SelectedTuningProfile(names[next].to_string()));
}

/// T / Shift+T: cycle RL model forward/backward for the followed plane.
#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
fn cycle_rl_model(
    mode: Res<CameraMode>,
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&ControllerKind, Option<&mut SelectedModel>)>,
    model_lib: Res<ModelLibrary>,
) {
    if !keys.just_pressed(KeyCode::KeyT) {
        return;
    }
    let CameraMode::Follow(entity) = *mode else {
        return;
    };
    let Ok((kind, Some(mut sel))) = query.get_mut(entity) else {
        return;
    };
    let Some(dir) = kind.model_dir() else { return };
    let Some(available) = model_lib.0.get(dir) else {
        return;
    };
    if available.is_empty() {
        return;
    }
    let n = available.len();
    let cur = available.iter().position(|p| p == &sel.0).unwrap_or(0);
    let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    sel.0 = available[if shift {
        (cur + n - 1) % n
    } else {
        (cur + 1) % n
    }]
    .clone();
}
