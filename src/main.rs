use bevy::prelude::*;
// Rapier is only used by the local-sim / WASM build; the pure networked client
// (`feature = "client"`) runs no physics and renders replicated state instead.
#[cfg(any(not(feature = "net"), feature = "server"))]
use bevy_rapier3d::prelude::*;

// The binary wires the core plugins; the controller-rebuild systems live in
// `SimControlPlugin` (shared with the headless server). The client has no
// `ActiveController` to rebuild, so `SimControlPlugin` is compiled out there.
#[cfg(any(not(feature = "net"), feature = "server"))]
use ml_planes::controllers::SimControlPlugin;
use ml_planes::environment::{EnvironmentPlugin, LifecyclePlugin};
use ml_planes::plane::PlanePlugin;

#[cfg(feature = "visual")]
use ml_planes::controllers::{ControllerKind, PlaneTuning, SelectedTuningProfile};
// `ActiveController` is polled directly only in the local-sim/WASM build; the
// networked client has none (it sends `ManualInputCommand` instead).
#[cfg(all(feature = "visual", not(feature = "net")))]
use ml_planes::controllers::ActiveController;
#[cfg(feature = "visual")]
use ml_planes::plane::{FlightState, PlaneTuningHandle};

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
use ml_planes::controllers::{ModelLibrary, SelectedModel};

// Phase 6: on the networked client the input/hotkey systems no longer mutate local
// components (the server is authoritative); they send client→server commands.
#[cfg(all(feature = "visual", feature = "net"))]
use bevy_replicon::prelude::ClientTriggerExt;
#[cfg(all(feature = "visual", feature = "net"))]
use ml_planes::controllers::ManualController;
#[cfg(all(
    feature = "visual",
    feature = "net",
    feature = "inference",
    not(target_arch = "wasm32")
))]
use ml_planes::net::SetModelCommand;
#[cfg(all(feature = "visual", feature = "net"))]
use ml_planes::net::{ManualInputCommand, SetTuningProfileCommand, SwitchControllerCommand};
#[cfg(all(feature = "visual", feature = "net"))]
use ml_planes::plane::PlaneId;

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

    // Local physics + the controller-rebuild systems run in every build except the
    // pure networked client, which renders replicated state (see
    // `plans/client_server.md` Phase 4). `PlanePlugin`/`EnvironmentPlugin`/
    // `LifecyclePlugin` are still added on the client for the asset loaders, the
    // plane gizmos, and the (Phase 6) lifecycle observers.
    #[cfg(any(not(feature = "net"), feature = "server"))]
    {
        app.insert_resource(TimestepMode::Fixed {
            dt: 1.0 / 64.0,
            substeps: 1,
        });
        app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule());
    }
    app.add_plugins(PlanePlugin);
    app.add_plugins(EnvironmentPlugin);
    app.add_plugins(LifecyclePlugin);
    // The controller-rebuild systems (apply_initial_tuning / apply_controller_switch /
    // apply_flight_plan + the RL load arms) live in `SimControlPlugin` so the headless
    // server runs them too. The client has no `ActiveController` to rebuild.
    #[cfg(any(not(feature = "net"), feature = "server"))]
    app.add_plugins(SimControlPlugin);

    #[cfg(feature = "visual")]
    {
        app.add_plugins(EguiPlugin::default());
        app.add_plugins(CameraPlugin);
        app.add_plugins(UiPlugin);
    }

    // Client networking: the shared replication protocol + the renet client
    // transport + replicated-state rendering. The transport is opened by the menu's
    // Start New Server / Connect to Server flow (`ui::menu`), which inserts a
    // `ConnectTarget` and enters `AppState::Connecting`.
    #[cfg(feature = "client")]
    {
        use bevy_replicon::prelude::RepliconPlugins;
        use bevy_replicon_renet::RepliconRenetPlugins;
        use ml_planes::net::{ClientNetPlugin, NetProtocolPlugin};

        app.add_plugins(RepliconPlugins)
            .add_plugins(RepliconRenetPlugins)
            .add_plugins(NetProtocolPlugin)
            .add_plugins(ClientNetPlugin);
    }

    // Client-side input/hotkey systems only run while a scenario is flying, so the menu
    // screens don't drive controller switching, input polling, etc. In the local-sim /
    // WASM build they mutate the components `SimControlPlugin` reacts to; in the networked
    // client (Phase 6) they send client→server commands and the server stays authoritative.
    #[cfg(all(feature = "visual", feature = "net"))]
    app.init_resource::<ManualFlightInput>();
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

// Compute the next index when cycling a list with `T` (forward) / `Shift+T`
// (backward); shared by the tuning-profile and RL-model cyclers.
#[cfg(feature = "visual")]
fn cycle_index(cur: usize, len: usize, reverse: bool) -> usize {
    if reverse {
        (cur + len - 1) % len
    } else {
        (cur + 1) % len
    }
}

/// Sorted tuning-profile names for a controller kind's family, or `None` if the
/// kind has no tuning pool.
#[cfg(feature = "visual")]
fn tuning_profile_names(kind: ControllerKind, pt: &PlaneTuning) -> Option<Vec<String>> {
    let mut names: Vec<String> = match kind {
        ControllerKind::LevelHold | ControllerKind::RlLevelHold => {
            pt.level_hold.keys().cloned().collect()
        }
        ControllerKind::Orbit | ControllerKind::RlOrbit => pt.orbit.keys().cloned().collect(),
        ControllerKind::HeadingHold => pt.heading_hold.keys().cloned().collect(),
        _ => return None,
    };
    names.sort();
    (!names.is_empty()).then_some(names)
}

// --- Local-sim / WASM build: mutate components directly (SimControlPlugin rebuilds) ---

#[cfg(all(feature = "visual", not(feature = "net")))]
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
#[cfg(all(feature = "visual", not(feature = "net")))]
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
#[cfg(all(feature = "visual", not(feature = "net")))]
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
    let Some(names) = tuning_profile_names(*kind, pt) else {
        return;
    };
    let cur = names.iter().position(|nm| *nm == profile.0).unwrap_or(0);
    let reverse = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    let next = cycle_index(cur, names.len(), reverse);
    profile.set_if_neq(SelectedTuningProfile(names[next].clone()));
}

/// T / Shift+T: cycle RL model forward/backward for the followed plane.
#[cfg(all(
    feature = "visual",
    feature = "inference",
    not(target_arch = "wasm32"),
    not(feature = "net")
))]
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
    let cur = available.iter().position(|p| p == &sel.0).unwrap_or(0);
    let reverse = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    sel.0 = available[cycle_index(cur, available.len(), reverse)].clone();
}

// --- Networked client: send client→server commands; server stays authoritative ---

/// Client-local manual-flight accumulator. The networked client has no
/// `ActiveController` to poll, so it runs the `ManualController` keyboard logic
/// locally for the followed plane and ships the resulting inputs each frame.
#[cfg(all(feature = "visual", feature = "net"))]
#[derive(Resource, Default)]
struct ManualFlightInput {
    controller: ManualController,
    target: Option<Entity>,
}

/// Poll the keyboard for the followed plane when it is `Manual`, and send the
/// accumulated inputs to the server (latest-wins).
#[cfg(all(feature = "visual", feature = "net"))]
fn poll_controller_inputs(
    mode: Res<CameraMode>,
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    query: Query<(&PlaneId, &ControllerKind), With<FlightState>>,
    mut state: ResMut<ManualFlightInput>,
    mut commands: Commands,
) {
    let CameraMode::Follow(entity) = *mode else {
        state.target = None;
        return;
    };
    let Ok((plane, kind)) = query.get(entity) else {
        state.target = None;
        return;
    };
    if *kind != ControllerKind::Manual {
        state.target = None;
        return;
    }
    // Reset the accumulator when the controlled plane changes so a new plane
    // starts from neutral surfaces.
    if state.target != Some(entity) {
        state.controller = ManualController::default();
        state.target = Some(entity);
    }
    state.controller.read_keyboard(&keys, time.delta_secs());
    commands.client_trigger(ManualInputCommand {
        plane: *plane,
        inputs: state.controller.inputs.clone(),
    });
}

/// C key: ask the server to cycle every plane's controller kind.
#[cfg(all(feature = "visual", feature = "net"))]
fn switch_controller(
    query: Query<(&PlaneId, &ControllerKind), With<FlightState>>,
    keys: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
) {
    if keys.just_pressed(KeyCode::KeyC) {
        for (plane, kind) in &query {
            commands.client_trigger(SwitchControllerCommand {
                plane: *plane,
                kind: kind.next(),
            });
        }
    }
}

/// T / Shift+T: ask the server to cycle the followed plane's tune profile.
#[cfg(all(feature = "visual", feature = "net"))]
fn cycle_tune_profile(
    mode: Res<CameraMode>,
    keys: Res<ButtonInput<KeyCode>>,
    query: Query<(
        &PlaneId,
        &ControllerKind,
        Option<&SelectedTuningProfile>,
        Option<&PlaneTuningHandle>,
    )>,
    tuning_assets: Res<Assets<PlaneTuning>>,
    mut commands: Commands,
) {
    if !keys.just_pressed(KeyCode::KeyT) {
        return;
    }
    let CameraMode::Follow(entity) = *mode else {
        return;
    };
    let Ok((plane, kind, profile, Some(handle))) = query.get(entity) else {
        return;
    };
    let Some(pt) = tuning_assets.get(&handle.0) else {
        return;
    };
    let Some(names) = tuning_profile_names(*kind, pt) else {
        return;
    };
    let current = profile.map(|p| p.0.as_str()).unwrap_or("");
    let cur = names.iter().position(|nm| nm == current).unwrap_or(0);
    let reverse = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    let next = cycle_index(cur, names.len(), reverse);
    commands.client_trigger(SetTuningProfileCommand {
        plane: *plane,
        profile: names[next].clone(),
    });
}

/// T / Shift+T: ask the server to cycle the followed plane's RL model.
#[cfg(all(
    feature = "visual",
    feature = "inference",
    not(target_arch = "wasm32"),
    feature = "net"
))]
fn cycle_rl_model(
    mode: Res<CameraMode>,
    keys: Res<ButtonInput<KeyCode>>,
    query: Query<(&PlaneId, &ControllerKind, Option<&SelectedModel>)>,
    model_lib: Res<ModelLibrary>,
    mut commands: Commands,
) {
    if !keys.just_pressed(KeyCode::KeyT) {
        return;
    }
    let CameraMode::Follow(entity) = *mode else {
        return;
    };
    let Ok((plane, kind, sel)) = query.get(entity) else {
        return;
    };
    let Some(dir) = kind.model_dir() else { return };
    let Some(available) = model_lib.0.get(dir) else {
        return;
    };
    if available.is_empty() {
        return;
    }
    let current = sel.map(|s| s.0.as_str()).unwrap_or("");
    let cur = available.iter().position(|p| p == current).unwrap_or(0);
    let reverse = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    commands.client_trigger(SetModelCommand {
        plane: *plane,
        model_stem: available[cycle_index(cur, available.len(), reverse)].clone(),
    });
}

#[cfg(all(test, feature = "visual"))]
mod tests {
    use super::cycle_index;

    #[test]
    fn cycle_index_wraps_both_directions() {
        // Forward wraps past the end back to 0.
        assert_eq!(cycle_index(0, 3, false), 1);
        assert_eq!(cycle_index(2, 3, false), 0);
        // Reverse wraps past 0 to the end.
        assert_eq!(cycle_index(0, 3, true), 2);
        assert_eq!(cycle_index(1, 3, true), 0);
        // Single-element list stays put.
        assert_eq!(cycle_index(0, 1, false), 0);
        assert_eq!(cycle_index(0, 1, true), 0);
    }
}
