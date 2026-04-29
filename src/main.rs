use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::{
    ControllerKind, LevelHoldController, ModelLibrary, WingmanController,
    FormationOffset, LeaderRef, LeaderState,
};
#[cfg(feature = "training")]
use ml_planes::controllers::{RlLevelHoldController, SelectedModel};
use ml_planes::environment::{EnvironmentPlugin, spawn_plane};
use ml_planes::plane::{config::PlaneConfig, FlightState, PlaneIndex, PlanePlugin};
use ml_planes::training::SpawnSpec;

#[cfg(feature = "visual")]
use ml_planes::controllers::{ActiveController, ControllerTuning, PlaneTuning, SelectedTuningProfile};
#[cfg(feature = "visual")]
use ml_planes::plane::ControlInputs;
#[cfg(feature = "visual")]
use ml_planes::plane::PlaneTuningHandle;

#[cfg(feature = "visual")]
use bevy_egui::EguiPlugin;
#[cfg(feature = "visual")]
use ml_planes::camera::CameraPlugin;
#[cfg(feature = "visual")]
use ml_planes::ui::UiPlugin;

fn main() {
    let mut app = App::new();

    #[cfg(feature = "visual")]
    app.add_plugins(DefaultPlugins);
    #[cfg(not(feature = "visual"))]
    app.add_plugins((
        MinimalPlugins,
        bevy::asset::AssetPlugin::default(),
        bevy::transform::TransformPlugin,
    ));

    app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default());
    app.add_plugins(PlanePlugin);
    app.add_plugins(EnvironmentPlugin);

    #[cfg(feature = "visual")]
    {
        app.add_plugins(EguiPlugin::default());
        app.add_plugins(RapierDebugRenderPlugin::default());
        app.add_plugins(CameraPlugin);
        app.add_plugins(UiPlugin);
    }

    app.init_resource::<ModelLibrary>();
    app.add_systems(Startup, setup);
    #[cfg(feature = "training")]
    app.add_systems(Startup, scan_models);

    #[cfg(feature = "visual")]
    app.add_systems(Update, (poll_controller_inputs, switch_controller));

    #[cfg(feature = "visual")]
    app.add_systems(PostUpdate, apply_controller_switch);
    #[cfg(all(feature = "visual", feature = "training"))]
    app.add_systems(PostUpdate, apply_model_switch);

    app.run();
}

fn setup(mut commands: Commands, asset_server: Res<AssetServer>) {
    // Mass/inertia values match assets/planes/generic_jet.plane.ron.
    // Only used for AdditionalMassProperties at spawn time.
    // Aerodynamic coefficients are ignored here; the async-loaded handle drives aero.
    let cfg = PlaneConfig {
        wing_area: 20.0, mean_chord: 2.0, wing_span: 10.0,
        mass: 5000.0, inertia: Vec3::new(10000.0, 40000.0, 45000.0),
        cl0: 0.0, cl_alpha: 0.0, cl_delta_e: 0.0, cl_max: 1.4,
        cd0: 0.0, cd_induced: 0.0,
        cm0: 0.0, cm_alpha: 0.0, cm_q: 0.0, cm_delta_e: 0.0,
        cl_beta: 0.0, cl_p: 0.0, cl_r: 0.0, cl_delta_a: 0.0,
        cn_beta: 0.0, cn_r: 0.0, cn_delta_r: 0.0,
        thrust_max: 0.0,
        aileron_limit: 0.4363, elevator_limit: 0.3491, rudder_limit: 0.2618,
    };

    // --- Leader plane: LevelHold at 1000 m / 100 m/s ---
    let leader_pos = Vec3::new(0.0, 1000.0, 0.0);
    let leader_vel = Vec3::new(100.0, 0.0, 0.0);
    let leader_attitude = Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2);

    let leader_initial = FlightState {
        position: leader_pos,
        velocity: leader_vel,
        attitude: leader_attitude,
        airspeed: 100.0,
        altitude: 1000.0,
        ..Default::default()
    };

    let leader = spawn_plane(
        &mut commands,
        &asset_server,
        &SpawnSpec {
            position: Some(leader_pos),
            velocity: Some(leader_vel),
            ..Default::default()
        },
        Box::new(LevelHoldController::new(1000.0, 100.0)),
        ControllerKind::LevelHold,
        &cfg,
    );
    commands.entity(leader).insert(PlaneIndex(1));

    #[cfg(feature = "visual")]
    {
        let tuning_handle: Handle<PlaneTuning> =
            asset_server.load("planes/generic_jet.tuning.ron");
        commands.entity(leader).insert((
            PlaneTuningHandle(tuning_handle),
            SelectedTuningProfile("normal".to_string()),
        ));
    }

    // --- Wingman plane: trails 20 m behind, 15 m to the right ---
    let offset = FormationOffset::default(); // (-20, 15, 0) in leader body frame
    // With rotation_x(-π/2): body +Y (right) maps to world +Z, body +X (fwd) maps to world +X.
    let wingman_pos = leader_pos + leader_attitude * offset.offset_body;
    let own_initial = FlightState {
        position: wingman_pos,
        velocity: leader_vel,
        attitude: leader_attitude,
        airspeed: 100.0,
        altitude: wingman_pos.y,
        ..Default::default()
    };

    let wingman = spawn_plane(
        &mut commands,
        &asset_server,
        &SpawnSpec {
            position: Some(wingman_pos),
            velocity: Some(leader_vel),
            ..Default::default()
        },
        Box::new(WingmanController::new(&leader_initial, &own_initial, offset.clone())),
        ControllerKind::Wingman,
        &cfg,
    );

    commands.entity(wingman).insert((
        LeaderRef(leader),
        LeaderState::default(),
        offset,
        PlaneIndex(2),
    ));

    // --- Optional RL comparison plane (requires `training` feature + trained weights) ---
    #[cfg(feature = "training")]
    {
        let model_path = "models/level_hold/ppo_level_hold";
        if std::path::Path::new(&format!("{model_path}.mpk")).exists() {
            match RlLevelHoldController::load(model_path, 1000.0, 100.0) {
                Ok(rl_ctrl) => {
                    // Spawn 30 m behind the leader so both are visible in follow-cam.
                    let rl_pos = Vec3::new(0.0, 1000.0, -30.0);
                    let rl_plane = spawn_plane(
                        &mut commands,
                        &asset_server,
                        &SpawnSpec {
                            position: Some(rl_pos),
                            velocity: Some(leader_vel),
                            ..Default::default()
                        },
                        Box::new(rl_ctrl),
                        ControllerKind::RlLevelHold,
                        &cfg,
                    );
                    commands.entity(rl_plane).insert((
                        PlaneIndex(3),
                        SelectedModel(model_path.to_string()),
                    ));
                }
                Err(e) => eprintln!("Failed to load RL model from {model_path}.mpk: {e}"),
            }
        }
    }
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

/// Scan `models/<category>/` subdirectories at startup and populate `ModelLibrary`.
#[cfg(feature = "training")]
fn scan_models(mut commands: Commands) {
    let mut lib: std::collections::HashMap<String, Vec<String>> = Default::default();
    if let Ok(categories) = std::fs::read_dir("models/") {
        for cat in categories.flatten() {
            if !cat.file_type().map(|t| t.is_dir()).unwrap_or(false) { continue; }
            let cat_name = match cat.file_name().into_string() {
                Ok(s) => s,
                Err(_) => continue,
            };
            if let Ok(files) = std::fs::read_dir(cat.path()) {
                let mut stems: Vec<String> = files.flatten()
                    .filter_map(|f| {
                        let name = f.file_name().into_string().ok()?;
                        let stem = name.strip_suffix(".mpk")?;
                        Some(format!("models/{cat_name}/{stem}"))
                    })
                    .collect();
                stems.sort();
                if !stems.is_empty() {
                    lib.insert(cat_name, stems);
                }
            }
        }
    }
    commands.insert_resource(ModelLibrary(lib));
}

/// Reload `ActiveController` whenever `SelectedModel` changes (HUD model dropdown).
#[cfg(all(feature = "visual", feature = "training"))]
fn apply_model_switch(
    mut query: Query<
        (&mut ActiveController, &SelectedModel),
        Changed<SelectedModel>,
    >,
) {
    for (mut ctrl, sel) in query.iter_mut() {
        let (target_alt, target_spd) = ctrl.0
            .as_any_mut()
            .downcast_mut::<RlLevelHoldController>()
            .map(|r| (r.target_altitude, r.target_airspeed))
            .unwrap_or((1000.0, 100.0));
        match RlLevelHoldController::load(&sel.0, target_alt, target_spd) {
            Ok(new_ctrl) => ctrl.0 = Box::new(new_ctrl),
            Err(e) => eprintln!("Failed to load model {}: {e}", sel.0),
        }
    }
}

/// Rebuild `ActiveController` whenever `ControllerKind` or `SelectedTuningProfile` changes.
/// Runs in `PostUpdate` so both `switch_controller` (Update) and `draw_flight_hud` (Update)
/// have already written their changes before this system fires.
#[cfg(feature = "visual")]
fn apply_controller_switch(
    mut query: Query<
        (
            &FlightState,
            &mut ActiveController,
            Ref<ControllerKind>,
            &ControlInputs,
            Option<&PlaneTuningHandle>,
            Option<Ref<SelectedTuningProfile>>,
        ),
        Or<(Changed<ControllerKind>, Changed<SelectedTuningProfile>)>,
    >,
    tuning_assets: Res<Assets<PlaneTuning>>,
) {
    for (state, mut controller, kind, prev_inputs, tuning_handle, profile) in query.iter_mut() {
        // Skip entities that matched only because components were just inserted at spawn —
        // their controllers were already constructed correctly before spawning.
        let kind_mutated = kind.is_changed() && !kind.is_added();
        let profile_mutated = profile.as_ref().map(|p| p.is_changed() && !p.is_added()).unwrap_or(false);
        if !kind_mutated && !profile_mutated {
            continue;
        }
        let profile_name = profile.as_deref().map(|p| p.0.as_str()).unwrap_or("normal");
        let tuning: Option<&dyn ControllerTuning> = tuning_handle
            .and_then(|h| tuning_assets.get(&h.0))
            .and_then(|pt| pt.get_level_hold(profile_name))
            .map(|t| t as &dyn ControllerTuning);
        controller.0 = kind.build(state, tuning, prev_inputs);
    }
}
