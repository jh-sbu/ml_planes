use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::{
    ControllerKind, FormationOffset, LevelHoldController, ModelLibrary, OrbitController,
    OrbitDirection, WaypointController, WingmanController,
};
#[cfg(any(feature = "inference", feature = "training"))]
#[allow(unused_imports)]
use ml_planes::controllers::{
    RlLevelHoldController, RlLstmOrbitConfig, RlLstmOrbitController, RlOrbitConfig,
    RlOrbitController, RlOrbitResidualConfig, RlOrbitResidualController, SelectedModel,
};
use ml_planes::environment::{spawn_plane, EnvironmentPlugin};
use ml_planes::plane::{
    config::PlaneConfig, ControlInputs, FlightState, NextPlaneId, PlaneIndex, PlanePlugin,
};
use ml_planes::training::SpawnSpec;

#[cfg(all(
    feature = "visual",
    any(feature = "inference", feature = "training"),
    not(target_arch = "wasm32")
))]
use ml_planes::controllers::OrbitTuning;
#[cfg(feature = "visual")]
use ml_planes::controllers::{
    ActiveController, ControllerTuning, OrbitParams, PlaneTuning, SelectedTuningProfile,
    TuningApplied,
};
#[cfg(feature = "visual")]
use ml_planes::plane::PlaneTuningHandle;

#[cfg(feature = "visual")]
use bevy::asset::{AssetMetaCheck, AssetPlugin};
#[cfg(feature = "visual")]
use bevy_egui::EguiPlugin;
#[cfg(feature = "visual")]
use ml_planes::camera::{CameraMode, CameraPlugin};
#[cfg(feature = "visual")]
use ml_planes::ui::UiPlugin;

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

    #[cfg(feature = "visual")]
    {
        app.add_plugins(EguiPlugin::default());
        app.add_plugins(CameraPlugin);
        app.add_plugins(UiPlugin);
    }

    app.init_resource::<ModelLibrary>();
    app.add_systems(Startup, setup);
    #[cfg(all(feature = "training", not(target_arch = "wasm32")))]
    app.add_systems(Startup, scan_models);

    #[cfg(feature = "visual")]
    app.add_systems(
        Update,
        (
            poll_controller_inputs,
            switch_controller,
            cycle_tune_profile,
            set_waypoint_from_click,
        ),
    );
    #[cfg(all(
        feature = "visual",
        any(feature = "inference", feature = "training"),
        not(target_arch = "wasm32")
    ))]
    app.add_systems(Update, cycle_rl_model);

    #[cfg(feature = "visual")]
    app.add_systems(
        PostUpdate,
        (
            apply_initial_tuning,
            apply_controller_switch.after(apply_initial_tuning),
        ),
    );
    #[cfg(all(
        feature = "visual",
        any(feature = "inference", feature = "training"),
        not(target_arch = "wasm32")
    ))]
    app.add_systems(
        PostUpdate,
        (
            apply_rl_controller_switch.after(apply_controller_switch),
            apply_model_switch,
        ),
    );

    app.run();
}

fn setup(mut commands: Commands, asset_server: Res<AssetServer>, mut ids: ResMut<NextPlaneId>) {
    // Mass/inertia values match assets/planes/generic_jet.plane.ron.
    // Only used for AdditionalMassProperties at spawn time.
    // Aerodynamic coefficients are ignored here; the async-loaded handle drives aero.
    let cfg = PlaneConfig {
        wing_area: 20.0,
        mean_chord: 2.0,
        wing_span: 10.0,
        mass: 5000.0,
        inertia: Vec3::new(10000.0, 40000.0, 45000.0),
        cl0: 0.0,
        cl_alpha: 0.0,
        cl_delta_e: 0.0,
        cl_max: 1.4,
        cd0: 0.0,
        cd_induced: 0.0,
        cm0: 0.0,
        cm_alpha: 0.0,
        cm_q: 0.0,
        cm_delta_e: 0.0,
        cl_beta: 0.0,
        cl_p: 0.0,
        cl_r: 0.0,
        cl_delta_a: 0.0,
        cn_beta: 0.0,
        cn_r: 0.0,
        cn_delta_r: 0.0,
        thrust_max: 0.0,
        aileron_limit: 0.4363,
        elevator_limit: 0.3491,
        rudder_limit: 0.2618,
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
        &mut ids,
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
    commands.entity(leader.entity).insert(PlaneIndex(1));

    #[cfg(feature = "visual")]
    {
        let tuning_handle: Handle<PlaneTuning> = asset_server.load("planes/generic_jet.tuning.ron");
        commands.entity(leader.entity).insert((
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
        &mut ids,
        &asset_server,
        &SpawnSpec {
            position: Some(wingman_pos),
            velocity: Some(leader_vel),
            ..Default::default()
        },
        Box::new(WingmanController::new(
            leader.id,
            &leader_initial,
            &own_initial,
            offset.clone(),
        )),
        ControllerKind::Wingman,
        &cfg,
    );

    commands
        .entity(wingman.entity)
        .insert((offset, PlaneIndex(2)));

    // --- PID orbit plane: 3000 m radius around origin at 1200 m / 100 m/s ---
    let orbit_radius = 3000.0;
    let orbit_speed = 100.0;
    let orbit_direction = OrbitDirection::CounterClockwise;

    let pid_orbit_pos = Vec3::new(0.0, 1200.0, orbit_direction.sign() * orbit_radius);
    let pid_orbit_vel = Vec3::new(orbit_speed, 0.0, 0.0);
    let pid_orbit_attitude = level_attitude_for_heading(pid_orbit_vel.x, pid_orbit_vel.z);
    let mut pid_orbit_state = FlightState {
        position: pid_orbit_pos,
        velocity: pid_orbit_vel,
        attitude: pid_orbit_attitude,
        angular_velocity: Vec3::ZERO,
        ..Default::default()
    };
    pid_orbit_state.update_air_data();

    let mut pid_orbit_controller =
        OrbitController::from_state(&pid_orbit_state, &ControlInputs::default());
    configure_origin_orbit(
        &mut pid_orbit_controller,
        orbit_radius,
        pid_orbit_state.altitude,
        orbit_speed,
        orbit_direction,
    );

    let pid_orbit = spawn_plane(
        &mut commands,
        &mut ids,
        &asset_server,
        &SpawnSpec {
            position: Some(pid_orbit_pos),
            velocity: Some(pid_orbit_vel),
            attitude: Some(pid_orbit_attitude),
            ..Default::default()
        },
        Box::new(pid_orbit_controller),
        ControllerKind::Orbit,
        &cfg,
    );
    commands.entity(pid_orbit.entity).insert(PlaneIndex(3));

    #[cfg(feature = "visual")]
    {
        let tuning_handle: Handle<PlaneTuning> = asset_server.load("planes/generic_jet.tuning.ron");
        commands.entity(pid_orbit.entity).insert((
            PlaneTuningHandle(tuning_handle),
            SelectedTuningProfile("normal".to_string()),
        ));
    }

    // --- Optional RL orbit plane: 3000 m radius around origin at 800 m / 100 m/s ---
    // Native: probe filesystem at runtime.
    #[cfg(all(
        any(feature = "inference", feature = "training"),
        not(target_arch = "wasm32")
    ))]
    {
        let model_path = "models/orbit/ppo_orbit_1";
        if std::path::Path::new(&format!("{model_path}.mpk")).exists() {
            let rl_orbit_pos = Vec3::new(0.0, 800.0, -orbit_direction.sign() * orbit_radius);
            let rl_orbit_vel = Vec3::new(-orbit_speed, 0.0, 0.0);
            let rl_orbit_attitude = level_attitude_for_heading(rl_orbit_vel.x, rl_orbit_vel.z);
            let config = RlOrbitConfig {
                center_x: 0.0,
                center_z: 0.0,
                target_radius: orbit_radius,
                target_altitude: rl_orbit_pos.y,
                target_airspeed: orbit_speed,
                direction: orbit_direction,
            };

            match RlOrbitController::load(model_path, config) {
                Ok(rl_ctrl) => {
                    let rl_orbit = spawn_plane(
                        &mut commands,
                        &mut ids,
                        &asset_server,
                        &SpawnSpec {
                            position: Some(rl_orbit_pos),
                            velocity: Some(rl_orbit_vel),
                            attitude: Some(rl_orbit_attitude),
                            ..Default::default()
                        },
                        Box::new(rl_ctrl),
                        ControllerKind::RlOrbit,
                        &cfg,
                    );
                    commands
                        .entity(rl_orbit.entity)
                        .insert((PlaneIndex(4), SelectedModel(model_path.to_string())));
                }
                Err(e) => eprintln!("Failed to load RL orbit model from {model_path}.mpk: {e}"),
            }
        }
    }
    // WASM: load from embedded bytes.
    #[cfg(all(feature = "inference", target_arch = "wasm32"))]
    {
        const ORBIT_MPK: &[u8] = include_bytes!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/models/orbit/ppo_orbit_1.mpk"
        ));
        let rl_orbit_pos = Vec3::new(0.0, 800.0, -orbit_direction.sign() * orbit_radius);
        let rl_orbit_vel = Vec3::new(-orbit_speed, 0.0, 0.0);
        let rl_orbit_attitude = level_attitude_for_heading(rl_orbit_vel.x, rl_orbit_vel.z);
        let config = RlOrbitConfig {
            center_x: 0.0,
            center_z: 0.0,
            target_radius: orbit_radius,
            target_altitude: rl_orbit_pos.y,
            target_airspeed: orbit_speed,
            direction: orbit_direction,
        };
        match RlOrbitController::load_bytes(ORBIT_MPK, config) {
            Ok(rl_ctrl) => {
                let rl_orbit = spawn_plane(
                    &mut commands,
                    &mut ids,
                    &asset_server,
                    &SpawnSpec {
                        position: Some(rl_orbit_pos),
                        velocity: Some(rl_orbit_vel),
                        attitude: Some(rl_orbit_attitude),
                        ..Default::default()
                    },
                    Box::new(rl_ctrl),
                    ControllerKind::RlOrbit,
                    &cfg,
                );
                commands.entity(rl_orbit.entity).insert((
                    PlaneIndex(4),
                    SelectedModel("models/orbit/ppo_orbit_1".to_string()),
                ));
            }
            Err(e) => eprintln!("Failed to load embedded RL orbit model: {e}"),
        }
    }

    // --- Optional RL comparison plane ---
    // Native: probe filesystem at runtime.
    #[cfg(all(
        any(feature = "inference", feature = "training"),
        not(target_arch = "wasm32")
    ))]
    {
        let model_path = "models/level_hold/ppo_level_hold";
        if std::path::Path::new(&format!("{model_path}.mpk")).exists() {
            match RlLevelHoldController::load(model_path, 1000.0, 100.0) {
                Ok(rl_ctrl) => {
                    // Spawn 30 m behind the leader so both are visible in follow-cam.
                    let rl_pos = Vec3::new(0.0, 1000.0, -30.0);
                    let rl_plane = spawn_plane(
                        &mut commands,
                        &mut ids,
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
                    commands
                        .entity(rl_plane.entity)
                        .insert((PlaneIndex(5), SelectedModel(model_path.to_string())));
                }
                Err(e) => eprintln!("Failed to load RL model from {model_path}.mpk: {e}"),
            }
        }
    }
    // WASM: load from embedded bytes.
    #[cfg(all(feature = "inference", target_arch = "wasm32"))]
    {
        const LEVEL_HOLD_MPK: &[u8] = include_bytes!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/models/level_hold/ppo_level_hold.mpk"
        ));
        match RlLevelHoldController::load_bytes(LEVEL_HOLD_MPK, 1000.0, 100.0) {
            Ok(rl_ctrl) => {
                let rl_pos = Vec3::new(0.0, 1000.0, -30.0);
                let rl_plane = spawn_plane(
                    &mut commands,
                    &mut ids,
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
                commands.entity(rl_plane.entity).insert((
                    PlaneIndex(5),
                    SelectedModel("models/level_hold/ppo_level_hold".to_string()),
                ));
            }
            Err(e) => eprintln!("Failed to load embedded RL level-hold model: {e}"),
        }
    }

    // --- Waypoint plane: starts 2 km from origin, heading toward it ---
    let waypoint_pos = Vec3::new(2000.0, 1200.0, 2000.0);
    let waypoint_vel = Vec3::new(-70.7, 0.0, -70.7); // ~100 m/s toward origin
    let waypoint_attitude = level_attitude_for_heading(waypoint_vel.x, waypoint_vel.z);
    let mut waypoint_initial_state = FlightState {
        position: waypoint_pos,
        velocity: waypoint_vel,
        attitude: waypoint_attitude,
        angular_velocity: Vec3::ZERO,
        ..Default::default()
    };
    waypoint_initial_state.update_air_data();

    let waypoint_controller = WaypointController::from_state(
        &waypoint_initial_state,
        0.0,    // target_x
        0.0,    // target_z
        1200.0, // target_altitude
        100.0,  // target_airspeed
        &ControlInputs::default(),
    );
    let waypoint_plane = spawn_plane(
        &mut commands,
        &mut ids,
        &asset_server,
        &SpawnSpec {
            position: Some(waypoint_pos),
            velocity: Some(waypoint_vel),
            attitude: Some(waypoint_attitude),
            ..Default::default()
        },
        Box::new(waypoint_controller),
        ControllerKind::Waypoint,
        &cfg,
    );
    commands.entity(waypoint_plane.entity).insert(PlaneIndex(6));
}

fn configure_origin_orbit(
    orbit: &mut OrbitController,
    radius: f32,
    altitude: f32,
    airspeed: f32,
    direction: OrbitDirection,
) {
    orbit.center_x = 0.0;
    orbit.center_z = 0.0;
    orbit.target_radius = radius;
    orbit.target_altitude = altitude;
    orbit.target_airspeed = airspeed;
    orbit.direction = direction;
    orbit.inner.target_altitude = altitude;
    orbit.inner.target_airspeed = airspeed;
    orbit.inner.target_roll = steady_orbit_bank(airspeed, radius, direction);
    orbit.radial_pid.reset();
    orbit.heading_pid.reset();
}

fn steady_orbit_bank(airspeed: f32, radius: f32, direction: OrbitDirection) -> f32 {
    const G: f32 = 9.81;
    -direction.sign() * (airspeed.powi(2) / (G * radius.max(1.0))).atan()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steady_orbit_bank_matches_pid_feedforward_chirality() {
        let ccw = steady_orbit_bank(100.0, 1000.0, OrbitDirection::CounterClockwise);
        let cw = steady_orbit_bank(100.0, 1000.0, OrbitDirection::Clockwise);

        assert!(ccw < 0.0, "CCW orbit should seed negative bank, got {ccw}");
        assert!(cw > 0.0, "CW orbit should seed positive bank, got {cw}");
        assert!(
            (ccw + cw).abs() < 1e-6,
            "CW/CCW seed banks should be symmetric: cw={cw}, ccw={ccw}"
        );
    }
}

fn level_attitude_for_heading(head_x: f32, head_z: f32) -> Quat {
    let forward = Vec3::new(head_x, 0.0, head_z).normalize_or_zero();
    let forward = if forward.length_squared() > 0.0 {
        forward
    } else {
        Vec3::X
    };
    let up = Vec3::Y;
    let right = up.cross(forward).normalize_or_zero();
    let right = if right.length_squared() > 0.0 {
        right
    } else {
        Vec3::NEG_Z
    };
    Quat::from_mat3(&Mat3::from_cols(forward, right, up)).normalize()
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
        ControllerKind::Waypoint => pt.waypoint.keys().map(|s| s.as_str()).collect(),
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
#[cfg(all(
    feature = "visual",
    any(feature = "inference", feature = "training"),
    not(target_arch = "wasm32")
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

/// Scan `models/<category>/` subdirectories at startup and populate `ModelLibrary`.
#[cfg(all(feature = "training", not(target_arch = "wasm32")))]
fn scan_models(mut commands: Commands) {
    let mut lib: std::collections::HashMap<String, Vec<String>> = Default::default();
    if let Ok(categories) = std::fs::read_dir("models/") {
        for cat in categories.flatten() {
            if !cat.file_type().map(|t| t.is_dir()).unwrap_or(false) {
                continue;
            }
            let cat_name = match cat.file_name().into_string() {
                Ok(s) => s,
                Err(_) => continue,
            };
            if let Ok(files) = std::fs::read_dir(cat.path()) {
                let mut stems: Vec<String> = files
                    .flatten()
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
#[cfg(all(
    feature = "visual",
    any(feature = "inference", feature = "training"),
    not(target_arch = "wasm32")
))]
fn apply_model_switch(
    mut query: Query<
        (
            &FlightState,
            &mut ActiveController,
            &ControllerKind,
            &SelectedModel,
            Option<&PlaneTuningHandle>,
            Option<&SelectedTuningProfile>,
        ),
        Changed<SelectedModel>,
    >,
    tuning_assets: Res<Assets<PlaneTuning>>,
) {
    for (state, mut ctrl, kind, sel, tuning_handle, profile) in query.iter_mut() {
        let Some(dir) = kind.model_dir() else {
            continue;
        };
        if !model_path_matches_dir(&sel.0, dir) {
            eprintln!("Ignoring model '{}' for controller {}", sel.0, kind.name());
            continue;
        }

        match *kind {
            ControllerKind::RlLevelHold => {
                let (target_alt, target_spd) = level_hold_targets_from_controller(&mut ctrl, state);
                match RlLevelHoldController::load(&sel.0, target_alt, target_spd) {
                    Ok(new_ctrl) => ctrl.0 = Box::new(new_ctrl),
                    Err(e) => eprintln!("Failed to load model {}: {e}", sel.0),
                }
            }
            ControllerKind::RlOrbit => {
                let config = orbit_config_from_controller(&mut ctrl, state);
                match RlOrbitController::load(&sel.0, config) {
                    Ok(new_ctrl) => ctrl.0 = Box::new(new_ctrl),
                    Err(e) => eprintln!("Failed to load model {}: {e}", sel.0),
                }
            }
            ControllerKind::RlOrbitResidual => {
                let config = residual_config_from_controller(&mut ctrl, state);
                let profile_name = profile.map(|p| p.0.as_str()).unwrap_or("normal");
                let orbit_tuning: Option<&OrbitTuning> = tuning_handle
                    .and_then(|h| tuning_assets.get(&h.0))
                    .and_then(|pt| pt.get_orbit(profile_name));
                match RlOrbitResidualController::load(&sel.0, config, state, orbit_tuning) {
                    Ok(new_ctrl) => ctrl.0 = Box::new(new_ctrl),
                    Err(e) => eprintln!("Failed to load model {}: {e}", sel.0),
                }
            }
            ControllerKind::RlLstmOrbit => {
                let config = lstm_orbit_config_from_controller(&mut ctrl, state);
                match RlLstmOrbitController::load(&sel.0, config) {
                    Ok(new_ctrl) => ctrl.0 = Box::new(new_ctrl),
                    Err(e) => eprintln!("Failed to load LSTM orbit model {}: {e}", sel.0),
                }
            }
            _ => {}
        }
    }
}

/// When `ControllerKind` changes to an RL kind, load the actual RL model,
/// overriding the PID fallback that `apply_controller_switch` produces.
/// If the entity lacks `SelectedModel`, inserts a default so `apply_model_switch`
/// loads the controller on the next frame.
#[cfg(all(
    feature = "visual",
    any(feature = "inference", feature = "training"),
    not(target_arch = "wasm32")
))]
fn apply_rl_controller_switch(
    mut commands: Commands,
    mut query: Query<
        (
            Entity,
            &FlightState,
            &mut ActiveController,
            Mut<ControllerKind>,
            Option<&SelectedModel>,
            Option<&PlaneTuningHandle>,
            Option<&SelectedTuningProfile>,
        ),
        Changed<ControllerKind>,
    >,
    model_lib: Res<ModelLibrary>,
    tuning_assets: Res<Assets<PlaneTuning>>,
) {
    for (entity, state, mut controller, mut kind, sel, tuning_handle, profile) in query.iter_mut() {
        if kind.is_added()
            || !matches!(
                *kind,
                ControllerKind::RlLevelHold
                    | ControllerKind::RlOrbit
                    | ControllerKind::RlOrbitResidual
                    | ControllerKind::RlLstmOrbit
            )
        {
            continue;
        }

        let Some(path) = selected_or_default_model_path(*kind, sel, &model_lib) else {
            if matches!(
                *kind,
                ControllerKind::RlOrbit
                    | ControllerKind::RlOrbitResidual
                    | ControllerKind::RlLstmOrbit
            ) {
                kind.set_if_neq(ControllerKind::Orbit);
            }
            continue;
        };

        if sel.map(|s| s.0.as_str()) != Some(path.as_str()) {
            commands.entity(entity).insert(SelectedModel(path.clone()));
        }

        match *kind {
            ControllerKind::RlLevelHold => {
                let (tgt_alt, tgt_spd) = level_hold_targets_from_controller(&mut controller, state);
                match RlLevelHoldController::load(&path, tgt_alt, tgt_spd) {
                    Ok(rl) => controller.0 = Box::new(rl),
                    Err(e) => eprintln!("Failed to load RL model '{}': {e}", path),
                }
            }
            ControllerKind::RlOrbit => {
                let config = orbit_config_from_controller(&mut controller, state);
                match RlOrbitController::load(&path, config) {
                    Ok(rl) => controller.0 = Box::new(rl),
                    Err(e) => {
                        eprintln!("Failed to load RL orbit model '{}': {e}", path);
                        kind.set_if_neq(ControllerKind::Orbit);
                    }
                }
            }
            ControllerKind::RlOrbitResidual => {
                let config = residual_config_from_controller(&mut controller, state);
                let profile_name = profile.map(|p| p.0.as_str()).unwrap_or("normal");
                let orbit_tuning: Option<&OrbitTuning> = tuning_handle
                    .and_then(|h| tuning_assets.get(&h.0))
                    .and_then(|pt| pt.get_orbit(profile_name));
                match RlOrbitResidualController::load(&path, config, state, orbit_tuning) {
                    Ok(rl) => controller.0 = Box::new(rl),
                    Err(e) => {
                        eprintln!("Failed to load RL orbit residual model '{}': {e}", path);
                        kind.set_if_neq(ControllerKind::Orbit);
                    }
                }
            }
            ControllerKind::RlLstmOrbit => {
                let config = lstm_orbit_config_from_controller(&mut controller, state);
                match RlLstmOrbitController::load(&path, config) {
                    Ok(rl) => controller.0 = Box::new(rl),
                    Err(e) => {
                        eprintln!("Failed to load RL LSTM orbit model '{}': {e}", path);
                        kind.set_if_neq(ControllerKind::Orbit);
                    }
                }
            }
            _ => {}
        }
    }
}

#[cfg(all(
    feature = "visual",
    any(feature = "inference", feature = "training"),
    not(target_arch = "wasm32")
))]
fn selected_or_default_model_path(
    kind: ControllerKind,
    selected: Option<&SelectedModel>,
    model_lib: &ModelLibrary,
) -> Option<String> {
    let dir = kind.model_dir()?;
    if let Some(sel) = selected {
        if model_path_matches_dir(&sel.0, dir) {
            return Some(sel.0.clone());
        }
    }
    model_lib
        .0
        .get(dir)
        .and_then(|paths| paths.first().cloned())
}

#[cfg(all(
    feature = "visual",
    any(feature = "inference", feature = "training"),
    not(target_arch = "wasm32")
))]
fn model_path_matches_dir(path: &str, dir: &str) -> bool {
    path.starts_with(&format!("models/{dir}/"))
}

#[cfg(all(
    feature = "visual",
    any(feature = "inference", feature = "training"),
    not(target_arch = "wasm32")
))]
fn level_hold_targets_from_controller(
    controller: &mut ActiveController,
    state: &FlightState,
) -> (f32, f32) {
    if let Some(rl) = controller
        .0
        .as_any_mut()
        .downcast_mut::<RlLevelHoldController>()
    {
        return (rl.target_altitude, rl.target_airspeed);
    }
    if let Some(lh) = controller
        .0
        .as_any_mut()
        .downcast_mut::<LevelHoldController>()
    {
        return (lh.target_altitude, lh.target_airspeed);
    }
    (state.altitude, state.airspeed)
}

#[cfg(all(
    feature = "visual",
    any(feature = "inference", feature = "training"),
    not(target_arch = "wasm32")
))]
fn orbit_config_from_controller(
    controller: &mut ActiveController,
    state: &FlightState,
) -> RlOrbitConfig {
    if let Some(rl) = controller
        .0
        .as_any_mut()
        .downcast_mut::<RlOrbitController>()
    {
        return rl.config();
    }
    if let Some(orbit) = controller
        .0
        .as_any_mut()
        .downcast_mut::<ml_planes::controllers::OrbitController>()
    {
        return RlOrbitConfig::from_orbit(orbit);
    }
    RlOrbitConfig::from_state(state)
}

#[cfg(all(
    feature = "visual",
    any(feature = "inference", feature = "training"),
    not(target_arch = "wasm32")
))]
fn residual_config_from_controller(
    controller: &mut ActiveController,
    state: &FlightState,
) -> RlOrbitResidualConfig {
    if let Some(rl) = controller
        .0
        .as_any_mut()
        .downcast_mut::<RlOrbitResidualController>()
    {
        return rl.config();
    }
    if let Some(orbit) = controller
        .0
        .as_any_mut()
        .downcast_mut::<ml_planes::controllers::OrbitController>()
    {
        return RlOrbitResidualConfig::from_orbit(orbit);
    }
    RlOrbitResidualConfig::from_state(state)
}

#[cfg(all(
    feature = "visual",
    any(feature = "inference", feature = "training"),
    not(target_arch = "wasm32")
))]
fn lstm_orbit_config_from_controller(
    controller: &mut ActiveController,
    state: &FlightState,
) -> RlLstmOrbitConfig {
    if let Some(rl) = controller
        .0
        .as_any_mut()
        .downcast_mut::<RlLstmOrbitController>()
    {
        return rl.config();
    }
    if let Some(orbit) = controller
        .0
        .as_any_mut()
        .downcast_mut::<ml_planes::controllers::OrbitController>()
    {
        return RlLstmOrbitConfig::from_orbit(orbit);
    }
    RlLstmOrbitConfig::from_state(state)
}

/// Extract orbit geometry from whatever controller variant is currently active.
///
/// Returns `None` when the active controller is not an orbit variant (e.g. LevelHold,
/// Manual), so the caller can fall back to `from_state()` geometry as before.
#[cfg(feature = "visual")]
fn extract_orbit_params(ctrl: &mut ActiveController) -> Option<OrbitParams> {
    if let Some(orbit) = ctrl.0.as_any_mut().downcast_mut::<OrbitController>() {
        return Some(OrbitParams {
            center_x: orbit.center_x,
            center_z: orbit.center_z,
            target_radius: orbit.target_radius,
            target_altitude: orbit.target_altitude,
            target_airspeed: orbit.target_airspeed,
            direction: orbit.direction,
        });
    }
    #[cfg(any(feature = "inference", feature = "training"))]
    {
        if let Some(rl) = ctrl.0.as_any_mut().downcast_mut::<RlOrbitController>() {
            let cfg = rl.config();
            return Some(OrbitParams {
                center_x: cfg.center_x,
                center_z: cfg.center_z,
                target_radius: cfg.target_radius,
                target_altitude: cfg.target_altitude,
                target_airspeed: cfg.target_airspeed,
                direction: cfg.direction,
            });
        }
        if let Some(rl) = ctrl
            .0
            .as_any_mut()
            .downcast_mut::<RlOrbitResidualController>()
        {
            let cfg = rl.config();
            return Some(OrbitParams {
                center_x: cfg.center_x,
                center_z: cfg.center_z,
                target_radius: cfg.target_radius,
                target_altitude: cfg.target_altitude,
                target_airspeed: cfg.target_airspeed,
                direction: cfg.direction,
            });
        }
        if let Some(rl) = ctrl.0.as_any_mut().downcast_mut::<RlLstmOrbitController>() {
            let cfg = rl.config();
            return Some(OrbitParams {
                center_x: cfg.center_x,
                center_z: cfg.center_z,
                target_radius: cfg.target_radius,
                target_altitude: cfg.target_altitude,
                target_airspeed: cfg.target_airspeed,
                direction: cfg.direction,
            });
        }
    }
    None
}

/// Apply the named tuning profile to a controller that was spawned before the PlaneTuning asset
/// finished loading. Runs once per entity (guarded by `Without<TuningApplied>`) and fires before
/// `apply_controller_switch` so any explicit profile switch in the same frame takes precedence.
#[cfg(feature = "visual")]
fn apply_initial_tuning(
    mut query: Query<
        (
            Entity,
            &FlightState,
            &mut ActiveController,
            &ControllerKind,
            &ControlInputs,
            &PlaneTuningHandle,
            &SelectedTuningProfile,
        ),
        Without<TuningApplied>,
    >,
    tuning_assets: Res<Assets<PlaneTuning>>,
    mut commands: Commands,
) {
    for (entity, state, mut controller, kind, prev_inputs, tuning_handle, profile) in
        query.iter_mut()
    {
        let Some(pt) = tuning_assets.get(&tuning_handle.0) else {
            continue;
        };
        let profile_name = profile.0.as_str();
        let tuning: Option<&dyn ControllerTuning> = match *kind {
            ControllerKind::Orbit
            | ControllerKind::RlOrbit
            | ControllerKind::RlOrbitResidual
            | ControllerKind::RlLstmOrbit => pt
                .get_orbit(profile_name)
                .map(|t| t as &dyn ControllerTuning),
            ControllerKind::Waypoint => pt
                .get_waypoint(profile_name)
                .map(|t| t as &dyn ControllerTuning),
            _ => pt
                .get_level_hold(profile_name)
                .map(|t| t as &dyn ControllerTuning),
        };

        // Preserve orbit geometry: extract before rebuild, restore after.
        let orbit_params = if matches!(
            *kind,
            ControllerKind::Orbit
                | ControllerKind::RlOrbit
                | ControllerKind::RlOrbitResidual
                | ControllerKind::RlLstmOrbit
        ) {
            extract_orbit_params(&mut controller)
        } else {
            None
        };

        controller.0 = kind.build(state, tuning, prev_inputs);

        if let Some(params) = orbit_params {
            if let Some(orbit) = controller.0.as_any_mut().downcast_mut::<OrbitController>() {
                orbit.apply_params(&params, state.airspeed);
            }
        }

        commands.entity(entity).insert(TuningApplied);
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
        let profile_mutated = profile
            .as_ref()
            .map(|p| p.is_changed() && !p.is_added())
            .unwrap_or(false);
        if !kind_mutated && !profile_mutated {
            continue;
        }
        let profile_name = profile.as_deref().map(|p| p.0.as_str()).unwrap_or("normal");
        let tuning: Option<&dyn ControllerTuning> = tuning_handle
            .and_then(|h| tuning_assets.get(&h.0))
            .and_then(|pt| match *kind {
                ControllerKind::Orbit
                | ControllerKind::RlOrbit
                | ControllerKind::RlOrbitResidual
                | ControllerKind::RlLstmOrbit => pt
                    .get_orbit(profile_name)
                    .map(|t| t as &dyn ControllerTuning),
                ControllerKind::HeadingHold => pt
                    .get_heading_hold(profile_name)
                    .map(|t| t as &dyn ControllerTuning),
                ControllerKind::Waypoint => pt
                    .get_waypoint(profile_name)
                    .map(|t| t as &dyn ControllerTuning),
                _ => pt
                    .get_level_hold(profile_name)
                    .map(|t| t as &dyn ControllerTuning),
            });

        // Preserve orbit geometry when switching between orbit variants.
        // None when the current controller is not an orbit type (e.g. LevelHold → Orbit
        // stays with the from_state() auto-center default).
        let orbit_params = if matches!(
            *kind,
            ControllerKind::Orbit
                | ControllerKind::RlOrbit
                | ControllerKind::RlOrbitResidual
                | ControllerKind::RlLstmOrbit
        ) {
            extract_orbit_params(&mut controller)
        } else {
            None
        };

        controller.0 = kind.build(state, tuning, prev_inputs);

        // Apply preserved geometry to the freshly-built OrbitController (always the PID
        // fallback produced by build() for orbit kinds). No-op when orbit_params is None.
        if let Some(params) = orbit_params {
            if let Some(orbit) = controller.0.as_any_mut().downcast_mut::<OrbitController>() {
                orbit.apply_params(&params, state.airspeed);
            }
        }
    }
}

/// Right-click on the ground sets the waypoint target X/Z for the followed plane.
///
/// Casts a ray from the camera through the cursor and intersects with Y = 0.
/// Resets guidance PIDs so the plane steers to the new target from scratch.
#[cfg(feature = "visual")]
fn set_waypoint_from_click(
    mode: Res<CameraMode>,
    mouse: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<bevy::window::PrimaryWindow>>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    mut plane_query: Query<(&ControllerKind, &mut ActiveController), With<FlightState>>,
) {
    if !mouse.just_pressed(MouseButton::Right) {
        return;
    }
    let CameraMode::Follow(entity) = *mode else {
        return;
    };
    let Ok((kind, mut controller)) = plane_query.get_mut(entity) else {
        return;
    };
    if *kind != ControllerKind::Waypoint {
        return;
    }
    let Ok(window) = windows.single() else {
        return;
    };
    let Some(cursor_pos) = window.cursor_position() else {
        return;
    };
    // Use the first camera in the scene (there is only one 3-D camera).
    let Some((camera, cam_transform)) = cameras.iter().next() else {
        return;
    };
    let Ok(ray) = camera.viewport_to_world(cam_transform, cursor_pos) else {
        return;
    };
    let dir = Vec3::from(*ray.direction);
    if dir.y.abs() < 1e-6 {
        return; // ray parallel to ground, skip
    }
    let t = -ray.origin.y / dir.y;
    if t <= 0.0 {
        return; // looking away from ground
    }
    let hit = ray.origin + t * dir;
    if let Some(wpt) = controller
        .0
        .as_any_mut()
        .downcast_mut::<WaypointController>()
    {
        wpt.target_x = hit.x;
        wpt.target_z = hit.z;
        wpt.reset_guidance();
    }
}
