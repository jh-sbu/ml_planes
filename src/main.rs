use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::{
    ControllerKind, FormationOffset, LevelHoldController, ModelLibrary, OrbitController,
    OrbitDirection, WingmanController,
};
#[cfg(feature = "inference")]
#[allow(unused_imports)]
use ml_planes::controllers::{
    ModelLoadError, RlLevelHoldController, RlLstmOrbitConfig, RlLstmOrbitController, RlOrbitConfig,
    RlOrbitController, RlOrbitResidualConfig, RlOrbitResidualController, SelectedModel,
};
use ml_planes::environment::{
    generic_jet_spawn_config, spawn_plane, EnvironmentPlugin, LifecyclePlugin,
};
use ml_planes::plane::{ControlInputs, FlightPlanHandle, FlightState, NextPlaneId, PlanePlugin};
use ml_planes::training::SpawnSpec;

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
use ml_planes::controllers::OrbitTuning;
#[cfg(feature = "visual")]
use ml_planes::controllers::{
    ActiveController, ControllerTuning, FlightPlan, L1Controller, OrbitParams, PlaneTuning,
    SelectedTuningProfile, TuningApplied,
};
#[cfg(feature = "visual")]
use ml_planes::plane::PlaneTuningHandle;
#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
use ml_planes::ui::notifications::Notifications;

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
    app.add_plugins(LifecyclePlugin);

    #[cfg(feature = "visual")]
    {
        app.add_plugins(EguiPlugin::default());
        app.add_plugins(CameraPlugin);
        app.add_plugins(UiPlugin);
    }

    app.init_resource::<ModelLibrary>();
    app.add_systems(Startup, setup);
    #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
    app.add_systems(Startup, scan_models);

    #[cfg(feature = "visual")]
    app.add_systems(
        Update,
        (
            poll_controller_inputs,
            switch_controller,
            cycle_tune_profile,
        ),
    );
    #[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
    app.add_systems(Update, cycle_rl_model);

    #[cfg(feature = "visual")]
    app.add_systems(
        PostUpdate,
        (
            apply_initial_tuning,
            apply_controller_switch.after(apply_initial_tuning),
            apply_flight_plan.after(apply_controller_switch),
        ),
    );
    #[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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
    // Mass/inertia for the Rapier body at spawn time; the async-loaded
    // `.plane.ron` handle drives aerodynamics once ready.
    let cfg = generic_jet_spawn_config();

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
        "planes/generic_jet.plane.ron",
        &SpawnSpec {
            position: Some(leader_pos),
            velocity: Some(leader_vel),
            ..Default::default()
        },
        Box::new(LevelHoldController::new(1000.0, 100.0)),
        ControllerKind::LevelHold,
        &cfg,
    );
    // `spawn_plane` attaches the per-config `PlaneTuningHandle` + a "normal"
    // `SelectedTuningProfile` automatically, so the visual `apply_initial_tuning`
    // system applies generic_jet's gains without any manual wiring here.

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
        "planes/generic_jet.plane.ron",
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

    commands.entity(wingman.entity).insert(offset);

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

    // `spawn_plane` indexes the plane and attaches generic_jet's tuning handle +
    // "normal" profile, so the orbit gains are applied automatically.
    spawn_plane(
        &mut commands,
        &mut ids,
        &asset_server,
        "planes/generic_jet.plane.ron",
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

    // --- Optional RL orbit plane: 3000 m radius around origin at 800 m / 100 m/s ---
    // Native: probe filesystem at runtime.
    #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
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
                        "planes/generic_jet.plane.ron",
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
                        .insert(SelectedModel(model_path.to_string()));
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
                    "planes/generic_jet.plane.ron",
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
                    .insert(SelectedModel("models/orbit/ppo_orbit_1".to_string()));
            }
            Err(e) => eprintln!("Failed to load embedded RL orbit model: {e}"),
        }
    }

    // --- Optional RL comparison plane ---
    // Native: probe filesystem at runtime.
    #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
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
                        "planes/generic_jet.plane.ron",
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
                        .insert(SelectedModel(model_path.to_string()));
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
                    "planes/generic_jet.plane.ron",
                    &SpawnSpec {
                        position: Some(rl_pos),
                        velocity: Some(leader_vel),
                        ..Default::default()
                    },
                    Box::new(rl_ctrl),
                    ControllerKind::RlLevelHold,
                    &cfg,
                );
                commands.entity(rl_plane.entity).insert(SelectedModel(
                    "models/level_hold/ppo_level_hold".to_string(),
                ));
            }
            Err(e) => eprintln!("Failed to load embedded RL level-hold model: {e}"),
        }
    }

    // --- Flight-plan (L1) plane: follows assets/plans/patrol.plan.ron ---
    // Spawns 2 km out heading toward the origin. The controller starts as a PID
    // orbit fallback and is swapped for an `L1Controller` by `apply_flight_plan`
    // once the `.plan.ron` asset finishes loading.
    let fp_pos = Vec3::new(2000.0, 1000.0, 2000.0);
    let fp_vel = Vec3::new(-70.7, 0.0, -70.7); // ~100 m/s toward origin
    let fp_attitude = level_attitude_for_heading(fp_vel.x, fp_vel.z);
    let mut fp_state = FlightState {
        position: fp_pos,
        velocity: fp_vel,
        attitude: fp_attitude,
        angular_velocity: Vec3::ZERO,
        ..Default::default()
    };
    fp_state.update_air_data();

    let fp_fallback = OrbitController::from_state(&fp_state, &ControlInputs::default());
    let fp_plane = spawn_plane(
        &mut commands,
        &mut ids,
        &asset_server,
        "planes/generic_jet.plane.ron",
        &SpawnSpec {
            position: Some(fp_pos),
            velocity: Some(fp_vel),
            attitude: Some(fp_attitude),
            ..Default::default()
        },
        Box::new(fp_fallback),
        ControllerKind::FlightPlan,
        &cfg,
    );
    commands
        .entity(fp_plane.entity)
        .insert(FlightPlanHandle(asset_server.load("plans/patrol.plan.ron")));
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

    #[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
    #[test]
    fn rl_kind_load_gate() {
        use ControllerKind::*;
        // Runtime (panel/hotkey) spawn: added, no model wired yet → load.
        assert!(rl_kind_needs_load_on_change(RlOrbit, true, false));
        // Startup spawn: added but already carries a SelectedModel → skip.
        assert!(!rl_kind_needs_load_on_change(RlOrbit, true, true));
        // Interactive cycle to an RL kind: not added → load.
        assert!(rl_kind_needs_load_on_change(RlOrbit, false, false));
        // Every RL kind is covered.
        for k in [RlLevelHold, RlOrbit, RlOrbitResidual, RlLstmOrbit] {
            assert!(rl_kind_needs_load_on_change(k, true, false));
        }
        // Non-RL kinds are never handled here.
        assert!(!rl_kind_needs_load_on_change(Orbit, true, false));
        assert!(!rl_kind_needs_load_on_change(LevelHold, false, false));
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

/// Scan `models/<category>/` subdirectories at startup and populate `ModelLibrary`.
#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
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
#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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
    mut notes: ResMut<Notifications>,
) {
    for (state, mut ctrl, kind, sel, tuning_handle, profile) in query.iter_mut() {
        let Some(dir) = kind.model_dir() else {
            continue;
        };
        if !model_path_matches_dir(&sel.0, dir) {
            warn!("Ignoring model '{}' for controller {}", sel.0, kind.name());
            continue;
        }

        match *kind {
            ControllerKind::RlLevelHold => {
                let (target_alt, target_spd) = level_hold_targets_from_controller(&mut ctrl, state);
                match RlLevelHoldController::load(&sel.0, target_alt, target_spd) {
                    Ok(new_ctrl) => ctrl.0 = Box::new(new_ctrl),
                    Err(e) => report_skipped_model(&mut notes, &sel.0, &e),
                }
            }
            ControllerKind::RlOrbit => {
                let config = orbit_config_from_controller(&mut ctrl, state);
                match RlOrbitController::load(&sel.0, config) {
                    Ok(new_ctrl) => ctrl.0 = Box::new(new_ctrl),
                    Err(e) => report_skipped_model(&mut notes, &sel.0, &e),
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
                    Err(e) => report_skipped_model(&mut notes, &sel.0, &e),
                }
            }
            ControllerKind::RlLstmOrbit => {
                let config = lstm_orbit_config_from_controller(&mut ctrl, state);
                match RlLstmOrbitController::load(&sel.0, config) {
                    Ok(new_ctrl) => ctrl.0 = Box::new(new_ctrl),
                    Err(e) => report_skipped_model(&mut notes, &sel.0, &e),
                }
            }
            _ => {}
        }
    }
}

/// Log a model-load failure and surface a transient HUD banner. Used when an
/// incompatible (e.g. stale-dimension) checkpoint is skipped and the controller
/// keeps its current (PID fallback) behavior.
#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
fn report_skipped_model(notes: &mut Notifications, path: &str, err: &ModelLoadError) {
    warn!("Skipping model '{path}': {err}");
    let name = std::path::Path::new(path)
        .file_name()
        .and_then(|s| s.to_str())
        .unwrap_or(path);
    notes.push(format!("Model '{name}' skipped — {err}"));
}

/// Whether `apply_rl_controller_switch` should (re)load a model for a plane
/// whose `ControllerKind` was just observed as changed.
///
/// Only RL kinds are handled here (non-RL kinds keep the PID path). For RL kinds
/// we load on any genuine kind change, and on the spawn frame only when no
/// `SelectedModel` is wired yet: runtime (panel/hotkey) spawns arrive on the PID
/// fallback with no model and need loading, whereas startup-spawned RL planes
/// already carry both the loaded controller and a `SelectedModel`, so they're
/// skipped to avoid a redundant reload.
#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
fn rl_kind_needs_load_on_change(
    kind: ControllerKind,
    kind_added: bool,
    has_selected_model: bool,
) -> bool {
    let is_rl = matches!(
        kind,
        ControllerKind::RlLevelHold
            | ControllerKind::RlOrbit
            | ControllerKind::RlOrbitResidual
            | ControllerKind::RlLstmOrbit
    );
    is_rl && (!kind_added || !has_selected_model)
}

/// When `ControllerKind` changes to an RL kind, load the actual RL model,
/// overriding the PID fallback that `apply_controller_switch` produces.
/// If the entity lacks `SelectedModel`, inserts a default so `apply_model_switch`
/// loads the controller on the next frame.
#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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
    mut notes: ResMut<Notifications>,
) {
    for (entity, state, mut controller, mut kind, sel, tuning_handle, profile) in query.iter_mut() {
        if !rl_kind_needs_load_on_change(*kind, kind.is_added(), sel.is_some()) {
            continue;
        }

        let Some(path) = selected_or_default_model_path(*kind, sel, &model_lib) else {
            // No checkpoint available: revert the label to the PID baseline the
            // controller is actually running, so a spawned RL plane doesn't show
            // an "RL" label while flying PID.
            match *kind {
                ControllerKind::RlOrbit
                | ControllerKind::RlOrbitResidual
                | ControllerKind::RlLstmOrbit => {
                    kind.set_if_neq(ControllerKind::Orbit);
                }
                ControllerKind::RlLevelHold => {
                    kind.set_if_neq(ControllerKind::LevelHold);
                }
                _ => {}
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
                    Err(e) => report_skipped_model(&mut notes, &path, &e),
                }
            }
            ControllerKind::RlOrbit => {
                let config = orbit_config_from_controller(&mut controller, state);
                match RlOrbitController::load(&path, config) {
                    Ok(rl) => controller.0 = Box::new(rl),
                    Err(e) => {
                        report_skipped_model(&mut notes, &path, &e);
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
                        report_skipped_model(&mut notes, &path, &e);
                        kind.set_if_neq(ControllerKind::Orbit);
                    }
                }
            }
            ControllerKind::RlLstmOrbit => {
                let config = lstm_orbit_config_from_controller(&mut controller, state);
                match RlLstmOrbitController::load(&path, config) {
                    Ok(rl) => controller.0 = Box::new(rl),
                    Err(e) => {
                        report_skipped_model(&mut notes, &path, &e);
                        kind.set_if_neq(ControllerKind::Orbit);
                    }
                }
            }
            _ => {}
        }
    }
}

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
fn model_path_matches_dir(path: &str, dir: &str) -> bool {
    path.starts_with(&format!("models/{dir}/"))
}

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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

#[cfg(all(feature = "visual", feature = "inference", not(target_arch = "wasm32")))]
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
    #[cfg(feature = "inference")]
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

/// Swap a `FlightPlan` plane's PID-orbit fallback for an `L1Controller` once its
/// `.plan.ron` asset finishes loading.
///
/// `ControllerKind::build()` cannot construct an `L1Controller` (it lacks the
/// plan asset), so it returns a PID orbit. This system runs after
/// `apply_controller_switch`, detects planes whose kind is `FlightPlan` but
/// whose active controller is not yet an `L1Controller`, and builds the real
/// controller from the loaded plan — preserving leg progress once installed by
/// leaving existing `L1Controller`s untouched.
#[cfg(feature = "visual")]
fn apply_flight_plan(
    mut query: Query<(
        &FlightState,
        &mut ActiveController,
        &ControllerKind,
        &ControlInputs,
        &FlightPlanHandle,
    )>,
    plans: Res<Assets<FlightPlan>>,
) {
    for (state, mut controller, kind, prev_inputs, handle) in query.iter_mut() {
        if *kind != ControllerKind::FlightPlan {
            continue;
        }
        // Already running the plan — keep it so leg progress is preserved.
        if controller
            .0
            .as_any_mut()
            .downcast_mut::<L1Controller>()
            .is_some()
        {
            continue;
        }
        let Some(plan) = plans.get(&handle.0) else {
            continue; // asset still loading; the PID orbit fallback keeps flying
        };
        controller.0 = Box::new(L1Controller::from_plan(state, plan.clone(), prev_inputs));
    }
}
