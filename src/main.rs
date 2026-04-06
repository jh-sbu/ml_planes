use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::{
    ControllerKind, LevelHoldController, WingmanController,
    FormationOffset, LeaderRef, LeaderState,
};
use ml_planes::environment::{EnvironmentPlugin, spawn_plane};
use ml_planes::plane::{config::PlaneConfig, FlightState, PlanePlugin};
use ml_planes::training::SpawnSpec;

#[cfg(feature = "visual")]
use ml_planes::controllers::{ActiveController, ControllerTuning, PlaneTuning, SelectedTuningProfile};
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
    app.add_plugins(MinimalPlugins);

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

    app.add_systems(Startup, setup);

    #[cfg(feature = "visual")]
    app.add_systems(Update, (poll_controller_inputs, switch_controller));

    #[cfg(feature = "visual")]
    app.add_systems(PostUpdate, apply_controller_switch);

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
    ));
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

/// Rebuild `ActiveController` whenever `ControllerKind` or `SelectedTuningProfile` changes.
/// Runs in `PostUpdate` so both `switch_controller` (Update) and `draw_flight_hud` (Update)
/// have already written their changes before this system fires.
#[cfg(feature = "visual")]
fn apply_controller_switch(
    mut query: Query<
        (
            &FlightState,
            &mut ActiveController,
            &ControllerKind,
            Option<&PlaneTuningHandle>,
            Option<&SelectedTuningProfile>,
        ),
        Or<(Changed<ControllerKind>, Changed<SelectedTuningProfile>)>,
    >,
    tuning_assets: Res<Assets<PlaneTuning>>,
) {
    for (state, mut controller, &kind, tuning_handle, profile) in query.iter_mut() {
        let profile_name = profile.map(|p| p.0.as_str()).unwrap_or("normal");
        let tuning: Option<&dyn ControllerTuning> = tuning_handle
            .and_then(|h| tuning_assets.get(&h.0))
            .and_then(|pt| pt.get_level_hold(profile_name))
            .map(|t| t as &dyn ControllerTuning);
        controller.0 = kind.build(state, tuning);
    }
}
