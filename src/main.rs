use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::{ActiveController, ManualController};
use ml_planes::environment::{EnvironmentPlugin, spawn_plane};
use ml_planes::plane::{config::PlaneConfig, FlightState, PlanePlugin};
use ml_planes::training::SpawnSpec;

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
    app.add_systems(Update, poll_controller_inputs);

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

    spawn_plane(
        &mut commands,
        &asset_server,
        &SpawnSpec::default(),
        Box::new(ManualController::new()),
        &cfg,
    );
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
