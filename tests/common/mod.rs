use std::time::Duration;

use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use bevy_rapier3d::prelude::*;

use ml_planes::plane::{PlaneConfig, PlanePlugin};

/// Build a headless Bevy app with Rapier physics and PlanePlugin.
///
/// Each `app.update()` advances virtual time by exactly 1/60 s,
/// making FixedUpdate steps deterministic regardless of wall-clock speed.
///
/// `app.finish()` is called before returning so that plugins which initialize
/// resources in their `finish()` impl are properly set up (required when
/// driving the app manually via `app.update()` instead of `app.run()`).
pub fn build_headless_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins)
        .add_plugins(bevy::transform::TransformPlugin)
        .add_plugins(bevy::asset::AssetPlugin::default())
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(PlanePlugin);

    app.insert_resource(TimeUpdateStrategy::ManualDuration(
        Duration::from_secs_f32(1.0 / 60.0),
    ));

    // Finalize plugin setup — equivalent to what app.run() does internally.
    // Required when driving the app manually via app.update().
    app.finish();

    app
}

/// Returns a PlaneConfig matching assets/planes/generic_jet.plane.ron exactly.
/// Used to insert into Assets<PlaneConfig> synchronously, bypassing async file loading.
pub fn generic_jet_config() -> PlaneConfig {
    PlaneConfig {
        wing_area: 20.0,
        mean_chord: 2.0,
        wing_span: 10.0,
        mass: 5000.0,
        inertia: Vec3::new(10000.0, 40000.0, 45000.0),
        cl0: 0.1,
        cl_alpha: 4.5,
        cl_delta_e: 0.4,
        cl_max: 1.4,
        cd0: 0.02,
        cd_induced: 0.05,
        cm0: -0.02,
        cm_alpha: -0.6,
        cm_q: -8.0,
        cm_delta_e: -1.2,
        cl_beta: -0.08,
        cl_p: -0.45,
        cl_r: 0.12,
        cl_delta_a: 0.18,
        cn_beta: 0.10,
        cn_r: -0.12,
        cn_delta_r: -0.10,
        thrust_max: 60000.0,
        aileron_limit: 0.4363,
        elevator_limit: 0.3491,
        rudder_limit: 0.2618,
    }
}
