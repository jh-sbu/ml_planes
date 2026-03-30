//! Headless simulation runner — streams FlightState + ControlInputs as CSV to stdout.
//!
//! Build and run (requires no window; --no-default-features drops the renderer):
//!
//!   cargo run --example observe_state --no-default-features -- [OPTIONS]
//!
//! Options:
//!   --plane PATH       Path to a .plane.ron config file (default: built-in generic_jet)
//!   --steps N          Total simulation steps at 60 Hz (default: 600 = 10 s)
//!   --controller NAME  level_hold (default) | manual (zero inputs)
//!   --interval N       Print every Nth step (default: 10)
//!   --altitude F       Target altitude [m] and spawn altitude (default: 500)
//!   --airspeed F       Target airspeed [m/s] (default: 100)
//!   --alt-kp F         Altitude outer loop Kp (default: 0.01)
//!   --alt-ki F         Altitude outer loop Ki (default: 0.12)
//!   --alt-kd F         Altitude outer loop Kd (default: 0.04)
//!   --alpha-kp F       Alpha inner loop Kp (default: 1.0)
//!   --alpha-kd F       Alpha inner loop Kd (default: 0.5)
//!   --spd-kp F         Airspeed loop Kp (default: 0.01)
//!   --spd-ki F         Airspeed loop Ki (default: 0.06)

use std::f32::consts::FRAC_PI_2;
use std::time::Duration;

use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use bevy_rapier3d::prelude::*;

use ml_planes::{
    controllers::{ActiveController, FlightController, LevelHoldController, ManualController},
    plane::{ControlInputs, FlightState, PlaneConfig, PlaneConfigHandle, PlanePlugin},
};

fn main() {
    let args = parse_args();

    println!("step,time_s,altitude_m,airspeed_ms,alpha_deg,beta_deg,pitch_rate,roll_rate,yaw_rate,elevator,throttle,aileron,rudder");

    let mut app = App::new();
    app.add_plugins(MinimalPlugins)
        .add_plugins(bevy::transform::TransformPlugin)
        .add_plugins(bevy::asset::AssetPlugin::default())
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(PlanePlugin);

    app.insert_resource(TimeUpdateStrategy::ManualDuration(
        Duration::from_secs_f32(1.0 / 60.0),
    ));

    // finish() initialises plugin resources; required before manual app.update() driving.
    app.finish();

    // Insert PlaneConfig synchronously so aero forces are available from step 0.
    // (Mirrors the pattern in tests/common/mod.rs — bypasses async asset loading.)
    let cfg = match &args.plane {
        Some(path) => load_plane_config(path),
        None       => generic_jet_config(),
    };
    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneConfig>>()
        .add(cfg.clone());

    let controller: Box<dyn FlightController> = match args.controller.as_str() {
        "level_hold" => {
            let mut ctrl = LevelHoldController::new(args.altitude, args.airspeed);
            if let Some(v) = args.alt_kp   { ctrl.altitude_pid.kp = v; }
            if let Some(v) = args.alt_ki   { ctrl.altitude_pid.ki = v; }
            if let Some(v) = args.alt_kd   { ctrl.altitude_pid.kd = v; }
            if let Some(v) = args.alpha_kp { ctrl.alpha_pid.kp    = v; }
            if let Some(v) = args.alpha_kd { ctrl.alpha_pid.kd    = v; }
            if let Some(v) = args.spd_kp   { ctrl.airspeed_pid.kp = v; }
            if let Some(v) = args.spd_ki   { ctrl.airspeed_pid.ki = v; }
            Box::new(ctrl)
        }
        _ => Box::new(ManualController::new()),
    };

    // Spawn at target altitude, target airspeed forward, level attitude.
    // Body +Z (cockpit up) → world +Y (up): Quat::from_rotation_x(-π/2).
    let attitude = Quat::from_rotation_x(-FRAC_PI_2);
    app.world_mut().spawn((
        RigidBody::Dynamic,
        Collider::cuboid(1.0, 0.5, 3.0),
        Velocity {
            linvel: Vec3::new(args.airspeed, 0.0, 0.0),
            angvel: Vec3::ZERO,
        },
        ExternalForce::default(),
        AdditionalMassProperties::MassProperties(MassProperties {
            local_center_of_mass: Vec3::ZERO,
            mass: cfg.mass,
            principal_inertia: cfg.inertia,
            principal_inertia_local_frame: Quat::IDENTITY,
        }),
        FlightState::default(),
        ControlInputs::default(),
        ActiveController(controller),
        PlaneConfigHandle(handle),
        Transform::from_translation(Vec3::new(0.0, args.altitude, 0.0)).with_rotation(attitude),
    ));

    for step in 0..args.steps {
        app.update();

        if step % args.interval == 0 {
            // Query via a temporary QueryState; copy f32/Vec3 values (all Copy) to escape
            // the world borrow before the next app.update() call.
            let mut q = app.world_mut().query::<(&FlightState, &ControlInputs)>();
            let world = app.world();
            if let Some((state, inputs)) = q.iter(world).next() {
                let altitude = state.altitude;
                let airspeed = state.airspeed;
                let alpha_deg = state.alpha.to_degrees();
                let beta_deg = state.beta.to_degrees();
                let av = state.angular_velocity; // body frame (p, q, r) = (x, y, z)
                let elevator = inputs.elevator;
                let throttle = inputs.throttle;
                let aileron = inputs.aileron;
                let rudder = inputs.rudder;

                println!(
                    "{},{:.3},{:.2},{:.2},{:.3},{:.3},{:.4},{:.4},{:.4},{:.3},{:.3},{:.3},{:.3}",
                    step,
                    step as f32 / 60.0,
                    altitude,
                    airspeed,
                    alpha_deg,
                    beta_deg,
                    av.y, // q = pitch rate
                    av.x, // p = roll rate
                    av.z, // r = yaw rate
                    elevator,
                    throttle,
                    aileron,
                    rudder,
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// CLI helpers
// ---------------------------------------------------------------------------

struct Args {
    plane: Option<String>,
    steps: usize,
    controller: String,
    interval: usize,
    altitude: f32,
    airspeed: f32,
    alt_kp: Option<f32>,
    alt_ki: Option<f32>,
    alt_kd: Option<f32>,
    alpha_kp: Option<f32>,
    alpha_kd: Option<f32>,
    spd_kp: Option<f32>,
    spd_ki: Option<f32>,
}

fn parse_args() -> Args {
    let args: Vec<String> = std::env::args().collect();
    Args {
        plane:      get_arg(&args, "--plane"),
        steps:      get_arg(&args, "--steps")    .and_then(|v| v.parse().ok()).unwrap_or(600),
        controller: get_arg(&args, "--controller").unwrap_or_else(|| "level_hold".to_string()),
        interval:   get_arg(&args, "--interval") .and_then(|v| v.parse().ok()).unwrap_or(10),
        altitude:   get_arg(&args, "--altitude") .and_then(|v| v.parse().ok()).unwrap_or(500.0),
        airspeed:   get_arg(&args, "--airspeed") .and_then(|v| v.parse().ok()).unwrap_or(100.0),
        alt_kp:     get_arg(&args, "--alt-kp")   .and_then(|v| v.parse().ok()),
        alt_ki:     get_arg(&args, "--alt-ki")   .and_then(|v| v.parse().ok()),
        alt_kd:     get_arg(&args, "--alt-kd")   .and_then(|v| v.parse().ok()),
        alpha_kp:   get_arg(&args, "--alpha-kp") .and_then(|v| v.parse().ok()),
        alpha_kd:   get_arg(&args, "--alpha-kd") .and_then(|v| v.parse().ok()),
        spd_kp:     get_arg(&args, "--spd-kp")   .and_then(|v| v.parse().ok()),
        spd_ki:     get_arg(&args, "--spd-ki")   .and_then(|v| v.parse().ok()),
    }
}

fn get_arg(args: &[String], flag: &str) -> Option<String> {
    args.windows(2)
        .find(|pair| pair[0] == flag)
        .map(|pair| pair[1].clone())
}

// ---------------------------------------------------------------------------
// Config loading
// ---------------------------------------------------------------------------

fn load_plane_config(path: &str) -> PlaneConfig {
    let bytes = std::fs::read(path)
        .unwrap_or_else(|e| panic!("Cannot read plane config '{path}': {e}"));
    ron::de::from_bytes(&bytes)
        .unwrap_or_else(|e| panic!("Cannot parse plane config '{path}': {e}"))
}

/// Fallback config matching `assets/planes/generic_jet.plane.ron` exactly.
fn generic_jet_config() -> PlaneConfig {
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
        cm_alpha: 0.6,
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
