//! Headless simulation runner — streams FlightState + ControlInputs as CSV to stdout.
//!
//! Build and run (requires no window; --no-default-features drops the renderer):
//!
//!   cargo run --example observe_state --no-default-features -- [OPTIONS]
//!
//! Options:
//!   --plane PATH         Path to a .plane.ron config file (default: built-in generic_jet)
//!   --tuning-file PATH   Path to a .tuning.ron file; loads gains for --profile
//!   --profile NAME       Named tuning profile to use (default: "normal")
//!   --steps N            Total simulation steps at 60 Hz (default: 600 = 10 s)
//!   --controller NAME    level_hold (default) | orbit | manual (zero inputs)
//!   --interval N         Print every Nth step (default: 10)
//!   --altitude F         Target altitude [m] and spawn altitude (default: 500)
//!   --airspeed F         Target airspeed [m/s] (default: 100)
//!
//! Level-hold gain overrides:
//!   --alt-kp F           Altitude outer loop Kp override
//!   --alt-ki F           Altitude outer loop Ki override
//!   --alt-kd F           Altitude outer loop Kd override
//!   --pitch-kp F         Pitch inner loop Kp override
//!   --pitch-kd F         Pitch inner loop Kd override
//!   --spd-kp F           Airspeed loop Kp override
//!   --spd-ki F           Airspeed loop Ki override
//!
//! Orbit geometry (only used with --controller orbit):
//!   --center-x F         Orbit center X [m] (default: auto from spawn)
//!   --center-z F         Orbit center Z [m] (default: auto from spawn)
//!   --radius F           Orbit radius [m] (default: 1000)
//!   --direction NAME     ccw (default) | cw
//!
//! Orbit outer-loop gain overrides (only used with --controller orbit):
//!   --radial-kp F        Radial error [m] → heading offset Kp override
//!   --radial-kd F        Radial error [m] → heading offset Kd override
//!   --heading-kp F       Heading error [rad] → bank correction Kp override
//!   --heading-kd F       Heading error [rad] → bank correction Kd override
//!
//! Level-hold gain flags (--alt-kp, --pitch-kp, etc.) also override the
//! inner loop when --controller orbit is used.
//!
//! Individual gain flags override values loaded from --tuning-file.
//!
//! Output columns for orbit mode append: radial_error_m, heading_error_rad, bank_ff_rad

use std::f32::consts::FRAC_PI_2;
use std::time::Duration;

use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use bevy_rapier3d::prelude::*;

use ml_planes::{
    controllers::{
        orbit_observation_terms, ActiveController, FlightController, LevelHoldController,
        ManualController, OrbitController, OrbitDirection, OrbitTuning, PlaneTuning,
    },
    plane::{ControlInputs, FlightState, PlaneConfig, PlaneConfigHandle, PlanePlugin},
};

fn main() {
    let args = parse_args();

    let header = if args.controller == "orbit" {
        "step,time_s,altitude_m,airspeed_ms,alpha_deg,beta_deg,pitch_rate,roll_rate,yaw_rate,elevator,throttle,aileron,rudder,radial_error_m,heading_error_rad,bank_ff_rad"
    } else {
        "step,time_s,altitude_m,airspeed_ms,alpha_deg,beta_deg,pitch_rate,roll_rate,yaw_rate,elevator,throttle,aileron,rudder"
    };
    println!("{header}");

    let mut app = App::new();
    app.add_plugins(MinimalPlugins)
        .add_plugins(bevy::transform::TransformPlugin)
        .add_plugins(bevy::asset::AssetPlugin::default())
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(PlanePlugin);

    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
        1.0 / 60.0,
    )));

    // finish() initialises plugin resources; required before manual app.update() driving.
    app.finish();

    // Insert PlaneConfig synchronously so aero forces are available from step 0.
    // (Mirrors the pattern in tests/common/mod.rs — bypasses async asset loading.)
    let cfg = match &args.plane {
        Some(path) => load_plane_config(path),
        None => generic_jet_config(),
    };
    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneConfig>>()
        .add(cfg.clone());

    let controller: Box<dyn FlightController> = match args.controller.as_str() {
        "level_hold" => {
            // Start from file tuning (or defaults), then apply individual flag overrides.
            let mut tuning = args
                .tuning_file
                .as_deref()
                .map(load_plan_tuning)
                .and_then(|pt| pt.get_level_hold(&args.profile).cloned())
                .unwrap_or_default();
            if let Some(v) = args.alt_kp {
                tuning.alt_kp = v;
            }
            if let Some(v) = args.alt_ki {
                tuning.alt_ki = v;
            }
            if let Some(v) = args.alt_kd {
                tuning.alt_kd = v;
            }
            if let Some(v) = args.pitch_kp {
                tuning.pitch_kp = v;
            }
            if let Some(v) = args.pitch_kd {
                tuning.pitch_kd = v;
            }
            if let Some(v) = args.spd_kp {
                tuning.spd_kp = v;
            }
            if let Some(v) = args.spd_ki {
                tuning.spd_ki = v;
            }
            let state = FlightState {
                altitude: args.altitude,
                airspeed: args.airspeed,
                ..FlightState::default()
            };
            Box::new(LevelHoldController::with_tuning(
                &state,
                &tuning,
                &ControlInputs::default(),
            ))
        }
        "orbit" => {
            let mut tuning: OrbitTuning = args
                .tuning_file
                .as_deref()
                .map(load_plan_tuning)
                .and_then(|pt| pt.get_orbit(&args.profile).cloned())
                .unwrap_or_default();
            // Outer-loop overrides
            if let Some(v) = args.radial_kp  { tuning.radial_kp  = v; }
            if let Some(v) = args.radial_kd  { tuning.radial_kd  = v; }
            if let Some(v) = args.heading_kp { tuning.heading_kp = v; }
            if let Some(v) = args.heading_kd { tuning.heading_kd = v; }
            // Inner-loop overrides (reuse level-hold flag fields)
            if let Some(v) = args.alt_kp   { tuning.inner.alt_kp   = v; }
            if let Some(v) = args.alt_ki   { tuning.inner.alt_ki   = v; }
            if let Some(v) = args.alt_kd   { tuning.inner.alt_kd   = v; }
            if let Some(v) = args.pitch_kp { tuning.inner.pitch_kp = v; }
            if let Some(v) = args.pitch_kd { tuning.inner.pitch_kd = v; }
            if let Some(v) = args.spd_kp   { tuning.inner.spd_kp   = v; }
            if let Some(v) = args.spd_ki   { tuning.inner.spd_ki   = v; }
            let seed_state = FlightState {
                altitude: args.altitude,
                airspeed: args.airspeed,
                ..FlightState::default()
            };
            let mut ctrl =
                OrbitController::with_tuning(&seed_state, &tuning, &ControlInputs::default());
            ctrl.center_x = args.center_x;
            ctrl.center_z = args.center_z;
            ctrl.target_radius = args.radius;
            ctrl.direction = args.direction;
            ctrl.target_altitude = args.altitude;
            ctrl.target_airspeed = args.airspeed;
            ctrl.radial_pid.reset();
            ctrl.heading_pid.reset();
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

    let is_orbit = args.controller == "orbit";

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

                if is_orbit {
                    let terms = orbit_observation_terms(
                        state,
                        args.center_x,
                        args.center_z,
                        args.radius,
                        args.direction,
                    );
                    println!(
                        "{},{:.3},{:.2},{:.2},{:.3},{:.3},{:.4},{:.4},{:.4},{:.3},{:.3},{:.3},{:.3},{:.3},{:.4},{:.4}",
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
                        terms.radial_error,
                        terms.heading_error,
                        terms.bank_ff,
                    );
                } else {
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
}

// ---------------------------------------------------------------------------
// CLI helpers
// ---------------------------------------------------------------------------

struct Args {
    plane: Option<String>,
    tuning_file: Option<String>,
    profile: String,
    steps: usize,
    controller: String,
    interval: usize,
    altitude: f32,
    airspeed: f32,
    // level_hold gain overrides
    alt_kp: Option<f32>,
    alt_ki: Option<f32>,
    alt_kd: Option<f32>,
    pitch_kp: Option<f32>,
    pitch_kd: Option<f32>,
    spd_kp: Option<f32>,
    spd_ki: Option<f32>,
    // orbit geometry
    center_x: f32,
    center_z: f32,
    radius: f32,
    direction: OrbitDirection,
    // orbit outer-loop gain overrides
    radial_kp: Option<f32>,
    radial_kd: Option<f32>,
    heading_kp: Option<f32>,
    heading_kd: Option<f32>,
}

fn parse_args() -> Args {
    let args: Vec<String> = std::env::args().collect();
    Args {
        plane: get_arg(&args, "--plane"),
        tuning_file: get_arg(&args, "--tuning-file"),
        profile: get_arg(&args, "--profile").unwrap_or_else(|| "normal".to_string()),
        steps: get_arg(&args, "--steps")
            .and_then(|v| v.parse().ok())
            .unwrap_or(600),
        controller: get_arg(&args, "--controller").unwrap_or_else(|| "level_hold".to_string()),
        interval: get_arg(&args, "--interval")
            .and_then(|v| v.parse().ok())
            .unwrap_or(10),
        altitude: get_arg(&args, "--altitude")
            .and_then(|v| v.parse().ok())
            .unwrap_or(500.0),
        airspeed: get_arg(&args, "--airspeed")
            .and_then(|v| v.parse().ok())
            .unwrap_or(100.0),
        alt_kp: get_arg(&args, "--alt-kp").and_then(|v| v.parse().ok()),
        alt_ki: get_arg(&args, "--alt-ki").and_then(|v| v.parse().ok()),
        alt_kd: get_arg(&args, "--alt-kd").and_then(|v| v.parse().ok()),
        pitch_kp: get_arg(&args, "--pitch-kp").and_then(|v| v.parse().ok()),
        pitch_kd: get_arg(&args, "--pitch-kd").and_then(|v| v.parse().ok()),
        spd_kp: get_arg(&args, "--spd-kp").and_then(|v| v.parse().ok()),
        spd_ki: get_arg(&args, "--spd-ki").and_then(|v| v.parse().ok()),
        center_x: get_arg(&args, "--center-x")
            .and_then(|v| v.parse().ok())
            .unwrap_or(0.0),
        center_z: get_arg(&args, "--center-z")
            .and_then(|v| v.parse().ok())
            .unwrap_or(0.0),
        radius: get_arg(&args, "--radius")
            .and_then(|v| v.parse().ok())
            .unwrap_or(1000.0),
        direction: match get_arg(&args, "--direction").as_deref() {
            Some("cw") => OrbitDirection::Clockwise,
            _ => OrbitDirection::CounterClockwise,
        },
        radial_kp:  get_arg(&args, "--radial-kp").and_then(|v| v.parse().ok()),
        radial_kd:  get_arg(&args, "--radial-kd").and_then(|v| v.parse().ok()),
        heading_kp: get_arg(&args, "--heading-kp").and_then(|v| v.parse().ok()),
        heading_kd: get_arg(&args, "--heading-kd").and_then(|v| v.parse().ok()),
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

fn load_plan_tuning(path: &str) -> PlaneTuning {
    let bytes =
        std::fs::read(path).unwrap_or_else(|e| panic!("Cannot read tuning file '{path}': {e}"));
    ron::de::from_bytes(&bytes).unwrap_or_else(|e| panic!("Cannot parse tuning file '{path}': {e}"))
}

fn load_plane_config(path: &str) -> PlaneConfig {
    let bytes =
        std::fs::read(path).unwrap_or_else(|e| panic!("Cannot read plane config '{path}': {e}"));
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
