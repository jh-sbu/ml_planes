//! Headless simulation runner — streams per-plane FlightState + ControlInputs
//! as CSV to stdout, driven by a multi-plane `.scenario.ron` file.
//!
//! Build and run (requires no window; --no-default-features drops the renderer):
//!
//!   cargo run --example observe_state --no-default-features -- \
//!     --scenario assets/scenarios/level_hold.scenario.ron
//!
//! To exercise trained RL policies (rl_* controller specs), add `--features inference`:
//!
//!   cargo run --example observe_state --no-default-features --features inference -- \
//!     --scenario <path-with-rl-plane>
//!
//! Options:
//!   --scenario PATH   Path to a `.scenario.ron` file (required).
//!   --steps N         Override the scenario's step count (64 Hz).
//!   --interval N      Override the scenario's print interval (every Nth step).
//!
//! The scenario file describes one or more planes — each with an initial state,
//! an optional `.plane.ron` config, and a controller. Controllers that follow
//! another plane (e.g. `Wingman`) reference their leader by `name`. See
//! `assets/scenarios/*.scenario.ron` for examples and `src/scenario.rs` for the
//! full format.
//!
//! Output: one CSV row per plane per sampled step (rows ordered by PlaneId).
//! The trailing `radial_error_m,heading_error_rad,bank_ff_rad` columns are
//! populated for orbit-like controllers and blank otherwise. See
//! `scenario::CSV_HEADER`.

use std::time::Duration;

use bevy::prelude::*;
use bevy::time::TimeUpdateStrategy;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::ActiveController;
use ml_planes::{
    plane::{ControlInputs, FlightState, PlaneConfig, PlaneConfigHandle, PlaneId, PlanePlugin},
    scenario::{Scenario, CSV_HEADER},
};

fn main() {
    let args = parse_args();

    let Some(scenario_path) = args.scenario.as_deref() else {
        usage_and_exit();
    };

    let scenario =
        Scenario::from_path(std::path::Path::new(scenario_path)).unwrap_or_else(|e| fatal(&e));
    let mut resolved = scenario.resolve().unwrap_or_else(|e| fatal(&e));

    // CLI overrides for sim-runner knobs.
    if let Some(s) = args.steps {
        resolved.steps = s;
    }
    if let Some(i) = args.interval {
        resolved.interval = i;
    }
    if resolved.interval == 0 {
        fatal("--interval (or scenario `interval`) must be >= 1");
    }

    println!("{CSV_HEADER}");

    let mut app = App::new();
    app.add_plugins(MinimalPlugins)
        .add_plugins(bevy::transform::TransformPlugin)
        .add_plugins(bevy::asset::AssetPlugin::default())
        .insert_resource(TimestepMode::Fixed {
            dt: 1.0 / 64.0,
            substeps: 1,
        })
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule())
        .add_plugins(PlanePlugin);

    // 1/64 s per update = exactly one fixed tick per update (no accumulator overflow),
    // matching the production sim (main.rs) and the test harness (tests/common/mod.rs).
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f32(
        1.0 / 64.0,
    )));

    // finish() initialises plugin resources; required before manual app.update() driving.
    app.finish();

    // Build controllers and spawn one entity per plane. PlaneConfig assets are
    // inserted synchronously so aero forces are available from step 0 (mirrors
    // tests/common/mod.rs — bypasses async asset loading).
    let plane_count = resolved.planes.len();
    for idx in 0..plane_count {
        let controller = resolved.build_controller(idx).unwrap_or_else(|e| fatal(&e));

        let plane = &resolved.planes[idx];
        let cfg = match &plane.config {
            Some(path) => load_plane_config(path),
            None => generic_jet_config(),
        };
        let handle = app
            .world_mut()
            .resource_mut::<Assets<PlaneConfig>>()
            .add(cfg.clone());

        // Body-frame angular velocity → world frame for Rapier.
        let angvel_world = plane.attitude.mul_vec3(plane.angular_velocity);

        // Load the tank to the requested fraction of capacity (default full),
        // mirroring the live spawner (environment::spawner::spawn_plane). The
        // PlanePlugin's `consume_fuel` / `update_plane_mass` systems then burn it
        // down and keep the Rapier body mass current over the run.
        let fuel_fraction = plane.fuel_fraction.unwrap_or(1.0);
        let consumable = cfg.powerplant.capacity() * fuel_fraction;

        app.world_mut().spawn((
            RigidBody::Dynamic,
            Collider::cuboid(3.0, 0.5, 1.0),
            Velocity {
                linvel: plane.velocity,
                angvel: angvel_world,
            },
            ExternalForce::default(),
            AdditionalMassProperties::MassProperties(MassProperties {
                local_center_of_mass: Vec3::ZERO,
                // Empty mass + fuel load (jets); empty mass alone for electric.
                mass: cfg.powerplant.effective_mass(cfg.mass, consumable),
                principal_inertia: cfg.inertia,
                principal_inertia_local_frame: Quat::IDENTITY,
            }),
            FlightState {
                consumable_remaining: consumable,
                ..FlightState::default()
            },
            ControlInputs::default(),
            ActiveController(controller),
            PlaneConfigHandle(handle),
            plane.id,
            Transform::from_translation(plane.position).with_rotation(plane.attitude),
        ));
    }

    // PlaneId → (name, orbit diagnostics) for output, sorted by PlaneId.
    let mut info: Vec<(PlaneId, String, Option<ml_planes::scenario::OrbitDiag>)> = resolved
        .planes
        .iter()
        .map(|p| (p.id, p.name.clone(), p.orbit_diag))
        .collect();
    info.sort_by_key(|(id, _, _)| id.0);

    for step in 0..resolved.steps {
        app.update();

        if step % resolved.interval == 0 {
            // Snapshot every plane's state for this step. Copy out before the
            // next update() invalidates the world borrow.
            let mut q = app
                .world_mut()
                .query::<(&PlaneId, &FlightState, &ControlInputs)>();
            let world = app.world();
            let mut rows: Vec<(u32, String)> = Vec::with_capacity(plane_count);
            for (id, state, inputs) in q.iter(world) {
                let Some((_, name, orbit_diag)) = info.iter().find(|(pid, _, _)| pid == id) else {
                    continue;
                };
                rows.push((
                    id.0,
                    ml_planes::scenario::csv_row(step, name, state, inputs, *orbit_diag),
                ));
            }
            rows.sort_by_key(|(id, _)| *id);
            for (_, row) in rows {
                println!("{row}");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// CLI helpers

struct Args {
    scenario: Option<String>,
    steps: Option<usize>,
    interval: Option<usize>,
}

fn parse_args() -> Args {
    let args: Vec<String> = std::env::args().collect();
    Args {
        scenario: get_arg(&args, "--scenario"),
        steps: parse_usize_arg(&args, "--steps"),
        interval: parse_usize_arg(&args, "--interval"),
    }
}

fn get_arg(args: &[String], flag: &str) -> Option<String> {
    args.windows(2)
        .find(|pair| pair[0] == flag)
        .map(|pair| pair[1].clone())
}

/// Parse a `usize` flag. Absent → `None`; present but unparseable → fatal error
/// (rather than silently falling back to the default).
fn parse_usize_arg(args: &[String], flag: &str) -> Option<usize> {
    let raw = get_arg(args, flag)?;
    match raw.parse() {
        Ok(v) => Some(v),
        Err(_) => fatal(&format!(
            "{flag} expects a non-negative integer, got '{raw}'"
        )),
    }
}

fn usage_and_exit() -> ! {
    eprintln!(
        "Usage: observe_state --scenario PATH [--steps N] [--interval N]\n\
         \n\
         PATH is a .scenario.ron file describing one or more planes.\n\
         Examples:\n\
         \x20 --scenario assets/scenarios/level_hold.scenario.ron\n\
         \x20 --scenario assets/scenarios/wingman_formation.scenario.ron\n\
         \n\
         rl_* controller specs require building with --features inference."
    );
    std::process::exit(2);
}

fn fatal(msg: &str) -> ! {
    eprintln!("{msg}");
    std::process::exit(2);
}

// ---------------------------------------------------------------------------
// Config loading

fn load_plane_config(path: &str) -> PlaneConfig {
    let bytes =
        std::fs::read(path).unwrap_or_else(|e| panic!("Cannot read plane config '{path}': {e}"));
    ron::de::from_bytes(&bytes)
        .unwrap_or_else(|e| panic!("Cannot parse plane config '{path}': {e}"))
}

/// Fallback config — embeds `assets/planes/generic_jet.plane.ron` at compile time so the
/// default can never drift from the asset.
fn generic_jet_config() -> PlaneConfig {
    const SRC: &str = include_str!("../assets/planes/generic_jet.plane.ron");
    ron::de::from_str(SRC).expect("embedded generic_jet config is valid RON")
}
