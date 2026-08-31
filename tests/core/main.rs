//! Core-sim integration tests. Run one module's tests with
//! `cargo test --no-default-features --test core <module>::`.

#[path = "../common/mod.rs"]
mod common;

mod controller_targets;
mod controller_telemetry;
mod eval_run;
mod lifecycle;
mod orbit_tune_sync;
mod physics_timestep;
mod pid_convergence;
mod plane_assets;
mod render_pose_order;
mod scenario;
mod sim_control;
mod spawn_reset;
mod training_assets;
mod training_task;

// 6-DOF sim chain required; compiles out on net-without-server builds (see build.rs).
#[cfg(sim_enabled)]
mod aero_physics;
#[cfg(sim_enabled)]
mod flight_plan;
#[cfg(sim_enabled)]
mod fuel;
#[cfg(sim_enabled)]
mod heading_hold;
#[cfg(sim_enabled)]
mod level_hold;
#[cfg(sim_enabled)]
mod wingman;
