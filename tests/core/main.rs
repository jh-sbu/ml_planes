//! Consolidated core-sim integration tests (formerly 13 separate test binaries; merged
//! per plans/test_compile_speed.md to cut link steps). Run one former file's tests with
//! `cargo test --no-default-features --test core <module>::`.

#[path = "../common/mod.rs"]
mod common;

mod controller_telemetry;
mod lifecycle;
mod orbit_tune_sync;
mod pid_convergence;
mod plane_assets;
mod scenario;
mod sim_control;
mod spawn_reset;

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
