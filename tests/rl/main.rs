//! Consolidated RL integration tests (formerly `rl_inference` + `ppo_training` binaries;
//! merged per plans/test_compile_speed.md to cut link steps). Entirely compiled out
//! without an RL backend. Run one former file's tests with e.g.
//! `cargo test --no-default-features --features inference --test rl rl_inference::`.
#![cfg(any(feature = "inference", feature = "training"))]

#[path = "../common/mod.rs"]
mod common;

mod rl_inference;
mod rl_sim_control;

#[cfg(feature = "training")]
mod ppo_training;
