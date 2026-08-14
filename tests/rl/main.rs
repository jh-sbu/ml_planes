//! RL integration tests. Entirely compiled out without an RL backend.
//! Run one module's tests with e.g.
//! `cargo test --no-default-features --features inference --test rl rl_inference::`.
#![cfg(any(feature = "inference", feature = "training"))]

#[path = "../common/mod.rs"]
mod common;

mod rl_inference;
mod rl_sim_control;

#[cfg(feature = "training")]
mod ppo_training;
