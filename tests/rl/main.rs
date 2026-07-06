//! Consolidated RL integration tests (formerly `rl_inference` + `ppo_training` binaries;
//! merged per plans/test_compile_speed.md to cut link steps). Entirely compiled out
//! without an RL backend. Run one former file's tests with e.g.
//! `cargo test --no-default-features --features inference --test rl rl_inference::`.
#![cfg(any(feature = "inference", feature = "training"))]

mod rl_inference;

#[cfg(feature = "training")]
mod ppo_training;
