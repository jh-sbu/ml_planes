//! The one simulation tick rate.
//!
//! Rapier's `TimestepMode::Fixed`, Bevy's `Time<Fixed>` (which drives `FixedUpdate`
//! and, on a `net` build, bevy_replicon's server tick), and the self-contained Euler
//! integrator in [`crate::training::flight_env`] all run at this rate.
//!
//! **Why this is shared rather than a per-site literal:** the training integrator used
//! to run at 60 Hz while the live sim ran at 64 Hz, deliberately. That made
//! `evaluate_policy` — which drives the training envs, never Rapier — structurally
//! unable to see how a policy behaves in the sim it actually flies in. A heading-hold
//! policy accepted at `mean_tail_abs_altitude_m = 0.333` measured 1.47 m in the live
//! sim, and roughly a third of that gap was the timestep alone. A training env that
//! integrates at any other rate is validating against physics the live sim does not
//! have, so both sides now read the same constant.
//!
//! This module is deliberately Bevy-free and un-feature-gated: `plane` is unconditional
//! in `lib.rs`, so `--no-default-features`, `--features training`, and
//! `--features "mcp server"` all see these constants without pulling in a renderer.

/// Simulation tick rate [Hz]. See the module doc before changing this — it is the
/// physics timestep, the `FixedUpdate` rate, *and* the replication tick rate.
pub const PHYSICS_HZ: f32 = 64.0;

/// Simulation timestep [s]. `1 / [PHYSICS_HZ]`.
pub const PHYSICS_DT: f32 = 1.0 / PHYSICS_HZ;

/// `f64` form of [`PHYSICS_HZ`], for `Duration::from_secs_f64` and the replication-tick
/// timeline in `net::client`.
pub const PHYSICS_HZ_F64: f64 = PHYSICS_HZ as f64;

/// `f64` form of [`PHYSICS_DT`]. `PHYSICS_HZ` is a power of two, so this is bit-exact
/// with the `f32` form — pinned by `timestep_f32_and_f64_agree`.
pub const PHYSICS_DT_F64: f64 = 1.0 / PHYSICS_HZ_F64;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physics_dt_is_the_reciprocal_of_physics_hz() {
        assert!((PHYSICS_DT * PHYSICS_HZ - 1.0).abs() < 1e-7);
    }

    #[test]
    fn timestep_f32_and_f64_agree() {
        // Exact, not approximate: PHYSICS_HZ is a power of two, so both forms are
        // representable without rounding. If someone picks a non-power-of-two rate
        // this fails loudly rather than silently desyncing the f32 and f64 call sites.
        assert_eq!(PHYSICS_DT as f64, PHYSICS_DT_F64);
    }
}
