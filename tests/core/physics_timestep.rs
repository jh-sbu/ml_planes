//! Guards that every clock in the live sim runs at the one shared physics rate.
//!
//! `ml_planes::plane::PHYSICS_DT` is consumed explicitly by Rapier's
//! `TimestepMode::Fixed` and by the training Euler integrator, so those two can't
//! drift. Bevy's `Time<Fixed>` is the third clock and the one nothing sets: it
//! *happens* to default to 64 Hz, which is why the mismatch went unnoticed. It drives
//! `FixedUpdate` — and therefore bevy_replicon's server tick, which increments in
//! `FixedPostUpdate` and whose rate `net::client`'s snapshot timeline assumes.
//!
//! So raising `PHYSICS_HZ` would move Rapier and training while leaving `Time<Fixed>`
//! at 64, silently desyncing replication from physics. This test is the tripwire.

use bevy::prelude::*;

use ml_planes::plane::PHYSICS_DT;

use crate::common::build_headless_app;

#[test]
fn bevy_fixed_schedule_matches_shared_physics_dt() {
    let app = build_headless_app();
    let fixed = app.world().resource::<Time<Fixed>>().timestep();
    assert!(
        (fixed.as_secs_f32() - PHYSICS_DT).abs() < 1e-9,
        "Time<Fixed> runs at {} s/tick but PHYSICS_DT is {PHYSICS_DT} s — FixedUpdate \
         (and the replicon server tick that rides on it) has drifted from the physics rate",
        fixed.as_secs_f32(),
    );
}
