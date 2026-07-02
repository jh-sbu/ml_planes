//! Build script: derives the `sim_enabled` cfg from the active feature set.
//!
//! The 6-DOF simulation chain (aero forces, controllers, fuel/mass integration) is compiled
//! into `PlanePlugin` **only** when `any(not(feature = "net"), feature = "server")` holds — i.e.
//! every build *except* the thin networked client, which renders replicated state and runs no
//! physics (see `plans/client_server.md` and the gate in `src/plane/plugin.rs`).
//!
//! Integration tests that assert on sim-updated `FlightState` share that precondition, so they
//! gate on `#[cfg(sim_enabled)]` and simply compile out on a `net`-without-`server` build (e.g.
//! bare `--features mcp`, which enables `net` but not `server`) instead of reading a default
//! `FlightState` and failing spuriously. To exercise those tests against a networked build, add
//! the matching server feature, e.g. `cargo test --no-default-features --features "mcp server"`
//! (as everywhere in this project, keep `--no-default-features` — the default `visual` feature
//! pulls in rendering plugins that panic in a headless test).
//!
//! **Keep this condition in sync** with the `#[cfg(any(not(feature = "net"), feature = "server"))]`
//! gate in `src/plane/plugin.rs` — build script and library must agree on when the sim runs.

fn main() {
    // Declare the cfg name so `#[cfg(sim_enabled)]` never trips the `unexpected_cfgs` lint.
    println!("cargo::rustc-check-cfg=cfg(sim_enabled)");

    // Cargo exposes each enabled feature as `CARGO_FEATURE_<NAME>` (uppercased, `-`→`_`).
    let net = std::env::var_os("CARGO_FEATURE_NET").is_some();
    let server = std::env::var_os("CARGO_FEATURE_SERVER").is_some();
    if !net || server {
        println!("cargo::rustc-cfg=sim_enabled");
    }

    println!("cargo::rerun-if-changed=build.rs");
}
