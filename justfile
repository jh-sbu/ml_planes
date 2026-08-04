# Supported test matrix. Feature combos not listed here are not supported
# test configurations — a green ad-hoc `cargo test --features …` run is not a gate
# (e.g. bare `--features mcp` compiles the sim tests out via the `sim_enabled` cfg;
# see build.rs). `--no-default-features` everywhere: the default features build the
# *app binary*, whose rendering plugins panic headless. The `visual` **tests** are
# headless-safe (pure math / a headless egui Context) — see `test-visual`.

# Quick inner-loop suite (core sim, headless)
test:
    cargo test --no-default-features

# The full supported matrix: run before committing / as the CI gate
test-all:
    cargo test --no-default-features
    cargo test --no-default-features --features "mcp server"
    cargo test --no-default-features --features inference

# The `visual` tests live behind `#[cfg(feature = "visual")]` (src/ui/**, src/camera/**),
# so every other recipe skips them silently — a green test-all says nothing about them.
# Kept out of test-all for the bevy/default (wgpu/winit/GTK) link cost, not because the
# tests need a display. No `net` here, so `sim_enabled` holds (build.rs): this also
# re-runs the core sim suite and compile-checks the local-sim/wasm bin path.

# UI/camera suite — run after any UI/camera work; not part of test-all
test-visual:
    cargo test --no-default-features --features visual

# RL training suite — CPU (ndarray) build, the default training backend; not part of test-all
test-training:
    cargo test --no-default-features --features training

# GPU training suite — opt-in wgpu/cubecl backend (heavy build, needs a GPU); not part of test-all
test-training-gpu:
    cargo test --no-default-features --features "training wgpu"

# Client (`cargo planes`) and server (`ml_planes_server`) are separate cargo targets
# with disjoint feature sets, so a plain `cargo run --bin ml_planes` never rebuilds
# the server the client's "Start New Server" spawns as a sibling process — they
# silently drift apart (a stale server behind a fresh client shows up as confusing
# runtime errors, e.g. an RL controller rejecting an otherwise-current checkpoint
# because the *server's* obs-dim code is stale, not the checkpoint). These recipes
# build both before running so the pair can't drift; `cargo planes` alone is still
# fine day-to-day, and a staleness banner in the client covers that path.
play:
    cargo build --release --no-default-features --features "server inference" --bin ml_planes_server
    cargo run --release --features training --bin ml_planes

play-debug:
    cargo build --no-default-features --features "server inference" --bin ml_planes_server
    cargo run --features training --bin ml_planes
