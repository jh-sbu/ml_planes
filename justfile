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

# RL training suite — heavy wgpu build, needs a GPU; not part of test-all
test-training:
    cargo test --no-default-features --features training
