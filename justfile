# Supported test matrix. Feature combos not listed here are not supported
# test configurations — a green ad-hoc `cargo test --features …` run is not a gate
# (e.g. bare `--features mcp` compiles the sim tests out via the `sim_enabled` cfg;
# see build.rs). `--no-default-features` everywhere: the default `visual` feature
# loads rendering plugins that panic headless.

# Quick inner-loop suite (core sim, headless)
test:
    cargo test --no-default-features

# The full supported matrix: run before committing / as the CI gate
test-all:
    cargo test --no-default-features
    cargo test --no-default-features --features "mcp server"
    cargo test --no-default-features --features inference

# RL training suite — heavy wgpu build, needs a GPU; not part of test-all
test-training:
    cargo test --no-default-features --features training
