# Rust promotion

`src/controllers/inversion_level_hold.rs` implements the frozen winning cascade
as `InversionLevelHoldController`, with the twelve selected gains in
`InversionLevelHoldGains::default()`. Integral and inversion arithmetic use f64;
flight state, observations, and commands use f32. The shared level-hold
observation builder preserves the Python experiment's normalization and signs.

The controller is available without inference/training features. It is registered
in the controller menu, lifecycle spawn panel, scenario parser, and MCP spawn
and switch paths as `InversionLevelHold`. Its altitude/speed targets use the
existing local and replicated target editors. PID tuning asset arrival or profile
changes preserve the new controller's targets, gains, and integral state.
Existing PID/heading/orbit/formation controllers are not replaced.

The new network enum variant is appended; protocol identity advances from 7 to 8
so older peers reject the connection instead of attempting to decode it.
Rebuild both server and client when using this mode over the network.

## Benchmark parity

The native evaluator uses the unchanged generic-jet plane, reward, target ranges,
3200-step episodes and shared `EvalRun` metrics. Python runs its environments in
a batch while the native example runs sequentially, so small float32 aggregation
differences are expected. The Rust port uses f64 for inversion arithmetic while
NumPy uses a mixture of f32 intermediate values and f64 integral calculations.

| Evaluation | Python return | Rust return | Rust successes / mean steps |
|---|---:|---:|---:|
| Original benchmark | -93.257179 | -93.257217 | 64/64 / 3200 |
| Held-out seeds | -95.322533 | -95.322540 | 1024/1024 / 3200 |

Native settled errors are 0.005579 m altitude and 0.152695 m/s speed on the
original benchmark, and round to 0.006 m / 0.157 m/s on holdout. Both retain the substantial
advantage over the PPO incumbent's 0.443 m / 1.054 m/s original benchmark.

```bash
cargo run --release --no-default-features --example inversion_level_hold_baseline
cargo run --release --no-default-features --example inversion_level_hold_baseline -- \
  --episodes 1024 --seed 100000 --seed-stride 104729
cargo run --release --no-default-features --example observe_state -- \
  --scenario assets/scenarios/inversion_level_hold.scenario.ron
```

The live Bevy/Rapier scenario starts 100 m below the commanded 1000 m altitude,
at 100 m/s against a 110 m/s target, on the generic jet with a full tank. The
observe-state trace remains finite, captures altitude, and reaches approximately
1000.00 m / 110.08 m/s at the final sampled time (49 seconds).

Verification passed: the full no-default-features core suite, `mcp server` suite,
`inference` suite, and `visual` suite, plus `cargo check --features inference`
for the default desktop client. The networking reconnect and renderer tests
required host UDP/GPU access outside the sandbox; both passed there. The existing
opt-in MCP end-to-end test remained ignored. New regressions cover benchmark
performance, factory identity despite legacy tuning, target editing, scenario
construction, late tuning loads, profile changes, and switching controller kinds.

## Limits carried over

This is a faithful promotion of the tested generic-jet architecture, including
its aggressive recovery and known actuator-lag sensitivity. The nominal aero
coefficients are fixed; selecting this mode on another airframe does not adapt
those coefficients. It does not enforce structural/comfort load limits or offer
a guaranteed bumpless mode transition. Changing a target resets that target's
integral; reapplying an identical target leaves its integral intact. The Python
reference and its artifacts remain available for future experimentation.
