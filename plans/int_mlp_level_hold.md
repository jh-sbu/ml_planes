# IntMLP level hold: promotion report

## What was promoted

The **architecture and the training pipeline** from `experiments/nn_arch`, not its
checkpoint. The Python run's `.pt` weights stay in the experiment directory; Rust
checkpoints are trained fresh with `train_int_mlp`.

| Piece | Rust | Python origin |
|---|---|---|
| Integrator + feature contract, flat forward pass | `src/training/int_mlp.rs` (ungated) | `common.py` `IntMLP`, `integrate` |
| burn module (checkpoints, gradients) | `src/training/int_mlp_model.rs` (`inference`) | `IntMLP` parameters |
| Controller | `src/controllers/int_mlp_level_hold.rs` (`inference`) | `common.act_fn` |
| DAgger + ES | `src/training/int_mlp_train.rs` (`training`) | `bc_dagger.py`, `es.py` |
| CLI | `src/bin/train_int_mlp.rs`, `evaluate_policy --arch int_mlp` | — |
| Expert hook | `InversionLevelHoldController::command_with_integrals` | `common.cascade_action` |

The network: `x = [obs(13), ih, iv] · INPUT_SCALE`, then
`pre = out(tanh(fc2(tanh(fc1(x))))) + skip(x)`, and `action = tanh(pre)`. The integrators
advance before the forward pass, clamped to ±30 / ±20. Because they are pure integrators of
the tracking errors, the closed loop can only rest at zero error. That is the property the
memoryless RL policies lacked (`experiments/rl_level_hold`, findings 10–13).

## Pipeline, as ported

1. **DAgger** (6 iterations × 128 envs × 3200 steps, 8 epochs, minibatch 4096, Adam lr
   `2e-3 · 0.6^k`, action noise 0.05).
   - Iteration 0 flies the expert; later iterations fly the learner.
   - Every visited state is labelled by `command_with_integrals` with the **learner's**
     integrators, so the label is exact and not an approximation of the expert's own
     history.
2. **ES** (500 generations, population 48 × 24 episodes, σ = 0.001, Adam lr 3e-4).
   - Antithetic sampling with centred ranks.
   - Every member flies the same episodes within a generation (common random numbers).
   - Fitness is the evaluation return, minus a hinge penalty on each tail metric above
     0.8 × the inversion benchmark bar, minus 1000 × the failure fraction.
   - Rollouts run on the flat kernel across `--threads` workers.
   - Validation (64 episodes, seeds 900000 + 7919·i) runs every 10 generations and writes a
     `_gNNNN` snapshot.

Deliberate differences from the Python run:
- Rust draws its own random numbers (SplitMix64 rather than NumPy/torch), so a seed does not
  reproduce the Python run's samples. Only the recipe carries over.
- The DAgger fit includes the final partial minibatch (as `torch.randperm(n).split(batch)`
  did) and uses Adam ε = 1e-8, matching `torch.optim.Adam`. burn's default is 1e-5.
- Training envs are ordinary `LevelHoldEnv`s with the shipped reward and default target
  envelope, the same environment the Python run drove through the bindings.

## Train and evaluate

```bash
cargo run --release --no-default-features --features training --bin train_int_mlp -- \
  --output int_mlp_level_hold --threads 12
# benchmark (64 episodes, seeds 42+i)
cargo run --release --no-default-features --features inference --bin evaluate_policy -- \
  --task level_hold --arch int_mlp --model models/int_mlp_level_hold/int_mlp_level_hold_best
# holdout (1024 episodes)
cargo run --release --no-default-features --features inference --bin evaluate_policy -- \
  --task level_hold --arch int_mlp --model models/int_mlp_level_hold/int_mlp_level_hold_best \
  --episodes 1024 --seed 100000 --seed-stride 104729
```

`--stage dagger|es|all` and `--init <stem>` let the stages run separately, for example to
rerun ES from a saved `_dagger` clone.

## The baseline to converge toward

From `experiments/nn_arch` (ES generation 270 of the IntMLP DAgger clone):

| | return | tail alt (m) | tail speed (m/s) | success |
|---|---:|---:|---:|---:|
| `.pt` IntMLP, benchmark (64) | −89.121 | 0.002254 | 0.012107 | 1.000 |
| `.pt` IntMLP, holdout (1024) | −92.826 | 0.002282 | 0.013059 | 1.000 |
| Inversion controller, benchmark | −93.257 | 0.005578 | 0.152694 | 1.000 |
| Inversion controller, holdout | −95.323 | 0.006379 | 0.157275 | 1.000 |

In that run, tail speed fell from about 0.15 to 0.013 m/s between ES generations 170 and 270,
and validation return was still improving when the run stopped at about generation 280.

## First Rust training run

`train_int_mlp --output int_mlp_level_hold --threads 8` (seed 0, all defaults) took 19
minutes on the 16-core laptop: DAgger 6 minutes, ES 500 generations at about 1.6 s each. It
**reached and passed the `.pt` baseline**.

DAgger validation went from −100.14 / 0.0135 m (iteration 0) to −89.44 / 0.0053 m /
0.148 m/s (iteration 5). In ES, tail speed collapsed from 0.12 to 0.012 m/s between
generations 170 and 360, the same shape as the Python run. The best validation fitness was
at generation 480 (`int_mlp_level_hold_best.mpk`).

Scored with `evaluate_policy --arch int_mlp`, full-precision tails from
`experiments/nn_arch/eval_mpk.py`:

| | return | tail alt (m) | tail speed (m/s) | success |
|---|---:|---:|---:|---:|
| Rust `_best` (gen 480), benchmark (64) | **−87.534** | **0.002705** | **0.011928** | 1.000 |
| Rust `_best`, holdout (1024) | **−91.830** | **0.002780** | **0.012692** | 1.000 |
| Rust final (gen 500), holdout | −92.185 | 0.003208 | 0.016103 | 1.000 |
| `.pt` IntMLP, holdout | −92.826 | 0.002282 | 0.013059 | 1.000 |
| Inversion controller, holdout | −95.323 | 0.006379 | 0.157275 | 1.000 |

On holdout, `_best` beats the inversion controller by +3.49 return, 2.29× on tail altitude
and 12.4× on tail speed. Against the `.pt` it is +1.00 return and about equal on speed, but
its altitude tail is 0.5 mm looser.

`eval_mpk.py` loads the burn record into the Python `IntMLP` and scores it through the
validated Python harness. Its returns match `evaluate_policy` to four decimals (−87.5343 vs
−87.534233; −91.8297 vs −91.829613), an independent check that the Rust kernel is the
experiment's network.

## Verification

- `just test-all`, `just test-training`, `just test-visual`: green. The only skipped test
  is `mcp_e2e`, which is ignored by design. `cargo check --features inference`: clean.
- New tests:
  - `training::int_mlp` (7): integrator math, clamps, feature scale, the flat layout.
  - `training::int_mlp_train` (10): rank shaping, ES gradient, Adam, antithetic sampling,
    toy-objective ES, the fitness hinge and survivors-only tails, rollout vs `EvalRun`
    agreement, DAgger and ES smoke runs.
  - `tests/rl/int_mlp.rs` (7) and two `rl_sim_control` cases.
  - `kind` (2).
  - `inversion_level_hold::command_with_integrals_is_the_second_half_of_update`.
- `observe_state --features inference` on `default.scenario.ron`: the
  `int_mlp_level_hold` plane (gen 500 checkpoint) spawns and holds 1000.00 m and
  100.03 m/s over 40 s in live Rapier. **But** it carries a steady ≈0.1° roll and ≈0.04° β,
  so its heading drifts about 0.014°/s: 0.56° and 16 m of cross-track after 40 s. The
  inversion plane beside it holds 0.000°. Level hold does not reward heading, and the roll
  term's cost at 0.1° is negligible, so nothing in training pushes that bias to zero. If a
  held heading matters, a lateral integrator or a heading-aware reward is the follow-up.

## Wiring

Everything follows the RL-controller checklist:
- `ControllerKind::IntMlpLevelHold` is appended, which bumped the protocol to v9.
- `model_dir() == "int_mlp_level_hold"`, and the kind is in the inference `ALL` list.
- `build()` falls back to the `LevelHold` PID.
- `sim_control` has the load, demote-on-failure, and preserve-across-tuning arms.
- Tuning dropdown matches, map colour, lifecycle panel, MCP bridge and docs are updated.
- Scenario spec `IntMlpLevelHold { model, altitude, airspeed }` always parses and builds only
  with `inference`.
- `default.scenario.ron` gains an `int_mlp_level_hold` plane at z = 60, skipped until
  `models/int_mlp_level_hold/int_mlp_level_hold.mpk` exists.

## Limits

- Calibrated like its teacher: generic jet, instantaneous actuators. The inversion
  controller's measured actuator-lag sensitivity was not re-tested for IntMLP.
- Training-environment results only (see the eval vs live-sim gap). Validate a trained
  checkpoint in `observe_state` / live Rapier before relying on it.
- No GPU path: DAgger runs on `Autodiff<NdArray>` and ES on the flat CPU kernel.
