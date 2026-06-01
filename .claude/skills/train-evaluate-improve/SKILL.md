---
description: Run a two-model PPO experiment for an ml_planes orbit controller — train a baseline, evaluate it, apply one targeted reward/PPO/init-from improvement, retrain, and report whether it helped. Release-mode ndarray only.
---

Run a two-model PPO experiment for this repo: baseline train, baseline
evaluation, one targeted improvement, second train, second evaluation, and
comparison. Invoke as `/train-evaluate-improve <type>`.

> Reference doc: the original Codex-format version of this workflow lives at
> `.agents/skills/train-evaluate-improve/SKILL.md`. This file is the Claude Code
> adaptation; the two should stay behaviorally in sync.

## Supported type gate

Normalize `<TYPE>` to lowercase and replace hyphens with underscores **before
doing any expensive work**.

| User type | `train_ppo --task` | `evaluate_policy --task` | Model subdir |
|---|---|---|---|
| `orbit`, `rl_orbit` | `orbit` | `orbit` | `orbit` |
| `residual_orbit`, `rl_orbit_residual` | `residual_orbit` | `residual_orbit` | `orbit_residual` |

Reject everything else before training. In particular reject `level_hold`,
`rl_level_hold`, `lstm_orbit`, `rl_lstm_orbit`, `bc_*`, and unknown values —
the existing `evaluate_policy` binary cannot complete this workflow for them.
If meaningful results would require code changes, ask the user before changing
any code.

Do **not** use `train_bc`, `--bc-steps`, or `--bc-epochs`. Do **not** switch a
plain `orbit` request to `residual_orbit` — use residual only when the requested
type itself is residual.

**Requires exactly one argument.** If `<TYPE>` is missing or rejected, stop
immediately and print:

```
Usage: /train-evaluate-improve <type>
  Supported types: orbit | rl_orbit | residual_orbit | rl_orbit_residual
Examples:
  /train-evaluate-improve orbit
  /train-evaluate-improve residual_orbit
```

Otherwise resolve `<TYPE>` through the table above, state the mapped task and
model subdirectory, and continue to Step 1.

---

## Step 1 — Pre-flight test

Before changing or generating any experiment files:

```bash
cargo test --no-default-features
```

Do not proceed if this fails — preserve the error and report it.

## Step 2 — Run directory

Create a run directory for logs and per-experiment configs (`<timestamp>` =
`date +%Y%m%d-%H%M%S` or similar). All generated files for this run live here.

```bash
mkdir -p target/skill-runs/<type>-<timestamp>
```

Pick defaults unless the user supplied them: `--steps 2000000`,
`--episodes 64`, the default task reward config, and compiled PPO defaults.
Choose output stems that will not overwrite existing checkpoints, such as
`skill_<type>_baseline_<timestamp>` and `skill_<type>_improved_<timestamp>`.
Models land at `models/<model_dir>/<stem>.mpk`.

## Step 3 — Train baseline

Train only in release mode, with training features and the `ndarray` backend:

```bash
cargo run --release --no-default-features --features training --bin train_ppo -- \
  --task <task> \
  --backend ndarray \
  --plain \
  --steps <steps> \
  --output <baseline_stem> \
  --log-file target/skill-runs/<run>/baseline_train.csv
```

## Step 4 — Evaluate baseline

Evaluate with the existing evaluator and the same task:

```bash
cargo run --release --no-default-features --features training --bin evaluate_policy -- \
  --task <task> \
  --backend ndarray \
  --model models/<model_dir>/<baseline_stem> \
  --episodes <episodes> \
  > target/skill-runs/<run>/baseline_eval.txt
```

`.mpk` or no extension are both accepted — be consistent. Read the eval output
and inspect these metrics:

`success_rate`, `mean_return`, `mean_length_steps`, `mean_abs_radial_m`,
`mean_abs_heading_rad`, `mean_abs_altitude_m`, `mean_abs_speed_mps`,
`mean_final_abs_radial_m`, `mean_final_abs_altitude_m`.

## Step 5 — Choose exactly one improvement

Pick the **smallest credible change** for the second model:

- **`--init-from <baseline>`** — continue from the baseline if the training log
  (`baseline_train.csv`) is still improving near the end.
- **Reward config** — if evaluation shows a dominant task error, copy
  `assets/training/orbit.reward.ron` into the run directory and edit only the
  field(s) the hypothesis needs (residual orbit also starts from
  `orbit.reward.ron`). Pass with `--reward-config <path>`.
- **PPO config** — if optimization looks unstable or undertrained, copy
  `assets/training/default.ppo.ron` into the run directory and make small
  changes (lower `lr`, higher `n_epochs`, larger `rollout_steps`, adjusted
  `entropy_coef`). Keep the full RON shape. Pass with `--ppo-config <path>`.

Use the metrics to justify the change:

| Symptom | Improvement |
|---|---|
| High radial error | Prioritize radial reward weight or scale |
| High heading error | Prioritize heading reward weight or scale |
| High altitude error | Prioritize altitude reward weight or scale |
| Low success rate, short episodes | Inspect failure metric; gentler `lr` or more rollout steps |
| Good success but weak return | Continue from baseline, or reduce instability/penalty mismatch |

## Step 6 — Train improved model

Same command shape as Step 3, into the improved stem, plus the chosen
`--init-from`, `--reward-config`, or `--ppo-config`:

```bash
cargo run --release --no-default-features --features training --bin train_ppo -- \
  --task <task> \
  --backend ndarray \
  --plain \
  --steps <steps> \
  --output <improved_stem> \
  [--init-from models/<model_dir>/<baseline_stem>] \
  [--reward-config target/skill-runs/<run>/improved.reward.ron] \
  [--ppo-config target/skill-runs/<run>/improved.ppo.ron] \
  --log-file target/skill-runs/<run>/improved_train.csv
```

## Step 7 — Evaluate improved model

Use the **same** `--episodes`, `--max-steps` (if supplied), and reward-config
policy as the baseline evaluation, so the two are comparable:

```bash
cargo run --release --no-default-features --features training --bin evaluate_policy -- \
  --task <task> \
  --backend ndarray \
  --model models/<model_dir>/<improved_stem> \
  --episodes <episodes> \
  > target/skill-runs/<run>/improved_eval.txt
```

## Step 8 — Post-flight test

```bash
cargo test --no-default-features
```

## Step 9 — Report

Report:

- Exact `<TYPE>` normalization, task, model paths, steps, episodes, reward
  config, and PPO config.
- Baseline metrics and improved metrics **side by side**.
- Whether performance improved. Prefer higher `success_rate`, higher
  `mean_return`, longer `mean_length_steps`, and lower absolute-error metrics.
- The improvement hypothesis and whether the result supported it.
- Recommended next experiments, including whether code changes would be needed
  for unsupported tasks or richer diagnostics.

If a training or evaluation command fails, preserve the error, explain what
failed, and do **not** silently change controller type or toolchain.
