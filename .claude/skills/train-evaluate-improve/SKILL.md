---
description: Run a two-model PPO experiment for an ml_planes learnable controller (level-hold, orbit, residual-orbit, or lstm-orbit) — train a baseline, evaluate it, apply one targeted reward/PPO/init-from improvement, retrain, and report whether it helped. Release-mode ndarray only.
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

| User type | `--task` | Model subdir | Default reward config | Metric family |
|---|---|---|---|---|
| `level_hold`, `rl_level_hold` | `level_hold` | `level_hold` | `assets/training/level_hold.reward.ron` | LevelHold |
| `orbit`, `rl_orbit` | `orbit` | `orbit` | `assets/training/orbit.reward.ron` | Orbit |
| `residual_orbit`, `rl_orbit_residual` | `residual_orbit` | `orbit_residual` | `assets/training/orbit.reward.ron` | Orbit |
| `lstm_orbit`, `rl_lstm_orbit` | `lstm_orbit` | `lstm_orbit` | `assets/training/wu_orbit.reward.ron` | Orbit |

Reject `bc_*` and unknown values before training — those are not learnable
controller types this workflow supports. If meaningful results would require
code changes, ask the user before changing any code.

Do **not** use `train_bc`, `--bc-steps`, or `--bc-epochs` — behavior cloning is
a warm-start technique, not a controller type. Do **not** switch the requested
type to a different one (e.g. a plain `orbit` request to `residual_orbit`) —
resolve and train exactly what was asked.

Per-task caveats:

- **`residual_orbit`** — `residual_scale` lives in `orbit.reward.ron`; modest
  `residual_scale` experiments are allowed for this task only.
- **`lstm_orbit`** — the recurrent trainer ignores `--ppo-config`, so the PPO-
  config improvement vector is unavailable; rely on reward-config or
  `--init-from`. Its reward schema is `WuOrbitRewardConfig` (`b_radial`,
  `b_heading_coarse`, `b_heading_fine`, `b_altitude`, `b_speed`, …), not the
  orbit `*_reward_weight` schema. `evaluate_policy` defaults to
  `--curriculum-stage full` (keep the default) and echoes `curriculum_stage`.

**Requires exactly one argument.** If `<TYPE>` is missing or rejected, stop
immediately and print:

```
Usage: /train-evaluate-improve <type>
  Supported types: level_hold | orbit | residual_orbit | lstm_orbit
    (rl_level_hold | rl_orbit | rl_orbit_residual | rl_lstm_orbit aliases too)
Examples:
  /train-evaluate-improve level_hold
  /train-evaluate-improve orbit
```

Otherwise resolve `<TYPE>` through the table above, state the mapped task, model
subdirectory, default reward config, and metric family, and continue to Step 1.

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
cargo run --release --no-default-features --features inference --bin evaluate_policy -- \
  --task <task> \
  --backend ndarray \
  --model models/<model_dir>/<baseline_stem> \
  --episodes <episodes> \
  > target/skill-runs/<run>/baseline_eval.txt
```

`.mpk` or no extension are both accepted — be consistent. Read the eval output
and inspect the metrics for the resolved metric family. The common core is
always present; the extras depend on the family:

- **Common core (every task):** `success_rate`, `mean_return`,
  `mean_length_steps`.
- **Orbit family** (`orbit`, `residual_orbit`, `lstm_orbit`):
  `mean_abs_radial_m`, `mean_abs_heading_rad`, `mean_abs_altitude_m`,
  `mean_abs_speed_mps`, `mean_final_abs_radial_m`, `mean_final_abs_altitude_m`.
- **LevelHold family** (`level_hold`): `mean_abs_altitude_m`,
  `mean_abs_speed_mps`, `mean_abs_roll_rad`, `mean_abs_beta_rad`,
  `mean_final_abs_altitude_m` (no radial/heading metrics).

## Step 5 — Choose exactly one improvement

Pick the **smallest credible change** for the second model:

- **`--init-from <baseline>`** — continue from the baseline if the training log
  (`baseline_train.csv`) is still improving near the end.
- **Reward config** — if evaluation shows a dominant task error, copy the
  task's **default reward config** (from the resolution table) into the run
  directory and edit only the field(s) the hypothesis needs. Edit fields that
  exist in the copied file: `LevelHoldRewardConfig` (`alt_error_weight`,
  `speed_error_weight`, `roll_weight`, `beta_weight`, …) for level_hold, the
  orbit `*_reward_weight`/`*_reward_scale` schema (plus `residual_scale` for
  residual_orbit) for orbit/residual_orbit, and the `WuOrbitRewardConfig`
  `b_*` weights for lstm_orbit. Pass with `--reward-config <path>`.
- **PPO config** — if optimization looks unstable or undertrained, copy
  `assets/training/default.ppo.ron` into the run directory and make small
  changes (lower `lr`, higher `n_epochs`, larger `rollout_steps`, adjusted
  `entropy_coef`). Keep the full RON shape. Pass with `--ppo-config <path>`.
  **Not available for `lstm_orbit`** — the recurrent trainer ignores
  `--ppo-config`; use a reward-config or `--init-from` improvement instead.

Use the metrics to justify the change, per metric family:

**Orbit family** (`orbit`, `residual_orbit`, `lstm_orbit`):

| Symptom | Improvement |
|---|---|
| High radial error (`mean_abs_radial_m`) | Prioritize radial reward weight or scale |
| High heading error (`mean_abs_heading_rad`) | Prioritize heading reward weight or scale |
| High altitude error (`mean_abs_altitude_m`) | Prioritize altitude reward weight or scale |
| High speed error (`mean_abs_speed_mps`) | Prioritize speed reward weight or scale |
| Low success rate, short episodes | Inspect failure metric; gentler `lr` or more rollout steps |
| Good success but weak return | Continue from baseline, or reduce instability/penalty mismatch |

**LevelHold family** (`level_hold`):

| Symptom | Improvement |
|---|---|
| High altitude error (`mean_abs_altitude_m`) | Prioritize `alt_error_weight`/`alt_error_scale` |
| High speed error (`mean_abs_speed_mps`) | Prioritize `speed_error_weight`/`speed_error_scale` |
| High bank (`mean_abs_roll_rad`) | Prioritize `roll_weight`/`roll_scale` |
| High sideslip (`mean_abs_beta_rad`) | Prioritize `beta_weight`/`beta_scale` |
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
cargo run --release --no-default-features --features inference --bin evaluate_policy -- \
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

- Exact `<TYPE>` normalization, task, model subdir, metric family, model paths,
  steps, episodes, reward config, and PPO config.
- Baseline metrics and improved metrics **side by side** (the task's
  tracking-error metrics for its metric family).
- Whether performance improved. Prefer higher `success_rate`, higher
  `mean_return`, longer `mean_length_steps`, and lower absolute-error metrics.
- The improvement hypothesis and whether the result supported it.
- Recommended next experiments, including whether code changes would be needed
  for unsupported tasks or richer diagnostics.

If a training or evaluation command fails, preserve the error, explain what
failed, and do **not** silently change controller type or toolchain.
