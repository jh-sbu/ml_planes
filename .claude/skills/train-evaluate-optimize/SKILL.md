---
description: Run an adaptive multi-experiment PPO loop for an ml_planes learnable controller (level-hold, orbit, residual-orbit, or lstm-orbit) — baseline train/eval, then targeted reward/PPO/init-from experiments up to a budget, with improvement and early-stop rules. Release-mode ndarray only.
---

Run an adaptive PPO experiment loop for this repo: baseline train/eval, then
multiple targeted train/eval experiments until the experiment budget is
exhausted or the remaining realistic improvements require code changes. Invoke
as `/train-evaluate-optimize <type>`.

> Reference doc: the original Codex-format version of this workflow lives at
> `.agents/skills/train-evaluate-optimize/SKILL.md`. This file is the Claude Code
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
  config experiment vector is unavailable; rely on reward-config or
  `--init-from`. Its reward schema is `WuOrbitRewardConfig` (`b_radial`,
  `b_heading_coarse`, `b_heading_fine`, `b_altitude`, `b_speed`, …), not the
  orbit `*_reward_weight` schema. `evaluate_policy` defaults to
  `--curriculum-stage full` (keep the default) and echoes `curriculum_stage`.

**Requires exactly one argument.** If `<TYPE>` is missing or rejected, stop
immediately and print:

```
Usage: /train-evaluate-optimize <type>
  Supported types: level_hold | orbit | residual_orbit | lstm_orbit
    (rl_level_hold | rl_orbit | rl_orbit_residual | rl_lstm_orbit aliases too)
Examples:
  /train-evaluate-optimize level_hold
  /train-evaluate-optimize orbit
```

Otherwise resolve `<TYPE>` through the table above, state the mapped task, model
subdirectory, default reward config, and metric family, and continue to the
budget step.

---

## Budget and defaults

Use the user's explicit experiment limit when supplied through wording such as
`max experiments`, `limit`, `budget`, or `run up to N`. **Count every training
run, including the baseline.**

Defaults unless supplied:

- Experiment limit: `6` total training runs.
- `--steps 2000000`.
- `--episodes 64`.
- Default task reward config.
- Compiled PPO defaults.

If the limit is `1`, run only the baseline train/eval and report that no
follow-up fit the budget. Reject `0`, negative, or non-numeric limits.

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

Choose output stems that will not overwrite existing checkpoints, such as
`skill_<type>_<label>_<timestamp>`. Models land at
`models/<model_dir>/<stem>.mpk`.

## Step 3 — Train and evaluate the baseline

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

## Step 4 — Adaptive experiment loop

Repeat until the experiment limit is reached or the early-stop rule (below)
applies. Each iteration is one training run plus one evaluation. Use the
**same** `--episodes`, optional `--max-steps`, and reward-config policy for
every evaluation so runs stay comparable.

For each follow-up experiment:

1. Write a short hypothesis and choose **exactly one** experiment direction.
2. **Early in the loop**, when no dominant failure is clear, explore distinct
   no-code vectors — small PPO config changes such as `lr`, `rollout_steps`,
   `n_epochs`, `minibatch`, `entropy_coef`, or `clip_epsilon`. **For
   `lstm_orbit` the recurrent trainer ignores `--ppo-config`**, so this vector
   is unavailable — explore reward-config and `--init-from` vectors instead.
3. **Once a signal appears**, dial in on the best vector: continue from the
   best checkpoint with `--init-from`, or copy and target the task's reward
   config when its family's tracking-error or stability metrics dominate the
   failure (radial/heading/altitude/speed for the Orbit family;
   altitude/speed/roll/beta for the LevelHold family).
4. For `residual_orbit` **only**, allow modest `residual_scale` experiments
   from a copied reward config.
5. Do **not** repeat a failed vector unless the next variant is narrower and
   metric-backed.

Train each experiment with the same command shape as Step 3, into a fresh
stem, adding the chosen `--init-from`, `--reward-config`, or `--ppo-config`:

```bash
cargo run --release --no-default-features --features training --bin train_ppo -- \
  --task <task> \
  --backend ndarray \
  --plain \
  --steps <steps> \
  --output <experiment_stem> \
  [--init-from models/<model_dir>/<incumbent_stem>] \
  [--reward-config target/skill-runs/<run>/<experiment>.reward.ron] \
  [--ppo-config target/skill-runs/<run>/<experiment>.ppo.ron] \
  --log-file target/skill-runs/<run>/<experiment>_train.csv
```

Evaluate each experiment exactly as in Step 3, writing to
`target/skill-runs/<run>/<experiment>_eval.txt`, then apply the improvement and
stop rules to decide the incumbent and whether to continue.

## Step 5 — Post-flight test

```bash
cargo test --no-default-features
```

## Step 6 — Report

(See the Reporting section below.)

---

## Config guidance

When creating a reward config, copy the task's **default reward config** (from
the resolution table) into the run directory and edit only the fields the
hypothesis needs. Edit fields that exist in the copied file:
`LevelHoldRewardConfig` (`alt_error_weight`, `speed_error_weight`,
`roll_weight`, `beta_weight`, …) for level_hold, the orbit
`*_reward_weight`/`*_reward_scale` schema (plus `residual_scale` for
residual_orbit) for orbit/residual_orbit, and the `WuOrbitRewardConfig` `b_*`
weights for lstm_orbit. Use the copy for both training and evaluation of that
experiment.

When creating a PPO config, start from `assets/training/default.ppo.ron`. Keep
the full RON shape and pass it with `--ppo-config <path>`. **Not available for
`lstm_orbit`** — the recurrent trainer ignores `--ppo-config`.

Use evaluation metrics to justify changes, per metric family.

**Orbit family** (`orbit`, `residual_orbit`, `lstm_orbit`):

| Symptom | Improvement |
|---|---|
| High radial error (`mean_abs_radial_m`) | Prioritize radial reward weight/scale or radius-stabilizing PPO changes |
| High heading error (`mean_abs_heading_rad`) | Prioritize heading reward weight/scale |
| High altitude error (`mean_abs_altitude_m`) | Prioritize altitude reward weight/scale |
| High speed error (`mean_abs_speed_mps`) | Prioritize speed reward weight/scale |
| Short episodes or low success with noisy returns | Lower `lr`, increase `rollout_steps`, or reduce update aggressiveness |
| Good success but weak return | Continue from the incumbent, or tune reward/penalty mismatch |

**LevelHold family** (`level_hold`):

| Symptom | Improvement |
|---|---|
| High altitude error (`mean_abs_altitude_m`) | Prioritize `alt_error_weight`/`alt_error_scale` |
| High speed error (`mean_abs_speed_mps`) | Prioritize `speed_error_weight`/`speed_error_scale` |
| High bank (`mean_abs_roll_rad`) | Prioritize `roll_weight`/`roll_scale` |
| High sideslip (`mean_abs_beta_rad`) | Prioritize `beta_weight`/`beta_scale` |
| Short episodes or low success with noisy returns | Lower `lr`, increase `rollout_steps`, or reduce update aggressiveness |
| Good success but weak return | Continue from the incumbent, or tune reward/penalty mismatch |

## Improvement and stop rules

Prefer higher `success_rate`. Treat a run as **improved** if:

- `success_rate` increases by at least `0.02`, or
- success is within `0.01` of the incumbent while `mean_return` improves by at
  least 5% **and** at least two key absolute error metrics improve by at least
  5%.

Reject apparent wins that materially regress mean episode length or core
tracking errors by more than 10%.

Stop **before** the experiment limit only when **all** are true:

- Baseline plus at least three follow-ups have run.
- At least two distinct no-code vectors failed to improve the incumbent.
- The remaining likely improvements require code changes or unsupported
  workflows.

Code-change examples include evaluator metric additions, environment
observation/action changes, reward schema changes, network architecture
changes, unsupported tasks, new controller variants, or new training binaries.
Report the exact proposed code direction and ask for explicit permission
instead of editing.

## Reporting

Report:

- Exact `<TYPE>` normalization, task, model directory, metric family, experiment
  limit, steps, episodes, reward configs, PPO configs, and checkpoint paths.
- A **compact leaderboard** of all experiments with success, return, length, and
  the task's tracking-error metrics for its metric family (radial/heading/
  altitude/speed + final radial/altitude for Orbit; altitude/speed/roll/beta +
  final altitude for LevelHold).
- The incumbent model and whether performance improved.
- Each experiment hypothesis and whether the result supported it.
- Why the loop stopped and the next realistic no-code or code-required step.

If a training or evaluation command fails, preserve the error, explain what
failed, and do **not** silently change controller type or toolchain.
