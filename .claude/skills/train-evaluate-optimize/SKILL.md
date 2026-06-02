---
description: Run an adaptive multi-experiment PPO loop for an ml_planes orbit controller — baseline train/eval, then targeted reward/PPO/init-from experiments up to a budget, with improvement and early-stop rules. Release-mode ndarray only.
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
Usage: /train-evaluate-optimize <type>
  Supported types: orbit | rl_orbit | residual_orbit | rl_orbit_residual
Examples:
  /train-evaluate-optimize orbit
  /train-evaluate-optimize residual_orbit
```

Otherwise resolve `<TYPE>` through the table above, state the mapped task and
model subdirectory, and continue to the budget step.

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

## Step 4 — Adaptive experiment loop

Repeat until the experiment limit is reached or the early-stop rule (below)
applies. Each iteration is one training run plus one evaluation. Use the
**same** `--episodes`, optional `--max-steps`, and reward-config policy for
every evaluation so runs stay comparable.

For each follow-up experiment:

1. Write a short hypothesis and choose **exactly one** experiment direction.
2. **Early in the loop**, when no dominant failure is clear, explore distinct
   no-code vectors — small PPO config changes such as `lr`, `rollout_steps`,
   `n_epochs`, `minibatch`, `entropy_coef`, or `clip_epsilon`.
3. **Once a signal appears**, dial in on the best vector: continue from the
   best checkpoint with `--init-from`, or copy and target the reward config
   when radial, heading, altitude, speed, or stability metrics dominate the
   failure.
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

When creating a reward config, copy `assets/training/orbit.reward.ron` into the
run directory and edit only the fields the hypothesis needs (residual orbit also
starts from `orbit.reward.ron`). Use the copy for both training and evaluation
of that experiment.

When creating a PPO config, start from `assets/training/default.ppo.ron`. Keep
the full RON shape and pass it with `--ppo-config <path>`.

Use evaluation metrics to justify changes:

| Symptom | Improvement |
|---|---|
| High radial error | Prioritize radial reward weight/scale or radius-stabilizing PPO changes |
| High heading error | Prioritize heading reward weight/scale |
| High altitude error | Prioritize altitude reward weight/scale |
| High speed error | Prioritize speed reward weight/scale |
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

- Exact `<TYPE>` normalization, task, model directory, experiment limit, steps,
  episodes, reward configs, PPO configs, and checkpoint paths.
- A **compact leaderboard** of all experiments with success, return, length,
  radial/heading/altitude/speed errors, and final radial/altitude errors.
- The incumbent model and whether performance improved.
- Each experiment hypothesis and whether the result supported it.
- Why the loop stopped and the next realistic no-code or code-required step.

If a training or evaluation command fails, preserve the error, explain what
failed, and do **not** silently change controller type or toolchain.
