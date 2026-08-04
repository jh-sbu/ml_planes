---
description: Run an adaptive multi-experiment PPO loop for an ml_planes learnable controller (level-hold, orbit, residual-orbit, or lstm-orbit) — baseline train/eval, then targeted reward/PPO/init-from experiments up to a budget, with improvement and early-stop rules. Supports running experiments in parallel batches to use idle compute. Release-mode ndarray only.
---

Run an adaptive PPO experiment loop for this repo: baseline train/eval, then
multiple targeted train/eval experiments until the experiment budget is
exhausted or the remaining realistic improvements require code changes. Invoke
as `/train-evaluate-optimize <type> [total_runs] [runs_per_batch]`.

This file is the single source of truth for the workflow — there is no
separate Codex-format doc to keep in sync.

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

**Requires at least the type argument.** If `<TYPE>` is missing or rejected, stop
immediately and print:

```
Usage: /train-evaluate-optimize <type> [total_runs] [runs_per_batch]
  Supported types: level_hold | orbit | residual_orbit | lstm_orbit
    (rl_level_hold | rl_orbit | rl_orbit_residual | rl_lstm_orbit aliases too)
Examples:
  /train-evaluate-optimize level_hold
  /train-evaluate-optimize orbit 10
  /train-evaluate-optimize level_hold 20 5
```

Otherwise resolve `<TYPE>` through the table above, state the mapped task, model
subdirectory, default reward config, and metric family, and continue to the
budget step.

---

## Budget, batching, and defaults

Invocation is `<type> [total_runs] [runs_per_batch]`, positionally, but also
recognize the same quantities from free-text wording anywhere in the
arguments — the user will often phrase this conversationally (e.g. "run up to
20, but do 5 in parallel at a time") rather than as bare positional numbers.
Resolve two independent quantities:

- **`total_runs`** — the total experiment budget, **including the baseline**.
  From an explicit first number, or wording like `max experiments`, `limit`,
  `budget`, `run up to N`, `N total runs`. Default: `6`.
- **`runs_per_batch`** — how many training runs to run *concurrently* per
  batch. From an explicit second number, or wording like `N parallel`,
  `N at a time`, `batches of N`, `N-wide`. Default: `1` (sequential — the
  original, one-run-at-a-time loop; every rule below that mentions "batch"
  degenerates to a single run when `runs_per_batch` is 1, so sequential mode
  needs no separate code path). If the user asks for parallelism without
  giving a number, default `runs_per_batch` to `5`.
- If the user instead phrases it as "N batches of M" (batch count × batch
  size) with no total given, set `runs_per_batch = M` and
  `total_runs = N * M`.

Also unless supplied:

- `--steps 2000000`.
- `--episodes 64`.
- Default task reward config.
- Default task PPO config (the task's tuned `assets/training/<task>.ppo.ron` if
  one exists — currently only `level_hold` — else compiled defaults; i.e. no
  `--ppo-config` flag).
- `level_hold` only: default target envelope (`--target-alt-range 500:5000
  --target-speed-range 90:140`) — i.e. no `--target-alt-range`/`--target-speed-range`
  flags unless the user's hypothesis is specifically about narrowing/widening the
  trained envelope.

Reject `0`, negative, or non-numeric values for `total_runs` or
`runs_per_batch`. Clamp `runs_per_batch` down to `total_runs` if it's larger
(one batch, fewer than requested wide). If `runs_per_batch` doesn't evenly
divide `total_runs`, the **last batch is smaller** — e.g. 20 total at 5/batch
is four batches of 5; 22 total at 5/batch is four batches of 5 plus one batch
of 2. Never round `total_runs` to fit the batch size.

If `total_runs` is `1`, run only the baseline train/eval and report that no
follow-up fit the budget (this is unaffected by `runs_per_batch`).

State the resolved `total_runs` and `runs_per_batch` (and derived batch count)
back to the user before starting Step 1.

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

## Step 3 — Train and evaluate one batch

A **batch** is `runs_per_batch` training runs launched concurrently, each into
its own stem, followed by evaluating every checkpoint the batch produced. In
sequential mode (`runs_per_batch = 1`) a batch is just one run — everything
below still applies, only the concurrency mechanics are moot.

**Batch 1 always starts with the baseline** (default reward config, default
PPO config, no `--init-from`) as one of its `runs_per_batch` slots. If
`runs_per_batch > 1`, fill the remaining slots in batch 1 with distinct no-code
PPO-hyperparameter variants (§ Step 4, rule 2) — don't waste batch-1
concurrency running only the baseline while cores sit idle.

Train each run in the batch with the same command shape, each to a fresh
stem and its own log file:

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
  [--target-alt-range <MIN:MAX>] [--target-speed-range <MIN:MAX>] \
  --log-file target/skill-runs/<run>/<experiment>_train.csv
```

`--target-alt-range`/`--target-speed-range` apply to `level_hold` only (ignored by other
tasks): the per-episode target-altitude/airspeed envelope the policy is trained across
(default `500:5000` / `90:140`). Leave unset unless the experiment is deliberately
narrowing/widening the envelope — every run in a batch being compared should use the same
envelope.

**Launching a batch concurrently:** build the `train_ppo` and
`evaluate_policy` binaries once up front (`cargo build --release ...`) so the
first run in a batch doesn't win a cargo build lock race against the rest.
Then run all of a batch's training commands as background shell jobs and wait
for all of them, e.g. one script backgrounding each command with `&` and
`wait`-ing at the end, launched as a single background task so the whole batch
reports back as one completion. Throttle each process's thread pool so
`runs_per_batch` concurrent processes don't oversubscribe the machine — `burn`'s
ndarray backend uses `rayon`, which honors `RAYON_NUM_THREADS`:

```bash
RAYON_NUM_THREADS=$(( $(nproc) / runs_per_batch > 0 ? $(nproc) / runs_per_batch : 1 ))
```

set as an env var on each training (and evaluation) invocation in the batch.

Evaluate every checkpoint the batch produced, also concurrently, with the same
throttling:

```bash
cargo run --release --no-default-features --features inference --bin evaluate_policy -- \
  --task <task> \
  --backend ndarray \
  --model models/<model_dir>/<stem> \
  --episodes <episodes> \
  [--target-alt-range <MIN:MAX>] [--target-speed-range <MIN:MAX>] \
  > target/skill-runs/<run>/<experiment>_eval.txt
```

`.mpk` or no extension are both accepted — be consistent. If an experiment
used a modified `--reward-config`, pass that **same** config file to its
evaluation too (its raw `mean_return` won't be comparable in magnitude to runs
under the default reward — compare its absolute tracking-error metrics
instead, and say so in the report). Likewise for `level_hold`: if training used
non-default `--target-alt-range`/`--target-speed-range`, evaluate with the **same**
ranges — the eval output echoes back `target_alt_range`/`target_speed_range` so this is
easy to check post hoc.

Read each eval output and inspect the metrics for the resolved metric family.
The common core is always present; the extras depend on the family:

- **Common core (every task):** `success_rate`, `mean_return`,
  `mean_length_steps`.
- **Orbit family** (`orbit`, `residual_orbit`, `lstm_orbit`):
  `mean_abs_radial_m`, `mean_abs_heading_rad`, `mean_abs_altitude_m`,
  `mean_abs_speed_mps`, plus the same four `mean_tail_abs_*` (settled final-20%-of-episode
  window) and `mean_final_abs_radial_m`, `mean_final_abs_altitude_m`.
- **LevelHold family** (`level_hold`): `mean_abs_altitude_m`,
  `mean_abs_speed_mps`, `mean_abs_roll_rad`, `mean_abs_beta_rad`, the same four as
  `mean_tail_abs_altitude_m`/`mean_tail_abs_speed_mps`/`mean_tail_abs_roll_rad`/
  `mean_tail_abs_beta_rad` (settled final-20%-of-episode window — usually the more
  relevant number for a steady-state-tracking goal, since the whole-episode average
  includes the spawn-offset transient), and `mean_final_abs_altitude_m` (no
  radial/heading metrics). `level_hold` additionally echoes `target_alt_range` /
  `target_speed_range` so a report can confirm which envelope was evaluated.

## Step 4 — Analyze the batch and design the next one

Repeat Step 3 with a freshly designed batch until `total_runs` is exhausted or
the early-stop rule (below) applies. **Evaluate the stop rule only at a batch
boundary** — finish and analyze the current batch before deciding whether to
stop, never abandon a batch partway through. Use the **same** `--episodes`,
optional `--max-steps`, and reward-config policy for every evaluation across
every batch so runs stay comparable.

Build a per-batch leaderboard (same columns as the final report), and update
the running incumbent using the improvement rule against the *best result seen
in any prior batch*, not just the previous batch.

Every run within a batch must be a **genuinely distinct experiment** — never
fill batch slots with repeats of the same config for noise-averaging (`burn`'s
module-init RNG is unseeded, so re-running an identical config *will* produce
a different result, but that's a code-required fix — see the stop rule — not
something to paper over by spending budget on duplicates).

For each run, write a short hypothesis and choose **exactly one** experiment
direction per run:

1. **Early**, when no dominant failure is clear (typically batch 1, and batch
   2 if batch 1 was inconclusive), explore distinct no-code vectors across the
   batch's runs — small PPO config changes such as `lr`, `rollout_steps`,
   `n_epochs`, `minibatch`, `entropy_coef`, or `clip_epsilon`, each varied on
   its own axis. **For `lstm_orbit` the recurrent trainer ignores
   `--ppo-config`**, so this vector is unavailable — explore reward-config and
   `--init-from` vectors instead.
2. **Once a signal appears**, dial in: combine the winning knobs from prior
   batches, continue the best checkpoint with `--init-from`, or copy and
   target the task's reward config when its family's tracking-error or
   stability metrics dominate the failure (radial/heading/altitude/speed for
   the Orbit family; altitude/speed/roll/beta for the LevelHold family). A
   batch in this phase can mix directions across its runs (e.g. one
   continuation run, one reward-config run, two knob-combination runs) — it no
   longer needs to be single-vector like early exploration.
3. If a run **fails outright** (diverges, crashes, success rate collapses),
   don't just discard the config — if it combined two or more changed knobs,
   spend slots in the *next* batch isolating each knob individually to learn
   whether the failure was one knob or an interaction between them.
4. For `residual_orbit` **only**, allow modest `residual_scale` experiments
   from a copied reward config.
5. Do **not** repeat a failed vector unless the next variant is narrower and
   metric-backed.
6. Naive continuation (`--init-from` the incumbent with no other change) is a
   valid hypothesis but is not guaranteed to help — treat a regression from it
   as a real (if mildly surprising) result, not a bug, and don't retry it
   unmodified.

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

Stop **before** the experiment limit only when **all** are true, checked at a
batch boundary (never mid-batch):

- Baseline plus at least three follow-up runs have run (in batch mode this can
  be satisfied within batch 1 alone if `runs_per_batch >= 4`, but still finish
  and analyze that whole batch before stopping).
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

- Exact `<TYPE>` normalization, task, model directory, metric family,
  `total_runs`, `runs_per_batch` (and derived batch count), steps, episodes,
  reward configs, PPO configs, and checkpoint paths.
- A **compact leaderboard** of all experiments (sorted by `mean_return`, noting
  which batch each ran in) with success, return, length, and the task's
  tracking-error metrics for its metric family (radial/heading/altitude/speed +
  final radial/altitude for Orbit; altitude/speed/roll/beta + final altitude
  for LevelHold). Flag any run evaluated under a non-default reward config
  since its `mean_return` isn't directly comparable.
- The incumbent model and whether performance improved, and over how many
  batches it held.
- Each experiment's hypothesis and whether the result supported it — call out
  any batch that revealed a knob-interaction failure (a combo that failed
  where its isolated components didn't, or vice versa).
- Why the loop stopped and the next realistic no-code or code-required step.

If a training or evaluation command fails, preserve the error, explain what
failed, and do **not** silently change controller type or toolchain.
