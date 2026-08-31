"""`ml_planes.EvalRun` — the shared evaluation accumulator.

The metric mapping, the settled-tail window, the success criterion and the core
aggregation used to exist twice: privately inside `src/bin/evaluate_policy.rs`,
and hand-copied into `train_ppo_torch.py::evaluate_checkpoint`. The copies
disagreed — most visibly by sampling different episode sets, which put ~15%
between the two evaluators' `mean_return` for one and the same policy.

`EvalRun` wraps `ml_planes::training::eval::EvalRun`, so both stacks now compute
the report with the same Rust code. `test_parity_with_the_rust_evaluator` is what
makes that a test rather than a claim: it drives the Python side with one env per
episode, seeded absolutely, against a Rust reference that resets a single env —
`evaluate_policy`'s own scheme. Agreement proves the seeding alignment as well as
the shared arithmetic.
"""

import json
import os
import pathlib
import subprocess

import numpy as np
import pytest

import ml_planes

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
MANIFEST = REPO_ROOT / "bindings" / "python" / "Cargo.toml"

# `ml_planes::training::eval::TAIL_FRACTION`
TAIL_FRACTION = 0.2

TASK_DIMS = {
    "level_hold": 13,
    "heading_hold": 16,
    "orbit": 14,
    "residual_orbit": 14,
    "lstm_orbit": 14,
}


def obs_for(task: str, **fields: float) -> np.ndarray:
    """A zeroed observation of the right width, with named indices set."""
    obs = np.zeros(TASK_DIMS[task], dtype=np.float32)
    for index, value in fields.items():
        obs[int(index[1:])] = value
    return obs


def test_module_exports_eval_run():
    assert "EvalRun" in ml_planes.__all__


@pytest.mark.parametrize("task", sorted(TASK_DIMS))
def test_every_task_builds_a_run(task):
    run = ml_planes.EvalRun(task, episodes=2, max_steps=10)
    assert run.task == task
    assert run.finished_episodes == 0
    assert run.is_complete is False


def test_a_full_length_episode_reports_success_and_exact_means():
    # max_steps=10 -> tail_start=8, so steps 9 and 10 are the tail window.
    run = ml_planes.EvalRun("level_hold", episodes=1, max_steps=10)
    for step in range(1, 11):
        scale = 0.5 if step > 8 else 1.0
        run.record(0, obs_for("level_hold", i0=scale, i1=scale), 1.0)
    run.finish(0, obs_for("level_hold", i0=0.25))

    assert run.is_complete is True
    report = run.report()

    assert report["episodes"] == 1
    assert report["success_rate"] == 1.0
    assert report["mean_return"] == 10.0
    assert report["mean_length_steps"] == 10.0
    # (8 * 1.0 + 2 * 0.5) / 10 = 0.9, scaled by 200 m.
    assert report["mean_abs_altitude_m"] == pytest.approx(0.9 * 200.0, abs=1e-4)
    # Tail is the two half-error steps only.
    assert report["mean_tail_abs_altitude_m"] == pytest.approx(100.0, abs=1e-4)
    assert report["mean_final_abs_altitude_m"] == pytest.approx(50.0, abs=1e-4)


def test_report_keys_are_ordered_core_then_metrics():
    run = ml_planes.EvalRun("level_hold", episodes=1, max_steps=4)
    for _ in range(4):
        run.record(0, obs_for("level_hold"), 0.0)
    run.finish(0, obs_for("level_hold"))
    keys = list(run.report())
    assert keys[:4] == [
        "episodes",
        "success_rate",
        "mean_return",
        "mean_length_steps",
    ]
    # Then per-step means, then tail means, then final means — the same order
    # `evaluate_policy` prints its rows in.
    assert keys[4] == "mean_abs_altitude_m"
    assert "mean_tail_abs_altitude_m" in keys
    assert keys[-1] == "mean_final_abs_altitude_m"


def test_an_episode_cut_short_is_not_a_success():
    run = ml_planes.EvalRun("level_hold", episodes=2, max_steps=10, slots=1)
    for _ in range(10):
        run.record(0, obs_for("level_hold"), 1.0)
    run.finish(0, obs_for("level_hold"))
    for _ in range(4):
        run.record(0, obs_for("level_hold"), 1.0)
    run.finish(0, obs_for("level_hold"))

    report = run.report()
    assert report["success_rate"] == 0.5
    assert report["mean_length_steps"] == 7.0


def test_slots_default_to_one_per_episode():
    """The natural Python shape is one in-flight episode per env."""
    run = ml_planes.EvalRun("level_hold", episodes=3, max_steps=5)
    # Slot 2 is addressable without passing `slots` explicitly.
    run.record(2, obs_for("level_hold"), 0.0)
    run.finish(2, obs_for("level_hold"))
    assert run.finished_episodes == 1


def test_report_returns_plain_floats():
    run = ml_planes.EvalRun("orbit", episodes=1, max_steps=2)
    for _ in range(2):
        run.record(0, obs_for("orbit", i0=1.0), 0.0)
    run.finish(0, obs_for("orbit", i0=1.0))
    report = run.report()
    assert isinstance(report["episodes"], int)
    assert isinstance(report["success_rate"], float)
    assert isinstance(report["mean_abs_radial_m"], float)
    # Orbit family, so no level-hold rows.
    assert "mean_abs_beta_rad" not in report


def test_accepts_a_float64_observation():
    run = ml_planes.EvalRun("level_hold", episodes=1, max_steps=1)
    run.record(0, np.zeros(13, dtype=np.float64), 0.0)
    run.finish(0, np.zeros(13, dtype=np.float64))
    assert run.is_complete is True


# ---------------------------------------------------------------------------
# Misuse must raise, never panic. A Rust panic surfaces as an uncatchable
# PanicException with a backtrace, which is a bug in the binding.
# ---------------------------------------------------------------------------


def test_unknown_task_raises_value_error():
    with pytest.raises(ValueError, match="barrel_roll"):
        ml_planes.EvalRun("barrel_roll", episodes=1, max_steps=1)


def test_zero_episodes_raises_value_error():
    with pytest.raises(ValueError, match="at least one"):
        ml_planes.EvalRun("level_hold", episodes=0, max_steps=10)


def test_zero_max_steps_raises_value_error():
    with pytest.raises(ValueError, match="max_steps"):
        ml_planes.EvalRun("level_hold", episodes=1, max_steps=0)


def test_out_of_range_slot_raises_index_error_not_a_panic():
    run = ml_planes.EvalRun("level_hold", episodes=1, max_steps=10, slots=2)
    with pytest.raises(IndexError):
        run.record(2, obs_for("level_hold"), 0.0)
    with pytest.raises(IndexError):
        run.finish(5, obs_for("level_hold"))


def test_a_short_observation_raises_value_error():
    run = ml_planes.EvalRun("level_hold", episodes=1, max_steps=10)
    with pytest.raises(ValueError, match="too short"):
        run.record(0, np.zeros(4, dtype=np.float32), 0.0)


def test_finishing_an_empty_episode_raises_value_error():
    run = ml_planes.EvalRun("level_hold", episodes=1, max_steps=10)
    with pytest.raises(ValueError, match="no recorded steps"):
        run.finish(0, obs_for("level_hold"))


def test_recording_past_the_declared_episode_count_raises():
    run = ml_planes.EvalRun("level_hold", episodes=1, max_steps=10)
    run.record(0, obs_for("level_hold"), 0.0)
    run.finish(0, obs_for("level_hold"))
    with pytest.raises(ValueError, match="already been finished"):
        run.record(0, obs_for("level_hold"), 0.0)


def test_reporting_early_raises_rather_than_understating_the_means():
    run = ml_planes.EvalRun("level_hold", episodes=2, max_steps=10)
    run.record(0, obs_for("level_hold"), 0.0)
    run.finish(0, obs_for("level_hold"))
    with pytest.raises(ValueError, match="not ready"):
        run.report()


# ---------------------------------------------------------------------------
# Parity with the Rust evaluator.
# ---------------------------------------------------------------------------


def reference_eval(task: str, episodes: int, max_steps: int) -> dict:
    """Run the Rust reference evaluator and parse its JSON report."""
    env = dict(os.environ, CARGO_TARGET_DIR=str(REPO_ROOT / "target"))
    result = subprocess.run(
        [
            "cargo", "run", "--quiet",
            "--manifest-path", str(MANIFEST),
            "--example", "reference_eval",
            "--",
            "--task", task,
            "--episodes", str(episodes),
            "--max-steps", str(max_steps),
        ],
        cwd=REPO_ROOT,  # the default .plane.ron path is repo-root-relative
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        pytest.fail(f"reference_eval failed:\n{result.stderr}")
    return json.loads(result.stdout)


def python_eval(task: str, episodes: int, max_steps: int, base_seed: int) -> dict:
    """The Python evaluator's shape: one env per episode, seeded absolutely.

    Episode `i` must draw from `base_seed + i + 1`, because `reset()` advances
    the seed before drawing — so seeding to `base_seed + i` reproduces the
    episode a single env would have reached on its (i+1)-th reset.

    `slots=1` keeps the float summation order identical to the sequential Rust
    loop, which is what makes `==` a fair comparison below.
    """
    run = ml_planes.EvalRun(task, episodes=episodes, max_steps=max_steps, slots=1)
    action = np.zeros(4, dtype=np.float32)
    for i in range(episodes):
        env = ml_planes.Env(task, max_episode_steps=max_steps)
        env.seed(base_seed + i)
        obs, _ = env.reset()
        steps, done = 0, False
        while not done and steps < max_steps:
            obs, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated
            steps += 1
            run.record(0, obs, reward)
        run.finish(0, obs)
    return run.report()


@pytest.mark.parametrize("task", sorted(TASK_DIMS))
@pytest.mark.parametrize(
    "max_steps",
    # 200: a zero-action policy survives the whole budget, so this covers
    # truncation, success, and the tail window (steps 161..200).
    # 3200: the same policy crashes at ~1500 steps, covering the failure path
    # and an episode that contributes no tail samples at all.
    [200, 3200],
    ids=["survives-the-budget", "crashes-early"],
)
def test_parity_with_the_rust_evaluator(task, max_steps):
    episodes = 4
    reference = reference_eval(task, episodes, max_steps)

    # The base seed is discovered, not hardcoded: it is 42 for the level-hold
    # family and 4242 for the orbit family, and a hardcoded literal would
    # silently mis-align three of the five tasks.
    env = ml_planes.Env(task, max_episode_steps=max_steps)
    assert env.rng_seed == reference["base_seed"]

    mine = python_eval(task, episodes, max_steps, reference["base_seed"])

    assert mine["episodes"] == reference["episodes"]
    # Exact equality on purpose: both sides run identical Rust over identical
    # episodes, so a tolerance here could only hide the divergence this test
    # exists to catch.
    for key, value in mine.items():
        assert value == reference[key], f"{task}/{max_steps}: {key}"
