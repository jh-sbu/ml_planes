"""Smoke test for the reference PyTorch PPO loop.

Deliberately tiny: this asserts the loop *runs, learns something, and writes a
checkpoint* through the real bindings, not that PPO converges. Convergence and
policy quality belong to the `evaluate_policy` / train-evaluate-optimize
workflows, which run full-length training and are not part of a test suite.
"""

import pathlib
import subprocess
import sys

import pytest

torch = pytest.importorskip("torch")

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "bindings" / "python" / "examples" / "train_ppo_torch.py"


def run(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(SCRIPT), *args],
        cwd=REPO_ROOT,  # reward/plane profiles are repo-root-relative paths
        capture_output=True,
        text=True,
        check=False,
    )


def test_the_loop_runs_and_writes_a_checkpoint(tmp_path):
    out = tmp_path / "level_hold.pt"
    result = run(
        "--task", "level_hold",
        "--num-envs", "4",
        "--rollout-steps", "64",
        "--total-steps", "1024",
        "--minibatch-size", "128",
        "--out", str(out),
    )
    assert result.returncode == 0, result.stderr
    assert out.exists()

    ckpt = torch.load(out, weights_only=False)
    assert ckpt["task"] == "level_hold"
    assert ckpt["observation_dim"] == 13
    assert ckpt["action_dim"] == 4
    assert "model" in ckpt


def test_mlp_hidden_layers_are_configurable_and_saved(tmp_path):
    out = tmp_path / "level_hold_3x64.pt"
    result = run(
        "--task", "level_hold",
        "--num-envs", "2",
        "--rollout-steps", "8",
        "--total-steps", "16",
        "--minibatch-size", "16",
        "--hidden", "64",
        "--layers", "3",
        "--out", str(out),
    )
    assert result.returncode == 0, result.stderr

    ckpt = torch.load(out, weights_only=False)
    assert ckpt["hidden"] == 64
    assert ckpt["layers"] == 3
    hidden_weights = [
        key for key in ckpt["model"]
        if key.startswith("actor.") and key.endswith(".weight")
    ]
    assert len(hidden_weights) == 4  # three hidden layers plus the action head


def test_eval_reconstructs_a_custom_depth_checkpoint(tmp_path):
    out = tmp_path / "level_hold_1x64.pt"
    trained = run(
        "--task", "level_hold",
        "--num-envs", "2",
        "--rollout-steps", "8",
        "--total-steps", "16",
        "--minibatch-size", "16",
        "--hidden", "64",
        "--layers", "1",
        "--out", str(out),
    )
    assert trained.returncode == 0, trained.stderr

    evaluated = run(
        "--eval-only", str(out),
        "--eval-episodes", "1",
        "--eval-max-steps", "4",
    )
    assert evaluated.returncode == 0, evaluated.stderr


def test_transformer_hyperparameters_are_configurable_and_saved(tmp_path):
    out = tmp_path / "level_hold_transformer.pt"
    result = run(
        "--task", "level_hold",
        "--arch", "transformer",
        "--num-envs", "2",
        "--rollout-steps", "8",
        "--total-steps", "16",
        "--minibatch-size", "16",
        "--transformer-d-model", "24",
        "--transformer-heads", "3",
        "--transformer-depth", "1",
        "--transformer-ff-mult", "3",
        "--transformer-tokenizer", "per_feature",
        "--transformer-pooling", "cls",
        "--out", str(out),
    )
    assert result.returncode == 0, result.stderr

    ckpt = torch.load(out, weights_only=False)
    assert ckpt["args"]["transformer_d_model"] == 24
    assert ckpt["args"]["transformer_heads"] == 3
    assert ckpt["args"]["transformer_depth"] == 1
    assert ckpt["args"]["transformer_ff_mult"] == 3
    assert ckpt["args"]["transformer_tokenizer"] == "per_feature"
    assert ckpt["args"]["transformer_pooling"] == "cls"
    assert ckpt["model"]["actor.0.feature_weight"].shape == (13, 24)
    assert ckpt["model"]["actor.0.cls_token"].shape == (1, 1, 24)

    evaluated = run(
        "--eval-only", str(out),
        "--eval-episodes", "1",
        "--eval-max-steps", "4",
    )
    assert evaluated.returncode == 0, evaluated.stderr


def test_transformer_defaults_use_per_feature_tokens_and_cls_pooling(tmp_path):
    out = tmp_path / "level_hold_transformer_default.pt"
    result = run(
        "--task", "level_hold",
        "--arch", "transformer",
        "--num-envs", "2",
        "--rollout-steps", "8",
        "--total-steps", "16",
        "--minibatch-size", "16",
        "--out", str(out),
    )
    assert result.returncode == 0, result.stderr

    ckpt = torch.load(out, weights_only=False)
    assert ckpt["args"]["transformer_tokenizer"] == "per_feature"
    assert ckpt["args"]["transformer_pooling"] == "cls"
    assert "actor.0.feature_weight" in ckpt["model"]
    assert "actor.0.cls_token" in ckpt["model"]


def test_the_ppo_ratio_is_sane_on_the_first_update():
    """A policy loss far from 0 on update 1 means the stored action and its
    stored log-prob disagree — the ratio is then measuring a marshalling bug
    rather than a policy change."""
    result = run(
        "--task", "level_hold",
        "--num-envs", "4",
        "--rollout-steps", "64",
        "--total-steps", "256",
        "--minibatch-size", "128",
    )
    assert result.returncode == 0, result.stderr

    first = next(l for l in result.stdout.splitlines() if l.startswith("update    1"))
    policy_loss = float(first.split("policy")[1].split()[0])
    assert abs(policy_loss) < 0.5, f"suspicious first-update policy loss: {first}"


def test_it_accepts_every_task():
    """Each task has its own obs width; the loop must not have level_hold baked in."""
    import ml_planes

    for task in ml_planes.TASKS:
        result = run(
            "--task", task,
            "--num-envs", "2",
            "--rollout-steps", "8",
            "--total-steps", "16",
            "--minibatch-size", "16",
        )
        assert result.returncode == 0, f"{task} failed:\n{result.stderr}"
