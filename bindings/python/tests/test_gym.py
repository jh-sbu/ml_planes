"""The Gymnasium adapter.

Skipped wholesale when gymnasium is absent — that is the point of the adapter
living in its own module with a lazy import: `ml_planes.Env`/`VecEnv` must work
without it.
"""

import numpy as np
import pytest

import ml_planes

gymnasium = pytest.importorskip("gymnasium")

from ml_planes.gym import MlPlanesEnv, register  # noqa: E402


def test_core_package_does_not_import_gymnasium():
    """The adapter is optional, so importing `ml_planes` must not pull it in."""
    import subprocess
    import sys

    code = "import sys, ml_planes; assert 'gymnasium' not in sys.modules"
    assert subprocess.run([sys.executable, "-c", code]).returncode == 0


def test_it_is_a_real_gymnasium_env():
    assert isinstance(MlPlanesEnv("level_hold"), gymnasium.Env)


@pytest.mark.parametrize("task", sorted(ml_planes.TASKS))
def test_spaces_match_the_env_dims(task):
    env = MlPlanesEnv(task)
    assert env.observation_space.shape == (env.unwrapped.env.observation_dim,)
    assert env.observation_space.dtype == np.float32
    # Every task uses the 4-channel [-1, 1] direct action space.
    assert env.action_space.shape == (4,)
    assert env.action_space.dtype == np.float32
    assert (env.action_space.low == -1.0).all()
    assert (env.action_space.high == 1.0).all()


def test_reset_and_step_follow_the_gymnasium_api():
    env = MlPlanesEnv("level_hold")
    obs, info = env.reset(seed=3)
    assert env.observation_space.contains(obs)
    assert "spawn" in info

    obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
    assert env.observation_space.contains(obs)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)


def test_reset_seed_is_reproducible():
    a, _ = MlPlanesEnv("level_hold").reset(seed=17)
    b, _ = MlPlanesEnv("level_hold").reset(seed=17)
    assert np.array_equal(a, b)


def test_passes_the_gymnasium_env_checker():
    from gymnasium.utils.env_checker import check_env

    # skip_render_check: the sandbox has no renderer and never will (CLAUDE.md
    # scope: rendering lives in the Bevy client, not the training envs).
    check_env(MlPlanesEnv("level_hold"), skip_render_check=True)


def test_kwargs_reach_the_underlying_env():
    env = MlPlanesEnv("heading_hold", target_alt_range=(1200.0, 1200.0))
    assert env.unwrapped.env.observation_dim == 16


def test_register_makes_gymnasium_make_work():
    register()
    env = gymnasium.make("MlPlanes/LevelHold-v0")
    obs, _ = env.reset(seed=1)
    assert obs.shape == (13,)
    env.close()


def test_register_covers_every_task():
    register()
    for task in ml_planes.TASKS:
        env_id = f"MlPlanes/{''.join(p.title() for p in task.split('_'))}-v0"
        assert env_id in gymnasium.registry, f"{task} is not registered"
