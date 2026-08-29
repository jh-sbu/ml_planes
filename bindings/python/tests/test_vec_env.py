"""The batched binding surface.

`VecEnv` is what a PyTorch PPO loop actually drives, so the assertions here are
the ones that would otherwise fail silently: the env-major action layout, the
`ENV_SEED_STRIDE` spacing, and the deliberate absence of auto-reset.
"""

import numpy as np
import pytest

import ml_planes

ENV_SEED_STRIDE = 1000  # ml_planes::training::ENV_SEED_STRIDE


def zeros(n, a=4):
    return np.zeros((n, a), dtype=np.float32)


def test_shape_and_dims():
    venv = ml_planes.VecEnv("level_hold", 4)
    assert venv.num_envs == 4
    assert venv.observation_dim == 13
    assert venv.action_dim == 4
    assert venv.task == "level_hold"


def test_reset_returns_a_batched_observation_and_info():
    venv = ml_planes.VecEnv("orbit", 3)
    obs, info = venv.reset()

    assert obs.dtype == np.float32
    assert obs.shape == (3, 14)
    assert np.isfinite(obs).all()
    assert isinstance(info, dict)


def test_reset_info_stacks_the_spawn_specs():
    obs, info = ml_planes.VecEnv("level_hold", 3).reset()
    spawn = info["spawn"]
    assert spawn["position"].shape == (3, 3)
    assert spawn["attitude"].shape == (3, 4)
    assert spawn["fuel_fraction"].shape == (3,)


def test_step_returns_batched_arrays_with_the_documented_dtypes():
    venv = ml_planes.VecEnv("level_hold", 4)
    venv.reset()
    obs, rewards, terminated, truncated, info = venv.step(zeros(4))

    assert obs.shape == (4, 13) and obs.dtype == np.float32
    assert rewards.shape == (4,) and rewards.dtype == np.float32
    assert terminated.shape == (4,) and terminated.dtype == np.bool_
    assert truncated.shape == (4,) and truncated.dtype == np.bool_
    assert info["episode_step"].shape == (4,)
    assert info["episode_step"].dtype == np.uint32
    assert (info["episode_step"] == 1).all()


def test_step_accepts_a_float64_action_batch():
    venv = ml_planes.VecEnv("level_hold", 2)
    venv.reset()
    obs, *_ = venv.step(np.zeros((2, 4), dtype=np.float64))
    assert obs.shape == (2, 13)


def test_step_rejects_a_mis_shaped_action_batch():
    venv = ml_planes.VecEnv("level_hold", 4)
    venv.reset()
    with pytest.raises(ValueError, match=r"\(4, 4\)"):
        venv.step(zeros(3))


def test_seeding_spaces_sub_envs_by_one_stride():
    """Pins ENV_SEED_STRIDE across the language boundary: sub-env i must be the
    standalone Env seeded i strides past the base."""
    venv = ml_planes.VecEnv("level_hold", 3, seed=500)
    batch, _ = venv.reset()

    for i in range(3):
        solo = ml_planes.Env("level_hold")
        solo.seed(500 + i * ENV_SEED_STRIDE)
        assert np.array_equal(batch[i], solo.reset()[0]), f"sub-env {i} diverged"


def test_the_same_base_seed_reproduces_the_whole_batch():
    a = ml_planes.VecEnv("level_hold", 3)
    b = ml_planes.VecEnv("level_hold", 3)
    a.reset()
    a.reset()
    assert np.array_equal(a.reset(seed=77)[0], b.reset(seed=77)[0])


def test_seed_at_leaves_the_other_sub_envs_alone():
    venv = ml_planes.VecEnv("level_hold", 3, seed=500)
    venv.seed_at(1, 9)
    batch, _ = venv.reset()

    for i, expected_seed in [(0, 500), (1, 9), (2, 500 + 2 * ENV_SEED_STRIDE)]:
        solo = ml_planes.Env("level_hold")
        solo.seed(expected_seed)
        assert np.array_equal(batch[i], solo.reset()[0]), f"sub-env {i}"


def test_actions_are_sliced_env_major():
    """A transposed buffer would train every env on another env's actions and
    never raise. Each row must land on its own env."""
    venv = ml_planes.VecEnv("level_hold", 2, seed=7)
    venv.reset()
    actions = np.array([[1.0, 0.5, 0.0, 0.0], [-1.0, -0.5, 0.0, 0.0]], dtype=np.float32)
    batch, *_ = venv.step(actions)

    for i, row in enumerate(actions):
        solo = ml_planes.Env("level_hold")
        solo.seed(7 + i * ENV_SEED_STRIDE)
        solo.reset()
        assert np.array_equal(batch[i], solo.step(row)[0]), f"env {i} got the wrong row"


def test_reset_at_resets_only_that_env():
    venv = ml_planes.VecEnv("level_hold", 3, seed=1)
    before, _ = venv.reset()
    venv.step(zeros(3))

    obs, info = venv.reset_at(1)
    assert obs.shape == (13,)
    assert "spawn" in info

    # env 1 restarted, so its next step is episode_step 1 again; the others are 2.
    _, _, _, _, step_info = venv.step(zeros(3))
    assert list(step_info["episode_step"]) == [2, 1, 2]


def test_reset_at_takes_a_seed():
    venv = ml_planes.VecEnv("level_hold", 2)
    solo = ml_planes.Env("level_hold")
    assert np.array_equal(venv.reset_at(0, seed=64)[0], solo.reset(seed=64)[0])


def test_out_of_range_index_raises_index_error_not_a_panic():
    """VecEnv's indexed accessors panic in Rust; a PanicException is neither
    catchable as IndexError nor readable."""
    venv = ml_planes.VecEnv("level_hold", 2)
    with pytest.raises(IndexError):
        venv.reset_at(5)
    with pytest.raises(IndexError):
        venv.seed_at(5, 1)


def test_negative_indices_count_from_the_end():
    """Seeded absolutely on both calls, since each reset advances the sub-env's
    seed and two bare resets of the same env draw different episodes."""
    venv = ml_planes.VecEnv("level_hold", 3, seed=1)
    assert np.array_equal(venv.reset_at(-1, seed=5)[0], venv.reset_at(2, seed=5)[0])
    with pytest.raises(IndexError):
        venv.reset_at(-4)


def test_zero_envs_is_a_value_error_not_a_panic():
    with pytest.raises(ValueError, match="at least one"):
        ml_planes.VecEnv("level_hold", 0)


def test_there_is_no_auto_reset():
    """Deliberate, and load-bearing: the terminal observation must stay readable
    so a truncated episode's value can be bootstrapped before the reset."""
    venv = ml_planes.VecEnv("level_hold", 1, seed=3)
    venv.reset()

    dive = np.array([[-1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
    for _ in range(4000):
        _, _, terminated, truncated, _ = venv.step(dive)
        if terminated[0] or truncated[0]:
            break
    else:
        pytest.fail("a full-nose-down level-hold episode should end")

    # Still done on the next step: nothing reset it behind our back.
    _, _, terminated_again, truncated_again, _ = venv.step(dive)
    assert terminated_again[0] or truncated_again[0]


def test_vec_env_is_exported_from_the_package():
    assert "VecEnv" in ml_planes.__all__
