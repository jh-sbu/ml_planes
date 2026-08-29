"""The single-environment binding surface.

`Env` marshals `ml_planes::training`'s erased `Box<dyn TrainingEnv>`. Everything
asserted here is about the *boundary* — shapes, dtypes, the Gymnasium-shaped
return tuples, error types, and that constructor kwargs actually reach `EnvSpec`.
Whether the environment itself behaves is `test_parity.py`'s job, and it proves
it by diffing against a Rust rollout rather than by restating physics here.
"""

import numpy as np
import pytest

import ml_planes

TASK_DIMS = {
    "level_hold": 13,
    "heading_hold": 16,
    "orbit": 14,
    "residual_orbit": 14,
    "lstm_orbit": 14,
}


def test_tasks_tuple_matches_the_rust_registry():
    # Task::ALL — Python must never hardcode the five names.
    assert set(ml_planes.TASKS) == set(TASK_DIMS)


@pytest.mark.parametrize("task,obs_dim", sorted(TASK_DIMS.items()))
def test_every_task_builds_at_its_documented_dims(task, obs_dim):
    env = ml_planes.Env(task)
    assert env.observation_dim == obs_dim
    assert env.action_dim == 4
    assert env.task == task


def test_reset_returns_observation_and_info():
    env = ml_planes.Env("level_hold")
    obs, info = env.reset()

    assert isinstance(obs, np.ndarray)
    assert obs.dtype == np.float32
    assert obs.shape == (13,)
    assert np.isfinite(obs).all()
    assert isinstance(info, dict)


def test_reset_info_carries_the_spawn_spec():
    """The SpawnSpec half of Rust's `reset()` tuple, which the scaffold discarded."""
    _, info = ml_planes.Env("level_hold").reset()
    spawn = info["spawn"]

    assert len(spawn["position"]) == 3
    assert len(spawn["velocity"]) == 3
    assert len(spawn["attitude"]) == 4, "quaternion is [x, y, z, w]"
    assert len(spawn["angular_velocity"]) == 3
    assert 0.0 <= spawn["fuel_fraction"] <= 1.0


def test_step_returns_the_gymnasium_five_tuple():
    env = ml_planes.Env("level_hold")
    env.reset()
    obs, reward, terminated, truncated, info = env.step([0.0, 0.0, 0.0, 0.0])

    assert obs.dtype == np.float32 and obs.shape == (13,)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert info["episode_step"] == 1


@pytest.mark.parametrize(
    "action",
    [
        [0.1, 0.2, 0.3, 0.4],
        np.array([0.1, 0.2, 0.3, 0.4], dtype=np.float32),
        np.array([0.1, 0.2, 0.3, 0.4], dtype=np.float64),
        (0.1, 0.2, 0.3, 0.4),
    ],
    ids=["list", "f32", "f64", "tuple"],
)
def test_step_accepts_any_sequence_or_array_dtype(action):
    """A torch loop hands over float32; a hand-written script hands over a list.
    Both must work, and a float64 array must coerce rather than raise."""
    env = ml_planes.Env("level_hold")
    env.reset()
    obs, *_ = env.step(action)
    assert np.isfinite(obs).all()


def test_step_accepts_a_non_contiguous_action_view():
    """`as_slice()` is None for a strided view, so the binding needs its fallback."""
    env = ml_planes.Env("level_hold")
    env.reset()
    strided = np.zeros(8, dtype=np.float32)[::2]
    assert not strided.flags["C_CONTIGUOUS"]
    obs, *_ = env.step(strided)
    assert obs.shape == (13,)


def test_step_rejects_a_mis_sized_action():
    env = ml_planes.Env("level_hold")
    env.reset()
    with pytest.raises(ValueError, match="expected 4 actions"):
        env.step([0.0, 0.0])


def test_unknown_task_raises_value_error():
    with pytest.raises(ValueError, match="barrel_roll"):
        ml_planes.Env("barrel_roll")


def test_unreadable_reward_profile_is_an_error_not_a_silent_default():
    """`make_env` errors where the training binaries warn: a library call has no
    operator watching stderr, so a bad profile must not fall back silently."""
    with pytest.raises(ValueError, match="reward"):
        ml_planes.Env("level_hold", reward_config="assets/training/nope.reward.ron")


def test_unreadable_plane_config_raises_rather_than_exiting():
    """Must not reach `load_plane_config_or_exit`, which would kill the interpreter."""
    with pytest.raises(ValueError, match="nope.plane.ron"):
        ml_planes.Env("level_hold", plane_config="assets/planes/nope.plane.ron")


def test_the_same_seed_reproduces_the_same_episode():
    a, b = ml_planes.Env("level_hold"), ml_planes.Env("level_hold")
    a.seed(4242)
    b.seed(4242)
    assert np.array_equal(a.reset()[0], b.reset()[0])


def test_reset_takes_a_seed_directly():
    a = ml_planes.Env("level_hold").reset(seed=99)[0]
    b = ml_planes.Env("level_hold").reset(seed=99)[0]
    c = ml_planes.Env("level_hold").reset(seed=100)[0]
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)


def test_seeding_is_absolute_not_relative():
    """`set_rng_seed` is the absolute primitive: the same seed reproduces the same
    episode however many resets came before it."""
    env = ml_planes.Env("level_hold")
    first = env.reset(seed=7)[0]
    for _ in range(3):
        env.reset()
    assert np.array_equal(env.reset(seed=7)[0], first)


def test_rng_seed_reports_back():
    env = ml_planes.Env("level_hold")
    env.seed(1234)
    assert env.rng_seed == 1234


def test_target_range_overrides_reach_the_env():
    """Pinned to two disjoint altitude bands, the reset observations must differ.
    Asserted without hardcoding the observation layout."""
    low = ml_planes.Env("level_hold", target_alt_range=(600.0, 600.0))
    high = ml_planes.Env("level_hold", target_alt_range=(4800.0, 4800.0))
    low.seed(5)
    high.seed(5)
    assert not np.array_equal(low.reset()[0], high.reset()[0])


def test_heading_range_kwarg_is_degrees():
    """EnvSpec.target_heading_range is radians and the kwarg is degrees. Pinning
    180 deg must not behave like pinning 180 rad (~57 full turns, which wraps to
    something else entirely)."""
    deg = ml_planes.Env("heading_hold", target_heading_range_deg=(180.0, 180.0))
    rad = ml_planes.Env("heading_hold", target_heading_range_deg=(np.pi, np.pi))
    deg.seed(3)
    rad.seed(3)
    assert not np.array_equal(deg.reset()[0], rad.reset()[0])


def test_orbit_geometry_overrides_reach_the_env():
    low = ml_planes.Env("orbit", orbit_altitude=800.0)
    high = ml_planes.Env("orbit", orbit_altitude=3000.0)
    low.seed(11)
    high.seed(11)
    assert not np.array_equal(low.reset()[0], high.reset()[0])


def test_orbit_radius_is_resampled_by_the_env_not_taken_from_the_kwarg():
    """Not a binding quirk: `OrbitEnv::reset` unconditionally redraws
    `target_radius` from its own 2500-4000 range (src/training/orbit_env.rs),
    discarding whatever it was constructed with. `train_ppo` sees exactly the
    same thing, so the kwarg is recorded here as inert rather than quietly
    appearing to work."""
    tight = ml_planes.Env("orbit", orbit_radius=500.0)
    wide = ml_planes.Env("orbit", orbit_radius=4000.0)
    tight.seed(11)
    wide.seed(11)
    assert np.array_equal(tight.reset()[0], wide.reset()[0])


def test_inverted_range_is_rejected():
    with pytest.raises(ValueError, match="inverted"):
        ml_planes.Env("level_hold", target_alt_range=(5000.0, 500.0))


def test_env_is_exported_from_the_package():
    """The scaffold's `_Env` was deliberately unexported because the surface was
    undesigned. It is designed now, so `Env` is the surface."""
    assert "Env" in ml_planes.__all__
    assert not hasattr(ml_planes._core, "_Env"), "the parity harness should be gone"
