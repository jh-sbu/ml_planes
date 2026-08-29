"""Python bindings for the ml_planes 6-DOF flight training environments.

The Rust extension module is `ml_planes._core`; names are re-exported here so
callers import from `ml_planes` and never touch the private module directly.

    import ml_planes

    env = ml_planes.Env("level_hold")
    obs, info = env.reset(seed=0)
    obs, reward, terminated, truncated, info = env.step([0.0, 0.0, 0.0, 0.0])

`VecEnv` is the batched form a training loop should drive. Neither auto-resets a
finished episode — that is deliberate, and mirrors the Rust side: it keeps the
terminal observation readable so a truncated episode's value can be bootstrapped
before `reset_at` restarts it.

The Gymnasium adapter lives in `ml_planes.gym` and is **not** imported here, so
gymnasium stays an optional dependency of this package.
"""

from . import _core
from ._core import TASKS, Env, VecEnv, physics_dt

__all__ = ["TASKS", "Env", "VecEnv", "physics_dt"]
