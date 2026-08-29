"""Gymnasium adapter for the ml_planes training environments.

Imported lazily and never from `ml_planes/__init__.py`, so gymnasium stays an
optional dependency: `ml_planes.Env` and `ml_planes.VecEnv` work without it.

    from ml_planes.gym import MlPlanesEnv, register

    env = MlPlanesEnv("level_hold")
    # or, after register():
    import gymnasium
    env = gymnasium.make("MlPlanes/LevelHold-v0")

There is deliberately **no** `gymnasium.vector.VectorEnv` adapter. Gymnasium's
vector API auto-resets a finished sub-episode, which is exactly the behavior
`ml_planes.VecEnv` declines to have: not auto-resetting is what keeps the
terminal observation readable so a truncated episode's value can be bootstrapped
with `gamma * V(s')` before the reset. Wrapping it in something that auto-resets
would quietly reintroduce the value-target bug the split return exists to
prevent. Drive `ml_planes.VecEnv` directly for batched rollouts —
`bindings/python/examples/train_ppo_torch.py` shows the loop.
"""

from __future__ import annotations

from typing import Any

try:
    import gymnasium
    from gymnasium import spaces
except ImportError as exc:  # pragma: no cover - exercised only without gymnasium
    raise ImportError(
        "ml_planes.gym needs gymnasium. Install it with `uv add gymnasium` "
        "(or `pip install gymnasium`). The core ml_planes.Env / ml_planes.VecEnv "
        "do not require it."
    ) from exc

import numpy as np

from . import TASKS, Env

__all__ = ["MlPlanesEnv", "env_id", "register"]

# Every task emits the same 4-channel direct action space, in
# [elevator, throttle, aileron, rudder] order, each clamped to [-1, 1].
ACTION_LOW, ACTION_HIGH = -1.0, 1.0


def env_id(task: str) -> str:
    """`"level_hold"` -> `"MlPlanes/LevelHold-v0"`."""
    camel = "".join(part.title() for part in task.split("_"))
    return f"MlPlanes/{camel}-v0"


class MlPlanesEnv(gymnasium.Env):
    """A single `ml_planes` environment as a `gymnasium.Env`.

    A thin shim by design: `ml_planes.Env` already returns `(obs, info)` from
    `reset` and the five-tuple from `step`, so this class contributes spaces and
    nothing else. Any behavior added here would be behavior the Rust environment
    does not have, which is the divergence the parity test exists to catch.
    """

    metadata: dict[str, Any] = {"render_modes": []}

    def __init__(self, task: str, **kwargs: Any) -> None:
        """`kwargs` are forwarded verbatim to `ml_planes.Env` (plane_config,
        reward_config, the target ranges, orbit geometry, seed)."""
        self.env = Env(task, **kwargs)
        self.task = task

        # Observations are normalized but not hard-bounded by the env, so the
        # box is unbounded rather than pretending to a range nothing enforces.
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.env.observation_dim,),
            dtype=np.float32,
        )
        self.action_space = spaces.Box(
            low=ACTION_LOW,
            high=ACTION_HIGH,
            shape=(self.env.action_dim,),
            dtype=np.float32,
        )

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        return self.env.reset(seed=seed)

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        return self.env.step(action)

    def render(self) -> None:
        """Rendering lives in the Bevy client, never in a training env."""
        raise NotImplementedError("ml_planes training environments do not render")

    def __repr__(self) -> str:
        return f"MlPlanesEnv(task={self.task!r})"


def register() -> None:
    """Register `MlPlanes/<Task>-v0` for all five tasks with Gymnasium.

    Idempotent, so calling it more than once (or from several modules) is safe.
    `max_episode_steps` is deliberately left unset: each environment enforces its
    own limit from its reward profile and reports the result as `truncated`, so a
    `TimeLimit` wrapper on top would impose a second, different horizon.
    """
    for task in TASKS:
        ident = env_id(task)
        if ident in gymnasium.registry:
            continue
        gymnasium.register(
            id=ident,
            entry_point="ml_planes.gym:MlPlanesEnv",
            kwargs={"task": task},
        )
