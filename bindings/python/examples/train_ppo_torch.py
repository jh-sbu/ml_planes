#!/usr/bin/env python3
"""Reference PyTorch PPO over the ml_planes environments.

This is the end-to-end demonstration that the bindings are usable, not a
replacement for anything: the Rust `burn` track (`train_ppo`, `train_bc`,
`evaluate_policy`) remains the supported in-repo path (CLAUDE.md §3, constraint
(c)). Python is a second consumer of the same environments.

    just py-train level_hold --total-steps 2000000 --num-envs 16

At those settings a CPU run does ~19k env-steps/s and takes mean episode return
from roughly -2000 to -80 over 2M steps, with the value loss falling several
orders of magnitude and entropy declining as the policy sharpens. That is a
working training curve, not a tuned result — for policy quality use the Rust
`evaluate_policy` and the train-evaluate-optimize workflow.

Lives in the binding crate's `examples/` because cargo ignores `.py` there and
maturin only packages `python/`, so it ships in neither the crate nor the wheel.

The one detail worth reading before adapting this loop is `compute_gae`. The
environments distinguish a *failure* (absorbing — bootstrap 0) from a *timeout*
(the step limit cut a still-flying episode short — bootstrap `gamma * V(s')`),
and the value target differs between them. Collapsing the two into one `done`
teaches the critic that an ordinary in-flight state is worth nothing, and since
the observation carries no time feature, that state is indistinguishable from
any other. This mirrors `src/training/ppo/buffer.rs::compute_gae` on the Rust
side; keep the two in agreement.

Checkpoints are plain PyTorch `.pt` files. Interchange with the Rust `.mpk`
format is deliberately not attempted — CLAUDE.md §3 records it as a separate,
undecided question.
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

import ml_planes


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--task", default="level_hold", choices=sorted(ml_planes.TASKS))
    p.add_argument("--num-envs", type=int, default=16)
    p.add_argument("--total-steps", type=int, default=1_000_000,
                   help="environment steps summed across all envs")
    p.add_argument("--rollout-steps", type=int, default=256,
                   help="steps per env per update")
    # These defaults mirror `assets/training/level_hold.ppo.ron`, so a Python run
    # and a `train_ppo --task level_hold` run start from the same
    # hyperparameters. `--minibatch-size` is the one deviation: the shipped
    # profile uses 64, which costs ~16x the optimizer steps per update and is
    # painful in Python. Pass `--minibatch-size 64` to match it exactly.
    p.add_argument("--epochs", type=int, default=4)
    p.add_argument("--minibatch-size", type=int, default=256)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--gae-lambda", type=float, default=0.95)
    p.add_argument("--clip", type=float, default=0.1)
    p.add_argument("--ent-coef", type=float, default=0.0)
    p.add_argument("--vf-coef", type=float, default=0.5)
    p.add_argument("--max-grad-norm", type=float, default=0.5)
    p.add_argument("--hidden", type=int, default=128)
    p.add_argument("--layers", type=int, default=2,
                   help="number of hidden layers in the plain MLP")
    p.add_argument(
        "--arch",
        choices=("mlp", "residual_mlp", "transformer"),
        default="mlp",
        help="PyTorch policy/value architecture (no Rust rebuild required)",
    )
    p.add_argument("--log-std-init", type=float, default=-0.5)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--device", default="cpu")
    p.add_argument("--plane-config", default=None,
                   help="airframe .plane.ron; must match what a checkpoint was trained on")
    p.add_argument("--reward-config", default=None)
    p.add_argument("--out", default=None, help="checkpoint path (.pt)")
    p.add_argument("--log-every", type=int, default=1)
    p.add_argument("--eval-only", default=None, metavar="CHECKPOINT",
                   help="skip training and deterministically evaluate a .pt checkpoint")
    p.add_argument("--eval-episodes", type=int, default=64)
    p.add_argument("--eval-max-steps", type=int, default=3200)
    p.add_argument("--eval-seed", type=int, default=None,
                   help="base episode seed; defaults to the env's own starting seed, "
                        "which makes the episode set identical to evaluate_policy's")
    p.add_argument("--eval-json", default=None,
                   help="optional path for machine-readable evaluation metrics")
    return p.parse_args()


def layer_init(layer: nn.Linear, gain: float = np.sqrt(2)) -> nn.Linear:
    """Orthogonal weights, zero bias — the standard PPO initialization.

    The gain on the *final* actor layer matters more than it looks. Left at the
    default, the initial policy emits large means on a [-1, 1] action space, so
    almost every sampled action saturates at the clamp, every episode looks alike
    and there is nothing for the advantage to discriminate. A gain of 0.01 starts
    the policy near zero-mean instead.
    """
    nn.init.orthogonal_(layer.weight, gain)
    nn.init.constant_(layer.bias, 0.0)
    return layer


class ActorCritic(nn.Module):
    """MLP actor-critic with a state-independent log-std, matching the shape of
    the Rust `ActorCritic` closely enough to be comparable."""

    def __init__(
        self, obs_dim: int, action_dim: int, hidden: int,
        log_std_init: float = -0.5, layers: int = 2,
    ) -> None:
        super().__init__()
        if layers < 1:
            raise ValueError("MLP layers must be at least 1")

        def trunk(output_dim: int, output_gain: float) -> nn.Sequential:
            modules: list[nn.Module] = []
            input_dim = obs_dim
            for _ in range(layers):
                modules.extend((layer_init(nn.Linear(input_dim, hidden)), nn.Tanh()))
                input_dim = hidden
            modules.append(layer_init(nn.Linear(hidden, output_dim), gain=output_gain))
            return nn.Sequential(*modules)

        self.actor = trunk(action_dim, 0.01)
        self.critic = trunk(1, 1.0)
        # std = exp(-0.5) ~ 0.61: wide enough to explore a [-1, 1] action space,
        # narrow enough that a sample is not usually clipped.
        self.log_std = nn.Parameter(torch.full((action_dim,), log_std_init))

    def distribution(self, obs: torch.Tensor) -> torch.distributions.Normal:
        return torch.distributions.Normal(self.actor(obs), self.log_std.exp())

    def value(self, obs: torch.Tensor) -> torch.Tensor:
        return self.critic(obs).squeeze(-1)

    @torch.no_grad()
    def act(self, obs: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        dist = self.distribution(obs)
        action = dist.sample()
        return action, dist.log_prob(action).sum(-1), self.value(obs)

    def evaluate(
        self, obs: torch.Tensor, action: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        dist = self.distribution(obs)
        return (
            dist.log_prob(action).sum(-1),
            dist.entropy().sum(-1),
            self.value(obs),
        )


class ResidualBlock(nn.Module):
    def __init__(self, width: int) -> None:
        super().__init__()
        self.norm = nn.LayerNorm(width)
        self.fc1 = layer_init(nn.Linear(width, width))
        self.fc2 = layer_init(nn.Linear(width, width), gain=1.0)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        y = self.norm(x)
        y = torch.nn.functional.silu(self.fc1(y))
        return x + 0.5 * self.fc2(y)


class ResidualMlpTrunk(nn.Module):
    def __init__(self, obs_dim: int, width: int) -> None:
        super().__init__()
        self.input = layer_init(nn.Linear(obs_dim, width))
        self.blocks = nn.Sequential(ResidualBlock(width), ResidualBlock(width))
        self.output_norm = nn.LayerNorm(width)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        x = torch.nn.functional.silu(self.input(obs))
        return torch.nn.functional.silu(self.output_norm(self.blocks(x)))


class ResidualActorCritic(ActorCritic):
    """A wider/deeper MLP with normalized residual blocks."""

    def __init__(
        self, obs_dim: int, action_dim: int, hidden: int,
        log_std_init: float = -0.5, layers: int = 2,
    ) -> None:
        nn.Module.__init__(self)
        del layers  # residual depth is fixed by ResidualMlpTrunk
        width = max(hidden, 256)
        self.actor = nn.Sequential(
            ResidualMlpTrunk(obs_dim, width),
            layer_init(nn.Linear(width, action_dim), gain=0.01),
        )
        self.critic = nn.Sequential(
            ResidualMlpTrunk(obs_dim, width),
            layer_init(nn.Linear(width, 1), gain=1.0),
        )
        self.log_std = nn.Parameter(torch.full((action_dim,), log_std_init))


class FeatureTransformerTrunk(nn.Module):
    """Treat each named scalar observation feature as one attention token."""

    def __init__(self, obs_dim: int, d_model: int = 32) -> None:
        super().__init__()
        self.value_projection = nn.Linear(1, d_model)
        self.feature_embedding = nn.Parameter(torch.empty(obs_dim, d_model))
        nn.init.normal_(self.feature_embedding, std=0.02)
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=d_model,
            nhead=4,
            dim_feedforward=2 * d_model,
            dropout=0.0,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.encoder = nn.TransformerEncoder(encoder_layer, num_layers=2)
        self.norm = nn.LayerNorm(d_model)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        tokens = self.value_projection(obs.unsqueeze(-1))
        tokens = tokens + self.feature_embedding.unsqueeze(0)
        return self.norm(self.encoder(tokens)).mean(dim=1)


class TransformerActorCritic(ActorCritic):
    """Small feature-token Transformer with independent actor/critic trunks."""

    def __init__(
        self, obs_dim: int, action_dim: int, hidden: int,
        log_std_init: float = -0.5, layers: int = 2,
    ) -> None:
        nn.Module.__init__(self)
        del hidden, layers  # the compact attention width/depth are intentionally fixed
        d_model = 32
        self.actor = nn.Sequential(
            FeatureTransformerTrunk(obs_dim, d_model),
            layer_init(nn.Linear(d_model, action_dim), gain=0.01),
        )
        self.critic = nn.Sequential(
            FeatureTransformerTrunk(obs_dim, d_model),
            layer_init(nn.Linear(d_model, 1), gain=1.0),
        )
        self.log_std = nn.Parameter(torch.full((action_dim,), log_std_init))


ARCHITECTURES = {
    "mlp": ActorCritic,
    "residual_mlp": ResidualActorCritic,
    "transformer": TransformerActorCritic,
}


def build_model(
    arch: str, obs_dim: int, action_dim: int, hidden: int, log_std_init: float,
    layers: int = 2,
) -> ActorCritic:
    return ARCHITECTURES[arch](obs_dim, action_dim, hidden, log_std_init, layers)


@dataclass
class Rollout:
    obs: torch.Tensor            # (T, N, D)
    actions: torch.Tensor        # (T, N, A)
    log_probs: torch.Tensor      # (T, N)
    values: torch.Tensor         # (T, N)
    rewards: torch.Tensor        # (T, N)
    dones: torch.Tensor          # (T, N) — episode boundary, either reason
    bootstrap: torch.Tensor      # (T, N) — gamma * V(s') where a timeout cut it short


def compute_gae(
    rollout: Rollout, last_values: torch.Tensor, gamma: float, lam: float
) -> tuple[torch.Tensor, torch.Tensor]:
    """Generalized advantage estimation with truncation bootstrapping.

    Mirrors `src/training/ppo/buffer.rs::compute_gae`:

    * `dones` resets the advantage carry at *every* episode boundary, failure or
      timeout alike — it really is a boundary either way.
    * `bootstrap` is non-zero only where the episode was *truncated*, and adds
      the `gamma * V(s')` the value target would otherwise be missing. A failure
      is absorbing, so its bootstrap stays 0.
    """
    T, N = rollout.rewards.shape
    advantages = torch.zeros_like(rollout.rewards)
    carry = torch.zeros(N, device=rollout.rewards.device)

    for t in reversed(range(T)):
        not_done = 1.0 - rollout.dones[t]
        next_value = last_values if t == T - 1 else rollout.values[t + 1]
        # A finished episode contributes no next-state value of its own; the
        # truncation bootstrap supplies it where one is owed.
        next_value = next_value * not_done + rollout.bootstrap[t]

        delta = rollout.rewards[t] + gamma * next_value - rollout.values[t]
        carry = delta + gamma * lam * not_done * carry
        advantages[t] = carry

    return advantages, advantages + rollout.values


def collect(
    venv: ml_planes.VecEnv,
    model: ActorCritic,
    obs_np: np.ndarray,
    steps: int,
    device: torch.device,
    gamma: float,
    episode_returns: np.ndarray,
    finished: list[float],
) -> tuple[Rollout, np.ndarray, torch.Tensor]:
    n, obs_dim, action_dim = venv.num_envs, venv.observation_dim, venv.action_dim

    buf_obs = torch.zeros(steps, n, obs_dim, device=device)
    buf_actions = torch.zeros(steps, n, action_dim, device=device)
    buf_logp = torch.zeros(steps, n, device=device)
    buf_values = torch.zeros(steps, n, device=device)
    buf_rewards = torch.zeros(steps, n, device=device)
    buf_dones = torch.zeros(steps, n, device=device)
    buf_bootstrap = torch.zeros(steps, n, device=device)

    for t in range(steps):
        obs = torch.from_numpy(obs_np).to(device)
        action, log_prob, value = model.act(obs)

        buf_obs[t], buf_actions[t] = obs, action
        buf_logp[t], buf_values[t] = log_prob, value

        # Store the RAW sampled action, and clip only what is flown. The policy
        # is a Gaussian over an unbounded space; clipping is the environment's.
        # Storing the clipped value instead would pair it with the raw action's
        # log-prob, so the PPO ratio would not be 1 on the first epoch and the
        # surrogate objective would be measuring a mismatch rather than a policy
        # change.
        next_obs, rewards, terminated, truncated, _ = venv.step(
            action.clamp(-1.0, 1.0).cpu().numpy().astype(np.float32)
        )

        buf_rewards[t] = torch.from_numpy(rewards).to(device)
        done = terminated | truncated
        buf_dones[t] = torch.from_numpy(done.astype(np.float32)).to(device)

        episode_returns += rewards

        # Only a timeout gets a bootstrap: it cut a still-flying episode short at
        # an arbitrary wall clock, so V(s') at the terminal observation is real.
        if truncated.any():
            with torch.no_grad():
                terminal_values = model.value(torch.from_numpy(next_obs).to(device))
            buf_bootstrap[t] = (
                gamma
                * terminal_values
                * torch.from_numpy(truncated.astype(np.float32)).to(device)
            )

        # No auto-reset, so restart finished episodes explicitly — after the
        # terminal observation above has already been used. `reset_done` is the
        # batched form of `reset_at`, and resets exactly the masked rows.
        if done.any():
            finished.extend(episode_returns[done].tolist())
            episode_returns[done] = 0.0
            next_obs, _ = venv.reset_done(next_obs, done)

        obs_np = next_obs

    with torch.no_grad():
        last_values = model.value(torch.from_numpy(obs_np).to(device))

    return (
        Rollout(buf_obs, buf_actions, buf_logp, buf_values,
                buf_rewards, buf_dones, buf_bootstrap),
        obs_np,
        last_values,
    )


def update(
    model: ActorCritic,
    optimizer: torch.optim.Optimizer,
    rollout: Rollout,
    advantages: torch.Tensor,
    returns: torch.Tensor,
    args: argparse.Namespace,
) -> dict[str, float]:
    obs = rollout.obs.reshape(-1, rollout.obs.shape[-1])
    actions = rollout.actions.reshape(-1, rollout.actions.shape[-1])
    old_log_probs = rollout.log_probs.reshape(-1)
    flat_adv = advantages.reshape(-1)
    flat_ret = returns.reshape(-1)
    flat_adv = (flat_adv - flat_adv.mean()) / (flat_adv.std() + 1e-8)

    total = obs.shape[0]
    stats = {"policy_loss": 0.0, "value_loss": 0.0, "entropy": 0.0}
    batches = 0

    for _ in range(args.epochs):
        for idx in torch.randperm(total, device=obs.device).split(args.minibatch_size):
            log_probs, entropy, values = model.evaluate(obs[idx], actions[idx])
            ratio = (log_probs - old_log_probs[idx]).exp()

            unclipped = ratio * flat_adv[idx]
            clipped = ratio.clamp(1 - args.clip, 1 + args.clip) * flat_adv[idx]
            policy_loss = -torch.min(unclipped, clipped).mean()
            value_loss = ((values - flat_ret[idx]) ** 2).mean()
            entropy_bonus = entropy.mean()

            loss = policy_loss + args.vf_coef * value_loss - args.ent_coef * entropy_bonus

            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), args.max_grad_norm)
            optimizer.step()

            stats["policy_loss"] += policy_loss.item()
            stats["value_loss"] += value_loss.item()
            stats["entropy"] += entropy_bonus.item()
            batches += 1

    return {k: v / max(batches, 1) for k, v in stats.items()}


@torch.no_grad()
def evaluate_checkpoint(args: argparse.Namespace) -> dict[str, float | int | str]:
    """Evaluate a checkpoint with the shared Rust metric accumulator."""
    checkpoint = torch.load(args.eval_only, map_location=args.device, weights_only=False)
    task = checkpoint["task"]
    saved_args = checkpoint.get("args", {})
    arch = checkpoint.get("arch", saved_args.get("arch", "mlp"))
    hidden = checkpoint.get("hidden", saved_args.get("hidden", 128))
    layers = checkpoint.get("layers", saved_args.get("layers", 2))
    log_std_init = saved_args.get("log_std_init", -0.5)

    env_kwargs = {}
    for key in ("plane_config", "reward_config"):
        if saved_args.get(key):
            env_kwargs[key] = saved_args[key]
    # Pin the env's own Timeout to this loop's budget, exactly as
    # `evaluate_policy.rs` does by assigning `env.max_episode_steps`. Without it
    # the two only agree when the reward profile happens to ship 3200, and
    # `success_rate` silently counts the wrong thing for every profile that does
    # not.
    env_kwargs["max_episode_steps"] = args.eval_max_steps
    envs = [ml_planes.Env(task, **env_kwargs) for _ in range(args.eval_episodes)]

    # reset() advances the seed, so base + i matches reset i + 1 of one env.
    base_seed = envs[0].rng_seed if args.eval_seed is None else args.eval_seed
    for i, env in enumerate(envs):
        env.seed(base_seed + i)

    model = build_model(
        arch,
        checkpoint["observation_dim"],
        checkpoint["action_dim"],
        hidden,
        log_std_init,
        layers,
    ).to(args.device)
    model.load_state_dict(checkpoint["model"])
    model.eval()

    run = ml_planes.EvalRun(
        task, episodes=args.eval_episodes, max_steps=args.eval_max_steps
    )
    lengths = np.zeros(args.eval_episodes, dtype=np.int64)
    finished = np.zeros(args.eval_episodes, dtype=bool)
    observations = [env.reset()[0] for env in envs]

    while not finished.all():
        active = np.flatnonzero(~finished)
        obs_batch = np.stack([observations[i] for i in active])
        actions = model.actor(torch.from_numpy(obs_batch).to(args.device))
        actions = actions.clamp(-1.0, 1.0).cpu().numpy().astype(np.float32)

        for row, i in enumerate(active):
            obs, reward, terminated, truncated, _ = envs[i].step(actions[row])
            observations[i] = obs
            lengths[i] += 1
            run.record(int(i), obs, reward)
            if terminated or truncated or lengths[i] >= args.eval_max_steps:
                finished[i] = True
                run.finish(int(i), obs)

    metrics: dict[str, float | int | str] = {
        "checkpoint": str(Path(args.eval_only)),
        "task": task,
        "architecture": arch,
        **run.report(),
    }
    for key, value in metrics.items():
        print(f"{key},{value}")
    if args.eval_json:
        Path(args.eval_json).write_text(json.dumps(metrics, indent=2) + "\n")
    return metrics


def main() -> None:
    args = parse_args()
    torch.manual_seed(args.seed)
    device = torch.device(args.device)

    if args.eval_only:
        evaluate_checkpoint(args)
        return

    env_kwargs = {}
    if args.plane_config:
        env_kwargs["plane_config"] = args.plane_config
    if args.reward_config:
        env_kwargs["reward_config"] = args.reward_config

    venv = ml_planes.VecEnv(args.task, args.num_envs, seed=args.seed, **env_kwargs)
    model = build_model(
        args.arch, venv.observation_dim, venv.action_dim,
        args.hidden, args.log_std_init, args.layers,
    ).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr, eps=1e-5)

    print(
        f"task={args.task} obs_dim={venv.observation_dim} action_dim={venv.action_dim} "
        f"num_envs={args.num_envs} arch={args.arch} hidden={args.hidden} "
        f"layers={args.layers} device={device} "
        f"params={sum(p.numel() for p in model.parameters()):,} "
        f"dt={ml_planes.physics_dt():.6f}s"
    )

    obs_np, _ = venv.reset()
    episode_returns = np.zeros(args.num_envs, dtype=np.float64)
    finished: list[float] = []

    steps_per_update = args.rollout_steps * args.num_envs
    updates = max(args.total_steps // steps_per_update, 1)
    start = time.time()

    for it in range(1, updates + 1):
        rollout, obs_np, last_values = collect(
            venv, model, obs_np, args.rollout_steps, device,
            args.gamma, episode_returns, finished,
        )
        advantages, returns = compute_gae(
            rollout, last_values, args.gamma, args.gae_lambda
        )
        stats = update(model, optimizer, rollout, advantages, returns, args)

        if it % args.log_every == 0 or it == updates:
            steps_done = it * steps_per_update
            recent = finished[-100:]
            mean_return = float(np.mean(recent)) if recent else float("nan")
            sps = steps_done / max(time.time() - start, 1e-9)
            print(
                f"update {it:4d}/{updates}  steps {steps_done:>9,}  "
                f"return {mean_return:10.2f}  episodes {len(finished):>5}  "
                f"policy {stats['policy_loss']:+.4f}  value {stats['value_loss']:.4f}  "
                f"entropy {stats['entropy']:+.3f}  {sps:,.0f} steps/s",
                flush=True,
            )

    if args.out:
        torch.save(
            {
                "model": model.state_dict(),
                "task": args.task,
                "observation_dim": venv.observation_dim,
                "action_dim": venv.action_dim,
                "hidden": args.hidden,
                "layers": args.layers,
                "arch": args.arch,
                "args": vars(args),
            },
            args.out,
        )
        print(f"saved {args.out}")


if __name__ == "__main__":
    main()
