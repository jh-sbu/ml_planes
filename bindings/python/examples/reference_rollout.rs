//! Rust-side reference rollout for the Python parity test.
//!
//! Runs a `ml_planes::training` environment through a deterministic action
//! sequence from a fixed seed and prints the whole trace as JSON. The pytest in
//! `bindings/python/tests/test_parity.py` replays *these exact actions* through
//! `ml_planes._core._Env` and demands bit-identical observations and rewards.
//!
//! The action sequence is generated here and echoed into the output rather than
//! being agreed on by both sides, so a drifting generator cannot masquerade as a
//! drifting environment.
//!
//! Lives in the binding crate because that crate already path-depends on
//! `ml_planes`; `pyo3/extension-module` is switched on by maturin only, so plain
//! `cargo run --example` still links against a real interpreter-free build.
//!
//! With `--n-envs <N>` it drives a `VecEnv` instead, giving the parity test a
//! reference for the two things most easily marshalled wrong: the env-major
//! action layout of `step_batch`, and the `ENV_SEED_STRIDE` spacing of a seeded
//! pool.
//!
//! Usage: `reference_rollout --task <name> --seed <u64> --steps <n> [--pattern <p>] [--n-envs <N>]`

use ml_planes::training::task::{self, EnvSpec, Task};
use ml_planes::training::{TerminationReason, TrainingEnv, VecEnv};

/// Deterministic, dependency-free action generator. Not meant to be a good
/// policy — only to be reproducible and to exercise all four channels.
fn action_at(pattern: &str, step: usize, action_dim: usize) -> Vec<f32> {
    action_at_env(pattern, step, action_dim, 0)
}

/// As [`action_at`], offset per env so no two envs of a pool ever receive the
/// same row — a transposed or mis-strided batch then cannot go unnoticed.
fn action_at_env(pattern: &str, step: usize, action_dim: usize, env: usize) -> Vec<f32> {
    let step = step + env * 13;
    (0..action_dim)
        .map(|ch| match pattern {
            // Nose hard down: reaches a `Failure` termination quickly.
            "dive" => {
                if ch == 0 {
                    -1.0
                } else {
                    0.0
                }
            }
            // Hands off. On a residual env that is the PID baseline flying
            // unaided, which survives to the step limit and reports `Timeout`.
            "hold" => 0.0,
            // A slow, bounded sweep across every channel.
            _ => {
                let phase = (step * (ch + 1) % 97) as f32 / 97.0;
                (phase * 2.0 - 1.0) * 0.25
            }
        })
        .collect()
}

fn arg(args: &[String], key: &str) -> Option<String> {
    args.windows(2).find(|w| w[0] == key).map(|w| w[1].clone())
}

/// Emit an `f32` as the shortest decimal that round-trips **as an `f64`**.
///
/// This is the subtle half of the parity test. `{:?}` on an `f32` prints the
/// shortest string that round-trips to the same `f32` — but Python parses JSON
/// numbers as `f64`, and `float("0.1")` is not the same `f64` as the widened
/// `f32` 0.1 that pyo3 hands back from `_Env.step`. Widening here first makes
/// both sides land on the identical `f64`, so `==` means what the test says it
/// means instead of failing on a formatting artifact.
fn json_f32(v: f32) -> String {
    assert!(
        v.is_finite(),
        "non-finite value {v} would not be valid JSON, and means the rollout diverged"
    );
    format!("{:?}", f64::from(v))
}

fn json_row(values: &[f32]) -> String {
    let inner: Vec<String> = values.iter().copied().map(json_f32).collect();
    format!("[{}]", inner.join(","))
}

fn single_rollout(task: Task, spec: &EnvSpec, seed: u64, steps: usize, pattern: &str) -> String {
    let mut env = task::make_env(task, spec, None).expect("shipped reward profile");
    env.set_rng_seed(seed);
    let reset_obs = env.reset().0;
    let action_dim = env.action_dim();

    let mut rows: Vec<String> = Vec::new();
    for step in 0..steps {
        let action = action_at(pattern, step, action_dim);
        let outcome = env.step(&action);
        rows.push(step_row(&action, &outcome));
        if outcome.done() {
            break;
        }
    }

    format!(
        r#""observation_dim":{},"action_dim":{},"reset_obs":{},"steps":[{}]"#,
        env.observation_dim(),
        action_dim,
        json_row(&reset_obs),
        rows.join(",")
    )
}

/// The batched reference: one seeded pool, per-env-distinct actions, one trace
/// per env in the same order `step_batch` returns them.
fn vec_rollout(
    task: Task,
    spec: &EnvSpec,
    seed: u64,
    steps: usize,
    pattern: &str,
    n_envs: usize,
) -> String {
    let envs: Vec<Box<dyn TrainingEnv>> = (0..n_envs)
        .map(|_| task::make_env(task, spec, None).expect("shipped reward profile"))
        .collect();
    let observation_dim = envs[0].observation_dim();
    let action_dim = envs[0].action_dim();

    let mut pool = VecEnv::new(envs);
    pool.set_rng_seed(seed);
    let reset_obs = pool.reset_all();

    let mut rows: Vec<Vec<String>> = vec![Vec::new(); n_envs];
    for step in 0..steps {
        let flat: Vec<f32> = (0..n_envs)
            .flat_map(|e| action_at_env(pattern, step, action_dim, e))
            .collect();
        let outcomes = pool.step_batch(&flat);
        for (e, outcome) in outcomes.iter().enumerate() {
            let action = action_at_env(pattern, step, action_dim, e);
            rows[e].push(step_row(&action, outcome));
        }
        // Nothing auto-resets, so stop the whole batch as soon as any env ends
        // rather than keep stepping a finished episode.
        if outcomes.iter().any(|o| o.done()) {
            break;
        }
    }

    let envs_json: Vec<String> = (0..n_envs)
        .map(|e| {
            format!(
                r#"{{"reset_obs":{},"steps":[{}]}}"#,
                json_row(&reset_obs[e]),
                rows[e].join(",")
            )
        })
        .collect();

    format!(
        r#""observation_dim":{},"action_dim":{},"n_envs":{},"envs":[{}]"#,
        observation_dim,
        action_dim,
        n_envs,
        envs_json.join(",")
    )
}

fn step_row(action: &[f32], outcome: &ml_planes::training::StepOutcome) -> String {
    format!(
        r#"{{"action":{},"obs":{},"reward":{},"terminated":{},"truncated":{},"episode_step":{}}}"#,
        json_row(action),
        json_row(&outcome.obs),
        json_f32(outcome.reward),
        outcome.end == Some(TerminationReason::Failure),
        outcome.truncated(),
        outcome.info.episode_step
    )
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let task_name = arg(&args, "--task").unwrap_or_else(|| "level_hold".to_string());
    let seed: u64 = arg(&args, "--seed")
        .unwrap_or_else(|| "7".to_string())
        .parse()
        .expect("--seed must be a u64");
    let steps: usize = arg(&args, "--steps")
        .unwrap_or_else(|| "64".to_string())
        .parse()
        .expect("--steps must be a usize");
    let pattern = arg(&args, "--pattern").unwrap_or_else(|| "sweep".to_string());
    let n_envs: usize = arg(&args, "--n-envs")
        .unwrap_or_else(|| "0".to_string())
        .parse()
        .expect("--n-envs must be a usize");

    let task = Task::parse(&task_name).expect("known task");
    let cfg = ml_planes::training::load_plane_config(ml_planes::training::DEFAULT_PLANE_CONFIG_PATH)
        .expect("default plane config; run from the repo root");
    let spec = EnvSpec::defaults_for(task, cfg);

    let body = if n_envs > 0 {
        vec_rollout(task, &spec, seed, steps, &pattern, n_envs)
    } else {
        single_rollout(task, &spec, seed, steps, &pattern)
    };

    println!(
        r#"{{"task":"{}","seed":{},"pattern":"{}",{}}}"#,
        task.as_str(),
        seed,
        pattern,
        body
    );
}
