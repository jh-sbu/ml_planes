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
//! Usage: `reference_rollout --task <name> --seed <u64> --steps <n> [--pattern <p>]`

use ml_planes::training::task::{self, EnvSpec, Task};
use ml_planes::training::{TerminationReason, TrainingEnv};

/// Deterministic, dependency-free action generator. Not meant to be a good
/// policy — only to be reproducible and to exercise all four channels.
fn action_at(pattern: &str, step: usize, action_dim: usize) -> Vec<f32> {
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

    let task = Task::parse(&task_name).expect("known task");
    let cfg =
        ml_planes::training::load_plane_config(ml_planes::training::DEFAULT_PLANE_CONFIG_PATH)
            .expect("default plane config; run from the repo root");
    let spec = EnvSpec::defaults_for(task, cfg);
    let mut env = task::make_env(task, &spec, None).expect("shipped reward profile");

    env.set_rng_seed(seed);
    let reset_obs = env.reset().0;
    let action_dim = env.action_dim();

    let mut rows: Vec<String> = Vec::new();
    for step in 0..steps {
        let action = action_at(&pattern, step, action_dim);
        let outcome = env.step(&action);
        let terminated = outcome.end == Some(TerminationReason::Failure);
        rows.push(format!(
            r#"{{"action":{},"obs":{},"reward":{},"terminated":{},"truncated":{},"episode_step":{}}}"#,
            json_row(&action),
            json_row(&outcome.obs),
            json_f32(outcome.reward),
            terminated,
            outcome.truncated(),
            outcome.info.episode_step
        ));
        if outcome.done() {
            break;
        }
    }

    println!(
        r#"{{"task":"{}","seed":{},"pattern":"{}","observation_dim":{},"action_dim":{},"reset_obs":{},"steps":[{}]}}"#,
        task.as_str(),
        seed,
        pattern,
        env.observation_dim(),
        action_dim,
        json_row(&reset_obs),
        rows.join(",")
    );
}
