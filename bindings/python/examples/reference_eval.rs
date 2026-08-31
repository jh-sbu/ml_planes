//! Reference evaluation report, printed as JSON — the Rust half of the
//! `EvalRun` parity test (`bindings/python/tests/test_eval_run.py`).
//!
//! This deliberately mirrors `src/bin/evaluate_policy.rs`'s loop *exactly*: one
//! environment, reset once per episode, driven with a fixed all-zero action so
//! no policy — and therefore no checkpoint, and no burn — is needed on either
//! side of the comparison. The Python half instead builds one environment per
//! episode and seeds each absolutely, which is the scheme
//! `train_ppo_torch.py::evaluate_checkpoint` now uses.
//!
//! That difference is the point. If the two agree bit-for-bit, then both the
//! shared metric code *and* the seeding alignment are correct: episode `i` of a
//! freshly built env draws from `base + i + 1`, because every `reset()` advances
//! the seed by one before drawing. If someone later "fixes" the Python seeding
//! back to a strided scheme, or reimplements a metric by hand, this test fails.
//!
//! Run:
//!   cargo run --quiet --manifest-path bindings/python/Cargo.toml \
//!     --example reference_eval -- --task level_hold --episodes 4 --max-steps 200

use ml_planes::training::task::{self, EnvSpec, Task};
use ml_planes::training::{EvalRun, TrainingEnv};

fn arg(args: &[String], key: &str) -> Option<String> {
    args.windows(2).find(|w| w[0] == key).map(|w| w[1].clone())
}

/// Widen to `f64` before printing: Python parses JSON numbers as `f64`, and
/// `float("0.1")` is not the widened `f32` 0.1. `{:?}` on the widened value
/// gives the shortest decimal that round-trips, so `==` on the Python side is a
/// legitimate comparison rather than a tolerance in disguise.
fn json_f32(v: f32) -> String {
    assert!(
        v.is_finite(),
        "non-finite metric {v} would not be valid JSON, and means the rollout diverged"
    );
    format!("{:?}", f64::from(v))
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let task_name = arg(&args, "--task").unwrap_or_else(|| "level_hold".to_string());
    let episodes: usize = arg(&args, "--episodes")
        .map(|v| v.parse().expect("--episodes"))
        .unwrap_or(4);
    let max_steps: u32 = arg(&args, "--max-steps")
        .map(|v| v.parse().expect("--max-steps"))
        .unwrap_or(200);

    let task = Task::parse(&task_name).expect("known task");
    let cfg = ml_planes::training::load_plane_config(ml_planes::training::DEFAULT_PLANE_CONFIG_PATH)
        .expect("default plane config; run from the repo root");
    let spec = EnvSpec {
        // Pin the env's own Timeout to the loop bound, exactly as
        // `evaluate_policy` does by assigning `env.max_episode_steps`.
        max_episode_steps: Some(max_steps),
        ..EnvSpec::defaults_for(task, cfg)
    };
    let mut env = task::make_env(task, &spec, None).expect("shipped reward profile");

    // The seed the env starts at, before any reset. Python discovers the same
    // value through `Env.rng_seed` rather than hardcoding it — it is 42 for the
    // level-hold family but 4242 for the orbit family.
    let base_seed = env.rng_seed();

    let action = vec![0.0_f32; env.action_dim()];
    let mut run = EvalRun::new(task.metric_family(), episodes, max_steps, 1);

    for _ in 0..episodes {
        let (mut obs, _) = env.reset();
        let mut ep_len = 0_u32;
        let mut done = false;
        while !done && ep_len < max_steps {
            let outcome = env.step(&action);
            done = outcome.done();
            obs = outcome.obs;
            ep_len += 1;
            run.record(0, &obs, outcome.reward).expect("record");
        }
        run.finish(0, &obs).expect("finish");
    }

    let report = run.report();
    let rows: Vec<String> = report
        .rows
        .iter()
        .map(|r| format!("\"{}\":{}", r.key, json_f32(r.value)))
        .collect();

    println!(
        "{{\"task\":\"{}\",\"base_seed\":{},\"max_steps\":{},\"episodes\":{},\
\"success_rate\":{},\"mean_return\":{},\"mean_length_steps\":{},{}}}",
        task.as_str(),
        base_seed,
        max_steps,
        report.episodes,
        json_f32(report.success_rate),
        json_f32(report.mean_return),
        json_f32(report.mean_length_steps),
        rows.join(",")
    );
}
