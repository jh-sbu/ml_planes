//! JSON reference for the Python `EvalRun` parity test.
//!
//! It uses one environment and zero actions, matching `evaluate_policy`'s
//! episode order without requiring a checkpoint.
//!
//! Run:
//!   cargo run --quiet --manifest-path bindings/python/Cargo.toml \
//!     --example reference_eval -- --task level_hold --episodes 4 --max-steps 200

use ml_planes::training::task::{self, EnvSpec, Task};
use ml_planes::training::{EvalRun, TrainingEnv};

fn arg(args: &[String], key: &str) -> Option<String> {
    args.windows(2).find(|w| w[0] == key).map(|w| w[1].clone())
}

/// Format an `f32` so JSON parsing round-trips to the same widened value.
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
        // Keep the environment timeout aligned with the evaluation budget.
        max_episode_steps: Some(max_steps),
        ..EnvSpec::defaults_for(task, cfg)
    };
    let mut env = task::make_env(task, &spec, None).expect("shipped reward profile");

    // Capture the seed before reset advances it.
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
