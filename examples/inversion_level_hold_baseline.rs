//! Benchmark the promoted generic-jet inversion controller without ML dependencies.
//! cargo run --release --no-default-features --example inversion_level_hold_baseline
//! Optional: --episodes 1024 --seed 100000 --seed-stride 104729 --max-steps 3200

use ml_planes::controllers::{FlightController, InversionLevelHoldController};
use ml_planes::plane::{ControllerContext, PlaneId, PHYSICS_DT};
use ml_planes::training::eval_metrics::MetricFamily;
use ml_planes::training::task::{self, EnvSpec, Task};
use ml_planes::training::{DemonstrationEnv, EvalRun, TrainingEnv};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (mut episodes, mut seed, mut stride, mut max_steps) = (64usize, 42u64, 1u64, 3200u32);
    let usage = || -> ! {
        eprintln!("Usage: inversion_level_hold_baseline [--episodes N] [--seed N] [--seed-stride N] [--max-steps N]");
        std::process::exit(2)
    };
    if args.len() % 2 != 0 {
        usage();
    }
    for pair in args.chunks_exact(2) {
        match pair[0].as_str() {
            "--episodes" => episodes = pair[1].parse().unwrap_or_else(|_| usage()),
            "--seed" => seed = pair[1].parse().unwrap_or_else(|_| usage()),
            "--seed-stride" => stride = pair[1].parse().unwrap_or_else(|_| usage()),
            "--max-steps" => max_steps = pair[1].parse().unwrap_or_else(|_| usage()),
            _ => usage(),
        }
    }
    if episodes == 0 || max_steps == 0 {
        usage();
    }
    let cfg = ml_planes::training::load_plane_config_or_exit("assets/planes/generic_jet.plane.ron");
    let reward = ml_planes::training::reward_config::load_reward_config(
        "assets/training/level_hold.reward.ron",
    )
    .expect("valid level-hold reward profile");
    let mut env = task::level_hold_env(&EnvSpec::defaults_for(Task::LevelHold, cfg), reward);
    env.max_episode_steps = max_steps;
    let ctx = ControllerContext::empty_for(PlaneId::TEST);
    let mut run = EvalRun::new(MetricFamily::LevelHold, episodes, max_steps, 1);
    let mut failures = 0;
    for i in 0..episodes {
        env.set_rng_seed(seed.wrapping_add((i as u64).wrapping_mul(stride)));
        env.reset();
        let mut ctrl = InversionLevelHoldController::new(env.target_altitude, env.target_airspeed);
        for _ in 0..max_steps {
            let u = ctrl.update(&env.current_state(), &ctx, PHYSICS_DT);
            let o = env.step(&[u.elevator, u.throttle * 2.0 - 1.0, u.aileron, u.rudder]);
            run.record(0, &o.obs, o.reward).unwrap();
            if o.done() {
                if !o.truncated() {
                    failures += 1;
                }
                run.finish(0, &o.obs).unwrap();
                break;
            }
        }
    }
    let report = run.report();
    println!("controller,InversionLevelHold");
    println!("plane_config,assets/planes/generic_jet.plane.ron");
    println!("reward_config,assets/training/level_hold.reward.ron");
    println!("target_alt_range,500:5000\ntarget_speed_range,90:140");
    println!("seed,{seed}\nseed_stride,{stride}\nmax_steps,{max_steps}");
    println!("episodes,{}\nfailures,{failures}", report.episodes);
    println!("success_rate,{:.6}", report.success_rate);
    println!("mean_length_steps,{:.3}", report.mean_length_steps);
    println!("mean_return,{:.6}", report.mean_return);
    for row in report.rows {
        println!("{},{:.*}", row.key, row.decimals.max(6), row.value);
    }
}
