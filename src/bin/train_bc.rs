#![recursion_limit = "256"]

//! Behavior-cloning pretraining binary. Build and run with:
//!   cargo run --no-default-features --features training --bin train_bc -- --task level_hold
//!
//! Collects `(observation, action)` demonstrations from the task's PID expert
//! controller, supervised-trains an `ActorCritic` policy to imitate them, and saves
//! the warm policy under `models/<task>/`. The checkpoint is consumable by the
//! existing PPO fine-tune path:
//!   cargo run ... --bin train_ppo -- --task <task> --init-from models/<task>/<stem>
//!
//! Flags:
//!   --task <level_hold|heading_hold|orbit>   Task / expert to clone (default: level_hold).
//!   --steps <n>                 Demonstration steps to collect (default: 200_000).
//!   --bc-epochs <n>             Supervised epochs over the dataset (default: 10).
//!   --minibatch <n>             Minibatch size (default: 256).
//!   --output <stem>             Save to models/<task>/<stem>.mpk (auto-increments if omitted).
//!   --backend <ndarray|wgpu>    Compute backend (default: ndarray; wgpu requires
//!                               --features wgpu).
//!   --reward-config <path>      Reward/termination profile path, overriding the task default
//!                               (assets/training/<task>.reward.ron). Missing file → defaults.
//!   --plane-config <path>       Airframe (.plane.ron) to clone the expert on (default:
//!                               assets/planes/generic_jet.plane.ron). Unreadable or
//!                               invalid → exit 2, never a silent fallback; see
//!                               `train_ppo`'s --plane-config doc for why.
//!   --seed <u64>                Fix the model's weight init and the minibatch-shuffle RNG
//!                               for a reproducible run. Omitted → unseeded.
//!                               The starting weights are exactly reproducible (verified
//!                               bit-exact), but burn's `ndarray` matmul backend has its own tiny
//!                               floating-point non-associativity that compounds over epochs, so
//!                               a long run's final checkpoint may not be byte-for-byte identical
//!                               — see `train_ppo`'s `--seed` doc for detail. `wgpu` is
//!                               best-effort only.
//!   --target-alt-range <MIN:MAX|VALUE>    level_hold/heading_hold only: target altitude [m] the
//!                               PID expert tracks is resampled from this range every demo episode
//!                               (default: 500:5000). A bare VALUE pins a fixed target.
//!   --target-speed-range <MIN:MAX|VALUE>  level_hold/heading_hold only: target airspeed [m/s]
//!                               (default: 90:140 for both level_hold and heading_hold).
//!   --target-heading-range <MIN:MAX|VALUE>  heading_hold only: target heading change [degrees]
//!                               the PID expert tracks (default: -180:180).

#[cfg(not(feature = "training"))]
fn main() {
    eprintln!("Build with --features training to run behavior-cloning pretraining.");
    eprintln!("  cargo run --no-default-features --features training --bin train_bc");
}

#[cfg(feature = "training")]
fn main() {
    #[cfg(feature = "wgpu")]
    use burn::backend::Wgpu;
    use burn::backend::{Autodiff, NdArray};
    use ml_planes::training::task::Task;
    use ml_planes::training::Backend;

    let args: Vec<String> = std::env::args().collect();

    let find = |flag: &str| -> Option<String> {
        args.windows(2).find(|w| w[0] == flag).map(|w| w[1].clone())
    };

    let task = find("--task")
        .map(|v| Task::parse(&v))
        .unwrap_or(Ok(Task::LevelHold))
        .unwrap_or_else(|e| {
            eprintln!("{e}");
            std::process::exit(2);
        });
    // The registry knows all five tasks; behavior cloning needs a PID expert to
    // roll out, which is exactly the set with a `bc_default_stem`.
    if task.bc_default_stem().is_none() {
        eprintln!(
            "Unsupported --task '{}'. Behavior cloning supports 'level_hold', 'heading_hold', or 'orbit'.",
            task.as_str()
        );
        std::process::exit(2);
    }

    let steps: usize = find("--steps")
        .map(|v| {
            v.parse::<usize>().unwrap_or_else(|_| {
                eprintln!("--steps must be a positive integer");
                std::process::exit(2);
            })
        })
        .unwrap_or(200_000);

    let bc_epochs: usize = find("--bc-epochs")
        .map(|v| {
            v.parse::<usize>().unwrap_or_else(|_| {
                eprintln!("--bc-epochs must be a positive integer");
                std::process::exit(2);
            })
        })
        .unwrap_or(10);

    let minibatch: usize = find("--minibatch")
        .map(|v| {
            v.parse::<usize>().unwrap_or_else(|_| {
                eprintln!("--minibatch must be a positive integer");
                std::process::exit(2);
            })
        })
        .unwrap_or(256);

    let backend = find("--backend")
        .map(|v| Backend::parse(&v))
        .unwrap_or(Ok(Backend::default()))
        .unwrap_or_else(|e| {
            eprintln!("{e}");
            std::process::exit(2);
        });
    println!("Backend: {}", backend.label());

    let save_path = save_path_for(task, find("--output"));

    // Optional reward-profile override; defaults to the task baseline profile.
    let reward_config = find("--reward-config");
    let plane_config = find("--plane-config")
        .unwrap_or_else(|| ml_planes::training::DEFAULT_PLANE_CONFIG_PATH.to_string());

    let seed: Option<u64> = find("--seed").map(|v| {
        v.parse::<u64>().unwrap_or_else(|_| {
            eprintln!("--seed must be a non-negative integer");
            std::process::exit(2);
        })
    });

    // level_hold only: the per-episode target-altitude/airspeed envelope the
    // PID expert demonstrates. See `LevelHoldEnv::with_target_ranges`.
    let target_alt_range = find("--target-alt-range")
        .map(|v| {
            ml_planes::training::parse_f32_range(&v).unwrap_or_else(|e| {
                eprintln!("--target-alt-range: {e}");
                std::process::exit(2);
            })
        })
        .unwrap_or_else(|| task.default_target_alt_range());
    let target_speed_range = find("--target-speed-range")
        .map(|v| {
            ml_planes::training::parse_f32_range(&v).unwrap_or_else(|e| {
                eprintln!("--target-speed-range: {e}");
                std::process::exit(2);
            })
        })
        .unwrap_or_else(|| task.default_target_speed_range());

    let target_heading_range = find("--target-heading-range")
        .map(|v| {
            ml_planes::training::parse_f32_range(&v).unwrap_or_else(|e| {
                eprintln!("--target-heading-range: {e}");
                std::process::exit(2);
            })
        })
        .unwrap_or_else(|| task.default_target_heading_range_deg());
    let target_heading_range =
        target_heading_range.start().to_radians()..=target_heading_range.end().to_radians();

    match backend {
        Backend::NdArray => run::<Autodiff<NdArray>>(
            task,
            steps,
            bc_epochs,
            minibatch,
            save_path,
            reward_config,
            plane_config,
            seed,
            target_alt_range,
            target_speed_range,
            target_heading_range,
        ),
        #[cfg(feature = "wgpu")]
        Backend::Wgpu => run::<Autodiff<Wgpu>>(
            task,
            steps,
            bc_epochs,
            minibatch,
            save_path,
            reward_config,
            plane_config,
            seed,
            target_alt_range,
            target_speed_range,
            target_heading_range,
        ),
    }
}

#[cfg(feature = "training")]
fn save_path_for(task: ml_planes::training::task::Task, output_stem: Option<String>) -> String {
    let dir = task.model_dir();
    match output_stem {
        Some(stem) => format!("models/{dir}/{stem}"),
        None => {
            let mut n = 1u32;
            loop {
                let candidate = format!(
                    "models/{dir}/{}_{n}",
                    task.bc_default_stem().expect("BC-capable task")
                );
                if !std::path::Path::new(&format!("{candidate}.mpk")).exists() {
                    break candidate;
                }
                n += 1;
            }
        }
    }
}

#[cfg(feature = "training")]
#[allow(clippy::too_many_arguments)]
fn run<B>(
    task: ml_planes::training::task::Task,
    steps: usize,
    bc_epochs: usize,
    minibatch: usize,
    save_path: String,
    reward_config: Option<String>,
    plane_config: String,
    seed: Option<u64>,
    target_alt_range: std::ops::RangeInclusive<f32>,
    target_speed_range: std::ops::RangeInclusive<f32>,
    target_heading_range: std::ops::RangeInclusive<f32>,
) where
    B: burn::tensor::backend::AutodiffBackend,
    B::Device: Default,
{
    use std::time::Instant;

    use ml_planes::training::ppo::PpoTrainer;
    use ml_planes::training::reward_config::{
        load_reward_config, HeadingHoldRewardConfig, LevelHoldRewardConfig, OrbitRewardConfig,
    };
    use ml_planes::training::task::{self, EnvSpec, Task};
    use ml_planes::training::{collect_demonstrations, DemonstrationEnv};

    let cfg = ml_planes::training::load_plane_config_or_exit(&plane_config);
    println!("Loaded plane config from {plane_config}");

    fn load_or_default<C: serde::de::DeserializeOwned + Default>(path: &str) -> C {
        match load_reward_config(path) {
            Ok(c) => {
                println!("Loaded reward config from {path}");
                c
            }
            Err(e) => {
                eprintln!("Warning: could not load {path}: {e}. Using defaults.");
                C::default()
            }
        }
    }

    let path = reward_config.unwrap_or_else(|| task.reward_config_path().to_string());
    let path = path.as_str();
    if let Some(s) = seed {
        println!("Seed: {s} (weight init, minibatch shuffling)");
    }

    let spec = EnvSpec {
        target_alt_range,
        target_speed_range,
        target_heading_range,
        ..EnvSpec::defaults_for(task, cfg)
    };

    match task {
        Task::LevelHold => {
            let reward_cfg: LevelHoldRewardConfig = load_or_default(path);
            let env = task::level_hold_env(&spec, reward_cfg);
            pretrain::<B, _>(env, steps, bc_epochs, minibatch, &save_path, seed);
        }
        Task::HeadingHold => {
            let reward_cfg: HeadingHoldRewardConfig = load_or_default(path);
            let env = task::heading_hold_env(&spec, reward_cfg);
            pretrain::<B, _>(env, steps, bc_epochs, minibatch, &save_path, seed);
        }
        Task::Orbit => {
            let reward_cfg: OrbitRewardConfig = load_or_default(path);
            let env = task::orbit_env(&spec, reward_cfg);
            pretrain::<B, _>(env, steps, bc_epochs, minibatch, &save_path, seed);
        }
        // Rejected at argument-parse time; `bc_default_stem` is the gate.
        Task::ResidualOrbit | Task::LstmOrbit => unreachable!("not BC-capable"),
    }

    #[allow(clippy::too_many_arguments)]
    fn pretrain<B, E>(
        env: E,
        steps: usize,
        bc_epochs: usize,
        minibatch: usize,
        save_path: &str,
        seed: Option<u64>,
    ) where
        B: burn::tensor::backend::AutodiffBackend,
        B::Device: Default,
        E: DemonstrationEnv + Clone,
    {
        let device: B::Device = Default::default();

        println!("Collecting {steps} demonstration steps from the PID expert...");
        let collect_start = Instant::now();
        let mut demo_env = env.clone();
        let data = collect_demonstrations(&mut demo_env, steps);
        println!(
            "Collected {} pairs in {:.1}s.",
            data.len(),
            collect_start.elapsed().as_secs_f64()
        );

        let mut trainer = PpoTrainer::<B, E>::new_seeded(env, device, seed);
        println!("Behavior cloning for {bc_epochs} epochs (minibatch {minibatch})...");
        let train_start = Instant::now();
        let print_every = (bc_epochs / 10).max(1);
        let mut last_mse = f32::NAN;
        for epoch in 0..bc_epochs {
            last_mse = trainer.pretrain_bc(&data, 1, minibatch);
            if epoch % print_every == 0 || epoch + 1 == bc_epochs {
                println!(
                    "  epoch {:>4}/{bc_epochs}  mse {last_mse:.6}  {:.1}s",
                    epoch + 1,
                    train_start.elapsed().as_secs_f64()
                );
            }
        }
        println!(
            "BC complete (final mse {last_mse:.6}, {:.1}s).",
            train_start.elapsed().as_secs_f64()
        );

        let save_dir = std::path::Path::new(save_path)
            .parent()
            .unwrap_or(std::path::Path::new("models"));
        std::fs::create_dir_all(save_dir).expect("create model output dir");
        trainer.save_policy(save_path);
    }
}
