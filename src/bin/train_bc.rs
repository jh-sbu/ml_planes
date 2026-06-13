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
//!   --task <level_hold|orbit>   Task / expert to clone (default: level_hold).
//!   --steps <n>                 Demonstration steps to collect (default: 200_000).
//!   --bc-epochs <n>             Supervised epochs over the dataset (default: 10).
//!   --minibatch <n>             Minibatch size (default: 256).
//!   --output <stem>             Save to models/<task>/<stem>.mpk (auto-increments if omitted).
//!   --backend <wgpu|ndarray>    Compute backend (default: wgpu).
//!   --reward-config <path>      Reward/termination profile path, overriding the task default
//!                               (assets/training/<task>.reward.ron). Missing file → defaults.

#[cfg(not(feature = "training"))]
fn main() {
    eprintln!("Build with --features training to run behavior-cloning pretraining.");
    eprintln!("  cargo run --no-default-features --features training --bin train_bc");
}

#[cfg(feature = "training")]
fn main() {
    use burn::backend::{Autodiff, NdArray, Wgpu};

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
        .unwrap_or(Ok(Backend::Wgpu))
        .unwrap_or_else(|e| {
            eprintln!("{e}");
            std::process::exit(2);
        });

    let save_path = save_path_for(task, find("--output"));

    // Optional reward-profile override; defaults to the task baseline profile.
    let reward_config = find("--reward-config");

    match backend {
        Backend::NdArray => {
            run::<Autodiff<NdArray>>(task, steps, bc_epochs, minibatch, save_path, reward_config)
        }
        Backend::Wgpu => {
            run::<Autodiff<Wgpu>>(task, steps, bc_epochs, minibatch, save_path, reward_config)
        }
    }
}

#[cfg(feature = "training")]
#[derive(Clone, Copy)]
enum Backend {
    NdArray,
    Wgpu,
}

#[cfg(feature = "training")]
impl Backend {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "ndarray" | "cpu" => Ok(Self::NdArray),
            "wgpu" => Ok(Self::Wgpu),
            other => Err(format!(
                "Unsupported --backend '{other}'. Use 'wgpu', 'ndarray', or 'cpu'."
            )),
        }
    }
}

#[cfg(feature = "training")]
#[derive(Clone, Copy)]
enum Task {
    LevelHold,
    Orbit,
}

#[cfg(feature = "training")]
impl Task {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "level_hold" => Ok(Self::LevelHold),
            "orbit" => Ok(Self::Orbit),
            other => Err(format!(
                "Unsupported --task '{other}'. Behavior cloning supports 'level_hold' or 'orbit'."
            )),
        }
    }

    fn reward_config_path(self) -> &'static str {
        match self {
            Self::LevelHold => "assets/training/level_hold.reward.ron",
            Self::Orbit => "assets/training/orbit.reward.ron",
        }
    }

    fn model_dir(self) -> &'static str {
        match self {
            Self::LevelHold => "level_hold",
            Self::Orbit => "orbit",
        }
    }

    fn default_stem(self) -> &'static str {
        match self {
            Self::LevelHold => "bc_level_hold",
            Self::Orbit => "bc_orbit",
        }
    }
}

#[cfg(feature = "training")]
fn save_path_for(task: Task, output_stem: Option<String>) -> String {
    let dir = task.model_dir();
    match output_stem {
        Some(stem) => format!("models/{dir}/{stem}"),
        None => {
            let mut n = 1u32;
            loop {
                let candidate = format!("models/{dir}/{}_{n}", task.default_stem());
                if !std::path::Path::new(&format!("{candidate}.mpk")).exists() {
                    break candidate;
                }
                n += 1;
            }
        }
    }
}

#[cfg(feature = "training")]
fn run<B>(
    task: Task,
    steps: usize,
    bc_epochs: usize,
    minibatch: usize,
    save_path: String,
    reward_config: Option<String>,
) where
    B: burn::tensor::backend::AutodiffBackend,
    B::Device: Default,
{
    use bevy::math::Vec3;
    use std::time::Instant;

    use ml_planes::plane::config::PlaneConfig;
    use ml_planes::training::ppo::PpoTrainer;
    use ml_planes::training::reward_config::{
        load_reward_config, LevelHoldRewardConfig, OrbitRewardConfig,
    };
    use ml_planes::training::{collect_demonstrations, DemonstrationEnv, LevelHoldEnv, OrbitEnv};

    // Generic jet values matching assets/planes/generic_jet.plane.ron
    let cfg = PlaneConfig {
        wing_area: 20.0,
        mean_chord: 2.0,
        wing_span: 10.0,
        mass: 5000.0,
        inertia: Vec3::new(10000.0, 40000.0, 45000.0),
        cl0: 0.1,
        cl_alpha: 4.5,
        cl_delta_e: 0.4,
        cl_max: 1.4,
        cd0: 0.02,
        cd_induced: 0.05,
        cm0: -0.02,
        cm_alpha: 0.6,
        cm_q: -14.0,
        cm_delta_e: -1.2,
        cl_beta: -0.08,
        cl_p: -0.45,
        cl_r: 0.12,
        cl_delta_a: 0.18,
        cn_beta: 0.10,
        cn_r: -0.12,
        cn_delta_r: -0.10,
        thrust_max: 60000.0,
        powerplant: Default::default(),
        aileron_limit: 0.4363,
        elevator_limit: 0.3491,
        rudder_limit: 0.2618,
    };

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
    match task {
        Task::LevelHold => {
            let reward_cfg: LevelHoldRewardConfig = load_or_default(path);
            let env = LevelHoldEnv::with_reward_config(1000.0, 100.0, cfg, reward_cfg);
            pretrain::<B, _>(env, steps, bc_epochs, minibatch, &save_path);
        }
        Task::Orbit => {
            let reward_cfg: OrbitRewardConfig = load_or_default(path);
            let env = OrbitEnv::with_reward_config(1000.0, 100.0, 1000.0, cfg, reward_cfg);
            pretrain::<B, _>(env, steps, bc_epochs, minibatch, &save_path);
        }
    }

    fn pretrain<B, E>(env: E, steps: usize, bc_epochs: usize, minibatch: usize, save_path: &str)
    where
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

        let mut trainer = PpoTrainer::<B, E>::new(env, device);
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
