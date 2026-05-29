#![recursion_limit = "256"]

//! PPO training binary.  Build and run with:
//!   cargo run --no-default-features --features training --bin train_ppo
//!
//! Trains a PPO controller for 2 000 000 environment steps and saves the
//! policy under `models/<task>/`.
//!
//! Flags:
//!   --task <level_hold|orbit|residual_orbit>  Training task (default: level_hold).
//!   --plain                   Print the traditional metrics table instead of the TUI display.
//!   --output <stem>           Save the model to `models/<task>/<stem>.mpk`.
//!                             If omitted, auto-increments: ppo_level_hold_1.mpk,
//!                             ppo_orbit_1.mpk, … (never overwrites an existing file).
//!   --steps <n>               Total environment steps (default: 2_000_000).
//!   --backend <wgpu|ndarray>  Compute backend (default: wgpu).
//!   --log-file <path>         Write a CSV training log (reward config header + per-iteration
//!                             metrics) to <path>. Compatible with pandas / gnuplot / Excel.
//!   --reward-config <path>    Load reward/termination profile from <path> instead of the
//!                             task default (assets/training/<task>.reward.ron). A missing
//!                             file falls back to the compiled defaults with a warning.

#[cfg(not(feature = "training"))]
fn main() {
    eprintln!("Build with --features training to run PPO training.");
    eprintln!("  cargo run --no-default-features --features training --bin train_ppo");
}

#[cfg(feature = "training")]
fn main() {
    use burn::backend::{Autodiff, NdArray, Wgpu};

    let plain = std::env::args().any(|a| a == "--plain");

    let args: Vec<String> = std::env::args().collect();

    let task = args
        .windows(2)
        .find(|w| w[0] == "--task")
        .map(|w| Task::parse(&w[1]))
        .unwrap_or(Ok(Task::LevelHold))
        .unwrap_or_else(|e| {
            eprintln!("{e}");
            std::process::exit(2);
        });

    let output_stem: Option<String> = args
        .windows(2)
        .find(|w| w[0] == "--output")
        .map(|w| w[1].clone());

    let backend = args
        .windows(2)
        .find(|w| w[0] == "--backend")
        .map(|w| Backend::parse(&w[1]))
        .unwrap_or(Ok(Backend::Wgpu))
        .unwrap_or_else(|e| {
            eprintln!("{e}");
            std::process::exit(2);
        });

    let total_timesteps: usize = args
        .windows(2)
        .find(|w| w[0] == "--steps")
        .map(|w| {
            w[1].parse::<usize>().unwrap_or_else(|_| {
                eprintln!("--steps must be a positive integer");
                std::process::exit(2);
            })
        })
        .unwrap_or(2_000_000);

    let init_from: Option<String> = args
        .windows(2)
        .find(|w| w[0] == "--init-from")
        .map(|w| w[1].clone());

    // Optional behavior-cloning warm-start: collect this many PID-expert demo steps
    // and supervised-pretrain the policy before the PPO loop (level_hold / orbit only).
    let bc_steps: usize = args
        .windows(2)
        .find(|w| w[0] == "--bc-steps")
        .map(|w| {
            w[1].parse::<usize>().unwrap_or_else(|_| {
                eprintln!("--bc-steps must be a non-negative integer");
                std::process::exit(2);
            })
        })
        .unwrap_or(0);

    let bc_epochs: usize = args
        .windows(2)
        .find(|w| w[0] == "--bc-epochs")
        .map(|w| {
            w[1].parse::<usize>().unwrap_or_else(|_| {
                eprintln!("--bc-epochs must be a positive integer");
                std::process::exit(2);
            })
        })
        .unwrap_or(10);

    let log_file: Option<String> = args
        .windows(2)
        .find(|w| w[0] == "--log-file")
        .map(|w| w[1].clone());

    // Optional reward-profile override. Defaults to the task's baseline profile
    // (see `Task::reward_config_path`). A missing file falls back to defaults.
    let reward_config: Option<String> = args
        .windows(2)
        .find(|w| w[0] == "--reward-config")
        .map(|w| w[1].clone());

    let save_path = save_path_for(task, output_stem);

    match backend {
        Backend::NdArray => run::<Autodiff<NdArray>>(
            plain,
            save_path,
            task,
            total_timesteps,
            init_from,
            log_file,
            reward_config,
            bc_steps,
            bc_epochs,
        ),
        Backend::Wgpu => run::<Autodiff<Wgpu>>(
            plain,
            save_path,
            task,
            total_timesteps,
            init_from,
            log_file,
            reward_config,
            bc_steps,
            bc_epochs,
        ),
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
    ResidualOrbit,
    LstmOrbit,
}

#[cfg(feature = "training")]
impl Task {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "level_hold" => Ok(Self::LevelHold),
            "orbit" => Ok(Self::Orbit),
            "residual_orbit" => Ok(Self::ResidualOrbit),
            "lstm_orbit" => Ok(Self::LstmOrbit),
            other => Err(format!(
                "Unsupported --task '{other}'. Use 'level_hold', 'orbit', 'residual_orbit', or 'lstm_orbit'."
            )),
        }
    }

    fn reward_config_path(self) -> &'static str {
        match self {
            Self::LevelHold => "assets/training/level_hold.reward.ron",
            Self::Orbit | Self::ResidualOrbit => "assets/training/orbit.reward.ron",
            Self::LstmOrbit => "assets/training/wu_orbit.reward.ron",
        }
    }

    fn model_dir(self) -> &'static str {
        match self {
            Self::LevelHold => "level_hold",
            Self::Orbit => "orbit",
            Self::ResidualOrbit => "orbit_residual",
            Self::LstmOrbit => "lstm_orbit",
        }
    }

    fn default_stem(self) -> &'static str {
        match self {
            Self::LevelHold => "ppo_level_hold",
            Self::Orbit => "ppo_orbit",
            Self::ResidualOrbit => "ppo_orbit_residual",
            Self::LstmOrbit => "ppo_lstm_orbit",
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
#[allow(clippy::too_many_arguments)]
fn run<B>(
    plain: bool,
    save_path: String,
    task: Task,
    total_timesteps: usize,
    init_from: Option<String>,
    log_file: Option<String>,
    reward_config: Option<String>,
    bc_steps: usize,
    bc_epochs: usize,
) where
    B: burn::tensor::backend::AutodiffBackend,
    B::Device: Default,
{
    use bevy::math::Vec3;

    use ml_planes::plane::config::PlaneConfig;
    use ml_planes::training::ppo::CsvLog;
    use ml_planes::training::reward_config::{
        load_reward_config, LevelHoldRewardConfig, OrbitRewardConfig,
    };
    use ml_planes::training::{
        LevelHoldEnv, OrbitEnv, ResidualOrbitEnv, WuOrbitEnv, WuOrbitRewardConfig,
    };

    fn open_log(
        path: &Option<String>,
        task_name: &str,
        config_fields: Vec<(&'static str, String)>,
    ) -> Option<CsvLog> {
        let path = path.as_deref()?;
        match CsvLog::new(path) {
            Ok(mut log) => {
                let mut comments = vec![("task", task_name.to_string())];
                comments.extend(config_fields);
                log.write_comments(&comments).ok();
                log.write_header().ok();
                Some(log)
            }
            Err(e) => {
                eprintln!("Warning: could not open log file {path}: {e}");
                None
            }
        }
    }

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
        aileron_limit: 0.4363,
        elevator_limit: 0.3491,
        rudder_limit: 0.2618,
    };

    // Effective reward-profile path: the CLI override if given, else the task default.
    let reward_path = reward_config.unwrap_or_else(|| task.reward_config_path().to_string());

    match task {
        Task::LevelHold => {
            let path = reward_path.as_str();
            let reward_cfg: LevelHoldRewardConfig = match load_reward_config(path) {
                Ok(cfg) => {
                    println!("Loaded reward config from {path}");
                    cfg
                }
                Err(e) => {
                    eprintln!("Warning: could not load {path}: {e}. Using defaults.");
                    LevelHoldRewardConfig::default()
                }
            };
            let log = open_log(&log_file, "level_hold", reward_cfg.log_fields());
            run_training_loop_bc::<B, _>(
                plain,
                save_path,
                total_timesteps,
                init_from,
                LevelHoldEnv::with_reward_config(1000.0, 100.0, cfg, reward_cfg),
                log,
                bc_steps,
                bc_epochs,
            )
        }
        Task::Orbit => {
            let path = reward_path.as_str();
            let reward_cfg: OrbitRewardConfig = match load_reward_config(path) {
                Ok(cfg) => {
                    println!("Loaded reward config from {path}");
                    cfg
                }
                Err(e) => {
                    eprintln!("Warning: could not load {path}: {e}. Using defaults.");
                    OrbitRewardConfig::default()
                }
            };
            let log = open_log(&log_file, "orbit", reward_cfg.log_fields());
            run_training_loop_bc::<B, _>(
                plain,
                save_path,
                total_timesteps,
                init_from,
                OrbitEnv::with_reward_config(1000.0, 100.0, 1000.0, cfg, reward_cfg),
                log,
                bc_steps,
                bc_epochs,
            )
        }
        Task::ResidualOrbit => {
            let path = reward_path.as_str();
            let reward_cfg: OrbitRewardConfig = match load_reward_config(path) {
                Ok(cfg) => {
                    println!("Loaded reward config from {path}");
                    cfg
                }
                Err(e) => {
                    eprintln!("Warning: could not load {path}: {e}. Using defaults.");
                    OrbitRewardConfig::default()
                }
            };
            let log = open_log(&log_file, "residual_orbit", reward_cfg.log_fields());
            run_training_loop::<B, _>(
                plain,
                save_path,
                total_timesteps,
                init_from,
                ResidualOrbitEnv::with_reward_config(1000.0, 100.0, 1000.0, cfg, reward_cfg),
                log,
            )
        }
        Task::LstmOrbit => {
            let path = reward_path.as_str();
            let reward_cfg: WuOrbitRewardConfig = match load_reward_config(path) {
                Ok(cfg) => {
                    println!("Loaded Wu orbit reward config from {path}");
                    cfg
                }
                Err(e) => {
                    eprintln!("Warning: could not load {path}: {e}. Using defaults.");
                    WuOrbitRewardConfig::default()
                }
            };
            let log = open_log(&log_file, "lstm_orbit", reward_cfg.log_fields());
            run_lstm_training_loop::<B>(
                plain,
                save_path,
                total_timesteps,
                init_from,
                WuOrbitEnv::with_reward_config(1000.0, 100.0, 3000.0, cfg, reward_cfg),
                log,
            )
        }
    }
}

#[cfg(feature = "training")]
fn run_training_loop<B, E>(
    plain: bool,
    save_path: String,
    total_timesteps: usize,
    init_from: Option<String>,
    env: E,
    log: Option<ml_planes::training::ppo::CsvLog>,
) where
    B: burn::tensor::backend::AutodiffBackend,
    B::Device: Default,
    E: ml_planes::training::TrainingEnv + Clone,
{
    use ml_planes::training::ppo::PpoTrainer;

    let device: B::Device = Default::default();
    let mut trainer = PpoTrainer::<B, E>::with_n_envs(env, 8, device);
    if let Some(ref path) = init_from {
        trainer.load_policy(path);
    }
    run_ppo_loop::<B, E>(trainer, plain, save_path, total_timesteps, log);
}

/// Like [`run_training_loop`], but optionally behavior-clones the policy from the
/// task's PID expert (`bc_steps` demo steps, `bc_epochs` supervised epochs) before
/// the PPO loop. With `total_timesteps == 0` this is effectively BC-only.
#[cfg(feature = "training")]
#[allow(clippy::too_many_arguments)]
fn run_training_loop_bc<B, E>(
    plain: bool,
    save_path: String,
    total_timesteps: usize,
    init_from: Option<String>,
    env: E,
    log: Option<ml_planes::training::ppo::CsvLog>,
    bc_steps: usize,
    bc_epochs: usize,
) where
    B: burn::tensor::backend::AutodiffBackend,
    B::Device: Default,
    E: ml_planes::training::DemonstrationEnv + Clone,
{
    use ml_planes::training::{collect_demonstrations, ppo::PpoTrainer};

    let device: B::Device = Default::default();
    let mut trainer = PpoTrainer::<B, E>::with_n_envs(env.clone(), 8, device);
    if let Some(ref path) = init_from {
        trainer.load_policy(path);
    }

    if bc_steps > 0 {
        use std::time::Instant;
        println!("BC warm-start: collecting {bc_steps} PID-expert demo steps...");
        let t0 = Instant::now();
        let mut demo_env = env;
        let data = collect_demonstrations(&mut demo_env, bc_steps);
        println!(
            "Collected {} pairs in {:.1}s; behavior cloning for {bc_epochs} epochs...",
            data.len(),
            t0.elapsed().as_secs_f64()
        );
        let mb = trainer.minibatch.min(data.len()).max(1);
        let mut mse = f32::NAN;
        for epoch in 0..bc_epochs {
            mse = trainer.pretrain_bc(&data, 1, mb);
            println!("  BC epoch {:>3}/{bc_epochs}  mse {mse:.6}", epoch + 1);
        }
        println!("BC warm-start complete (final mse {mse:.6}). Starting PPO fine-tune...");
    }

    run_ppo_loop::<B, E>(trainer, plain, save_path, total_timesteps, log);
}

#[cfg(feature = "training")]
fn run_ppo_loop<B, E>(
    mut trainer: ml_planes::training::ppo::PpoTrainer<B, E>,
    plain: bool,
    save_path: String,
    total_timesteps: usize,
    mut log: Option<ml_planes::training::ppo::CsvLog>,
) where
    B: burn::tensor::backend::AutodiffBackend,
    B::Device: Default,
    E: ml_planes::training::TrainingEnv + Clone,
{
    use std::sync::Arc;
    use std::time::Instant;

    use burn::data::dataloader::Progress;
    use burn::train::metric::{MetricAttributes, MetricDefinition, MetricId, NumericAttributes};
    use burn::train::renderer::tui::TuiMetricsRenderer;
    use burn::train::renderer::{MetricsRenderer, TrainingProgress};
    use burn::train::Interrupter;

    let total_iterations = total_timesteps.div_ceil(trainer.rollout_steps);

    // --- metric IDs ---
    let id_mean_return = MetricId::new(Arc::new("mean_return".to_string()));
    let id_ep_len = MetricId::new(Arc::new("ep_len".to_string()));
    let id_policy_loss = MetricId::new(Arc::new("policy_loss".to_string()));
    let id_value_loss = MetricId::new(Arc::new("value_loss".to_string()));
    let id_entropy = MetricId::new(Arc::new("entropy".to_string()));
    let id_steps_per_sec = MetricId::new(Arc::new("steps_per_sec".to_string()));

    let interrupter = Interrupter::new();

    let mut renderer: Box<dyn MetricsRenderer> = if plain {
        Box::new(PlainMetricsRenderer::new(
            total_timesteps,
            id_mean_return.clone(),
            id_ep_len.clone(),
            id_policy_loss.clone(),
            id_value_loss.clone(),
            id_entropy.clone(),
            id_steps_per_sec.clone(),
        ))
    } else {
        Box::new(TuiMetricsRenderer::new(interrupter.clone(), None))
    };

    // Register metrics — must happen before any update_train call.
    let definitions = [
        MetricDefinition {
            metric_id: id_mean_return.clone(),
            name: "Mean Return".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes {
                unit: None,
                higher_is_better: true,
            }),
        },
        MetricDefinition {
            metric_id: id_ep_len.clone(),
            name: "Ep Length".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes {
                unit: Some("steps".into()),
                higher_is_better: true,
            }),
        },
        MetricDefinition {
            metric_id: id_policy_loss.clone(),
            name: "Policy Loss".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes {
                unit: None,
                higher_is_better: false,
            }),
        },
        MetricDefinition {
            metric_id: id_value_loss.clone(),
            name: "Value Loss".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes {
                unit: None,
                higher_is_better: false,
            }),
        },
        MetricDefinition {
            metric_id: id_entropy.clone(),
            name: "Entropy".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes {
                unit: None,
                higher_is_better: true,
            }),
        },
        MetricDefinition {
            metric_id: id_steps_per_sec.clone(),
            name: "Steps/s".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes {
                unit: Some("sps".into()),
                higher_is_better: true,
            }),
        },
    ];
    for def in definitions {
        renderer.register_metric(def);
    }

    let mut steps = 0usize;
    let mut iteration = 0usize;
    let start = Instant::now();

    while steps < total_timesteps {
        let (buffer, mean_return, mean_ep_len) = trainer.collect_rollout();
        steps += buffer.len();
        let metrics = trainer.update(&buffer);
        iteration += 1;

        let elapsed_s = start.elapsed().as_secs_f64();
        let steps_per_sec = steps as f64 / elapsed_s.max(1e-6);

        if let Some(ref mut log) = log {
            log.write_row(
                iteration,
                steps,
                total_timesteps,
                elapsed_s,
                steps_per_sec,
                mean_return,
                mean_ep_len,
                metrics.policy_loss,
                metrics.value_loss,
                metrics.entropy,
            )
            .unwrap_or_else(|e| eprintln!("log write error: {e}"));
        }

        renderer.update_train(numeric_state(
            id_mean_return.clone(),
            format!("{mean_return:.3}"),
            mean_return as f64,
        ));
        renderer.update_train(numeric_state(
            id_ep_len.clone(),
            format!("{mean_ep_len:.0}"),
            mean_ep_len as f64,
        ));
        renderer.update_train(numeric_state(
            id_policy_loss.clone(),
            format!("{:.4}", metrics.policy_loss),
            metrics.policy_loss as f64,
        ));
        renderer.update_train(numeric_state(
            id_value_loss.clone(),
            format!("{:.4}", metrics.value_loss),
            metrics.value_loss as f64,
        ));
        renderer.update_train(numeric_state(
            id_entropy.clone(),
            format!("{:.4}", metrics.entropy),
            metrics.entropy as f64,
        ));
        renderer.update_train(numeric_state(
            id_steps_per_sec.clone(),
            format!("{steps_per_sec:.0}"),
            steps_per_sec,
        ));

        renderer.render_train(TrainingProgress {
            progress: Progress {
                items_processed: steps,
                items_total: total_timesteps,
            },
            epoch: iteration,
            epoch_total: total_iterations,
            iteration,
        });

        if interrupter.should_stop() {
            break;
        }
    }

    renderer.on_train_end(None).ok();
    drop(renderer); // restore terminal before printing to it

    let elapsed_secs = start.elapsed().as_secs();
    println!(
        "Training complete ({steps} steps, {iteration} iterations, elapsed {}).",
        fmt_duration(elapsed_secs),
    );

    if let Some(ref mut log) = log {
        log.flush().ok();
    }

    let save_dir = std::path::Path::new(&save_path)
        .parent()
        .unwrap_or(std::path::Path::new("models"));
    std::fs::create_dir_all(save_dir).expect("create model output dir");
    trainer.save_policy(&save_path);
}

#[cfg(feature = "training")]
fn run_lstm_training_loop<B>(
    plain: bool,
    save_path: String,
    total_timesteps: usize,
    init_from: Option<String>,
    env: ml_planes::training::WuOrbitEnv,
    mut log: Option<ml_planes::training::ppo::CsvLog>,
) where
    B: burn::tensor::backend::AutodiffBackend,
    B::Device: Default,
{
    use std::sync::Arc;
    use std::time::Instant;

    use burn::data::dataloader::Progress;
    use burn::train::metric::{MetricAttributes, MetricDefinition, MetricId, NumericAttributes};
    use burn::train::renderer::tui::TuiMetricsRenderer;
    use burn::train::renderer::{MetricsRenderer, TrainingProgress};
    use burn::train::Interrupter;

    use ml_planes::training::ppo::LstmPpoTrainer;

    let device: B::Device = Default::default();
    let mut trainer = LstmPpoTrainer::<B, _>::with_n_envs(env, 32, device);
    if let Some(ref path) = init_from {
        trainer.load_policy(path);
    }

    let total_iterations = total_timesteps.div_ceil(trainer.rollout_steps);

    let id_mean_return = MetricId::new(Arc::new("mean_return".to_string()));
    let id_ep_len = MetricId::new(Arc::new("ep_len".to_string()));
    let id_policy_loss = MetricId::new(Arc::new("policy_loss".to_string()));
    let id_value_loss = MetricId::new(Arc::new("value_loss".to_string()));
    let id_entropy = MetricId::new(Arc::new("entropy".to_string()));
    let id_steps_per_sec = MetricId::new(Arc::new("steps_per_sec".to_string()));

    let interrupter = Interrupter::new();

    let mut renderer: Box<dyn MetricsRenderer> = if plain {
        Box::new(PlainMetricsRenderer::new(
            total_timesteps,
            id_mean_return.clone(),
            id_ep_len.clone(),
            id_policy_loss.clone(),
            id_value_loss.clone(),
            id_entropy.clone(),
            id_steps_per_sec.clone(),
        ))
    } else {
        Box::new(TuiMetricsRenderer::new(interrupter.clone(), None))
    };

    let definitions = [
        MetricDefinition {
            metric_id: id_mean_return.clone(),
            name: "Mean Return".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes {
                unit: None,
                higher_is_better: true,
            }),
        },
        MetricDefinition {
            metric_id: id_ep_len.clone(),
            name: "Ep Length".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes {
                unit: Some("steps".into()),
                higher_is_better: true,
            }),
        },
        MetricDefinition {
            metric_id: id_policy_loss.clone(),
            name: "Policy Loss".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes {
                unit: None,
                higher_is_better: false,
            }),
        },
        MetricDefinition {
            metric_id: id_value_loss.clone(),
            name: "Value Loss".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes {
                unit: None,
                higher_is_better: false,
            }),
        },
        MetricDefinition {
            metric_id: id_entropy.clone(),
            name: "Entropy".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes {
                unit: None,
                higher_is_better: true,
            }),
        },
        MetricDefinition {
            metric_id: id_steps_per_sec.clone(),
            name: "Steps/s".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes {
                unit: Some("sps".into()),
                higher_is_better: true,
            }),
        },
    ];
    for def in definitions {
        renderer.register_metric(def);
    }

    let mut steps = 0usize;
    let mut iteration = 0usize;
    let start = Instant::now();

    while steps < total_timesteps {
        let (buffer, mean_return, mean_ep_len) = trainer.collect_rollout();
        steps += buffer.len();
        let metrics = trainer.update(&buffer);
        trainer.advance_curriculum_if_ready(mean_return);
        iteration += 1;

        let elapsed_s = start.elapsed().as_secs_f64();
        let steps_per_sec = steps as f64 / elapsed_s.max(1e-6);

        if let Some(ref mut log) = log {
            log.write_row(
                iteration,
                steps,
                total_timesteps,
                elapsed_s,
                steps_per_sec,
                mean_return,
                mean_ep_len,
                metrics.policy_loss,
                metrics.value_loss,
                metrics.entropy,
            )
            .unwrap_or_else(|e| eprintln!("log write error: {e}"));
        }

        renderer.update_train(numeric_state(
            id_mean_return.clone(),
            format!("{mean_return:.3}"),
            mean_return as f64,
        ));
        renderer.update_train(numeric_state(
            id_ep_len.clone(),
            format!("{mean_ep_len:.0}"),
            mean_ep_len as f64,
        ));
        renderer.update_train(numeric_state(
            id_policy_loss.clone(),
            format!("{:.4}", metrics.policy_loss),
            metrics.policy_loss as f64,
        ));
        renderer.update_train(numeric_state(
            id_value_loss.clone(),
            format!("{:.4}", metrics.value_loss),
            metrics.value_loss as f64,
        ));
        renderer.update_train(numeric_state(
            id_entropy.clone(),
            format!("{:.4}", metrics.entropy),
            metrics.entropy as f64,
        ));
        renderer.update_train(numeric_state(
            id_steps_per_sec.clone(),
            format!("{steps_per_sec:.0}"),
            steps_per_sec,
        ));

        renderer.render_train(TrainingProgress {
            progress: Progress {
                items_processed: steps,
                items_total: total_timesteps,
            },
            epoch: iteration,
            epoch_total: total_iterations,
            iteration,
        });

        if interrupter.should_stop() {
            break;
        }
    }

    renderer.on_train_end(None).ok();
    drop(renderer);

    let elapsed_secs = start.elapsed().as_secs();
    println!(
        "Training complete ({steps} steps, {iteration} iterations, elapsed {}).",
        fmt_duration(elapsed_secs),
    );

    if let Some(ref mut log) = log {
        log.flush().ok();
    }

    let save_dir = std::path::Path::new(&save_path)
        .parent()
        .unwrap_or(std::path::Path::new("models"));
    std::fs::create_dir_all(save_dir).expect("create model output dir");
    trainer.save_policy(&save_path);
}

#[cfg(feature = "training")]
fn numeric_state(
    id: burn::train::metric::MetricId,
    formatted: String,
    value: f64,
) -> burn::train::renderer::MetricState {
    use burn::train::metric::{MetricEntry, NumericEntry, SerializedEntry};
    use burn::train::renderer::MetricState;
    MetricState::Numeric(
        MetricEntry::new(id, SerializedEntry::new(formatted, value.to_string())),
        NumericEntry::Value(value),
    )
}

#[cfg(feature = "training")]
fn fmt_duration(secs: u64) -> String {
    let h = secs / 3600;
    let m = (secs % 3600) / 60;
    let s = secs % 60;
    if h > 0 {
        format!("{}:{:02}:{:02}", h, m, s)
    } else {
        format!("{:02}:{:02}", m, s)
    }
}

/// Plain table renderer — replicates the original metrics printout.
#[cfg(feature = "training")]
struct PlainMetricsRenderer {
    start: std::time::Instant,
    total_timesteps: usize,
    rows_since_header: usize,
    id_mean_return: burn::train::metric::MetricId,
    id_ep_len: burn::train::metric::MetricId,
    id_policy_loss: burn::train::metric::MetricId,
    id_value_loss: burn::train::metric::MetricId,
    id_entropy: burn::train::metric::MetricId,
    id_steps_per_sec: burn::train::metric::MetricId,
    mean_return: f64,
    ep_len: f64,
    policy_loss: f64,
    value_loss: f64,
    entropy: f64,
    steps_per_sec: f64,
}

#[cfg(feature = "training")]
impl PlainMetricsRenderer {
    fn new(
        total_timesteps: usize,
        id_mean_return: burn::train::metric::MetricId,
        id_ep_len: burn::train::metric::MetricId,
        id_policy_loss: burn::train::metric::MetricId,
        id_value_loss: burn::train::metric::MetricId,
        id_entropy: burn::train::metric::MetricId,
        id_steps_per_sec: burn::train::metric::MetricId,
    ) -> Self {
        println!("Starting PPO training — target {} steps", total_timesteps);
        Self {
            start: std::time::Instant::now(),
            total_timesteps,
            rows_since_header: 0,
            id_mean_return,
            id_ep_len,
            id_policy_loss,
            id_value_loss,
            id_entropy,
            id_steps_per_sec,
            mean_return: 0.0,
            ep_len: 0.0,
            policy_loss: 0.0,
            value_loss: 0.0,
            entropy: 0.0,
            steps_per_sec: 0.0,
        }
    }
}

#[cfg(feature = "training")]
impl burn::train::renderer::MetricsRendererTraining for PlainMetricsRenderer {
    fn update_train(&mut self, state: burn::train::renderer::MetricState) {
        use burn::train::renderer::MetricState;
        if let MetricState::Numeric(entry, num) = state {
            let v = num.current();
            if entry.metric_id == self.id_mean_return {
                self.mean_return = v;
            } else if entry.metric_id == self.id_ep_len {
                self.ep_len = v;
            } else if entry.metric_id == self.id_policy_loss {
                self.policy_loss = v;
            } else if entry.metric_id == self.id_value_loss {
                self.value_loss = v;
            } else if entry.metric_id == self.id_entropy {
                self.entropy = v;
            } else if entry.metric_id == self.id_steps_per_sec {
                self.steps_per_sec = v;
            }
        }
    }

    fn update_valid(&mut self, _state: burn::train::renderer::MetricState) {}

    fn render_train(&mut self, item: burn::train::renderer::TrainingProgress) {
        let iteration = item.epoch;
        if iteration % 10 != 0 && iteration > 5 {
            return;
        }

        if self.rows_since_header == 0 {
            println!(
                "{:<6}  {:<11}  {:>5}  {:>8}  {:>9}  {:>9}  {:>8}  {:>6}  {:>8}  {:>8}  {:>8}",
                "iter",
                "steps",
                "pct",
                "steps/s",
                "elapsed",
                "eta",
                "mean_ret",
                "ep_len",
                "p_loss",
                "v_loss",
                "entropy",
            );
        }

        let steps = item.progress.items_processed;
        let elapsed_secs = self.start.elapsed().as_secs_f64();
        let pct = 100.0 * steps as f64 / self.total_timesteps as f64;
        let remaining = self.total_timesteps.saturating_sub(steps);
        let eta_secs = if self.steps_per_sec > 0.0 {
            (remaining as f64 / self.steps_per_sec) as u64
        } else {
            0
        };

        println!(
            "{:<6}  {:<11}  {:>4.1}%  {:>8.0}  {:>9}  {:>9}  {:>8.3}  {:>6.0}  {:>8.4}  {:>8.4}  {:>8.4}",
            iteration,
            steps,
            pct,
            self.steps_per_sec,
            fmt_duration(elapsed_secs as u64),
            fmt_duration(eta_secs),
            self.mean_return,
            self.ep_len,
            self.policy_loss,
            self.value_loss,
            self.entropy,
        );

        self.rows_since_header += 1;
        if self.rows_since_header >= 50 {
            self.rows_since_header = 0;
        }
    }

    fn render_valid(&mut self, _item: burn::train::renderer::TrainingProgress) {}
}

#[cfg(feature = "training")]
impl burn::train::renderer::MetricsRendererEvaluation for PlainMetricsRenderer {
    fn update_test(
        &mut self,
        _name: burn::train::renderer::EvaluationName,
        _state: burn::train::renderer::MetricState,
    ) {
    }
    fn render_test(&mut self, _item: burn::train::renderer::EvaluationProgress) {}
}

#[cfg(feature = "training")]
impl burn::train::renderer::MetricsRenderer for PlainMetricsRenderer {
    fn manual_close(&mut self) {}
    fn register_metric(&mut self, _definition: burn::train::metric::MetricDefinition) {}
}
