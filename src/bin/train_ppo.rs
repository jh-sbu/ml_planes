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
//!   --backend <ndarray|wgpu>  Compute backend (default: ndarray; wgpu requires
//!                             --features wgpu).
//!   --log-file <path>         Write a CSV training log (reward config header + per-iteration
//!                             metrics) to <path>. Compatible with pandas / gnuplot / Excel.
//!   --reward-config <path>    Load reward/termination profile from <path> instead of the
//!                             task default (assets/training/<task>.reward.ron). A missing
//!                             file falls back to the compiled defaults with a warning.
//!   --plane-config <path>     Airframe (.plane.ron) to train against (default:
//!                             assets/planes/generic_jet.plane.ron). Unlike the reward
//!                             and PPO profiles, an unreadable or invalid file is FATAL
//!                             (exit 2) rather than falling back to a default: the
//!                             airframe is the plant the policy is fitted to, so a silent
//!                             substitution would produce a checkpoint for a plane nobody
//!                             asked for. Recorded in the --log-file CSV header.
//!   --ppo-config <path>       Load PPO training-loop hyperparameters (gamma, gae_lambda,
//!                             clip_epsilon, value/entropy coefs, lr, rollout_steps,
//!                             n_epochs, minibatch, seed) from a RON file. Applies to the MLP
//!                             tasks (level_hold/orbit/residual_orbit); the lstm_orbit task
//!                             ignores everything except `seed`. Absent → the task's tuned
//!                             default if it has one (level_hold:
//!                             assets/training/level_hold.ppo.ron), else compiled defaults.
//!                             Invalid file → compiled defaults, with a warning.
//!   --seed <u64>              Fix the model's weight init, the minibatch-shuffle RNG, and
//!                             per-env episode resets, for a reproducible run. Applies to every
//!                             task (incl. lstm_orbit) and overrides `--ppo-config`'s `seed`
//!                             field if both are given. Omitted → unseeded. This makes the
//!                             *starting point* (weights, env sample
//!                             order, minibatch order) exactly reproducible, verified bit-exact
//!                             — but NOT the full multi-iteration trajectory: burn's `ndarray`
//!                             matmul backend has its own tiny (~1 ULP/iteration) floating-point
//!                             non-associativity that compounds over a run, independent of this
//!                             seed. Two `--seed`-matched runs will start identically and track
//!                             closely, but a long run's final checkpoint may not be byte-for-byte
//!                             identical. `wgpu` is best-effort only (GPU reduction order isn't
//!                             guaranteed stable either).
//!   --target-alt-range <MIN:MAX|VALUE>    level_hold/heading_hold only: target altitude [m] is
//!                             resampled from this range every episode (default: 500:5000). A
//!                             bare VALUE pins a single fixed target.
//!   --target-speed-range <MIN:MAX|VALUE>  level_hold/heading_hold only: target airspeed [m/s],
//!                             same MIN:MAX|VALUE form (default: 90:140 for both level_hold and
//!                             heading_hold).
//!   --target-heading-range <MIN:MAX|VALUE>  heading_hold only: target heading change
//!                             [**degrees**] resampled every episode (default: -180:180). A bare
//!                             VALUE pins a single fixed heading change. Spawns always start on
//!                             ground track 0, so the sampled value IS the required turn.

#[cfg(not(feature = "training"))]
fn main() {
    eprintln!("Build with --features training to run PPO training.");
    eprintln!("  cargo run --no-default-features --features training --bin train_ppo");
}

#[cfg(feature = "training")]
fn main() {
    #[cfg(feature = "wgpu")]
    use burn::backend::Wgpu;
    use burn::backend::{Autodiff, NdArray};
    use ml_planes::training::Backend;

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
        .unwrap_or(Ok(Backend::default()))
        .unwrap_or_else(|e| {
            eprintln!("{e}");
            std::process::exit(2);
        });
    println!("Backend: {}", backend.label());

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

    // The airframe to train against. Unlike --reward-config, an unreadable value is
    // FATAL (see `load_plane_config_or_exit`): the airframe is the plant, and a run
    // that silently flew a substitute would yield a checkpoint fitted to a plane
    // nobody asked for.
    let plane_config: String = args
        .windows(2)
        .find(|w| w[0] == "--plane-config")
        .map(|w| w[1].clone())
        .unwrap_or_else(|| ml_planes::training::DEFAULT_PLANE_CONFIG_PATH.to_string());

    // Optional PPO hyperparameter override (level_hold / orbit / residual_orbit).
    // Absent → the task's tuned default (`Task::default_ppo_config_path`), else
    // compiled defaults; a missing/invalid file falls back with a warning.
    let ppo_config: Option<String> = args
        .windows(2)
        .find(|w| w[0] == "--ppo-config")
        .map(|w| w[1].clone());

    // Optional fixed seed: fixes the model's weight init (and any later unseeded
    // `sample_action` noise, on backends whose `Backend::seed` sets a
    // process-global generator), the minibatch-shuffle RNG, and per-env episode
    // resets — see `ml_planes::training::PpoHyperparams::seed`. Overrides
    // whatever `--ppo-config`'s `seed` field is set to, if both are given.
    // Applies to every task, including `lstm_orbit` (which otherwise ignores
    // `--ppo-config`). Best-effort (not guaranteed bit-reproducible) on the
    // `wgpu` backend — burn's cubecl `Backend::seed` is likewise process-global,
    // but GPU reduction order isn't guaranteed stable.
    let cli_seed: Option<u64> = args.windows(2).find(|w| w[0] == "--seed").map(|w| {
        w[1].parse::<u64>().unwrap_or_else(|_| {
            eprintln!("--seed must be a non-negative integer");
            std::process::exit(2);
        })
    });

    // level_hold only: the per-episode target-altitude/airspeed envelope. See
    // `LevelHoldEnv::with_target_ranges`. Defaults are the jointly
    // stall-feasible envelope for the generic jet at full fuel.
    let target_alt_range = args
        .windows(2)
        .find(|w| w[0] == "--target-alt-range")
        .map(|w| {
            ml_planes::training::parse_f32_range(&w[1]).unwrap_or_else(|e| {
                eprintln!("--target-alt-range: {e}");
                std::process::exit(2);
            })
        })
        .unwrap_or(
            ml_planes::training::level_hold_env::DEFAULT_TARGET_ALT_MIN
                ..=ml_planes::training::level_hold_env::DEFAULT_TARGET_ALT_MAX,
        );
    let target_speed_range = args
        .windows(2)
        .find(|w| w[0] == "--target-speed-range")
        .map(|w| {
            ml_planes::training::parse_f32_range(&w[1]).unwrap_or_else(|e| {
                eprintln!("--target-speed-range: {e}");
                std::process::exit(2);
            })
        })
        .unwrap_or_else(|| match task {
            Task::HeadingHold => {
                ml_planes::training::heading_hold_env::DEFAULT_TARGET_AIRSPEED_MIN
                    ..=ml_planes::training::heading_hold_env::DEFAULT_TARGET_AIRSPEED_MAX
            }
            _ => {
                ml_planes::training::level_hold_env::DEFAULT_TARGET_AIRSPEED_MIN
                    ..=ml_planes::training::level_hold_env::DEFAULT_TARGET_AIRSPEED_MAX
            }
        });

    // heading_hold only: target heading CHANGE [deg], converted to radians at the boundary
    // (CLI/HUD convention is degrees; the env stores radians).
    let target_heading_range = args
        .windows(2)
        .find(|w| w[0] == "--target-heading-range")
        .map(|w| {
            ml_planes::training::parse_f32_range(&w[1]).unwrap_or_else(|e| {
                eprintln!("--target-heading-range: {e}");
                std::process::exit(2);
            })
        })
        .unwrap_or(
            ml_planes::training::heading_hold_env::DEFAULT_TARGET_HEADING_DEG_MIN
                ..=ml_planes::training::heading_hold_env::DEFAULT_TARGET_HEADING_DEG_MAX,
        );
    let target_heading_range =
        target_heading_range.start().to_radians()..=target_heading_range.end().to_radians();

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
            plane_config,
            ppo_config,
            cli_seed,
            bc_steps,
            bc_epochs,
            target_alt_range,
            target_speed_range,
            target_heading_range,
        ),
        #[cfg(feature = "wgpu")]
        Backend::Wgpu => run::<Autodiff<Wgpu>>(
            plain,
            save_path,
            task,
            total_timesteps,
            init_from,
            log_file,
            reward_config,
            plane_config,
            ppo_config,
            cli_seed,
            bc_steps,
            bc_epochs,
            target_alt_range,
            target_speed_range,
            target_heading_range,
        ),
    }
}

#[cfg(feature = "training")]
#[derive(Clone, Copy)]
enum Task {
    LevelHold,
    HeadingHold,
    Orbit,
    ResidualOrbit,
    LstmOrbit,
}

#[cfg(feature = "training")]
impl Task {
    fn parse(value: &str) -> Result<Self, String> {
        match value {
            "level_hold" => Ok(Self::LevelHold),
            "heading_hold" => Ok(Self::HeadingHold),
            "orbit" => Ok(Self::Orbit),
            "residual_orbit" => Ok(Self::ResidualOrbit),
            "lstm_orbit" => Ok(Self::LstmOrbit),
            other => Err(format!(
                "Unsupported --task '{other}'. Use 'level_hold', 'heading_hold', 'orbit', 'residual_orbit', or 'lstm_orbit'."
            )),
        }
    }

    fn reward_config_path(self) -> &'static str {
        match self {
            Self::LevelHold => "assets/training/level_hold.reward.ron",
            Self::HeadingHold => "assets/training/heading_hold.reward.ron",
            Self::Orbit | Self::ResidualOrbit => "assets/training/orbit.reward.ron",
            Self::LstmOrbit => "assets/training/wu_orbit.reward.ron",
        }
    }

    /// Per-task default `--ppo-config` path, loaded when `--ppo-config` is
    /// not passed on the CLI. `None` means the task has no tuned default and
    /// falls back to `PpoHyperparams::default()` (mirrors
    /// `assets/training/default.ppo.ron`).
    fn default_ppo_config_path(self) -> Option<&'static str> {
        match self {
            Self::LevelHold => Some("assets/training/level_hold.ppo.ron"),
            Self::HeadingHold => Some("assets/training/heading_hold.ppo.ron"),
            Self::Orbit | Self::ResidualOrbit | Self::LstmOrbit => None,
        }
    }

    fn model_dir(self) -> &'static str {
        match self {
            Self::LevelHold => "level_hold",
            Self::HeadingHold => "heading_hold",
            Self::Orbit => "orbit",
            Self::ResidualOrbit => "orbit_residual",
            Self::LstmOrbit => "lstm_orbit",
        }
    }

    fn default_stem(self) -> &'static str {
        match self {
            Self::LevelHold => "ppo_level_hold",
            Self::HeadingHold => "ppo_heading_hold",
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
    plane_config: String,
    ppo_config: Option<String>,
    cli_seed: Option<u64>,
    bc_steps: usize,
    bc_epochs: usize,
    target_alt_range: std::ops::RangeInclusive<f32>,
    target_speed_range: std::ops::RangeInclusive<f32>,
    target_heading_range: std::ops::RangeInclusive<f32>,
) where
    B: burn::tensor::backend::AutodiffBackend,
    B::Device: Default,
{
    use ml_planes::training::ppo::CsvLog;
    use ml_planes::training::reward_config::{
        load_reward_config, load_ron_config, HeadingHoldRewardConfig, LevelHoldRewardConfig,
        OrbitRewardConfig,
    };
    use ml_planes::training::{
        HeadingHoldEnv, LevelHoldEnv, OrbitEnv, PpoHyperparams, ResidualOrbitEnv, WuOrbitEnv,
        WuOrbitRewardConfig,
    };

    fn open_log(
        path: &Option<String>,
        task_name: &str,
        reward_config_path: &str,
        plane_config_path: &str,
        config_fields: Vec<(&'static str, String)>,
    ) -> Option<CsvLog> {
        let path = path.as_deref()?;
        match CsvLog::new(path) {
            Ok(mut log) => {
                let mut comments = vec![
                    ("task", task_name.to_string()),
                    ("reward_config", reward_config_path.to_string()),
                    ("plane_config", plane_config_path.to_string()),
                ];
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

    let cfg = ml_planes::training::load_plane_config_or_exit(&plane_config);
    println!("Loaded plane config from {plane_config}");

    // Effective reward-profile path: the CLI override if given, else the task default.
    let reward_path = reward_config.unwrap_or_else(|| task.reward_config_path().to_string());

    // PPO hyperparameters: CLI override if given, else the task's tuned
    // default (`Task::default_ppo_config_path`), else compiled defaults.
    // Applies to the MLP `PpoTrainer` tasks only (level_hold / orbit / residual_orbit);
    // the LSTM task uses its own trainer and ignores this.
    let ppo_path = ppo_config.or_else(|| task.default_ppo_config_path().map(str::to_string));
    let mut hp: PpoHyperparams = match ppo_path.as_deref() {
        Some(p) => match load_ron_config(p) {
            Ok(cfg) => {
                println!("Loaded PPO config from {p}");
                cfg
            }
            Err(e) => {
                eprintln!("Warning: could not load {p}: {e}. Using defaults.");
                PpoHyperparams::default()
            }
        },
        None => PpoHyperparams::default(),
    };
    // A CLI `--seed` overrides whatever `--ppo-config`'s `seed` field is set to.
    if let Some(s) = cli_seed {
        hp.seed = Some(s);
    }
    if let Some(s) = hp.seed {
        println!("Seed: {s} (weight init, minibatch shuffling, env resets)");
    }

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
            let mut log_fields = reward_cfg.log_fields();
            log_fields.extend(hp.log_fields());
            log_fields.push((
                "target_alt_range",
                format!("{}:{}", target_alt_range.start(), target_alt_range.end()),
            ));
            log_fields.push((
                "target_speed_range",
                format!(
                    "{}:{}",
                    target_speed_range.start(),
                    target_speed_range.end()
                ),
            ));
            let log = open_log(&log_file, "level_hold", path, &plane_config, log_fields);
            run_training_loop_bc::<B, _>(
                plain,
                save_path,
                total_timesteps,
                init_from,
                LevelHoldEnv::with_target_ranges(
                    target_alt_range,
                    target_speed_range,
                    cfg,
                    reward_cfg,
                ),
                log,
                &hp,
                bc_steps,
                bc_epochs,
            )
        }
        Task::HeadingHold => {
            let path = reward_path.as_str();
            let reward_cfg: HeadingHoldRewardConfig = match load_reward_config(path) {
                Ok(cfg) => {
                    println!("Loaded reward config from {path}");
                    cfg
                }
                Err(e) => {
                    eprintln!("Warning: could not load {path}: {e}. Using defaults.");
                    HeadingHoldRewardConfig::default()
                }
            };
            let mut log_fields = reward_cfg.log_fields();
            log_fields.extend(hp.log_fields());
            log_fields.push((
                "target_alt_range",
                format!("{}:{}", target_alt_range.start(), target_alt_range.end()),
            ));
            log_fields.push((
                "target_speed_range",
                format!(
                    "{}:{}",
                    target_speed_range.start(),
                    target_speed_range.end()
                ),
            ));
            log_fields.push((
                "target_heading_range_deg",
                format!(
                    "{}:{}",
                    target_heading_range.start().to_degrees(),
                    target_heading_range.end().to_degrees()
                ),
            ));
            let log = open_log(&log_file, "heading_hold", path, &plane_config, log_fields);
            run_training_loop_bc::<B, _>(
                plain,
                save_path,
                total_timesteps,
                init_from,
                HeadingHoldEnv::with_target_ranges(
                    target_heading_range,
                    target_alt_range,
                    target_speed_range,
                    cfg,
                    reward_cfg,
                ),
                log,
                &hp,
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
            let mut log_fields = reward_cfg.log_fields();
            log_fields.extend(hp.log_fields());
            let log = open_log(&log_file, "orbit", path, &plane_config, log_fields);
            run_training_loop_bc::<B, _>(
                plain,
                save_path,
                total_timesteps,
                init_from,
                OrbitEnv::with_reward_config(1000.0, 100.0, 1000.0, cfg, reward_cfg),
                log,
                &hp,
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
            let mut log_fields = reward_cfg.log_fields();
            log_fields.extend(hp.log_fields());
            let log = open_log(&log_file, "residual_orbit", path, &plane_config, log_fields);
            run_training_loop::<B, _>(
                plain,
                save_path,
                total_timesteps,
                init_from,
                ResidualOrbitEnv::with_reward_config(1000.0, 100.0, 1000.0, cfg, reward_cfg),
                log,
                &hp,
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
            let log = open_log(
                &log_file,
                "lstm_orbit",
                path,
                &plane_config,
                reward_cfg.log_fields(),
            );
            run_lstm_training_loop::<B>(
                plain,
                save_path,
                total_timesteps,
                init_from,
                WuOrbitEnv::with_reward_config(1000.0, 100.0, 3000.0, cfg, reward_cfg),
                log,
                hp.seed,
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
    hp: &ml_planes::training::PpoHyperparams,
) where
    B: burn::tensor::backend::AutodiffBackend,
    B::Device: Default,
    E: ml_planes::training::TrainingEnv + Clone,
{
    use ml_planes::training::ppo::PpoTrainer;

    let device: B::Device = Default::default();
    let mut trainer = PpoTrainer::<B, E>::with_n_envs_seeded(env, 8, device, hp.seed);
    trainer.apply_hyperparams(hp);
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
    hp: &ml_planes::training::PpoHyperparams,
    bc_steps: usize,
    bc_epochs: usize,
) where
    B: burn::tensor::backend::AutodiffBackend,
    B::Device: Default,
    E: ml_planes::training::DemonstrationEnv + Clone,
{
    use ml_planes::training::{collect_demonstrations, ppo::PpoTrainer};

    let device: B::Device = Default::default();
    let mut trainer = PpoTrainer::<B, E>::with_n_envs_seeded(env.clone(), 8, device, hp.seed);
    trainer.apply_hyperparams(hp);
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
#[allow(clippy::too_many_arguments)]
fn run_lstm_training_loop<B>(
    plain: bool,
    save_path: String,
    total_timesteps: usize,
    init_from: Option<String>,
    env: ml_planes::training::WuOrbitEnv,
    mut log: Option<ml_planes::training::ppo::CsvLog>,
    // Passed explicitly (rather than via `PpoHyperparams`) since `lstm_orbit`
    // otherwise ignores `--ppo-config` entirely — see the flag docs above.
    seed: Option<u64>,
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
    let mut trainer = LstmPpoTrainer::<B, _>::with_n_envs_seeded(env, 32, device, seed);
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

/// Plain table metrics renderer.
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
