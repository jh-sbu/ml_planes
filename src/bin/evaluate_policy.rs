//! Deterministic policy evaluator for PPO checkpoints.
//!
//! Reports a stable common core (`success_rate`, `mean_return`,
//! `mean_length_steps`) for every task, plus task-specific tracking-error
//! metrics whose names depend on the observation layout (see
//! `ml_planes::training::eval_metrics`).
//!
//! Supported tasks: `level_hold`, `heading_hold`, `orbit`, `residual_orbit`, `lstm_orbit`.
//! `lstm_orbit` loads a recurrent `LstmActorCritic` and carries hidden state
//! across each episode (reset at every `env.reset()`); the others use the
//! feed-forward `ActorCritic`.
//!
//! Example:
//!   cargo run --release --no-default-features --features inference --bin evaluate_policy -- \
//!     --task orbit --model models/orbit/ppo_orbit_1 --episodes 64 --backend ndarray
//!
//! Flags:
//!   --task <task>           level_hold | heading_hold | orbit | residual_orbit | lstm_orbit
//!   --model <path>          Checkpoint path (with or without .mpk)
//!   --plane-config <path>   Airframe (.plane.ron) to evaluate against (default:
//!                           assets/planes/generic_jet.plane.ron). Unreadable or invalid →
//!                           exit 2. Pass the same airframe a checkpoint was trained with,
//!                           for the same reason the target ranges must match. Echoed back
//!                           as the `plane_config` row of the report.
//!   --episodes <n>          Episodes to roll out (default 64)
//!   --max-steps <n>         Override the per-episode step cap (default: task config)
//!   --backend ndarray|cpu   Inference backend (only ndarray/cpu supported)
//!   --reward-config <path>  Reward/termination profile path (default: the task's
//!                           assets/training/<task>.reward.ron). Missing file → defaults.
//!   --curriculum-stage <s>  lstm_orbit only: coarse | heading_fine | full
//!                           (default full). Selects the WuOrbit reward stage the
//!                           policy is scored under; reported as curriculum_stage.
//!   --target-alt-range <MIN:MAX|VALUE>    level_hold/heading_hold only: per-episode target
//!                           altitude [m] range to evaluate across (default: 500:5000). Use the
//!                           same range the policy was trained with for a comparable report.
//!   --target-speed-range <MIN:MAX|VALUE>  level_hold/heading_hold only: target airspeed [m/s]
//!                           range (default: 90:140 for both level_hold and heading_hold).
//!   --target-heading-range <MIN:MAX|VALUE>  heading_hold only: target heading change [degrees]
//!                           range to evaluate across (default: -180:180).

#[cfg(not(feature = "inference"))]
fn main() {
    eprintln!("Build with --features inference to evaluate PPO policies.");
}

#[cfg(feature = "inference")]
fn main() {
    use burn::backend::NdArray;
    use ml_planes::training::reward_config::{
        HeadingHoldRewardConfig, LevelHoldRewardConfig, OrbitRewardConfig,
    };
    use ml_planes::training::task::{self, EnvSpec, Task};
    use ml_planes::training::wu_orbit_reward::WuOrbitRewardConfig;

    type B = NdArray;

    let args: Vec<String> = std::env::args().collect();
    let task = Task::parse(&find_arg(&args, "--task").unwrap_or_else(|| "orbit".to_string()))
        .unwrap_or_else(|e| {
            eprintln!("{e}");
            std::process::exit(2);
        });
    let model_path = find_arg(&args, "--model").unwrap_or_else(|| {
        eprintln!("--model <path> is required; pass the path without or with .mpk");
        std::process::exit(2);
    });
    let backend = find_arg(&args, "--backend").unwrap_or_else(|| "ndarray".to_string());
    if backend != "ndarray" && backend != "cpu" {
        eprintln!("evaluate_policy currently supports only --backend ndarray/cpu");
        std::process::exit(2);
    }
    let episodes = parse_usize(&args, "--episodes", 64);
    let path = model_path
        .strip_suffix(".mpk")
        .unwrap_or(&model_path)
        .to_string();
    let plane_config = find_arg(&args, "--plane-config")
        .unwrap_or_else(|| ml_planes::training::DEFAULT_PLANE_CONFIG_PATH.to_string());
    let cfg = ml_planes::training::load_plane_config_or_exit(&plane_config);

    // Curriculum stage to evaluate `lstm_orbit` under (ignored by other tasks).
    // Defaults to `full` — the stage checkpoints are trained through to — so
    // `mean_return` reflects the final training objective rather than the coarse
    // default stage. Reported back in the output for reproducibility.
    let curriculum_stage = parse_curriculum_stage(&args);

    // Each task selects its env, reward-config type + default profile path, the
    // policy architecture (feed-forward vs recurrent), and the metric family.
    // The env's internal step cap is overridden to the resolved `--max-steps`
    // so the loop bound and the env's Timeout coincide (success is correct for
    // any `--max-steps`).
    let mut reported_stage: Option<&'static str> = None;
    let mut reported_target_alt_range: Option<std::ops::RangeInclusive<f32>> = None;
    let mut reported_target_speed_range: Option<std::ops::RangeInclusive<f32>> = None;
    let mut reported_target_heading_range_deg: Option<std::ops::RangeInclusive<f32>> = None;
    // The registry supplies the reward profile, the default envelope, and the
    // metric family, so the only thing that varies per arm here is the reward
    // config's *type* and the policy architecture.
    let defaults = EnvSpec::defaults_for(task, cfg);
    let alt_range =
        parse_target_range(&args, "--target-alt-range", task.default_target_alt_range());
    let speed_range = parse_target_range(
        &args,
        "--target-speed-range",
        task.default_target_speed_range(),
    );
    let heading_range_deg = parse_target_range(
        &args,
        "--target-heading-range",
        task.default_target_heading_range_deg(),
    );
    let spec = EnvSpec {
        target_alt_range: alt_range.clone(),
        target_speed_range: speed_range.clone(),
        target_heading_range: heading_range_deg.start().to_radians()
            ..=heading_range_deg.end().to_radians(),
        ..defaults
    };

    let metrics = match task {
        Task::LevelHold => {
            let reward_cfg: LevelHoldRewardConfig = load_task_reward(&args, task);
            let max_steps = parse_u32(&args, "--max-steps", reward_cfg.max_episode_steps);
            let mut env = task::level_hold_env(&spec, reward_cfg);
            env.max_episode_steps = max_steps;
            reported_target_alt_range = Some(alt_range);
            reported_target_speed_range = Some(speed_range);
            let mut policy = load_ff_policy::<B>(&path, env_obs_dim(&env));
            run_eval(env, episodes, max_steps, &mut policy, task.metric_family())
        }
        Task::HeadingHold => {
            let reward_cfg: HeadingHoldRewardConfig = load_task_reward(&args, task);
            let max_steps = parse_u32(&args, "--max-steps", reward_cfg.max_episode_steps);
            let mut env = task::heading_hold_env(&spec, reward_cfg);
            env.max_episode_steps = max_steps;
            reported_target_alt_range = Some(alt_range);
            reported_target_speed_range = Some(speed_range);
            reported_target_heading_range_deg = Some(heading_range_deg);
            let mut policy = load_ff_policy::<B>(&path, env_obs_dim(&env));
            run_eval(env, episodes, max_steps, &mut policy, task.metric_family())
        }
        Task::Orbit => {
            let reward_cfg: OrbitRewardConfig = load_task_reward(&args, task);
            let max_steps = parse_u32(&args, "--max-steps", reward_cfg.max_episode_steps);
            let mut env = task::orbit_env(&spec, reward_cfg);
            env.max_episode_steps = max_steps;
            let mut policy = load_ff_policy::<B>(&path, env_obs_dim(&env));
            run_eval(env, episodes, max_steps, &mut policy, task.metric_family())
        }
        Task::ResidualOrbit => {
            let reward_cfg: OrbitRewardConfig = load_task_reward(&args, task);
            let max_steps = parse_u32(&args, "--max-steps", reward_cfg.max_episode_steps);
            let mut env = task::residual_orbit_env(&spec, reward_cfg);
            env.max_episode_steps = max_steps;
            let mut policy = load_ff_policy::<B>(&path, env_obs_dim(&env));
            run_eval(env, episodes, max_steps, &mut policy, task.metric_family())
        }
        Task::LstmOrbit => {
            let reward_cfg: WuOrbitRewardConfig = load_task_reward(&args, task);
            let max_steps = parse_u32(&args, "--max-steps", reward_cfg.max_episode_steps);
            let mut env = task::wu_orbit_env(&spec, reward_cfg);
            env.max_episode_steps = max_steps;
            env.advance_to_stage(curriculum_stage);
            reported_stage = Some(env.curriculum_stage.name());
            let mut policy = load_lstm_policy::<B>(&path, env_obs_dim(&env));
            run_eval(env, episodes, max_steps, &mut policy, task.metric_family())
        }
    };

    // Common core — identical keys for every task.
    println!("task,{}", task.as_str());
    println!("model,{path}.mpk");
    println!("plane_config,{plane_config}");
    if let Some(stage) = reported_stage {
        println!("curriculum_stage,{stage}");
    }
    if let Some(r) = &reported_target_alt_range {
        println!("target_alt_range,{}:{}", r.start(), r.end());
    }
    if let Some(r) = &reported_target_speed_range {
        println!("target_speed_range,{}:{}", r.start(), r.end());
    }
    if let Some(r) = &reported_target_heading_range_deg {
        println!("target_heading_range_deg,{}:{}", r.start(), r.end());
    }
    println!("episodes,{}", metrics.episodes);
    println!("success_rate,{:.6}", metrics.success_rate);
    println!("mean_return,{:.6}", metrics.mean_return);
    println!("mean_length_steps,{:.3}", metrics.mean_length_steps);
    // Task-specific extras (each row carries its own print precision).
    for row in metrics.rows {
        println!("{},{:.*}", row.key, row.decimals, row.value);
    }
}

/// Load a task's reward config from `--reward-config` or the supplied default
/// path, falling back to compiled defaults on any error.
#[cfg(feature = "inference")]
fn load_task_reward<T>(args: &[String], task: ml_planes::training::task::Task) -> T
where
    T: serde::de::DeserializeOwned + Default,
{
    use ml_planes::training::reward_config::load_reward_config;
    let path =
        find_arg(args, "--reward-config").unwrap_or_else(|| task.reward_config_path().to_string());
    load_reward_config(&path).unwrap_or_else(|e| {
        eprintln!("Warning: could not load {path}: {e}. Using defaults.");
        T::default()
    })
}

#[cfg(feature = "inference")]
fn env_obs_dim<E: ml_planes::training::TrainingEnv>(env: &E) -> usize {
    env.observation_dim()
}

/// Resolve `--curriculum-stage <coarse|heading_fine|full>` for `lstm_orbit`.
/// Defaults to `Full`; rejects unknown values via the library parser. The
/// string→stage mapping and stage-advance ladder live in the library
/// (`CurriculumStage::from_cli_arg`, `WuOrbitEnv::advance_to_stage`) so they are
/// unit-tested; this wrapper only handles the missing-arg default and exit code.
#[cfg(feature = "inference")]
fn parse_curriculum_stage(args: &[String]) -> ml_planes::training::CurriculumStage {
    use ml_planes::training::CurriculumStage;
    match find_arg(args, "--curriculum-stage") {
        None => CurriculumStage::Full,
        Some(v) => CurriculumStage::from_cli_arg(&v).unwrap_or_else(|msg| {
            eprintln!("{msg}");
            std::process::exit(2);
        }),
    }
}

// ---------------------------------------------------------------------------
// Policy runners (feed-forward and recurrent)
// ---------------------------------------------------------------------------

/// A deterministic policy that maps an observation to a mean action. Recurrent
/// policies carry hidden state between steps; `reset` clears it at the start of
/// every episode.
#[cfg(feature = "inference")]
trait EvalPolicy {
    fn reset(&mut self);
    fn act(&mut self, obs: &[f32]) -> Vec<f32>;
}

#[cfg(feature = "inference")]
struct FeedForwardRunner<Bk: burn::tensor::backend::Backend> {
    model: ml_planes::training::ActorCritic<Bk>,
    device: Bk::Device,
}

#[cfg(feature = "inference")]
impl<Bk: burn::tensor::backend::Backend> EvalPolicy for FeedForwardRunner<Bk> {
    fn reset(&mut self) {}

    fn act(&mut self, obs: &[f32]) -> Vec<f32> {
        use burn::tensor::{Tensor, TensorData};
        let obs_t = Tensor::<Bk, 2>::from_data(
            TensorData::new(obs.to_vec(), vec![1, obs.len()]),
            &self.device,
        );
        self.model
            .mean_action(obs_t)
            .into_data()
            .to_vec::<f32>()
            .expect("policy action data")
    }
}

#[cfg(feature = "inference")]
struct LstmRunner<Bk: burn::tensor::backend::Backend> {
    model: ml_planes::training::LstmActorCritic<Bk>,
    device: Bk::Device,
    state: Option<burn::nn::LstmState<Bk, 2>>,
}

#[cfg(feature = "inference")]
impl<Bk: burn::tensor::backend::Backend> EvalPolicy for LstmRunner<Bk> {
    fn reset(&mut self) {
        // Drop the carried hidden state; the next step starts from zeros.
        self.state = None;
    }

    fn act(&mut self, obs: &[f32]) -> Vec<f32> {
        use burn::tensor::{Tensor, TensorData};
        let obs_t = Tensor::<Bk, 2>::from_data(
            TensorData::new(obs.to_vec(), vec![1, obs.len()]),
            &self.device,
        );
        let (action_t, new_state) = self.model.mean_action_step(obs_t, self.state.take());
        self.state = Some(new_state);
        action_t
            .into_data()
            .to_vec::<f32>()
            .expect("policy action data")
    }
}

#[cfg(feature = "inference")]
fn load_ff_policy<Bk: burn::tensor::backend::Backend>(
    path: &str,
    obs_dim: usize,
) -> FeedForwardRunner<Bk>
where
    Bk::Device: Default,
{
    use burn::module::Module;
    use burn::record::{DefaultFileRecorder, FullPrecisionSettings};
    let device: Bk::Device = Default::default();
    let model = ml_planes::training::ActorCritic::<Bk>::new(&device, obs_dim)
        .load_file(
            path,
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
            &device,
        )
        .unwrap_or_else(|e| panic!("failed to load model from {path}.mpk: {e}"));
    FeedForwardRunner { model, device }
}

#[cfg(feature = "inference")]
fn load_lstm_policy<Bk: burn::tensor::backend::Backend>(
    path: &str,
    obs_dim: usize,
) -> LstmRunner<Bk>
where
    Bk::Device: Default,
{
    use burn::module::Module;
    use burn::record::{DefaultFileRecorder, FullPrecisionSettings};
    let device: Bk::Device = Default::default();
    let model = ml_planes::training::LstmActorCritic::<Bk>::new(&device, obs_dim)
        .load_file(
            path,
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
            &device,
        )
        .unwrap_or_else(|e| panic!("failed to load LSTM model from {path}.mpk: {e}"));
    LstmRunner {
        model,
        device,
        state: None,
    }
}

// ---------------------------------------------------------------------------
// Evaluation loop
// ---------------------------------------------------------------------------

/// Roll out `episodes` deterministic episodes, accumulating the common core
/// metrics plus the task-specific tracking-error metrics for `family`.
#[cfg(feature = "inference")]
fn run_eval<E, P>(
    mut env: E,
    episodes: usize,
    max_steps: u32,
    policy: &mut P,
    family: ml_planes::training::eval_metrics::MetricFamily,
) -> ml_planes::training::EvalReport
where
    E: ml_planes::training::TrainingEnv,
    P: EvalPolicy,
{
    use ml_planes::training::EvalRun;

    let mut run = EvalRun::new(family, episodes, max_steps, 1);
    if episodes == 0 || max_steps == 0 {
        return run.report();
    }

    for _ in 0..episodes {
        let (mut obs, _) = env.reset();
        policy.reset();
        let mut ep_len = 0_u32;
        let mut done = false;

        while !done && ep_len < max_steps {
            let action = policy.act(&obs);
            let outcome = env.step(&action);
            done = outcome.done();
            obs = outcome.obs;
            ep_len += 1;
            run.record(0, &obs, outcome.reward)
                .expect("single-slot evaluation loop");
        }

        run.finish(0, &obs).expect("episode had at least one step");
    }

    run.report()
}

// ---------------------------------------------------------------------------
// Argument parsing helpers
// ---------------------------------------------------------------------------

#[cfg(feature = "inference")]
fn find_arg(args: &[String], key: &str) -> Option<String> {
    args.windows(2).find(|w| w[0] == key).map(|w| w[1].clone())
}

#[cfg(feature = "inference")]
fn parse_usize(args: &[String], key: &str, default: usize) -> usize {
    find_arg(args, key)
        .map(|v| {
            v.parse::<usize>().unwrap_or_else(|_| {
                eprintln!("{key} must be a positive integer");
                std::process::exit(2);
            })
        })
        .unwrap_or(default)
}

/// Resolve a `MIN:MAX|VALUE` range argument, defaulting to `(default_min, default_max)`.
#[cfg(feature = "inference")]
fn parse_target_range(
    args: &[String],
    key: &str,
    default: std::ops::RangeInclusive<f32>,
) -> std::ops::RangeInclusive<f32> {
    find_arg(args, key)
        .map(|v| {
            ml_planes::training::parse_f32_range(&v).unwrap_or_else(|e| {
                eprintln!("{key}: {e}");
                std::process::exit(2);
            })
        })
        .unwrap_or(default)
}

#[cfg(feature = "inference")]
fn parse_u32(args: &[String], key: &str, default: u32) -> u32 {
    find_arg(args, key)
        .map(|v| {
            v.parse::<u32>().unwrap_or_else(|_| {
                eprintln!("{key} must be a positive integer");
                std::process::exit(2);
            })
        })
        .unwrap_or(default)
}
