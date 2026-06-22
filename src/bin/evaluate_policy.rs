//! Deterministic policy evaluator for PPO checkpoints.
//!
//! Reports a stable common core (`success_rate`, `mean_return`,
//! `mean_length_steps`) for every task, plus task-specific tracking-error
//! metrics whose names depend on the observation layout (see
//! `ml_planes::training::eval_metrics`).
//!
//! Supported tasks: `level_hold`, `orbit`, `residual_orbit`, `lstm_orbit`.
//! `lstm_orbit` loads a recurrent `LstmActorCritic` and carries hidden state
//! across each episode (reset at every `env.reset()`); the others use the
//! feed-forward `ActorCritic`.
//!
//! Example:
//!   cargo run --release --no-default-features --features inference --bin evaluate_policy -- \
//!     --task orbit --model models/orbit/ppo_orbit_1 --episodes 64 --backend ndarray
//!
//! Flags:
//!   --task <task>           level_hold | orbit | residual_orbit | lstm_orbit
//!   --model <path>          Checkpoint path (with or without .mpk)
//!   --episodes <n>          Episodes to roll out (default 64)
//!   --max-steps <n>         Override the per-episode step cap (default: task config)
//!   --backend ndarray|cpu   Inference backend (only ndarray/cpu supported)
//!   --reward-config <path>  Reward/termination profile path (default: the task's
//!                           assets/training/<task>.reward.ron). Missing file → defaults.
//!   --curriculum-stage <s>  lstm_orbit only: coarse | heading_fine | full
//!                           (default full). Selects the WuOrbit reward stage the
//!                           policy is scored under; reported as curriculum_stage.

#[cfg(not(feature = "inference"))]
fn main() {
    eprintln!("Build with --features inference to evaluate PPO policies.");
}

#[cfg(feature = "inference")]
fn main() {
    use burn::backend::NdArray;
    use ml_planes::training::eval_metrics::MetricFamily;
    use ml_planes::training::reward_config::{LevelHoldRewardConfig, OrbitRewardConfig};
    use ml_planes::training::wu_orbit_reward::WuOrbitRewardConfig;
    use ml_planes::training::{LevelHoldEnv, OrbitEnv, ResidualOrbitEnv, WuOrbitEnv};

    type B = NdArray;

    let args: Vec<String> = std::env::args().collect();
    let task = find_arg(&args, "--task").unwrap_or_else(|| "orbit".to_string());
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
    let cfg = generic_jet_config();

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
    let metrics = match task.as_str() {
        "level_hold" => {
            let reward_cfg: LevelHoldRewardConfig =
                load_task_reward(&args, "assets/training/level_hold.reward.ron");
            let max_steps = parse_u32(&args, "--max-steps", reward_cfg.max_episode_steps);
            let mut env = LevelHoldEnv::with_reward_config(1000.0, 100.0, cfg, reward_cfg);
            env.max_episode_steps = max_steps;
            let mut policy = load_ff_policy::<B>(&path, env_obs_dim(&env));
            run_eval(
                env,
                episodes,
                max_steps,
                &mut policy,
                MetricFamily::LevelHold,
            )
        }
        "orbit" => {
            let reward_cfg: OrbitRewardConfig =
                load_task_reward(&args, "assets/training/orbit.reward.ron");
            let max_steps = parse_u32(&args, "--max-steps", reward_cfg.max_episode_steps);
            let mut env = OrbitEnv::with_reward_config(1000.0, 100.0, 1000.0, cfg, reward_cfg);
            env.max_episode_steps = max_steps;
            let mut policy = load_ff_policy::<B>(&path, env_obs_dim(&env));
            run_eval(env, episodes, max_steps, &mut policy, MetricFamily::Orbit)
        }
        "residual_orbit" => {
            let reward_cfg: OrbitRewardConfig =
                load_task_reward(&args, "assets/training/orbit.reward.ron");
            let max_steps = parse_u32(&args, "--max-steps", reward_cfg.max_episode_steps);
            let mut env =
                ResidualOrbitEnv::with_reward_config(1000.0, 100.0, 1000.0, cfg, reward_cfg);
            env.max_episode_steps = max_steps;
            let mut policy = load_ff_policy::<B>(&path, env_obs_dim(&env));
            run_eval(env, episodes, max_steps, &mut policy, MetricFamily::Orbit)
        }
        "lstm_orbit" => {
            let reward_cfg: WuOrbitRewardConfig =
                load_task_reward(&args, "assets/training/wu_orbit.reward.ron");
            let max_steps = parse_u32(&args, "--max-steps", reward_cfg.max_episode_steps);
            let mut env = WuOrbitEnv::with_reward_config(1000.0, 100.0, 3000.0, cfg, reward_cfg);
            env.max_episode_steps = max_steps;
            env.advance_to_stage(curriculum_stage);
            reported_stage = Some(env.curriculum_stage.name());
            let mut policy = load_lstm_policy::<B>(&path, env_obs_dim(&env));
            run_eval(env, episodes, max_steps, &mut policy, MetricFamily::Orbit)
        }
        other => {
            eprintln!(
                "Unsupported --task '{other}'. Use 'level_hold', 'orbit', 'residual_orbit', or 'lstm_orbit'."
            );
            std::process::exit(2);
        }
    };

    // Common core — identical keys for every task.
    println!("task,{task}");
    println!("model,{path}.mpk");
    if let Some(stage) = reported_stage {
        println!("curriculum_stage,{stage}");
    }
    println!("episodes,{}", metrics.core.episodes);
    println!("success_rate,{:.6}", metrics.core.success_rate);
    println!("mean_return,{:.6}", metrics.core.mean_return);
    println!("mean_length_steps,{:.3}", metrics.core.mean_length_steps);
    // Task-specific extras (each row carries its own print precision).
    for row in metrics.extra_rows {
        println!("{},{:.*}", row.key, row.decimals, row.value);
    }
}

/// Load a task's reward config from `--reward-config` or the supplied default
/// path, falling back to compiled defaults on any error.
#[cfg(feature = "inference")]
fn load_task_reward<T>(args: &[String], default_path: &str) -> T
where
    T: serde::de::DeserializeOwned + Default,
{
    use ml_planes::training::reward_config::load_reward_config;
    let path = find_arg(args, "--reward-config").unwrap_or_else(|| default_path.to_string());
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

#[cfg(feature = "inference")]
#[derive(Debug, Clone, Copy, Default)]
struct CoreMetrics {
    episodes: usize,
    success_rate: f32,
    mean_return: f32,
    mean_length_steps: f32,
}

#[cfg(feature = "inference")]
struct EvalResult {
    core: CoreMetrics,
    extra_rows: Vec<ml_planes::training::eval_metrics::MetricRow>,
}

/// Roll out `episodes` deterministic episodes, accumulating the common core
/// metrics plus the task-specific tracking-error metrics for `family`.
#[cfg(feature = "inference")]
fn run_eval<E, P>(
    mut env: E,
    episodes: usize,
    max_steps: u32,
    policy: &mut P,
    family: ml_planes::training::eval_metrics::MetricFamily,
) -> EvalResult
where
    E: ml_planes::training::TrainingEnv,
    P: EvalPolicy,
{
    use ml_planes::training::TaskMetrics;

    let mut task_metrics = TaskMetrics::new(family);
    if episodes == 0 || max_steps == 0 {
        return EvalResult {
            core: CoreMetrics {
                episodes,
                ..Default::default()
            },
            extra_rows: task_metrics.into_rows(),
        };
    }

    let mut total_return = 0.0_f32;
    let mut total_len = 0_u64;
    let mut success = 0_usize;

    for _ in 0..episodes {
        let (mut obs, _) = env.reset();
        policy.reset();
        let mut ep_return = 0.0_f32;
        let mut ep_len = 0_u32;
        let mut done = false;

        while !done && ep_len < max_steps {
            let action = policy.act(&obs);
            let (next_obs, reward, next_done, _info) = env.step(&action);
            obs = next_obs;
            ep_return += reward;
            ep_len += 1;
            done = next_done;

            task_metrics.step(&obs);
        }

        if ep_len >= max_steps {
            success += 1;
        }
        task_metrics.finish_episode(&obs);
        total_return += ep_return;
        total_len += ep_len as u64;
    }

    EvalResult {
        core: CoreMetrics {
            episodes,
            success_rate: success as f32 / episodes as f32,
            mean_return: total_return / episodes as f32,
            mean_length_steps: total_len as f32 / episodes as f32,
        },
        extra_rows: task_metrics.into_rows(),
    }
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

#[cfg(feature = "inference")]
fn generic_jet_config() -> ml_planes::plane::config::PlaneConfig {
    ml_planes::plane::config::PlaneConfig {
        wing_area: 20.0,
        mean_chord: 2.0,
        wing_span: 10.0,
        mass: 5000.0,
        inertia: bevy::math::Vec3::new(10000.0, 40000.0, 45000.0),
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
    }
}
