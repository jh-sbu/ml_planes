//! Deterministic policy evaluator for orbit PPO checkpoints.
//!
//! Example:
//!   cargo run --release --no-default-features --features training --bin evaluate_policy -- \
//!     --task orbit --model models/orbit/ppo_orbit_1 --episodes 64 --backend ndarray
//!
//! Flags:
//!   --reward-config <path>  Reward/termination profile path (default:
//!                           assets/training/orbit.reward.ron). Missing file → defaults.

#[cfg(not(feature = "training"))]
fn main() {
    eprintln!("Build with --features training to evaluate PPO policies.");
}

#[cfg(feature = "training")]
fn main() {
    use burn::backend::NdArray;
    use burn::module::Module;
    use burn::record::{DefaultFileRecorder, FullPrecisionSettings};
    use burn::tensor::backend::Backend;
    use ml_planes::training::ppo::ActorCritic;
    use ml_planes::training::reward_config::{load_reward_config, OrbitRewardConfig};
    use ml_planes::training::{OrbitEnv, ResidualOrbitEnv};

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

    // Optional reward-profile override; defaults to the orbit baseline profile.
    let reward_path = find_arg(&args, "--reward-config")
        .unwrap_or_else(|| "assets/training/orbit.reward.ron".to_string());
    let reward_cfg: OrbitRewardConfig = load_reward_config(&reward_path).unwrap_or_else(|e| {
        eprintln!("Warning: could not load {reward_path}: {e}. Using defaults.");
        OrbitRewardConfig::default()
    });
    let max_steps = parse_u32(&args, "--max-steps", reward_cfg.max_episode_steps);

    let cfg = generic_jet_config();
    let device: <B as Backend>::Device = Default::default();
    let path = model_path.strip_suffix(".mpk").unwrap_or(&model_path);
    let model = ActorCritic::<B>::new(&device, 13)
        .load_file(
            path,
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
            &device,
        )
        .unwrap_or_else(|e| panic!("failed to load model from {path}.mpk: {e}"));

    let mut runner = ModelRunner { model, device };
    let metrics = match task.as_str() {
        "orbit" => {
            let env = OrbitEnv::with_reward_config(1000.0, 100.0, 1000.0, cfg, reward_cfg);
            evaluate_orbit_env(env, episodes, max_steps, |obs| runner.action(obs))
        }
        "residual_orbit" => {
            let env = ResidualOrbitEnv::with_reward_config(1000.0, 100.0, 1000.0, cfg, reward_cfg);
            evaluate_orbit_env(env, episodes, max_steps, |obs| runner.action(obs))
        }
        other => {
            eprintln!("Unsupported --task '{other}'. Use 'orbit' or 'residual_orbit'.");
            std::process::exit(2);
        }
    };

    println!("task,{task}");
    println!("model,{path}.mpk");
    println!("episodes,{}", metrics.episodes);
    println!("success_rate,{:.6}", metrics.success_rate);
    println!("mean_return,{:.6}", metrics.mean_return);
    println!("mean_length_steps,{:.3}", metrics.mean_length_steps);
    println!("mean_abs_radial_m,{:.3}", metrics.mean_abs_radial_m);
    println!("mean_abs_heading_rad,{:.6}", metrics.mean_abs_heading_rad);
    println!("mean_abs_altitude_m,{:.3}", metrics.mean_abs_altitude_m);
    println!("mean_abs_speed_mps,{:.3}", metrics.mean_abs_speed_mps);
    println!(
        "mean_final_abs_radial_m,{:.3}",
        metrics.mean_final_abs_radial_m
    );
    println!(
        "mean_final_abs_altitude_m,{:.3}",
        metrics.mean_final_abs_altitude_m
    );
}

#[cfg(feature = "training")]
struct ModelRunner {
    model: ml_planes::training::ppo::ActorCritic<burn::backend::NdArray>,
    device: <burn::backend::NdArray as burn::tensor::backend::Backend>::Device,
}

#[cfg(feature = "training")]
impl ModelRunner {
    fn action(&mut self, obs: &[f32]) -> Vec<f32> {
        use burn::tensor::{Tensor, TensorData};

        let obs_t = Tensor::<burn::backend::NdArray, 2>::from_data(
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

#[cfg(feature = "training")]
#[derive(Debug, Clone, Copy, Default)]
struct OrbitEvalMetrics {
    episodes: usize,
    success_rate: f32,
    mean_return: f32,
    mean_length_steps: f32,
    mean_abs_radial_m: f32,
    mean_abs_heading_rad: f32,
    mean_abs_altitude_m: f32,
    mean_abs_speed_mps: f32,
    mean_final_abs_radial_m: f32,
    mean_final_abs_altitude_m: f32,
}

#[cfg(feature = "training")]
fn evaluate_orbit_env<E, F>(
    mut env: E,
    episodes: usize,
    max_steps: u32,
    mut policy: F,
) -> OrbitEvalMetrics
where
    E: ml_planes::training::TrainingEnv,
    F: FnMut(&[f32]) -> Vec<f32>,
{
    if episodes == 0 || max_steps == 0 {
        return OrbitEvalMetrics::default();
    }

    let mut total_return = 0.0_f32;
    let mut total_len = 0_u64;
    let mut success = 0_usize;
    let mut radial_sum = 0.0_f32;
    let mut heading_sum = 0.0_f32;
    let mut altitude_sum = 0.0_f32;
    let mut speed_sum = 0.0_f32;
    let mut metric_samples = 0_u64;
    let mut final_radial_sum = 0.0_f32;
    let mut final_altitude_sum = 0.0_f32;

    for _ in 0..episodes {
        let (mut obs, _) = env.reset();
        let mut ep_return = 0.0_f32;
        let mut ep_len = 0_u32;
        let mut done = false;

        while !done && ep_len < max_steps {
            let action = policy(&obs);
            let (next_obs, reward, next_done, _info) = env.step(&action);
            obs = next_obs;
            ep_return += reward;
            ep_len += 1;
            done = next_done;

            radial_sum += obs[0].abs() * 500.0;
            heading_sum += obs[1].abs() * 0.5;
            altitude_sum += obs[3].abs() * 200.0;
            speed_sum += obs[4].abs() * 50.0;
            metric_samples += 1;
        }

        if ep_len >= max_steps {
            success += 1;
        }
        final_radial_sum += obs[0].abs() * 500.0;
        final_altitude_sum += obs[3].abs() * 200.0;
        total_return += ep_return;
        total_len += ep_len as u64;
    }

    let samples = metric_samples.max(1) as f32;
    OrbitEvalMetrics {
        episodes,
        success_rate: success as f32 / episodes as f32,
        mean_return: total_return / episodes as f32,
        mean_length_steps: total_len as f32 / episodes as f32,
        mean_abs_radial_m: radial_sum / samples,
        mean_abs_heading_rad: heading_sum / samples,
        mean_abs_altitude_m: altitude_sum / samples,
        mean_abs_speed_mps: speed_sum / samples,
        mean_final_abs_radial_m: final_radial_sum / episodes as f32,
        mean_final_abs_altitude_m: final_altitude_sum / episodes as f32,
    }
}

#[cfg(feature = "training")]
fn find_arg(args: &[String], key: &str) -> Option<String> {
    args.windows(2).find(|w| w[0] == key).map(|w| w[1].clone())
}

#[cfg(feature = "training")]
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

#[cfg(feature = "training")]
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

#[cfg(feature = "training")]
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
        aileron_limit: 0.4363,
        elevator_limit: 0.3491,
        rudder_limit: 0.2618,
    }
}
