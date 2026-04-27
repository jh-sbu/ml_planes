//! PPO training binary.  Build and run with:
//!   cargo run --no-default-features --features training --bin train_ppo
//!
//! Trains a PPO level-hold controller for 2 000 000 environment steps and
//! saves the policy to `models/ppo_level_hold.mpk`.

#[cfg(not(feature = "training"))]
fn main() {
    eprintln!("Build with --features training to run PPO training.");
    eprintln!("  cargo run --no-default-features --features training --bin train_ppo");
}

#[cfg(feature = "training")]
fn main() {
    use std::time::Instant;

    use burn::backend::{Autodiff, Wgpu};
    use bevy::math::Vec3;

    use ml_planes::plane::config::PlaneConfig;
    use ml_planes::training::LevelHoldEnv;
    use ml_planes::training::ppo::PpoTrainer;

    type B = Autodiff<Wgpu>;

    // Generic jet values matching assets/planes/generic_jet.plane.ron
    let cfg = PlaneConfig {
        wing_area: 20.0, mean_chord: 2.0, wing_span: 10.0,
        mass: 5000.0, inertia: Vec3::new(10000.0, 40000.0, 45000.0),
        cl0: 0.1,   cl_alpha: 4.5,  cl_delta_e: 0.4,  cl_max: 1.4,
        cd0: 0.02,  cd_induced: 0.05,
        cm0: -0.02, cm_alpha: 0.6,  cm_q: -14.0,       cm_delta_e: -1.2,
        cl_beta: -0.08, cl_p: -0.45, cl_r: 0.12, cl_delta_a: 0.18,
        cn_beta: 0.10, cn_r: -0.12, cn_delta_r: -0.10,
        thrust_max: 60000.0,
        aileron_limit: 0.4363, elevator_limit: 0.3491, rudder_limit: 0.2618,
    };
    let env = LevelHoldEnv::new(1000.0, 100.0, cfg);

    let device: <B as burn::tensor::backend::Backend>::Device = Default::default();
    let mut trainer = PpoTrainer::<B>::new(env, device);

    let total_timesteps: usize = 2_000_000;
    let mut steps = 0usize;
    let mut iteration = 0usize;
    let mut rows_since_header = 0usize;
    let start = Instant::now();

    println!("Starting PPO training — target {} steps", total_timesteps);

    while steps < total_timesteps {
        let (buffer, mean_return, mean_ep_len) = trainer.collect_rollout();
        steps += buffer.len();
        let metrics = trainer.update(&buffer);
        iteration += 1;

        if iteration % 10 == 0 || iteration <= 5 {
            if rows_since_header == 0 {
                println!(
                    "{:<6}  {:<11}  {:>5}  {:>8}  {:>9}  {:>9}  {:>8}  {:>6}  {:>8}  {:>8}  {:>8}",
                    "iter", "steps", "pct", "steps/s", "elapsed", "eta",
                    "mean_ret", "ep_len", "p_loss", "v_loss", "entropy",
                );
            }

            let elapsed_secs = start.elapsed().as_secs_f64();
            let steps_per_sec = steps as f64 / elapsed_secs;
            let pct = 100.0 * steps as f64 / total_timesteps as f64;
            let remaining = total_timesteps.saturating_sub(steps);
            let eta_secs = if steps_per_sec > 0.0 {
                (remaining as f64 / steps_per_sec) as u64
            } else {
                0
            };

            println!(
                "{:<6}  {:<11}  {:>4.1}%  {:>8.0}  {:>9}  {:>9}  {:>8.3}  {:>6.0}  {:>8.4}  {:>8.4}  {:>8.4}",
                iteration,
                steps,
                pct,
                steps_per_sec,
                fmt_duration(elapsed_secs as u64),
                fmt_duration(eta_secs),
                mean_return,
                mean_ep_len,
                metrics.policy_loss,
                metrics.value_loss,
                metrics.entropy,
            );

            rows_since_header += 1;
            if rows_since_header >= 50 {
                rows_since_header = 0;
            }
        }
    }

    let elapsed_secs = start.elapsed().as_secs();
    println!(
        "Training complete ({steps} steps, {iteration} iterations, elapsed {}).",
        fmt_duration(elapsed_secs),
    );

    std::fs::create_dir_all("models").expect("create models dir");
    trainer.save_policy("models/ppo_level_hold");
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
