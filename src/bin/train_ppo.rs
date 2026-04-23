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
        cm0: -0.02, cm_alpha: 0.6,  cm_q: -8.0,        cm_delta_e: -1.2,
        cl_beta: -0.08, cl_p: -0.45, cl_r: 0.12, cl_delta_a: 0.18,
        cn_beta: 0.10, cn_r: -0.12, cn_delta_r: -0.10,
        thrust_max: 60000.0,
        aileron_limit: 0.4363, elevator_limit: 0.3491, rudder_limit: 0.2618,
    };
    let env = LevelHoldEnv::new(1000.0, 80.0, cfg);

    let device: <B as burn::tensor::backend::Backend>::Device = Default::default();
    let mut trainer = PpoTrainer::<B>::new(env, device);

    let total_timesteps: usize = 2_000_000;
    let mut steps = 0usize;
    let mut iteration = 0usize;

    println!("Starting PPO training — target {} steps", total_timesteps);
    println!("{:<6}  {:<10}  {}", "iter", "steps", "mean_return");

    while steps < total_timesteps {
        let (buffer, mean_return) = trainer.collect_rollout();
        steps += buffer.len();
        trainer.update(&buffer);
        iteration += 1;

        if iteration % 10 == 0 || iteration <= 5 {
            println!("{:<6}  {:<10}  {:.3}", iteration, steps, mean_return);
        }
    }

    println!("Training complete ({steps} steps, {iteration} iterations).");

    std::fs::create_dir_all("models").expect("create models dir");
    trainer.save_policy("models/ppo_level_hold");
}
