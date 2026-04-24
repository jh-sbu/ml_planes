//! PPO integration tests.
//!
//! All tests require the `training` feature:
//!   cargo test --no-default-features --features training --test ppo_training

#![cfg(feature = "training")]

use burn::backend::{Autodiff, NdArray};
use bevy::math::Vec3;

use ml_planes::plane::config::PlaneConfig;
use ml_planes::training::LevelHoldEnv;
use ml_planes::training::ppo::PpoTrainer;

type B = Autodiff<NdArray>;

fn jet_cfg() -> PlaneConfig {
    PlaneConfig {
        wing_area: 20.0, mean_chord: 2.0, wing_span: 10.0,
        mass: 5000.0, inertia: Vec3::new(10000.0, 40000.0, 45000.0),
        cl0: 0.1, cl_alpha: 4.5, cl_delta_e: 0.4, cl_max: 1.4,
        cd0: 0.02, cd_induced: 0.05,
        cm0: -0.02, cm_alpha: 0.6, cm_q: -14.0, cm_delta_e: -1.2,
        cl_beta: -0.08, cl_p: -0.45, cl_r: 0.12, cl_delta_a: 0.18,
        cn_beta: 0.10, cn_r: -0.12, cn_delta_r: -0.10,
        thrust_max: 60000.0,
        aileron_limit: 0.4363, elevator_limit: 0.3491, rudder_limit: 0.2618,
    }
}

/// 50 PPO iterations with a tiny rollout (128 steps/iter).
/// Checks: no NaN in policy output, reward doesn't blow up.
#[test]
fn ppo_50_iterations_no_nan_no_reward_collapse() {
    use burn::module::AutodiffModule;
    use burn::tensor::Tensor;

    let device: <B as burn::tensor::backend::Backend>::Device = Default::default();
    let env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
    let mut trainer = PpoTrainer::<B>::new(env, device);
    trainer.rollout_steps = 128;
    trainer.minibatch     = 32;
    trainer.n_epochs      = 2;

    let mut returns: Vec<f32> = Vec::new();

    for _i in 0..50 {
        let (buffer, mean_ret) = trainer.collect_rollout();
        trainer.update(&buffer);
        returns.push(mean_ret);
    }

    // Policy output must remain finite.
    let inner = trainer.model.valid();
    let inner_device = inner.log_std.val().device();
    let test_obs = Tensor::<<B as burn::tensor::backend::AutodiffBackend>::InnerBackend, 2>
        ::zeros([1, 8], &inner_device);
    let (action, lp) = inner.sample_action(test_obs);
    for v in action.into_data().to_vec::<f32>().unwrap() {
        assert!(v.is_finite(), "action NaN after 50 PPO iterations: {v}");
    }
    for v in lp.into_data().to_vec::<f32>().unwrap() {
        assert!(v.is_finite(), "log_prob NaN after 50 PPO iterations: {v}");
    }

    // Weak reward check: last 10 iterations shouldn't be wildly worse than first 10.
    let first_mean = returns[..10].iter().sum::<f32>() / 10.0;
    let last_mean  = returns[40..].iter().sum::<f32>() / 10.0;
    assert!(
        last_mean > first_mean - 5.0,
        "reward collapsed: first_mean={first_mean:.3}, last_mean={last_mean:.3}"
    );
}
