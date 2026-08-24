//! PPO integration tests. Module of the consolidated `rl` test binary (see
//! plans/test_compile_speed.md).
//!
//! All tests require the `training` feature:
//!   cargo test --no-default-features --features training --test rl ppo_training::

use burn::backend::{Autodiff, NdArray};

use ml_planes::plane::config::PlaneConfig;
use ml_planes::training::level_hold_env::LEVEL_HOLD_OBS_DIM;
use ml_planes::training::ppo::PpoTrainer;
use ml_planes::training::{LevelHoldEnv, OrbitEnv};

type B = Autodiff<NdArray>;
const ORBIT_OBS_DIM: usize = 14;

/// The shared frozen test airframe. Delegates to the `tests/common` helper rather
/// than re-typing the literal — see `fixtures/generic_jet.plane.ron`.
fn jet_cfg() -> PlaneConfig {
    crate::common::generic_jet_config()
}

/// 50 PPO iterations with a tiny rollout (128 steps/iter). Checks: no NaN in
/// policy output after training.
///
/// Does **not** assert anything about the reward trend. `PpoTrainer::new` builds
/// its `ActorCritic` via unseeded module initialization. At 6,400 steps the policy
/// remains in random exploration, so reward-trend assertions are unstable. Convergence
/// validation lives in the slower `evaluate_policy` / `train-evaluate-optimize`
/// workflows instead, which run full-length training.
#[test]
fn ppo_50_iterations_no_nan() {
    use burn::module::AutodiffModule;
    use burn::tensor::Tensor;

    let device: <B as burn::tensor::backend::Backend>::Device = Default::default();
    // Randomized-target env, exercising the same path `train_ppo` uses in
    // production (see `LevelHoldEnv::with_target_ranges`).
    let env = LevelHoldEnv::with_target_ranges(
        500.0..=5000.0,
        90.0..=140.0,
        jet_cfg(),
        ml_planes::training::LevelHoldRewardConfig::default(),
    );
    let mut trainer = PpoTrainer::<B>::new(env, device);
    trainer.rollout_steps = 128;
    trainer.minibatch = 32;
    trainer.n_epochs = 2;

    for _i in 0..50 {
        let (buffer, _mean_ret, _ep_len) = trainer.collect_rollout();
        let _ = trainer.update(&buffer);
    }

    // Policy output must remain finite.
    let inner = trainer.model.valid();
    let inner_device = inner.log_std.val().device();
    let test_obs = Tensor::<<B as burn::tensor::backend::AutodiffBackend>::InnerBackend, 2>::zeros(
        [1, LEVEL_HOLD_OBS_DIM],
        &inner_device,
    );
    let (action, lp) = inner.sample_action(test_obs);
    for v in action.into_data().to_vec::<f32>().unwrap() {
        assert!(v.is_finite(), "action NaN after 50 PPO iterations: {v}");
    }
    for v in lp.into_data().to_vec::<f32>().unwrap() {
        assert!(v.is_finite(), "log_prob NaN after 50 PPO iterations: {v}");
    }
}

#[test]
fn orbit_ppo_short_loop_no_nan() {
    use burn::module::AutodiffModule;
    use burn::tensor::Tensor;

    let device: <B as burn::tensor::backend::Backend>::Device = Default::default();
    let env = OrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
    let mut trainer = PpoTrainer::<B, OrbitEnv>::new(env, device);
    trainer.rollout_steps = 64;
    trainer.minibatch = 32;
    trainer.n_epochs = 1;

    let reset_obs = trainer.envs.reset_all();
    assert_eq!(reset_obs[0].len(), ORBIT_OBS_DIM);
    trainer.envs.reset_at(0);

    for _ in 0..3 {
        let (buffer, _mean_ret, _ep_len) = trainer.collect_rollout();
        assert_eq!(buffer.steps[0].obs.len(), ORBIT_OBS_DIM);
        let _ = trainer.update(&buffer);
    }

    let inner = trainer.model.valid();
    let inner_device = inner.log_std.val().device();
    let test_obs = Tensor::<<B as burn::tensor::backend::AutodiffBackend>::InnerBackend, 2>::zeros(
        [1, ORBIT_OBS_DIM],
        &inner_device,
    );
    let (action, lp) = inner.sample_action(test_obs);
    for v in action.into_data().to_vec::<f32>().unwrap() {
        assert!(v.is_finite(), "orbit action NaN after PPO iterations: {v}");
    }
    for v in lp.into_data().to_vec::<f32>().unwrap() {
        assert!(
            v.is_finite(),
            "orbit log_prob NaN after PPO iterations: {v}"
        );
    }
}
