//! PPO trainer: rollout collection and gradient updates.

use burn::{
    grad_clipping::GradientClippingConfig,
    module::{AutodiffModule, Module},
    optim::{AdamConfig, GradientsParams, Optimizer},
    optim::adaptor::OptimizerAdaptor,
    optim::Adam,
    tensor::{TensorData, Tensor, backend::AutodiffBackend},
};

use crate::training::{LevelHoldEnv, TrainingEnv};
use super::{model::ActorCritic, buffer::{RolloutBuffer, RolloutStep}};

// ---------------------------------------------------------------------------
// Trainer
// ---------------------------------------------------------------------------

pub struct PpoMetrics {
    pub policy_loss: f32,
    pub value_loss:  f32,
    pub entropy:     f32,
}

pub struct PpoTrainer<B: AutodiffBackend> {
    pub model:      ActorCritic<B>,
    optimizer:      OptimizerAdaptor<Adam, ActorCritic<B>, B>,
    pub env:        LevelHoldEnv,
    device:         B::Device,
    // Hyper-parameters
    pub gamma:         f32,
    pub gae_lambda:    f32,
    pub clip_epsilon:  f32,
    pub value_coef:    f32,
    pub entropy_coef:  f32,
    pub lr:            f64,
    pub rollout_steps: usize,
    pub n_epochs:      usize,
    pub minibatch:     usize,
    // Internal RNG seed for minibatch shuffling
    rng_seed:       u64,
}

impl<B: AutodiffBackend> PpoTrainer<B> {
    pub fn new(env: LevelHoldEnv, device: B::Device) -> Self {
        let model = ActorCritic::<B>::new(&device);
        let optimizer = AdamConfig::new()
            .with_grad_clipping(Some(GradientClippingConfig::Norm(0.5)))
            .init::<B, ActorCritic<B>>();
        Self {
            model,
            optimizer,
            env,
            device,
            gamma:         0.99,
            gae_lambda:    0.95,
            clip_epsilon:  0.2,
            value_coef:    0.5,
            entropy_coef:  0.01,
            lr:            3e-4,
            rollout_steps: 2048,
            n_epochs:      4,
            minibatch:     64,
            rng_seed:      12345,
        }
    }

    /// Collect `rollout_steps` environment interactions using the current policy.
    ///
    /// Returns (buffer with GAE computed, mean episode return, mean episode length).
    pub fn collect_rollout(&mut self) -> (RolloutBuffer, f32, f32) {
        let inference_model = self.model.valid();
        let inner_device = inference_model.log_std.val().device();

        let mut buffer = RolloutBuffer::new();
        let (mut obs, _spec) = self.env.reset();
        let mut episode_returns: Vec<f32> = Vec::new();
        let mut episode_lengths: Vec<u32> = Vec::new();
        let mut ep_ret = 0.0_f32;
        let mut ep_len = 0u32;

        for _ in 0..self.rollout_steps {
            let obs_t = obs_to_tensor::<B::InnerBackend>(&obs, &inner_device);
            let (action_t, log_prob_t) = inference_model.sample_action(obs_t.clone());
            let value_t = inference_model.value.forward(obs_t).squeeze_dims::<1>(&[1]);

            let action_vec = action_t.into_data().to_vec::<f32>().expect("action data");
            let log_prob   = log_prob_t.into_data().to_vec::<f32>().expect("log_prob data")[0];
            let value      = value_t.into_data().to_vec::<f32>().expect("value data")[0];

            let (next_obs, reward, done, _info) = self.env.step(&action_vec);

            buffer.push(RolloutStep {
                obs:      obs.clone(),
                action:   action_vec,
                log_prob,
                reward,
                value,
                done,
            });

            ep_ret += reward;
            ep_len += 1;
            obs = next_obs;

            if done {
                episode_returns.push(ep_ret);
                episode_lengths.push(ep_len);
                ep_ret = 0.0;
                ep_len = 0;
                let (reset_obs, _) = self.env.reset();
                obs = reset_obs;
            }
        }

        // Bootstrap last value.
        let last_obs_t = obs_to_tensor::<B::InnerBackend>(&obs, &inner_device);
        let last_value = inference_model.value.forward(last_obs_t)
            .squeeze_dims::<1>(&[1])
            .into_data()
            .to_vec::<f32>()
            .expect("last value")[0];

        buffer.compute_gae(last_value, self.gamma, self.gae_lambda);
        buffer.normalize_advantages();

        let mean_return = if episode_returns.is_empty() {
            ep_ret / self.rollout_steps as f32
        } else {
            episode_returns.iter().sum::<f32>() / episode_returns.len() as f32
        };

        let mean_ep_len = if episode_lengths.is_empty() {
            ep_len as f32
        } else {
            episode_lengths.iter().sum::<u32>() as f32 / episode_lengths.len() as f32
        };

        (buffer, mean_return, mean_ep_len)
    }

    /// Run PPO gradient updates over the collected buffer.
    ///
    /// Returns averaged loss metrics across all minibatch updates.
    pub fn update(&mut self, buffer: &RolloutBuffer) -> PpoMetrics {
        let n = buffer.len();
        let mb = self.minibatch;
        let lr = self.lr;
        let clip = self.clip_epsilon;
        let value_coef = self.value_coef;
        let entropy_coef = self.entropy_coef;

        let mut total_policy_loss = 0.0_f32;
        let mut total_value_loss  = 0.0_f32;
        let mut total_entropy     = 0.0_f32;
        let mut num_updates       = 0usize;

        for _epoch in 0..self.n_epochs {
            // Shuffle indices.
            let mut indices: Vec<usize> = (0..n).collect();
            lcg_shuffle(&mut indices, &mut self.rng_seed);

            let num_batches = n / mb;
            for batch_idx in 0..num_batches {
                let start = batch_idx * mb;
                let end   = start + mb;
                let idx   = &indices[start..end];

                // Build batch tensors.
                let obs_flat: Vec<f32> = idx.iter()
                    .flat_map(|&i| buffer.steps[i].obs.iter().copied())
                    .collect();
                let act_flat: Vec<f32> = idx.iter()
                    .flat_map(|&i| buffer.steps[i].action.iter().copied())
                    .collect();
                let lp_old: Vec<f32>   = idx.iter().map(|&i| buffer.steps[i].log_prob).collect();
                let adv_batch: Vec<f32> = idx.iter().map(|&i| buffer.advantages[i]).collect();
                let ret_batch: Vec<f32>  = idx.iter().map(|&i| buffer.returns[i]).collect();

                let obs_t  = Tensor::<B, 2>::from_data(
                    TensorData::new(obs_flat, vec![mb, 10]), &self.device);
                let act_t  = Tensor::<B, 2>::from_data(
                    TensorData::new(act_flat, vec![mb, 4]), &self.device);
                let lp_old_t = Tensor::<B, 1>::from_data(
                    TensorData::new(lp_old, vec![mb]), &self.device);
                let adv_t   = Tensor::<B, 1>::from_data(
                    TensorData::new(adv_batch, vec![mb]), &self.device);
                let ret_t   = Tensor::<B, 1>::from_data(
                    TensorData::new(ret_batch, vec![mb]), &self.device);

                // Forward pass.
                let (lp_new, entropy) = self.model.evaluate_actions(obs_t.clone(), act_t);
                let values_new = self.model.value.forward(obs_t).squeeze_dims::<1>(&[1]);

                // PPO clipped surrogate loss.
                let ratio = (lp_new - lp_old_t).exp();
                let surr1 = ratio.clone() * adv_t.clone();
                let surr2 = ratio.clamp(1.0_f32 - clip, 1.0_f32 + clip) * adv_t;
                let policy_loss = -surr1.min_pair(surr2).mean();

                // Value loss (MSE).
                let value_loss = (values_new - ret_t).powf_scalar(2.0).mean();

                // Mean entropy (positive; used for display before negating for loss).
                let mean_entropy = entropy.mean();

                // Extract scalar metrics before tensors are consumed.
                total_policy_loss += policy_loss.clone().into_data().to_vec::<f32>().expect("policy_loss")[0];
                total_value_loss  += value_loss.clone().into_data().to_vec::<f32>().expect("value_loss")[0];
                total_entropy     += mean_entropy.clone().into_data().to_vec::<f32>().expect("entropy")[0];
                num_updates       += 1;

                // Combined loss.
                let loss = policy_loss
                    + value_loss * value_coef
                    + (-mean_entropy) * entropy_coef;

                let grads = loss.backward();
                let grads_params = GradientsParams::from_grads(grads, &self.model);
                self.model = self.optimizer.step(lr, self.model.clone(), grads_params);
            }
        }

        let n = num_updates as f32;
        PpoMetrics {
            policy_loss: total_policy_loss / n,
            value_loss:  total_value_loss  / n,
            entropy:     total_entropy     / n,
        }
    }

    /// Save the model to `path` using MessagePack format (burn default).
    /// The file will have `.mpk` appended automatically.
    pub fn save_policy(&self, path: &str) {
        use burn::record::{DefaultFileRecorder, FullPrecisionSettings};
        self.model.clone()
            .save_file(path, &DefaultFileRecorder::<FullPrecisionSettings>::default())
            .unwrap_or_else(|e| panic!("Failed to save model to {path}: {e}"));
        println!("Saved policy to {path}.mpk");
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn obs_to_tensor<B: burn::tensor::backend::Backend>(
    obs:    &[f32],
    device: &B::Device,
) -> Tensor<B, 2> {
    Tensor::from_data(TensorData::new(obs.to_vec(), vec![1, 10]), device)
}

/// In-place Fisher-Yates shuffle using a linear congruential generator.
fn lcg_shuffle(indices: &mut Vec<usize>, seed: &mut u64) {
    let n = indices.len();
    for i in (1..n).rev() {
        *seed = seed.wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let j = (*seed >> 33) as usize % (i + 1);
        indices.swap(i, j);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use burn::backend::{Autodiff, NdArray};
    use crate::training::LevelHoldEnv;
    use crate::plane::config::PlaneConfig;
    use bevy::math::Vec3;

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

    #[test]
    fn ppo_update_no_nan() {
        let device = Default::default();
        let env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
        let mut trainer = PpoTrainer::<B>::new(env, device);
        // Use a tiny rollout so the test is fast.
        trainer.rollout_steps = 64;
        trainer.minibatch     = 64;
        trainer.n_epochs      = 1;

        let (buffer, _mean_ret, _ep_len) = trainer.collect_rollout();
        let _ = trainer.update(&buffer);

        // All model params must be finite after one update.
        let inner = trainer.model.valid();
        let test_obs = Tensor::<<B as AutodiffBackend>::InnerBackend, 2>::zeros(
            [1, 10], &inner.log_std.val().device());
        let (action, lp) = inner.sample_action(test_obs);
        for v in action.into_data().to_vec::<f32>().unwrap() {
            assert!(v.is_finite(), "action NaN after update: {v}");
        }
        for v in lp.into_data().to_vec::<f32>().unwrap() {
            assert!(v.is_finite(), "log_prob NaN after update: {v}");
        }
    }
}
