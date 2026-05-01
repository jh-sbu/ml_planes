//! PPO trainer: rollout collection and gradient updates.

use burn::{
    grad_clipping::GradientClippingConfig,
    module::{AutodiffModule, Module},
    optim::{AdamConfig, GradientsParams, Optimizer},
    optim::adaptor::OptimizerAdaptor,
    optim::Adam,
    tensor::{TensorData, Tensor, backend::AutodiffBackend},
};

use crate::training::{LevelHoldEnv, Observation, VecEnv};
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
    pub envs:       VecEnv<LevelHoldEnv>,
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
    // Last observations from the previous rollout — avoids hard-resetting all envs each call.
    last_obs:       Option<Vec<Observation>>,
}

impl<B: AutodiffBackend> PpoTrainer<B> {
    /// Create a trainer with a single environment.
    pub fn new(env: LevelHoldEnv, device: B::Device) -> Self {
        Self::with_n_envs(env, 1, device)
    }

    /// Create a trainer with `n` independent copies of `template_env`.
    pub fn with_n_envs(template_env: LevelHoldEnv, n: usize, device: B::Device) -> Self {
        assert!(n >= 1, "n_envs must be at least 1");
        let model = ActorCritic::<B>::new(&device);
        let optimizer = AdamConfig::new()
            .with_grad_clipping(Some(GradientClippingConfig::Norm(0.5)))
            .init::<B, ActorCritic<B>>();
        let envs = VecEnv::new((0..n).map(|i| {
            let mut e = template_env.clone();
            if i > 0 { e.offset_rng_seed(i as u64 * 1_000); }
            e
        }).collect());
        Self {
            model,
            optimizer,
            envs,
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
            last_obs:      None,
        }
    }

    /// Collect `rollout_steps` environment interactions using the current policy.
    ///
    /// Inference is batched over all N environments at once ([N, 10] → [N, 4]),
    /// reducing GPU readbacks from 3 × rollout_steps to 3 × (rollout_steps / N).
    /// GAE is computed per-environment so episode boundaries don't bleed across
    /// env indices, then all buffers are merged for minibatch updates.
    ///
    /// Returns (buffer with GAE computed, mean episode return, mean episode length).
    pub fn collect_rollout(&mut self) -> (RolloutBuffer, f32, f32) {
        let n = self.envs.n();
        let inference_model = self.model.valid();
        let inner_device = inference_model.log_std.val().device();
        let steps_per_env = self.rollout_steps / n;
        assert!(steps_per_env > 0,
            "rollout_steps ({}) must be >= n_envs ({})", self.rollout_steps, n);

        let mut env_buffers: Vec<RolloutBuffer> = (0..n)
            .map(|_| RolloutBuffer::with_capacity(steps_per_env))
            .collect();

        let mut obs_batch: Vec<Observation> = self.last_obs.take()
            .unwrap_or_else(|| self.envs.reset_all());
        let mut episode_returns: Vec<f32> = Vec::with_capacity(8 * n);
        let mut episode_lengths: Vec<u32> = Vec::with_capacity(8 * n);
        let mut ep_ret: Vec<f32> = vec![0.0; n];
        let mut ep_len: Vec<u32> = vec![0; n];

        for _ in 0..steps_per_env {
            // Batch [n, 10] obs tensor — one inference call for all envs.
            let obs_flat: Vec<f32> = obs_batch.iter()
                .flat_map(|o| o.iter().copied())
                .collect();
            let obs_t = Tensor::<B::InnerBackend, 2>::from_data(
                TensorData::new(obs_flat, vec![n, 10]), &inner_device);

            let (action_t, log_prob_t) = inference_model.sample_action(obs_t.clone());
            let value_t = inference_model.value.forward(obs_t).squeeze_dims::<1>(&[1]);

            // Three readbacks — each reads N values in a single transfer.
            let action_data   = action_t.into_data().to_vec::<f32>().expect("action data");
            let log_prob_data = log_prob_t.into_data().to_vec::<f32>().expect("log_prob data");
            let value_data    = value_t.into_data().to_vec::<f32>().expect("value data");

            let actions: Vec<[f32; 4]> = (0..n)
                .map(|i| [action_data[i*4], action_data[i*4+1],
                          action_data[i*4+2], action_data[i*4+3]])
                .collect();

            let (next_obs, rewards, dones) = self.envs.step_batch(&actions);

            for i in 0..n {
                env_buffers[i].push(RolloutStep {
                    obs:      obs_batch[i],
                    action:   actions[i],
                    log_prob: log_prob_data[i],
                    reward:   rewards[i],
                    value:    value_data[i],
                    done:     dones[i],
                });
                ep_ret[i] += rewards[i];
                ep_len[i] += 1;
                if dones[i] {
                    episode_returns.push(ep_ret[i]);
                    episode_lengths.push(ep_len[i]);
                    ep_ret[i] = 0.0;
                    ep_len[i] = 0;
                }
            }

            obs_batch = next_obs;
            for i in 0..n {
                if dones[i] {
                    obs_batch[i] = self.envs.reset_at(i);
                }
            }
        }

        // Bootstrap: one batched inference for all N last observations.
        let last_flat: Vec<f32> = obs_batch.iter()
            .flat_map(|o| o.iter().copied())
            .collect();
        let last_obs_t = Tensor::<B::InnerBackend, 2>::from_data(
            TensorData::new(last_flat, vec![n, 10]), &inner_device);
        let last_values = inference_model.value.forward(last_obs_t)
            .squeeze_dims::<1>(&[1])
            .into_data()
            .to_vec::<f32>()
            .expect("last values");

        // GAE per env so episode boundaries don't bleed across env indices.
        for (i, buf) in env_buffers.iter_mut().enumerate() {
            buf.compute_gae(last_values[i], self.gamma, self.gae_lambda);
        }

        // Merge into one buffer for minibatch updates.
        let total = steps_per_env * n;
        let mut merged = RolloutBuffer::with_capacity(total);
        merged.advantages = Vec::with_capacity(total);
        merged.returns    = Vec::with_capacity(total);
        for buf in &env_buffers {
            for step in &buf.steps {
                merged.steps.push(*step);
            }
            merged.advantages.extend_from_slice(&buf.advantages);
            merged.returns.extend_from_slice(&buf.returns);
        }
        merged.normalize_advantages();

        let mean_return = if episode_returns.is_empty() {
            ep_ret.iter().sum::<f32>() / n as f32 / steps_per_env as f32
        } else {
            episode_returns.iter().sum::<f32>() / episode_returns.len() as f32
        };
        let mean_ep_len = if episode_lengths.is_empty() {
            ep_len.iter().sum::<u32>() as f32 / n as f32
        } else {
            episode_lengths.iter().sum::<u32>() as f32 / episode_lengths.len() as f32
        };

        self.last_obs = Some(obs_batch);

        (merged, mean_return, mean_ep_len)
    }

    /// Run PPO gradient updates over the collected buffer.
    ///
    /// Returns loss metrics sampled once per epoch. Sampling every minibatch
    /// would force a wgpu readback on each scalar metric.
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
        let mut num_metric_samples = 0usize;
        let mut indices: Vec<usize> = (0..n).collect();

        for _epoch in 0..self.n_epochs {
            // Shuffle indices.
            for (i, value) in indices.iter_mut().enumerate() {
                *value = i;
            }
            lcg_shuffle(&mut indices, &mut self.rng_seed);

            let num_batches = n / mb;
            for batch_idx in 0..num_batches {
                let start = batch_idx * mb;
                let end   = start + mb;
                let idx   = &indices[start..end];

                // Build batch tensors.
                let mut obs_flat = Vec::with_capacity(mb * 10);
                let mut act_flat = Vec::with_capacity(mb * 4);
                let mut lp_old = Vec::with_capacity(mb);
                let mut adv_batch = Vec::with_capacity(mb);
                let mut ret_batch = Vec::with_capacity(mb);

                for &i in idx {
                    let step = &buffer.steps[i];
                    obs_flat.extend_from_slice(&step.obs);
                    act_flat.extend_from_slice(&step.action);
                    lp_old.push(step.log_prob);
                    adv_batch.push(buffer.advantages[i]);
                    ret_batch.push(buffer.returns[i]);
                }

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

                // Extract scalar metrics only once per epoch. Each into_data()
                // on wgpu synchronizes with the device.
                if batch_idx + 1 == num_batches {
                    total_policy_loss += policy_loss.clone().into_data().to_vec::<f32>().expect("policy_loss")[0];
                    total_value_loss  += value_loss.clone().into_data().to_vec::<f32>().expect("value_loss")[0];
                    total_entropy     += mean_entropy.clone().into_data().to_vec::<f32>().expect("entropy")[0];
                    num_metric_samples += 1;
                }

                // Combined loss.
                let loss = policy_loss
                    + value_loss * value_coef
                    + (-mean_entropy) * entropy_coef;

                let grads = loss.backward();
                let grads_params = GradientsParams::from_grads(grads, &self.model);
                self.model = self.optimizer.step(lr, self.model.clone(), grads_params);
            }
        }

        let n = num_metric_samples as f32;
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
    fn n_envs_have_distinct_initial_obs() {
        let env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
        let mut trainer = PpoTrainer::<B>::with_n_envs(env, 4, Default::default());
        // First collect_rollout resets all envs; each should start from a distinct spawn.
        let obs0 = trainer.envs.reset_all();
        // All 4 observations must not all be identical.
        let all_same = obs0.windows(2).all(|w| w[0] == w[1]);
        assert!(!all_same, "all envs produced identical initial observations — RNG seeds are not offset");
    }

    #[test]
    #[should_panic(expected = "rollout_steps")]
    fn collect_rollout_panics_when_n_envs_exceeds_rollout_steps() {
        let env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
        let mut trainer = PpoTrainer::<B>::with_n_envs(env, 4, Default::default());
        trainer.rollout_steps = 2; // fewer steps than envs → steps_per_env = 0
        let _ = trainer.collect_rollout();
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

    #[test]
    fn ppo_multi_env_no_nan() {
        let device = Default::default();
        let env = LevelHoldEnv::new(1000.0, 80.0, jet_cfg());
        let mut trainer = PpoTrainer::<B>::with_n_envs(env, 4, device);
        trainer.rollout_steps = 64;  // 16 steps_per_env × 4 envs
        trainer.minibatch     = 64;
        trainer.n_epochs      = 1;

        let (buffer, _mean_ret, _ep_len) = trainer.collect_rollout();
        assert_eq!(buffer.len(), 64);
        let _ = trainer.update(&buffer);

        let inner = trainer.model.valid();
        let test_obs = Tensor::<<B as AutodiffBackend>::InnerBackend, 2>::zeros(
            [1, 10], &inner.log_std.val().device());
        let (action, lp) = inner.sample_action(test_obs);
        for v in action.into_data().to_vec::<f32>().unwrap() {
            assert!(v.is_finite(), "action NaN after multi-env update: {v}");
        }
        for v in lp.into_data().to_vec::<f32>().unwrap() {
            assert!(v.is_finite(), "log_prob NaN after multi-env update: {v}");
        }
    }
}
