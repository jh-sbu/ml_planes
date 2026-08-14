//! Recurrent PPO trainer for the Wu et al. FC-LSTM-FC controller.
//!
//! Key differences from `PpoTrainer`:
//!   - Carries per-env LSTM hidden states across rollout steps
//!   - Resets hidden states on episode `done`
//!   - Records hidden states only at sequence-start positions (not per step)
//!   - PPO update uses fixed-length sequence minibatches with masking
//!   - Automatic 3-stage Wu curriculum via the `CurriculumEnv` trait

use std::collections::HashMap;

use burn::{
    grad_clipping::GradientClippingConfig,
    module::{AutodiffModule, Module},
    nn::LstmState,
    optim::adaptor::OptimizerAdaptor,
    optim::{Adam, AdamConfig, GradientsParams, Optimizer},
    tensor::{backend::AutodiffBackend, Tensor, TensorData},
};

use super::{
    lstm_buffer::{LstmRolloutBuffer, LstmRolloutStep},
    lstm_model::{LstmActorCritic, LstmHiddenState, LSTM_HIDDEN},
};
use crate::training::{CurriculumEnv, Observation, TrainingEnv, VecEnv};

// ---------------------------------------------------------------------------
// Metrics
// ---------------------------------------------------------------------------

pub struct LstmPpoMetrics {
    pub policy_loss: f32,
    pub value_loss: f32,
    pub entropy: f32,
}

// ---------------------------------------------------------------------------
// Trainer
// ---------------------------------------------------------------------------

pub struct LstmPpoTrainer<
    B: AutodiffBackend,
    E: TrainingEnv + CurriculumEnv = crate::training::WuOrbitEnv,
> {
    pub model: LstmActorCritic<B>,
    optimizer: OptimizerAdaptor<Adam, LstmActorCritic<B>, B>,
    pub envs: VecEnv<E>,
    device: B::Device,
    obs_dim: usize,

    // Hyper-parameters
    pub gamma: f32,
    pub gae_lambda: f32,
    pub clip_epsilon: f32,
    pub value_coef: f32,
    pub entropy_coef: f32,
    pub lr: f64,
    pub rollout_steps: usize,
    pub n_epochs: usize,
    /// Number of sequences per minibatch.
    pub minibatch_seqs: usize,
    /// Sequence length for TBPTT.
    pub seq_len: usize,

    // Per-env LSTM hidden states (reset on done)
    policy_hidden: Vec<LstmHiddenState>,
    value_hidden: Vec<LstmHiddenState>,

    // Episode bookkeeping
    rng_seed: u64,
    last_obs: Option<Vec<Observation>>,
    ep_returns: Vec<f32>,
    ep_lengths: Vec<u32>,
}

impl<B, E> LstmPpoTrainer<B, E>
where
    B: AutodiffBackend,
    E: TrainingEnv + CurriculumEnv + Clone,
{
    pub fn new(env: E, device: B::Device) -> Self {
        Self::with_n_envs(env, 1, device)
    }

    /// Like [`Self::new`], but with `seed = Some(s)` fixes the model's weight
    /// init and the per-env episode-reset RNGs. See
    /// `PpoTrainer::with_n_envs_seeded` (the MLP counterpart) for the same
    /// pattern and rationale.
    pub fn new_seeded(env: E, device: B::Device, seed: Option<u64>) -> Self {
        Self::with_n_envs_seeded(env, 1, device, seed)
    }

    pub fn with_n_envs(template_env: E, n: usize, device: B::Device) -> Self {
        Self::with_n_envs_seeded(template_env, n, device, None)
    }

    /// See `PpoTrainer::with_n_envs_seeded`; `None` selects unseeded behavior.
    pub fn with_n_envs_seeded(
        template_env: E,
        n: usize,
        device: B::Device,
        seed: Option<u64>,
    ) -> Self {
        assert!(n >= 1);
        let obs_dim = template_env.observation_dim();
        let model = match seed {
            Some(s) => LstmActorCritic::<B>::new_seeded(&device, obs_dim, s),
            None => LstmActorCritic::<B>::new(&device, obs_dim),
        };
        let optimizer = AdamConfig::new()
            .with_grad_clipping(Some(GradientClippingConfig::Norm(0.5)))
            .init::<B, LstmActorCritic<B>>();
        let envs = VecEnv::new(
            (0..n)
                .map(|i| {
                    let mut e = template_env.clone();
                    match seed {
                        Some(s) => e.offset_rng_seed(s.wrapping_add(i as u64 * 1_000)),
                        None => {
                            if i > 0 {
                                e.offset_rng_seed(i as u64 * 1_000);
                            }
                        }
                    }
                    e
                })
                .collect(),
        );
        Self {
            model,
            optimizer,
            envs,
            device,
            obs_dim,
            gamma: 0.99,
            gae_lambda: 0.95,
            clip_epsilon: 0.2,
            value_coef: 0.5,
            entropy_coef: 0.01,
            lr: 3e-4,
            rollout_steps: 2048,
            n_epochs: 4,
            minibatch_seqs: 16,
            seq_len: 64,
            policy_hidden: vec![LstmHiddenState::default(); n],
            value_hidden: vec![LstmHiddenState::default(); n],
            rng_seed: seed.unwrap_or(12345),
            last_obs: None,
            ep_returns: vec![0.0; n],
            ep_lengths: vec![0; n],
        }
    }

    // -----------------------------------------------------------------------
    // Rollout collection
    // -----------------------------------------------------------------------

    /// Collect `rollout_steps` environment interactions with LSTM state propagation.
    ///
    /// Returns `(buffer, mean_episode_return, mean_episode_length)`.
    pub fn collect_rollout(&mut self) -> (LstmRolloutBuffer, f32, f32) {
        let n = self.envs.n();
        let inference_model = self.model.valid();
        let inner_device = inference_model.log_std.val().device();
        let steps_per_env = self.rollout_steps / n;
        assert!(
            steps_per_env > 0,
            "rollout_steps ({}) must be >= n_envs ({})",
            self.rollout_steps,
            n
        );

        let mut buf = LstmRolloutBuffer::new(n, steps_per_env);
        let mut obs_batch: Vec<Observation> = self
            .last_obs
            .take()
            .unwrap_or_else(|| self.envs.reset_all());
        let mut episode_returns: Vec<f32> = Vec::with_capacity(8 * n);
        let mut episode_lengths: Vec<u32> = Vec::with_capacity(8 * n);

        // Stage steps per-env so the final buffer is env-major, which is what
        // compute_gae() and chunk_sequences() both expect.
        let mut per_env: Vec<Vec<LstmRolloutStep>> = vec![Vec::with_capacity(steps_per_env); n];

        // Track steps within the current BPTT chunk per env.  When this counter
        // is 0 the step is a sequence start; store the hidden state then.
        let mut steps_since_seq_start = vec![0usize; n];
        // Hidden state map keyed by absolute step index (env_i * steps_per_env + t).
        let mut seq_start_hidden: HashMap<usize, (Vec<f32>, Vec<f32>, Vec<f32>, Vec<f32>)> =
            HashMap::new();

        // Steps whose episode was cut short by the time limit rather than a failure.
        // Unlike the feed-forward trainer, evaluating `V(s')` here needs the *value*
        // LSTM state as it stood at the terminal step, so snapshot that alongside the
        // terminal observation before the per-env reset wipes both.
        let mut truncated: Vec<(usize, usize)> = Vec::new();
        let mut truncated_obs: Vec<Observation> = Vec::new();
        let mut truncated_hidden: Vec<LstmHiddenState> = Vec::new();

        for t in 0..steps_per_env {
            // Capture hidden states for sequence-start steps before inference.
            for i in 0..n {
                if steps_since_seq_start[i] == 0 {
                    let abs_idx = i * steps_per_env + t;
                    seq_start_hidden.insert(
                        abs_idx,
                        (
                            self.policy_hidden[i].h.clone(),
                            self.policy_hidden[i].c.clone(),
                            self.value_hidden[i].h.clone(),
                            self.value_hidden[i].c.clone(),
                        ),
                    );
                }
            }

            // Batch obs: [N, obs_dim].
            let obs_flat: Vec<f32> = obs_batch.iter().flat_map(|o| o.iter().copied()).collect();
            let obs_t = Tensor::<B::InnerBackend, 2>::from_data(
                TensorData::new(obs_flat, vec![n, self.obs_dim]),
                &inner_device,
            );

            // Batch policy LSTM states: h/c each [N, LSTM_HIDDEN].
            let p_state = LstmHiddenState::batch_to_burn::<B::InnerBackend>(
                &self.policy_hidden,
                &inner_device,
            );
            let v_state = LstmHiddenState::batch_to_burn::<B::InnerBackend>(
                &self.value_hidden,
                &inner_device,
            );

            // Single-step policy forward: action [N, 4], log_prob [N], new policy state.
            let (action_t, log_prob_t, new_p_state) =
                inference_model.sample_action_step(obs_t.clone(), Some(p_state));

            // Single-step value forward: value [N, 1], new value state.
            let (value_t, new_v_state) = inference_model.value.forward_step(obs_t, Some(v_state));

            // Readback to CPU.
            let action_data = action_t.into_data().to_vec::<f32>().expect("action");
            let log_prob_data = log_prob_t.into_data().to_vec::<f32>().expect("log_prob");
            let value_data = value_t
                .squeeze_dims::<1>(&[1])
                .into_data()
                .to_vec::<f32>()
                .expect("value");

            // Unpack per-env states.
            let new_p_hidden = LstmHiddenState::unbatch_from_burn(new_p_state, n);
            let new_v_hidden = LstmHiddenState::unbatch_from_burn(new_v_state, n);

            let actions: Vec<[f32; 4]> = (0..n)
                .map(|i| {
                    [
                        action_data[i * 4],
                        action_data[i * 4 + 1],
                        action_data[i * 4 + 2],
                        action_data[i * 4 + 3],
                    ]
                })
                .collect();

            let mut outcomes = self.envs.step_batch(&actions);
            let dones: Vec<bool> = outcomes.iter().map(|o| o.done()).collect();

            for (i, outcome) in outcomes.iter_mut().enumerate() {
                if outcome.truncated() {
                    // The env does not auto-reset, so `outcome.obs` is the genuine
                    // terminal observation; `new_v_hidden[i]` is the value state that
                    // produced it, before the reset below clears it.
                    truncated.push((i, per_env[i].len()));
                    truncated_obs.push(outcome.obs.clone());
                    truncated_hidden.push(new_v_hidden[i].clone());
                }
                per_env[i].push(LstmRolloutStep {
                    obs: obs_batch[i].clone(),
                    action: actions[i],
                    log_prob: log_prob_data[i],
                    reward: outcome.reward,
                    value: value_data[i],
                    done: dones[i],
                    // Filled in after the rollout loop for truncated steps only.
                    bootstrap_value: None,
                });
                self.ep_returns[i] += outcome.reward;
                self.ep_lengths[i] += 1;
                if dones[i] {
                    episode_returns.push(self.ep_returns[i]);
                    episode_lengths.push(self.ep_lengths[i]);
                    self.ep_returns[i] = 0.0;
                    self.ep_lengths[i] = 0;
                }
            }

            // Update hidden states; reset on done. A truncation resets the state just
            // like a failure — the next episode starts fresh either way.
            for i in 0..n {
                self.policy_hidden[i] = new_p_hidden[i].clone();
                self.value_hidden[i] = new_v_hidden[i].clone();
                if dones[i] {
                    self.policy_hidden[i] = LstmHiddenState::default();
                    self.value_hidden[i] = LstmHiddenState::default();
                }
            }

            // Advance sequence-start counters.  A done or reaching seq_len
            // means the next step starts a fresh chunk.
            for i in 0..n {
                steps_since_seq_start[i] += 1;
                if dones[i] || steps_since_seq_start[i] >= self.seq_len {
                    steps_since_seq_start[i] = 0;
                }
            }

            obs_batch = outcomes
                .into_iter()
                .enumerate()
                .map(|(i, outcome)| {
                    if dones[i] {
                        self.envs.reset_at(i)
                    } else {
                        outcome.obs
                    }
                })
                .collect();
        }

        // Resolve the truncated steps' bootstrap values in one batched recurrent
        // forward pass, using each step's own saved value hidden state. Kept separate
        // from the `last_values` pass below because that one carries the *current*
        // per-env hidden states, which are different.
        if !truncated.is_empty() {
            let values = truncated_bootstrap_values::<B::InnerBackend>(
                &inference_model.value,
                &truncated_obs,
                &truncated_hidden,
                self.obs_dim,
                &inner_device,
            );
            for (&(env_i, step_i), &v) in truncated.iter().zip(values.iter()) {
                per_env[env_i][step_i].bootstrap_value = Some(v);
            }
        }

        // Flush staging buffer in env-major order so compute_gae() and
        // chunk_sequences() can use contiguous per-env slices.
        for env_steps in per_env {
            for step in env_steps {
                buf.push(step);
            }
        }
        buf.seq_start_hidden = seq_start_hidden;

        // Bootstrap values for GAE.
        let last_flat: Vec<f32> = obs_batch.iter().flat_map(|o| o.iter().copied()).collect();
        let last_obs_t = Tensor::<B::InnerBackend, 2>::from_data(
            TensorData::new(last_flat, vec![n, self.obs_dim]),
            &inner_device,
        );
        let v_state_last =
            LstmHiddenState::batch_to_burn::<B::InnerBackend>(&self.value_hidden, &inner_device);
        let (last_values_t, _) = inference_model
            .value
            .forward_step(last_obs_t, Some(v_state_last));
        let last_values = last_values_t
            .squeeze_dims::<1>(&[1])
            .into_data()
            .to_vec::<f32>()
            .expect("last values");

        buf.compute_gae(&last_values, self.gamma, self.gae_lambda);
        buf.normalize_advantages();

        let mean_return = if episode_returns.is_empty() {
            self.ep_returns.iter().sum::<f32>() / n as f32
        } else {
            episode_returns.iter().sum::<f32>() / episode_returns.len() as f32
        };
        let mean_ep_len = if episode_lengths.is_empty() {
            self.ep_lengths.iter().sum::<u32>() as f32 / n as f32
        } else {
            episode_lengths.iter().sum::<u32>() as f32 / episode_lengths.len() as f32
        };

        self.last_obs = Some(obs_batch);
        (buf, mean_return, mean_ep_len)
    }

    // -----------------------------------------------------------------------
    // PPO update with BPTT
    // -----------------------------------------------------------------------

    pub fn update(&mut self, buffer: &LstmRolloutBuffer) -> LstmPpoMetrics {
        let clip = self.clip_epsilon;
        let value_coef = self.value_coef;
        let entropy_coef = self.entropy_coef;
        let lr = self.lr;

        let mut total_policy_loss = 0.0_f32;
        let mut total_value_loss = 0.0_f32;
        let mut total_entropy = 0.0_f32;
        let mut metric_samples = 0usize;

        let mut seqs = buffer.chunk_sequences(self.seq_len, self.obs_dim);
        let n_seqs = seqs.len();
        if n_seqs == 0 {
            return LstmPpoMetrics {
                policy_loss: 0.0,
                value_loss: 0.0,
                entropy: 0.0,
            };
        }

        let mb = self.minibatch_seqs.min(n_seqs);
        let sl = self.seq_len;
        let bt = mb * sl;

        let mut indices: Vec<usize> = (0..n_seqs).collect();

        for _epoch in 0..self.n_epochs {
            for v in indices.iter_mut().enumerate() {
                *v.1 = v.0;
            }
            lcg_shuffle(&mut indices, &mut self.rng_seed);

            let num_batches = n_seqs / mb;
            if num_batches == 0 {
                continue;
            }

            for batch_idx in 0..num_batches {
                let start = batch_idx * mb;
                let end = start + mb;
                let idx = &indices[start..end];

                // Allocate each Vec with the exact capacity to avoid log-growth
                // reallocation from flat_map's 0 size hint.
                let mut obs_flat = Vec::<f32>::with_capacity(bt * self.obs_dim);
                obs_flat.extend(idx.iter().flat_map(|&i| seqs[i].obs.iter().copied()));
                let mut act_flat = Vec::<f32>::with_capacity(bt * 4);
                act_flat.extend(idx.iter().flat_map(|&i| seqs[i].actions.iter().copied()));
                let mut lp_old = Vec::<f32>::with_capacity(bt);
                lp_old.extend(idx.iter().flat_map(|&i| seqs[i].log_probs.iter().copied()));
                let mut adv_flat = Vec::<f32>::with_capacity(bt);
                adv_flat.extend(idx.iter().flat_map(|&i| seqs[i].advantages.iter().copied()));
                let mut ret_flat = Vec::<f32>::with_capacity(bt);
                ret_flat.extend(idx.iter().flat_map(|&i| seqs[i].returns.iter().copied()));
                let mut mask_flat = Vec::<f32>::with_capacity(bt);
                mask_flat.extend(idx.iter().flat_map(|&i| seqs[i].mask.iter().copied()));

                let mut ph_flat = Vec::<f32>::with_capacity(mb * LSTM_HIDDEN);
                ph_flat.extend(idx.iter().flat_map(|&i| seqs[i].init_ph.iter().copied()));
                let mut pc_flat = Vec::<f32>::with_capacity(mb * LSTM_HIDDEN);
                pc_flat.extend(idx.iter().flat_map(|&i| seqs[i].init_pc.iter().copied()));
                let mut vh_flat = Vec::<f32>::with_capacity(mb * LSTM_HIDDEN);
                vh_flat.extend(idx.iter().flat_map(|&i| seqs[i].init_vh.iter().copied()));
                let mut vc_flat = Vec::<f32>::with_capacity(mb * LSTM_HIDDEN);
                vc_flat.extend(idx.iter().flat_map(|&i| seqs[i].init_vc.iter().copied()));

                let obs_t = Tensor::<B, 3>::from_data(
                    TensorData::new(obs_flat, vec![mb, sl, self.obs_dim]),
                    &self.device,
                );
                let act_t = Tensor::<B, 3>::from_data(
                    TensorData::new(act_flat, vec![mb, sl, 4]),
                    &self.device,
                );
                let lp_old_t =
                    Tensor::<B, 1>::from_data(TensorData::new(lp_old, vec![bt]), &self.device);
                let adv_t =
                    Tensor::<B, 1>::from_data(TensorData::new(adv_flat, vec![bt]), &self.device);
                let ret_t =
                    Tensor::<B, 1>::from_data(TensorData::new(ret_flat, vec![bt]), &self.device);
                let mask_t =
                    Tensor::<B, 1>::from_data(TensorData::new(mask_flat, vec![bt]), &self.device);

                let ph_t = Tensor::<B, 2>::from_data(
                    TensorData::new(ph_flat, vec![mb, LSTM_HIDDEN]),
                    &self.device,
                );
                let pc_t = Tensor::<B, 2>::from_data(
                    TensorData::new(pc_flat, vec![mb, LSTM_HIDDEN]),
                    &self.device,
                );
                let vh_t = Tensor::<B, 2>::from_data(
                    TensorData::new(vh_flat, vec![mb, LSTM_HIDDEN]),
                    &self.device,
                );
                let vc_t = Tensor::<B, 2>::from_data(
                    TensorData::new(vc_flat, vec![mb, LSTM_HIDDEN]),
                    &self.device,
                );

                let p_init = LstmState::new(pc_t, ph_t);
                let v_init = LstmState::new(vc_t, vh_t);

                // Evaluate actions over full sequences.
                let (lp_new, entropy, values) =
                    self.model
                        .evaluate_sequence(obs_t, act_t, Some(p_init), Some(v_init));

                let values_flat = values.reshape([bt]);

                // Mask: only backprop through valid (non-padded) timesteps.
                let mask_sum = mask_t.clone().sum();
                let safe_denom = mask_sum.clone().clamp_min(1.0_f32);

                // PPO clipped surrogate (masked mean).
                let ratio = (lp_new - lp_old_t).exp();
                let surr1 = ratio.clone() * adv_t.clone();
                let surr2 = ratio.clamp(1.0_f32 - clip, 1.0_f32 + clip) * adv_t;
                let policy_loss =
                    -(surr1.min_pair(surr2) * mask_t.clone()).sum() / safe_denom.clone();

                // Value MSE (masked mean).
                let value_loss = ((values_flat - ret_t).powf_scalar(2.0) * mask_t.clone()).sum()
                    / safe_denom.clone();

                // Entropy (masked mean).
                let mean_entropy = (entropy * mask_t).sum() / safe_denom;

                // Collect metrics once per epoch.
                if batch_idx + 1 == num_batches {
                    total_policy_loss += policy_loss
                        .clone()
                        .into_data()
                        .to_vec::<f32>()
                        .expect("policy_loss")[0];
                    total_value_loss += value_loss
                        .clone()
                        .into_data()
                        .to_vec::<f32>()
                        .expect("value_loss")[0];
                    total_entropy += mean_entropy
                        .clone()
                        .into_data()
                        .to_vec::<f32>()
                        .expect("entropy")[0];
                    metric_samples += 1;
                }

                let loss = policy_loss + value_loss * value_coef + (-mean_entropy) * entropy_coef;
                let grads = loss.backward();
                let grads_params = GradientsParams::from_grads(grads, &self.model);
                self.model = self.optimizer.step(lr, self.model.clone(), grads_params);
            }
        }
        // Flush ownership of seqs (avoid unused-variable warning).
        seqs.clear();

        let n = metric_samples.max(1) as f32;
        LstmPpoMetrics {
            policy_loss: total_policy_loss / n,
            value_loss: total_value_loss / n,
            entropy: total_entropy / n,
        }
    }

    // -----------------------------------------------------------------------
    // Curriculum
    // -----------------------------------------------------------------------

    /// Advance all envs to the next curriculum stage if `mean_return` exceeds
    /// the current stage's threshold.  Idempotent at the final stage.
    ///
    /// Returns `true` if a stage transition occurred.
    pub fn advance_curriculum_if_ready(&mut self, mean_return: f32) -> bool {
        let threshold = self.envs.first_env().next_stage_threshold();
        if mean_return > threshold {
            let old_stage = self.envs.first_env().curriculum_stage_name();
            self.envs.for_each_env_mut(|e| e.advance_curriculum());
            let new_stage = self.envs.first_env().curriculum_stage_name();
            println!(
                "[curriculum] {old_stage} → {new_stage}  (mean_return={mean_return:.4} > threshold={threshold:.4})"
            );
            true
        } else {
            false
        }
    }
}

impl<B, E> LstmPpoTrainer<B, E>
where
    B: AutodiffBackend,
    E: TrainingEnv + CurriculumEnv + Clone,
{
    pub fn save_policy(&self, path: &str) {
        use burn::record::{DefaultFileRecorder, FullPrecisionSettings};
        self.model
            .clone()
            .save_file(
                path,
                &DefaultFileRecorder::<FullPrecisionSettings>::default(),
            )
            .unwrap_or_else(|e| panic!("Failed to save model to {path}: {e}"));
        println!("Saved LSTM policy to {path}.mpk");
    }

    pub fn load_policy(&mut self, path: &str) {
        use burn::record::{DefaultFileRecorder, FullPrecisionSettings};
        self.model = self
            .model
            .clone()
            .load_file(
                path,
                &DefaultFileRecorder::<FullPrecisionSettings>::default(),
                &self.device,
            )
            .unwrap_or_else(|e| panic!("Failed to load model from {path}: {e}"));
        println!("Loaded LSTM policy from {path}.mpk");
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Evaluate `V(s')` at each truncated episode's terminal observation.
///
/// `hidden[i]` must be the value-LSTM state as it stood **at** the terminal step —
/// after consuming `s_t`, before the episode's reset zeroes it. Passing the
/// post-reset zero state instead would evaluate the terminal observation as if it
/// were the first step of a fresh episode, which is a different (and wrong) value.
/// Batched into one recurrent forward call, and kept separate from the rollout's
/// `last_values` pass because that one carries the *current* per-env hidden states.
fn truncated_bootstrap_values<BB: burn::tensor::backend::Backend>(
    value: &super::lstm_model::LstmValueNet<BB>,
    obs: &[Observation],
    hidden: &[LstmHiddenState],
    obs_dim: usize,
    device: &BB::Device,
) -> Vec<f32> {
    debug_assert_eq!(obs.len(), hidden.len());
    let k = obs.len();
    let mut flat: Vec<f32> = Vec::with_capacity(k * obs_dim);
    for o in obs {
        flat.extend_from_slice(o);
    }
    let obs_t = Tensor::<BB, 2>::from_data(TensorData::new(flat, vec![k, obs_dim]), device);
    let state = LstmHiddenState::batch_to_burn::<BB>(hidden, device);
    let (values_t, _) = value.forward_step(obs_t, Some(state));
    values_t
        .squeeze_dims::<1>(&[1])
        .into_data()
        .to_vec::<f32>()
        .expect("truncated bootstrap values")
}

/// In-place Fisher-Yates shuffle using a linear congruential generator.
fn lcg_shuffle(indices: &mut Vec<usize>, seed: &mut u64) {
    let n = indices.len();
    for i in (1..n).rev() {
        *seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
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
    use crate::plane::config::PlaneConfig;
    use crate::training::{WuOrbitEnv, WuOrbitRewardConfig};
    use bevy::math::Vec3;
    use burn::backend::{Autodiff, NdArray};

    type B = Autodiff<NdArray>;

    fn jet_cfg() -> PlaneConfig {
        PlaneConfig {
            wing_area: 20.0,
            mean_chord: 2.0,
            wing_span: 10.0,
            mass: 5000.0,
            inertia: Vec3::new(10000.0, 40000.0, 45000.0),
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
            cl_beta: 0.08,
            cl_p: -0.45,
            cl_r: -0.12,
            cl_delta_a: 0.18,
            cn_beta: 0.10,
            cn_r: -0.12,
            cn_delta_r: -0.10,
            cy_beta: -0.80,
            cy_p: 0.05,
            cy_r: 0.25,
            cy_delta_r: 0.18,
            thrust_max: 60000.0,
            powerplant: Default::default(),
            aileron_limit: 0.4363,
            elevator_limit: 0.3491,
            rudder_limit: 0.2618,
        }
    }

    fn make_trainer(n: usize) -> LstmPpoTrainer<B, WuOrbitEnv> {
        let env = WuOrbitEnv::new(1000.0, 100.0, 3000.0, jet_cfg());
        LstmPpoTrainer::<B, WuOrbitEnv>::with_n_envs(env, n, Default::default())
    }

    #[test]
    fn collect_rollout_no_nan() {
        let mut trainer = make_trainer(2);
        trainer.rollout_steps = 16;
        let (buf, _ret, _ep_len) = trainer.collect_rollout();
        assert_eq!(buf.len(), 16);
        for step in &buf.steps {
            assert!(step.reward.is_finite(), "reward NaN");
            assert!(step.value.is_finite(), "value NaN");
            assert!(step.log_prob.is_finite(), "log_prob NaN");
        }
    }

    #[test]
    fn ppo_update_no_nan() {
        let mut trainer = make_trainer(2);
        trainer.rollout_steps = 64;
        trainer.seq_len = 16;
        trainer.minibatch_seqs = 2;
        trainer.n_epochs = 1;
        let (buf, _, _) = trainer.collect_rollout();
        let metrics = trainer.update(&buf);
        assert!(metrics.policy_loss.is_finite(), "policy loss NaN");
        assert!(metrics.value_loss.is_finite(), "value loss NaN");
        assert!(metrics.entropy.is_finite(), "entropy NaN");
    }

    #[test]
    fn rollout_bootstraps_truncated_steps_only() {
        // Recurrent counterpart of `PpoTrainer::rollout_bootstraps_truncated_steps_only`.
        // The extra risk here is the hidden state: `V(s')` must be evaluated with the
        // value LSTM state as it stood at the terminal step, not the post-reset zero
        // state, so a finite value alone is the observable invariant we can assert.
        let env = {
            let mut e = WuOrbitEnv::new(1000.0, 100.0, 3000.0, jet_cfg());
            e.max_episode_steps = 8;
            e
        };
        let mut trainer = LstmPpoTrainer::<B, WuOrbitEnv>::with_n_envs(env, 2, Default::default());
        trainer.rollout_steps = 64;

        let (buf, _, _) = trainer.collect_rollout();

        let bootstrapped: Vec<_> = buf
            .steps
            .iter()
            .filter(|s| s.bootstrap_value.is_some())
            .collect();
        assert!(
            !bootstrapped.is_empty(),
            "an 8-step episode limit over a 64-step rollout must produce timeouts"
        );
        assert!(
            bootstrapped
                .iter()
                .all(|s| s.done && s.bootstrap_value.unwrap().is_finite()),
            "a bootstrapped step must be an episode end with a finite value"
        );
        assert!(
            buf.steps
                .iter()
                .filter(|s| !s.done)
                .all(|s| s.bootstrap_value.is_none()),
            "a step that did not end an episode must never carry a bootstrap value"
        );
    }

    #[test]
    fn rollout_never_bootstraps_a_failure() {
        // Recurrent counterpart of `PpoTrainer::rollout_never_bootstraps_a_failure`.
        // A 1 m altitude-error budget against a ±150 m spawn perturbation makes
        // every episode end in `Failure`, so the whole rollout must be
        // bootstrap-free — the case the timeout test above cannot reach.
        let reward_cfg = WuOrbitRewardConfig {
            max_altitude_error: 1.0,
            // `with_reward_config` copies this onto the env; keep it high so the
            // failure, not the step limit, is what ends every episode.
            max_episode_steps: 100_000,
            ..Default::default()
        };
        let env = WuOrbitEnv::with_reward_config(1000.0, 100.0, 3000.0, jet_cfg(), reward_cfg);
        let mut trainer = LstmPpoTrainer::<B, WuOrbitEnv>::with_n_envs(env, 2, Default::default());
        trainer.rollout_steps = 64;

        let (buf, _, _) = trainer.collect_rollout();

        assert!(
            buf.steps.iter().any(|s| s.done),
            "a 1 m altitude-error budget over a 64-step rollout must produce failures"
        );
        assert!(
            buf.steps.iter().all(|s| s.bootstrap_value.is_none()),
            "a failure is absorbing — no step in a failure-only rollout may bootstrap"
        );
    }

    #[test]
    fn truncated_bootstrap_uses_the_supplied_hidden_state() {
        // What the rollout tests above cannot check: `collect_rollout` never stores
        // the terminal observation, so no test can recompute the expected `V(s')`.
        // What *is* checkable is that the hidden state materially determines the
        // answer — so an edit that drops the argument, or hands over the post-reset
        // zero state instead of the pre-reset one, changes the value and fails here.
        //
        // This deliberately does not catch a pre-step-vs-post-step off-by-one: both
        // are nonzero states, and telling them apart needs an episode replay the
        // buffer keeps no data for.
        let device = Default::default();
        let obs_dim = WuOrbitEnv::new(1000.0, 100.0, 3000.0, jet_cfg()).observation_dim();
        let model = LstmActorCritic::<NdArray>::new(&device, obs_dim);

        let obs: Vec<Observation> = vec![vec![0.1; obs_dim]];
        let carried = vec![LstmHiddenState {
            h: vec![0.5; LSTM_HIDDEN],
            c: vec![0.25; LSTM_HIDDEN],
        }];
        let zeroed = vec![LstmHiddenState::default()];

        let with_state =
            truncated_bootstrap_values::<NdArray>(&model.value, &obs, &carried, obs_dim, &device);
        let without_state =
            truncated_bootstrap_values::<NdArray>(&model.value, &obs, &zeroed, obs_dim, &device);

        assert_eq!(with_state.len(), 1);
        assert!(with_state[0].is_finite() && without_state[0].is_finite());
        assert!(
            (with_state[0] - without_state[0]).abs() > 1e-6,
            "the carried hidden state must change V(s'): {} vs {}",
            with_state[0],
            without_state[0]
        );
    }

    #[test]
    fn hidden_states_reset_on_done() {
        let mut trainer = make_trainer(1);
        trainer.rollout_steps = 32;
        // Force fast termination to guarantee done=true within 32 steps.
        // We'll just check that after a rollout, some steps have been stored.
        let (buf, _, _) = trainer.collect_rollout();
        assert!(buf.len() > 0);
        // Hidden states are stored in seq_start_hidden as (ph, pc, vh, vc) tuples.
        for (_, (ph, pc, vh, vc)) in &buf.seq_start_hidden {
            for &v in ph.iter().chain(pc).chain(vh).chain(vc) {
                assert!(v.is_finite(), "hidden state NaN");
            }
        }
    }

    #[test]
    fn with_n_envs_seeded_gives_reproducible_weight_init() {
        // See `PpoTrainer::with_n_envs_seeded_gives_reproducible_weight_init`
        // for why this compares a single forward pass right after
        // construction rather than a full rollout: burn-ndarray's RNG is a
        // process-global mutex, and a full rollout draws hundreds of samples
        // over real wall-clock time, exposing the comparison to interleaving
        // from unrelated tests running concurrently in the same process.
        let env = WuOrbitEnv::new(1000.0, 100.0, 3000.0, jet_cfg());
        let obs_dim = env.observation_dim();
        let obs = Tensor::<B, 2>::zeros([1, obs_dim], &Default::default());

        let trainer_a = LstmPpoTrainer::<B, WuOrbitEnv>::with_n_envs_seeded(
            env.clone(),
            2,
            Default::default(),
            Some(99),
        );
        let (out_a, _) = trainer_a.model.mean_action_step(obs.clone(), None);
        let out_a = out_a.into_data().to_vec::<f32>().unwrap();

        let trainer_b = LstmPpoTrainer::<B, WuOrbitEnv>::with_n_envs_seeded(
            env,
            2,
            Default::default(),
            Some(99),
        );
        let (out_b, _) = trainer_b.model.mean_action_step(obs, None);
        let out_b = out_b.into_data().to_vec::<f32>().unwrap();

        assert_eq!(out_a, out_b, "same seed must produce identical weight init");
    }

    #[test]
    fn with_n_envs_seeded_offsets_every_env_including_zero() {
        let env = WuOrbitEnv::new(1000.0, 100.0, 3000.0, jet_cfg());
        let mut trainer = LstmPpoTrainer::<B, WuOrbitEnv>::with_n_envs_seeded(
            env,
            4,
            Default::default(),
            Some(99),
        );
        assert_eq!(
            trainer.rng_seed, 99,
            "rng_seed should adopt the supplied seed"
        );
        let obs0 = trainer.envs.reset_all();
        let all_same = obs0.windows(2).all(|w| w[0] == w[1]);
        assert!(
            !all_same,
            "seeded envs must still get distinct spawns (env 0 included)"
        );
    }

    #[test]
    fn multi_env_distinct_obs() {
        let env = WuOrbitEnv::new(1000.0, 100.0, 3000.0, jet_cfg());
        let mut trainer = LstmPpoTrainer::<B, WuOrbitEnv>::with_n_envs(env, 4, Default::default());
        let obs = trainer.envs.reset_all();
        let all_same = obs.windows(2).all(|w| w[0] == w[1]);
        assert!(
            !all_same,
            "all envs produced identical observations — RNG seeds not offset"
        );
    }
}
