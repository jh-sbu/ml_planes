//! Recurrent rollout buffer with per-step LSTM hidden state storage.
//!
//! Extends the flat `RolloutBuffer` with:
//!   - `policy_h/c` and `value_h/c` vectors stored at each step (before inference)
//!   - `chunk_sequences()` which slices each env's trajectory into fixed-length
//!     sequences for truncated BPTT, padding the last incomplete chunk with zeros.

use crate::training::Observation;

use super::lstm_model::LSTM_HIDDEN;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

#[derive(Clone)]
pub struct LstmRolloutStep {
    pub obs: Observation,
    pub action: [f32; 4],
    pub log_prob: f32,
    pub reward: f32,
    pub value: f32,
    pub done: bool,
    /// LSTM policy h state *before* this step was taken. Shape: [LSTM_HIDDEN].
    pub policy_h: Vec<f32>,
    /// LSTM policy c state *before* this step was taken.
    pub policy_c: Vec<f32>,
    /// LSTM value  h state *before* this step was taken.
    pub value_h: Vec<f32>,
    /// LSTM value  c state *before* this step was taken.
    pub value_c: Vec<f32>,
}

/// A fixed-length sequence chunk ready for a minibatch BPTT update.
pub struct LstmSequence {
    /// Flat obs: `[seq_len * obs_dim]`.
    pub obs: Vec<f32>,
    /// Flat actions: `[seq_len * 4]`.
    pub actions: Vec<f32>,
    /// Log-probs from rollout: `[seq_len]`.
    pub log_probs: Vec<f32>,
    /// GAE advantages: `[seq_len]`.
    pub advantages: Vec<f32>,
    /// TD-lambda returns: `[seq_len]`.
    pub returns: Vec<f32>,
    /// 1.0 = valid timestep, 0.0 = zero-padded. `[seq_len]`.
    pub mask: Vec<f32>,
    /// Policy LSTM h at the start of this sequence. `[LSTM_HIDDEN]`.
    pub init_ph: Vec<f32>,
    /// Policy LSTM c at the start of this sequence.
    pub init_pc: Vec<f32>,
    /// Value  LSTM h at the start of this sequence.
    pub init_vh: Vec<f32>,
    /// Value  LSTM c at the start of this sequence.
    pub init_vc: Vec<f32>,
}

pub struct LstmRolloutBuffer {
    pub steps: Vec<LstmRolloutStep>,
    pub advantages: Vec<f32>,
    pub returns: Vec<f32>,
    pub n_envs: usize,
    pub steps_per_env: usize,
}

// ---------------------------------------------------------------------------
// Implementation
// ---------------------------------------------------------------------------

impl LstmRolloutBuffer {
    pub fn new(n_envs: usize, steps_per_env: usize) -> Self {
        let cap = n_envs * steps_per_env;
        Self {
            steps: Vec::with_capacity(cap),
            advantages: Vec::with_capacity(cap),
            returns: Vec::with_capacity(cap),
            n_envs,
            steps_per_env,
        }
    }

    pub fn push(&mut self, step: LstmRolloutStep) {
        self.steps.push(step);
    }

    pub fn len(&self) -> usize {
        self.steps.len()
    }

    pub fn is_empty(&self) -> bool {
        self.steps.is_empty()
    }

    /// Compute GAE advantages and TD-lambda returns per env.
    ///
    /// `last_values`: bootstrap value for each env (length = `n_envs`).
    pub fn compute_gae(&mut self, last_values: &[f32], gamma: f32, lam: f32) {
        let n = self.steps.len();
        self.advantages = vec![0.0; n];
        self.returns = vec![0.0; n];

        for env_i in 0..self.n_envs {
            let start = env_i * self.steps_per_env;
            let end = (start + self.steps_per_env).min(n);
            let last_val = if env_i < last_values.len() {
                last_values[env_i]
            } else {
                0.0
            };
            let mut gae = 0.0_f32;
            for t in (start..end).rev() {
                let next_value = if self.steps[t].done {
                    0.0
                } else if t + 1 < end {
                    self.steps[t + 1].value
                } else {
                    last_val
                };
                let delta = self.steps[t].reward + gamma * next_value - self.steps[t].value;
                gae = delta + gamma * lam * if self.steps[t].done { 0.0 } else { gae };
                self.advantages[t] = gae;
                self.returns[t] = gae + self.steps[t].value;
            }
        }
    }

    /// Normalise advantages to zero mean, unit variance (in-place).
    pub fn normalize_advantages(&mut self) {
        let n = self.advantages.len() as f32;
        if n == 0.0 {
            return;
        }
        let mean: f32 = self.advantages.iter().sum::<f32>() / n;
        let var: f32 = self
            .advantages
            .iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f32>()
            / n;
        let std = var.sqrt() + 1e-8;
        for a in &mut self.advantages {
            *a = (*a - mean) / std;
        }
    }

    /// Slice each env's trajectory into consecutive chunks of `seq_len`.
    ///
    /// The last chunk of each env is zero-padded and marked with `mask[t]=0.0`
    /// for padding steps. The initial hidden states come from the stored state
    /// at the first (real) step of each chunk.
    ///
    /// `obs_dim` must equal the length of each `step.obs` vector.
    pub fn chunk_sequences(&self, seq_len: usize, obs_dim: usize) -> Vec<LstmSequence> {
        assert!(seq_len > 0, "seq_len must be > 0");
        let mut sequences = Vec::new();

        let zero_obs: Vec<f32> = vec![0.0; obs_dim];
        let zero_action: Vec<f32> = vec![0.0; 4];
        let zero_h: Vec<f32> = vec![0.0; LSTM_HIDDEN];

        for env_i in 0..self.n_envs {
            let start = env_i * self.steps_per_env;
            let end = (start + self.steps_per_env).min(self.steps.len());
            if start >= end {
                continue;
            }

            let mut chunk_start = start;
            while chunk_start < end {
                let chunk_end = (chunk_start + seq_len).min(end);
                let valid_len = chunk_end - chunk_start;

                // Initial hidden state = state at the first step of this chunk.
                let first = &self.steps[chunk_start];
                let init_ph = first.policy_h.clone();
                let init_pc = first.policy_c.clone();
                let init_vh = first.value_h.clone();
                let init_vc = first.value_c.clone();

                let mut obs_flat: Vec<f32> = Vec::with_capacity(seq_len * obs_dim);
                let mut acts_flat: Vec<f32> = Vec::with_capacity(seq_len * 4);
                let mut log_probs = Vec::with_capacity(seq_len);
                let mut advantages = Vec::with_capacity(seq_len);
                let mut returns = Vec::with_capacity(seq_len);
                let mut mask = Vec::with_capacity(seq_len);

                // Valid timesteps.
                for t in chunk_start..chunk_end {
                    obs_flat.extend_from_slice(&self.steps[t].obs);
                    acts_flat.extend_from_slice(&self.steps[t].action);
                    log_probs.push(self.steps[t].log_prob);
                    advantages.push(self.advantages[t]);
                    returns.push(self.returns[t]);
                    mask.push(1.0);
                }

                // Zero-padding for the last (incomplete) chunk.
                for _ in valid_len..seq_len {
                    obs_flat.extend_from_slice(&zero_obs);
                    acts_flat.extend_from_slice(&zero_action);
                    log_probs.push(0.0);
                    advantages.push(0.0);
                    returns.push(0.0);
                    mask.push(0.0);
                }

                sequences.push(LstmSequence {
                    obs: obs_flat,
                    actions: acts_flat,
                    log_probs,
                    advantages,
                    returns,
                    mask,
                    init_ph,
                    init_pc,
                    init_vh,
                    init_vc,
                });

                chunk_start += seq_len;
            }
        }

        sequences
    }

    pub fn clear(&mut self) {
        self.steps.clear();
        self.advantages.clear();
        self.returns.clear();
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_step(obs_dim: usize, reward: f32, value: f32, done: bool) -> LstmRolloutStep {
        LstmRolloutStep {
            obs: vec![0.0; obs_dim],
            action: [0.0; 4],
            log_prob: 0.0,
            reward,
            value,
            done,
            policy_h: vec![0.0; LSTM_HIDDEN],
            policy_c: vec![0.0; LSTM_HIDDEN],
            value_h: vec![0.0; LSTM_HIDDEN],
            value_c: vec![0.0; LSTM_HIDDEN],
        }
    }

    fn make_step_with_h(
        obs_dim: usize,
        reward: f32,
        value: f32,
        done: bool,
        h_val: f32,
    ) -> LstmRolloutStep {
        LstmRolloutStep {
            obs: vec![0.0; obs_dim],
            action: [0.0; 4],
            log_prob: 0.0,
            reward,
            value,
            done,
            policy_h: vec![h_val; LSTM_HIDDEN],
            policy_c: vec![0.0; LSTM_HIDDEN],
            value_h: vec![0.0; LSTM_HIDDEN],
            value_c: vec![0.0; LSTM_HIDDEN],
        }
    }

    #[test]
    fn chunk_exact_multiple() {
        let obs_dim = 13;
        let mut buf = LstmRolloutBuffer::new(1, 64);
        for _ in 0..64 {
            buf.push(make_step(obs_dim, 1.0, 0.5, false));
        }
        buf.advantages = vec![1.0; 64];
        buf.returns = vec![1.0; 64];

        let seqs = buf.chunk_sequences(32, obs_dim);
        assert_eq!(seqs.len(), 2, "64 steps / seq_len=32 = 2 sequences");
        for seq in &seqs {
            assert_eq!(
                seq.mask.iter().sum::<f32>() as usize,
                32,
                "all 32 steps valid"
            );
        }
    }

    #[test]
    fn chunk_with_remainder() {
        let obs_dim = 13;
        let mut buf = LstmRolloutBuffer::new(1, 130);
        for _ in 0..130 {
            buf.push(make_step(obs_dim, 0.0, 0.0, false));
        }
        buf.advantages = vec![0.0; 130];
        buf.returns = vec![0.0; 130];

        let seqs = buf.chunk_sequences(64, obs_dim);
        // 130 / 64 = 2 full + 1 partial → 3 sequences
        assert_eq!(seqs.len(), 3);

        let last = &seqs[2];
        let valid: usize = last.mask.iter().map(|&m| m as usize).sum();
        assert_eq!(valid, 2, "last chunk has 2 valid steps (130 - 2*64)");
        let padded: usize = last.mask.iter().map(|&m| (1.0 - m) as usize).sum();
        assert_eq!(padded, 62, "last chunk has 62 padded steps");
    }

    #[test]
    fn chunk_init_h_matches_first_step() {
        let obs_dim = 13;
        let mut buf = LstmRolloutBuffer::new(1, 64);
        for t in 0..64 {
            buf.push(make_step_with_h(obs_dim, 0.0, 0.0, false, t as f32));
        }
        buf.advantages = vec![0.0; 64];
        buf.returns = vec![0.0; 64];

        let seqs = buf.chunk_sequences(32, obs_dim);
        assert_eq!(seqs.len(), 2);
        // First sequence init_h should be h from step 0
        assert!(
            (seqs[0].init_ph[0] - 0.0).abs() < 1e-5,
            "seq0 init_h = step 0 h"
        );
        // Second sequence init_h should be h from step 32
        assert!(
            (seqs[1].init_ph[0] - 32.0).abs() < 1e-5,
            "seq1 init_h = step 32 h"
        );
    }

    #[test]
    fn gae_done_resets_carry() {
        // Single env, 3 steps. Step 1 (t=1) is done=true.
        let mut buf = LstmRolloutBuffer::new(1, 3);
        buf.push(make_step(13, 1.0, 0.0, false)); // t=0
        buf.push(make_step(13, 1.0, 0.0, true)); // t=1, done
        buf.push(make_step(13, 1.0, 0.0, true)); // t=2, done
        buf.compute_gae(&[0.0], 0.99, 0.95);

        // t=2: done, delta = 1 + 0 - 0 = 1, gae = 1
        // t=1: done, delta = 1 + 0 - 0 = 1, gae = 1 (carry reset)
        // t=0: not done, next_value = steps[1].value = 0, delta = 1 + 0.99*0 - 0 = 1
        //   gae = 1 + 0.99*0.95*1 (carry from t=1) = 1 + 0.9405 = 1.9405
        assert!(
            (buf.advantages[2] - 1.0).abs() < 1e-5,
            "adv[2]={}",
            buf.advantages[2]
        );
        assert!(
            (buf.advantages[1] - 1.0).abs() < 1e-5,
            "adv[1]={}",
            buf.advantages[1]
        );
        assert!(
            (buf.advantages[0] - 1.9405).abs() < 1e-4,
            "adv[0]={}",
            buf.advantages[0]
        );
    }

    #[test]
    fn multi_env_gae_independent() {
        // 2 envs, 4 steps each. Different rewards.
        let mut buf = LstmRolloutBuffer::new(2, 4);
        // Env 0 (indices 0–3): all reward=1, value=0
        for _ in 0..4 {
            buf.push(make_step(13, 1.0, 0.0, false));
        }
        // Env 1 (indices 4–7): all reward=2, value=0
        for _ in 0..4 {
            buf.push(make_step(13, 2.0, 0.0, false));
        }
        buf.compute_gae(&[0.0, 0.0], 0.99, 0.95);

        // Env 0 last step advantage should be less than env 1 last step advantage.
        assert!(
            buf.advantages[3] < buf.advantages[7],
            "env0 adv < env1 adv: {} < {}",
            buf.advantages[3],
            buf.advantages[7]
        );
    }

    #[test]
    fn normalize_advantages() {
        let mut buf = LstmRolloutBuffer::new(1, 5);
        for r in [1.0_f32, 2.0, 3.0, 4.0, 5.0] {
            buf.push(make_step(13, r, 0.0, false));
        }
        buf.compute_gae(&[0.0], 0.99, 0.95);
        buf.normalize_advantages();
        let mean: f32 = buf.advantages.iter().sum::<f32>() / buf.advantages.len() as f32;
        let std: f32 = (buf
            .advantages
            .iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f32>()
            / buf.advantages.len() as f32)
            .sqrt();
        assert!(mean.abs() < 1e-5, "mean not zero: {mean}");
        assert!((std - 1.0).abs() < 0.01, "std not 1: {std}");
    }
}
