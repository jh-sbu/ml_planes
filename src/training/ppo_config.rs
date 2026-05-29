//! Runtime-loadable PPO training-loop hyperparameters.
//!
//! These are the "category 1" scalars that affect only rollout collection and
//! the gradient update — they are never persisted in a trained `.mpk` and the
//! deterministic inference path (`ActorCritic::mean_action`) ignores them.
//! Loaded from a RON file via `--ppo-config <path>` to `train_ppo`; a missing
//! file falls back to these compiled defaults (which mirror the literals in
//! `PpoTrainer::with_n_envs`). See `PpoTrainer::apply_hyperparams`.

use serde::{Deserialize, Serialize};

/// PPO training-loop scalars. Field values and types mirror the corresponding
/// `pub` fields on `PpoTrainer` set in `with_n_envs`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PpoHyperparams {
    pub gamma: f32,
    pub gae_lambda: f32,
    pub clip_epsilon: f32,
    pub value_coef: f32,
    pub entropy_coef: f32,
    pub lr: f64,
    pub rollout_steps: usize,
    pub n_epochs: usize,
    pub minibatch: usize,
}

impl PpoHyperparams {
    /// CSV-log header fields, matching the reward-config `log_fields` convention.
    pub fn log_fields(&self) -> Vec<(&'static str, String)> {
        vec![
            ("gamma", self.gamma.to_string()),
            ("gae_lambda", self.gae_lambda.to_string()),
            ("clip_epsilon", self.clip_epsilon.to_string()),
            ("value_coef", self.value_coef.to_string()),
            ("entropy_coef", self.entropy_coef.to_string()),
            ("lr", self.lr.to_string()),
            ("rollout_steps", self.rollout_steps.to_string()),
            ("n_epochs", self.n_epochs.to_string()),
            ("minibatch", self.minibatch.to_string()),
        ]
    }
}

impl Default for PpoHyperparams {
    fn default() -> Self {
        Self {
            gamma: 0.99,
            gae_lambda: 0.95,
            clip_epsilon: 0.2,
            value_coef: 0.5,
            entropy_coef: 0.01,
            lr: 3e-4,
            rollout_steps: 2048,
            n_epochs: 4,
            minibatch: 64,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ron_round_trip() {
        let src = r#"(
            gamma: 0.97,
            gae_lambda: 0.9,
            clip_epsilon: 0.15,
            value_coef: 0.4,
            entropy_coef: 0.02,
            lr: 0.0005,
            rollout_steps: 1024,
            n_epochs: 8,
            minibatch: 128,
        )"#;
        let cfg: PpoHyperparams = ron::de::from_str(src).expect("parse failed");
        assert_eq!(cfg.gamma, 0.97);
        assert_eq!(cfg.gae_lambda, 0.9);
        assert_eq!(cfg.clip_epsilon, 0.15);
        assert_eq!(cfg.value_coef, 0.4);
        assert_eq!(cfg.entropy_coef, 0.02);
        assert_eq!(cfg.lr, 0.0005);
        assert_eq!(cfg.rollout_steps, 1024);
        assert_eq!(cfg.n_epochs, 8);
        assert_eq!(cfg.minibatch, 128);
    }

    #[test]
    fn log_fields_cover_all_params() {
        let cfg = PpoHyperparams::default();
        assert_eq!(cfg.log_fields().len(), 9);
    }
}
