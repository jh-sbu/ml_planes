use crate::training::{Observation, StepOutcome, TrainingEnv};

/// Wraps a pool of independent environments for batched rollout collection.
///
/// Each call to `step_batch` steps all N environments in sequence (no parallelism
/// is needed here since the env is pure-Rust Euler integration on CPU).
pub struct VecEnv<E: TrainingEnv> {
    envs: Vec<E>,
}

impl<E: TrainingEnv> VecEnv<E> {
    pub fn new(envs: Vec<E>) -> Self {
        assert!(!envs.is_empty(), "VecEnv requires at least one environment");
        Self { envs }
    }

    pub fn n(&self) -> usize {
        self.envs.len()
    }

    pub fn reset_all(&mut self) -> Vec<Observation> {
        self.envs.iter_mut().map(|e| e.reset().0).collect()
    }

    pub fn reset_at(&mut self, i: usize) -> Observation {
        self.envs[i].reset().0
    }

    /// Apply a closure to every environment mutably.
    pub fn for_each_env_mut(&mut self, mut f: impl FnMut(&mut E)) {
        for env in &mut self.envs {
            f(env);
        }
    }

    /// Inspect the first environment (read-only).
    pub fn first_env(&self) -> &E {
        &self.envs[0]
    }

    /// Step all N environments, returning one [`StepOutcome`] per env.
    ///
    /// Note that this does **not** auto-reset a finished env the way Gymnasium's
    /// vector envs do — the caller resets explicitly via [`Self::reset_at`]. That is
    /// what lets a caller read the terminal observation off the outcome and bootstrap
    /// a truncated episode before resetting.
    pub fn step_batch(&mut self, actions: &[[f32; 4]]) -> Vec<StepOutcome> {
        debug_assert_eq!(actions.len(), self.envs.len());
        self.envs
            .iter_mut()
            .zip(actions.iter())
            .map(|(env, action)| env.step(action))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plane::config::PlaneConfig;
    use crate::training::LevelHoldEnv;

    fn jet_cfg() -> PlaneConfig {
        crate::plane::config::fixture_jet_config()
    }

    #[test]
    fn vec_env_step_matches_single() {
        let cfg = jet_cfg();
        let mut single = LevelHoldEnv::new(1000.0, 80.0, cfg.clone());
        let mut vec = VecEnv::new(vec![LevelHoldEnv::new(1000.0, 80.0, cfg)]);

        let (obs_s, _) = single.reset();
        let obs_v = vec.reset_all();
        assert_eq!(obs_v.len(), 1);
        assert_eq!(obs_s, obs_v[0]);

        let action = [0.0_f32; 4];
        let single_out = single.step(&action);
        let batch_out = vec.step_batch(&[[0.0; 4]]);
        assert_eq!(batch_out[0].obs, single_out.obs);
        assert!((batch_out[0].reward - single_out.reward).abs() < 1e-6);
        assert_eq!(batch_out[0].end, single_out.end);
    }

    #[test]
    fn vec_env_n_envs_step() {
        let cfg = jet_cfg();
        let envs: Vec<_> = (0..4)
            .map(|_| LevelHoldEnv::new(1000.0, 80.0, cfg.clone()))
            .collect();
        let mut vec = VecEnv::new(envs);
        assert_eq!(vec.n(), 4);
        let obs = vec.reset_all();
        assert_eq!(obs.len(), 4);
        let actions = [[0.0_f32; 4]; 4];
        let out = vec.step_batch(&actions);
        assert_eq!(out.len(), 4);
        assert!(out.iter().all(|o| o.reward.is_finite()));
        assert!(out.iter().all(|o| o.obs.len() == 13));
    }

    #[test]
    fn reset_at_produces_new_obs() {
        let cfg = jet_cfg();
        let envs: Vec<_> = (0..2)
            .map(|_| LevelHoldEnv::new(1000.0, 80.0, cfg.clone()))
            .collect();
        let mut vec = VecEnv::new(envs);
        vec.reset_all();
        let obs_after_reset = vec.reset_at(0);
        assert_eq!(obs_after_reset.len(), 13);
        assert!(obs_after_reset.iter().all(|v| v.is_finite()));
    }
}
