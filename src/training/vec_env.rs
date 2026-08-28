use crate::training::{Observation, StepOutcome, TrainingEnv};

/// Seed spacing between neighbouring envs in a pool.
///
/// Each `reset()` advances an env's seed by 1, so this is how many episodes a
/// given env can run before its seed walks onto its neighbour's starting value
/// and the two begin drawing identical episodes. That is a correlation in the
/// rollout batch rather than a correctness bug, and 1000 episodes is far past
/// any shipped run length — but it is a ceiling, not an impossibility.
pub const ENV_SEED_STRIDE: u64 = 1_000;

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

    /// Seed every sub-env deterministically from one absolute base seed, spacing
    /// them [`ENV_SEED_STRIDE`] apart so each still draws a distinct episode.
    ///
    /// The batch counterpart of [`TrainingEnv::set_rng_seed`], and what a
    /// Gymnasium-style vectorized `reset(seed=...)` maps onto.
    pub fn set_rng_seed(&mut self, base: u64) {
        for (i, env) in self.envs.iter_mut().enumerate() {
            env.set_rng_seed(base.wrapping_add(i as u64 * ENV_SEED_STRIDE));
        }
    }

    /// Seed one sub-env to an absolute seed, leaving its neighbours alone.
    ///
    /// [`Self::set_rng_seed`] can only re-seed the whole pool off a single base;
    /// a caller handed a per-index seed list has no way to express that through
    /// it. Panics if `i` is out of range, like the other indexed accessors here.
    pub fn set_rng_seed_at(&mut self, i: usize, seed: u64) {
        self.envs[i].set_rng_seed(seed);
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
    /// `actions` is one flat `n * action_dim()` buffer, laid out env-major — the
    /// shape the callers already hold and the shape a contiguous `(N, action_dim)`
    /// array marshals as. Panics on a mis-sized buffer: the length is checked in
    /// release too, because zipping a short slice would otherwise step only the
    /// first few envs and return a silently truncated batch.
    pub fn step_batch(&mut self, actions: &[f32]) -> Vec<StepOutcome> {
        let width = self.envs[0].action_dim();
        assert_eq!(
            actions.len(),
            self.envs.len() * width,
            "step_batch expects {} actions ({} envs x {width}), got {}",
            self.envs.len() * width,
            self.envs.len(),
            actions.len()
        );
        self.envs
            .iter_mut()
            .zip(actions.chunks_exact(width))
            .map(|(env, action)| env.step(action))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plane::config::PlaneConfig;
    use crate::training::{LevelHoldEnv, OrbitEnv};

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
        let batch_out = vec.step_batch(&[0.0; 4]);
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
        let actions = [0.0_f32; 16];
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

    fn level_hold_pool(cfg: &PlaneConfig, n: usize) -> VecEnv<LevelHoldEnv> {
        VecEnv::new(
            (0..n)
                .map(|_| LevelHoldEnv::new(1000.0, 80.0, cfg.clone()))
                .collect(),
        )
    }

    #[test]
    fn set_rng_seed_spaces_sub_envs_by_one_stride() {
        let mut pool = level_hold_pool(&jet_cfg(), 3);
        pool.set_rng_seed(500);
        for i in 0..3 {
            assert_eq!(
                pool.envs[i].rng_seed(),
                500 + i as u64 * ENV_SEED_STRIDE,
                "env {i} should sit one stride past its predecessor"
            );
        }
    }

    /// The batch form of the absolute-seed guarantee: a base seed reproduces the
    /// whole batch, no matter what the pool did beforehand.
    #[test]
    fn same_base_seed_reproduces_the_same_batch() {
        let cfg = jet_cfg();
        let mut a = level_hold_pool(&cfg, 3);
        let mut b = level_hold_pool(&cfg, 3);
        a.reset_all();
        a.reset_all();

        a.set_rng_seed(77);
        b.set_rng_seed(77);
        assert_eq!(a.reset_all(), b.reset_all());
    }

    #[test]
    fn set_rng_seed_at_leaves_the_other_sub_envs_alone() {
        let mut pool = level_hold_pool(&jet_cfg(), 3);
        pool.set_rng_seed(500);
        pool.set_rng_seed_at(1, 9);
        assert_eq!(pool.envs[0].rng_seed(), 500);
        assert_eq!(pool.envs[1].rng_seed(), 9);
        assert_eq!(pool.envs[2].rng_seed(), 500 + 2 * ENV_SEED_STRIDE);
    }

    /// A short buffer used to zip-truncate and silently under-step the pool in
    /// release, where the old `debug_assert!` was compiled out.
    #[test]
    #[should_panic(expected = "step_batch expects 8 actions")]
    fn step_batch_rejects_a_mis_sized_action_buffer() {
        let mut pool = level_hold_pool(&jet_cfg(), 2);
        pool.reset_all();
        pool.step_batch(&[0.0; 4]);
    }

    /// The flat buffer is env-major: env `i` reads `[i * action_dim ..]`, so two
    /// envs handed different actions must diverge in the documented direction.
    #[test]
    fn step_batch_slices_the_flat_buffer_env_major() {
        let cfg = jet_cfg();
        let mut pool = level_hold_pool(&cfg, 2);
        pool.set_rng_seed(7);
        pool.reset_all();

        let mut solo_a = level_hold_pool(&cfg, 1);
        let mut solo_b = level_hold_pool(&cfg, 1);
        solo_a.set_rng_seed(7);
        solo_b.set_rng_seed(7 + ENV_SEED_STRIDE);
        solo_a.reset_all();
        solo_b.reset_all();

        let batch = pool.step_batch(&[1.0, 0.5, 0.0, 0.0, -1.0, -0.5, 0.0, 0.0]);
        assert_eq!(
            batch[0].obs,
            solo_a.step_batch(&[1.0, 0.5, 0.0, 0.0])[0].obs
        );
        assert_eq!(
            batch[1].obs,
            solo_b.step_batch(&[-1.0, -0.5, 0.0, 0.0])[0].obs
        );
    }

    /// The pool a `#[pyclass]` needs: two *different* concrete env types behind
    /// one erased handle, each still reporting its own observation width.
    #[test]
    fn vec_env_holds_a_heterogeneous_pool_of_boxed_envs() {
        let cfg = jet_cfg();
        let envs: Vec<Box<dyn TrainingEnv>> = vec![
            Box::new(LevelHoldEnv::new(1000.0, 80.0, cfg.clone())),
            Box::new(OrbitEnv::new(1000.0, 100.0, 1000.0, cfg)),
        ];
        let mut pool = VecEnv::new(envs);

        assert_eq!(pool.envs[0].observation_dim(), 13);
        assert_eq!(pool.envs[1].observation_dim(), 14);

        let obs = pool.reset_all();
        assert_eq!(obs[0].len(), 13);
        assert_eq!(obs[1].len(), 14);

        let out = pool.step_batch(&[0.0; 8]);
        assert_eq!(out[0].obs.len(), 13);
        assert_eq!(out[1].obs.len(), 14);
        assert!(out.iter().all(|o| o.reward.is_finite()));
    }
}
