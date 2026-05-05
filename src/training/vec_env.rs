use crate::training::{Observation, TrainingEnv};

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

    /// Step all N environments and return (obs[N], reward[N], done[N]).
    pub fn step_batch(&mut self, actions: &[[f32; 4]]) -> (Vec<Observation>, Vec<f32>, Vec<bool>) {
        debug_assert_eq!(actions.len(), self.envs.len());
        let mut obs_out = Vec::with_capacity(self.envs.len());
        let mut rew_out = Vec::with_capacity(self.envs.len());
        let mut done_out = Vec::with_capacity(self.envs.len());
        for (env, action) in self.envs.iter_mut().zip(actions.iter()) {
            let (obs, rew, done, _) = env.step(action);
            obs_out.push(obs);
            rew_out.push(rew);
            done_out.push(done);
        }
        (obs_out, rew_out, done_out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plane::config::PlaneConfig;
    use crate::training::LevelHoldEnv;
    use bevy::math::Vec3;

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
            cl_beta: -0.08,
            cl_p: -0.45,
            cl_r: 0.12,
            cl_delta_a: 0.18,
            cn_beta: 0.10,
            cn_r: -0.12,
            cn_delta_r: -0.10,
            thrust_max: 60000.0,
            aileron_limit: 0.4363,
            elevator_limit: 0.3491,
            rudder_limit: 0.2618,
        }
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
        let (o_s, r_s, d_s, _) = single.step(&action);
        let (obs_out, rew_out, done_out) = vec.step_batch(&[[0.0; 4]]);
        assert_eq!(obs_out[0], o_s);
        assert!((rew_out[0] - r_s).abs() < 1e-6);
        assert_eq!(done_out[0], d_s);
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
        let (o, r, d) = vec.step_batch(&actions);
        assert_eq!(o.len(), 4);
        assert_eq!(r.len(), 4);
        assert_eq!(d.len(), 4);
        assert!(r.iter().all(|v| v.is_finite()));
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
        assert_eq!(obs_after_reset.len(), 10);
        assert!(obs_after_reset.iter().all(|v| v.is_finite()));
    }
}
