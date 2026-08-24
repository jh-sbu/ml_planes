//! Behavior cloning (BC): collect `(observation, action)` demonstrations from the
//! PID expert controllers so an `ActorCritic` policy can be supervised-pretrained
//! before PPO fine-tuning.
//!
//! No `burn` dependency lives here — collection is pure simulation. The supervised
//! training step that consumes a [`BcDataset`] lives in `PpoTrainer::pretrain_bc`.

use crate::controllers::FlightController;
use crate::plane::{ControllerContext, FlightState, PlaneId};
use crate::training::flight_env::inputs_to_direct_action;
use crate::training::{Observation, TrainingEnv};

/// A training environment that can supply an expert (PID) controller for the
/// current episode, used to generate behavior-cloning demonstrations.
pub trait DemonstrationEnv: TrainingEnv {
    /// Snapshot of the env's current flight state (fed to the expert controller).
    fn current_state(&self) -> FlightState;
    /// Fixed integration timestep [s].
    fn dt(&self) -> f32;
    /// Build a freshly-seeded expert controller for the env's current state and
    /// target geometry. Call after `reset()` (and again on episode boundaries) so
    /// the PID integrators re-seed for the new spawn.
    fn make_expert(&self) -> Box<dyn FlightController>;
}

/// Supervised dataset of normalized `(observation, action)` pairs.
///
/// `actions` are already in the `[-1, 1]` direct-action space the network outputs
/// (throttle pre-converted via `throttle * 2 - 1`), so they can be regressed
/// directly against `ActorCritic::mean_action`.
#[derive(Clone, Default)]
pub struct BcDataset {
    pub obs: Vec<Observation>,
    pub actions: Vec<[f32; 4]>,
}

impl BcDataset {
    pub fn len(&self) -> usize {
        self.obs.len()
    }

    pub fn is_empty(&self) -> bool {
        self.obs.is_empty()
    }
}

/// Roll out `env` under its PID expert for `n_steps`, recording the observation the
/// network would see and the expert's action at each step.
///
/// The environment is stepped with the expert's own action so the demonstrations
/// cover the expert's state distribution (on-policy expert data). Episodes that
/// terminate are reset and the expert rebuilt.
pub fn collect_demonstrations<E: DemonstrationEnv>(env: &mut E, n_steps: usize) -> BcDataset {
    let ctx = ControllerContext::empty_for(PlaneId::TEST);
    let dt = env.dt();
    let mut data = BcDataset {
        obs: Vec::with_capacity(n_steps),
        actions: Vec::with_capacity(n_steps),
    };

    let (mut obs, _) = env.reset();
    let mut expert = env.make_expert();

    for _ in 0..n_steps {
        let state = env.current_state();
        let inputs = expert.update(&state, &ctx, dt);
        let action = inputs_to_direct_action(&inputs);

        data.obs.push(obs.clone());
        data.actions.push(action);

        let outcome = env.step(&action);
        let (done, next_obs) = (outcome.done(), outcome.obs);
        if done {
            let (reset_obs, _) = env.reset();
            obs = reset_obs;
            expert = env.make_expert();
        } else {
            obs = next_obs;
        }
    }

    data
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plane::PlaneConfig;
    use crate::training::{LevelHoldEnv, OrbitEnv};

    fn jet_cfg() -> PlaneConfig {
        crate::plane::config::fixture_jet_config()
    }

    #[test]
    fn expert_targets_track_resampled_episode_targets() {
        use crate::controllers::targets::ControllerTargets;
        use crate::training::LevelHoldRewardConfig;

        let mut env = LevelHoldEnv::with_target_ranges(
            500.0..=5000.0,
            90.0..=140.0,
            jet_cfg(),
            LevelHoldRewardConfig::default(),
        );
        for _ in 0..10 {
            env.reset();
            let expert = env.make_expert();
            match expert.targets() {
                ControllerTargets::LevelHold { altitude, airspeed } => {
                    assert!((altitude - env.target_altitude).abs() < 1e-6);
                    assert!((airspeed - env.target_airspeed).abs() < 1e-6);
                }
                other => panic!("expected LevelHold targets, got {other:?}"),
            }
        }
    }

    #[test]
    fn level_hold_env_exposes_demonstration_hooks() {
        let mut env = LevelHoldEnv::new(1000.0, 100.0, jet_cfg());
        env.reset();
        // The PID expert is discretized at the env's dt, so demonstrations are now
        // generated at the same rate the gains were tuned at (`tune` drives observe_state).
        assert_eq!(env.dt(), crate::plane::PHYSICS_DT);
        let state = env.current_state();
        assert!(state.altitude.is_finite());
        // Expert must produce finite control inputs for the current state.
        let ctx = ControllerContext::empty_for(PlaneId::TEST);
        let inputs = env.make_expert().update(&state, &ctx, env.dt());
        assert!(inputs.elevator.is_finite() && inputs.throttle.is_finite());
    }

    #[test]
    fn orbit_env_exposes_demonstration_hooks() {
        let mut env = OrbitEnv::new(1000.0, 100.0, 1000.0, jet_cfg());
        env.reset();
        let state = env.current_state();
        let ctx = ControllerContext::empty_for(PlaneId::TEST);
        let inputs = env.make_expert().update(&state, &ctx, env.dt());
        assert!(inputs.aileron.is_finite() && inputs.elevator.is_finite());
    }

    #[test]
    fn collect_level_hold_has_correct_length_and_range() {
        let mut env = LevelHoldEnv::new(1000.0, 100.0, jet_cfg());
        let data = collect_demonstrations(&mut env, 300);
        assert_eq!(data.len(), 300);
        assert_eq!(data.obs.len(), data.actions.len());
        for a in &data.actions {
            for v in a {
                assert!(
                    v.is_finite() && v.abs() <= 1.0 + 1e-5,
                    "action out of range: {v}"
                );
            }
            assert_eq!(env.observation_dim(), 13);
        }
        for o in &data.obs {
            assert_eq!(o.len(), 13);
        }
    }

    #[test]
    fn level_hold_expert_reduces_altitude_error() {
        // Spawn a fixed 100 m above target; the PID expert should drive the
        // (normalized) altitude error toward zero over a single episode.
        let mut env = LevelHoldEnv::new(1000.0, 100.0, jet_cfg());
        env.alt_spawn_offset_range = 100.0..=100.0;
        let data = collect_demonstrations(&mut env, 1200);

        // obs[0] is alt_err / 200.
        let first: f32 = data.obs[..100].iter().map(|o| o[0].abs()).sum::<f32>() / 100.0;
        let last: f32 = data.obs[data.len() - 100..]
            .iter()
            .map(|o| o[0].abs())
            .sum::<f32>()
            / 100.0;
        assert!(
            last < first * 0.5,
            "expert did not reduce altitude error: first={first}, last={last}"
        );
    }
}
