use bevy::math::{Quat, Vec3};
use std::collections::HashMap;

#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "net", derive(serde::Serialize, serde::Deserialize))]
pub struct SpawnSpec {
    pub position: Option<Vec3>,
    pub velocity: Option<Vec3>,
    pub attitude: Option<Quat>,
    pub angular_velocity: Option<Vec3>,
    /// Fraction of the powerplant's capacity the plane spawns with, in [0, 1].
    /// `None` ⇒ full tanks. Lets ML resets / refueling scenarios spawn partially
    /// fuelled planes.
    pub fuel_fraction: Option<f32>,
}

pub type Observation = Vec<f32>;

#[derive(Debug, Clone, Default)]
pub struct StepInfo {
    pub episode_step: u32,
    pub extra: HashMap<String, f32>,
}

/// Why an episode ended.
///
/// The distinction is not cosmetic: it decides the PPO value target. A `Failure`
/// state really is absorbing, so its return is exactly the reward collected there.
/// A `Timeout` cut a still-flying episode short at an arbitrary wall clock, so its
/// return must bootstrap `gamma * V(s')` — otherwise the critic is taught that a
/// perfectly ordinary in-flight state is worth nothing, and since the observation
/// carries no time feature, that state is indistinguishable from any other.
/// See `ppo::RolloutBuffer::compute_gae`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerminationReason {
    /// The plane crashed, diverged, or otherwise violated a termination threshold.
    Failure,
    /// The episode hit `max_episode_steps` while still flying.
    Timeout,
}

/// The result of one environment step.
#[derive(Debug, Clone)]
pub struct StepOutcome {
    pub obs: Observation,
    pub reward: f32,
    /// `None` while the episode is still running.
    pub end: Option<TerminationReason>,
    pub info: StepInfo,
}

impl StepOutcome {
    /// The episode ended, for either reason. Use this wherever the caller only
    /// needs to know that it is time to `reset()`.
    pub fn done(&self) -> bool {
        self.end.is_some()
    }

    /// The episode ended by hitting its time limit, so `V(s')` should be bootstrapped.
    pub fn truncated(&self) -> bool {
        matches!(self.end, Some(TerminationReason::Timeout))
    }
}

pub trait TrainingEnv: Send + Sync + 'static {
    /// Start a new episode. Returns the initial observation and spawn spec.
    fn reset(&mut self) -> (Observation, SpawnSpec);

    /// Advance one step.
    ///
    /// The returned `obs` is the observation *at the state reached by this step*,
    /// including when the episode ended — environments never auto-reset, so a
    /// caller that wants to bootstrap a truncated episode can read the terminal
    /// observation straight off the outcome before calling `reset()`.
    fn step(&mut self, action: &[f32]) -> StepOutcome;

    fn observation_dim(&self) -> usize;
    fn action_dim(&self) -> usize;

    /// Shift deterministic episode variation for cloned vectorized envs.
    fn offset_rng_seed(&mut self, _offset: u64) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spawn_spec_defaults_are_none() {
        let s = SpawnSpec::default();
        assert!(s.position.is_none());
        assert!(s.velocity.is_none());
        assert!(s.attitude.is_none());
        assert!(s.angular_velocity.is_none());
    }

    #[test]
    fn step_info_defaults() {
        let i = StepInfo::default();
        assert_eq!(i.episode_step, 0);
        assert!(i.extra.is_empty());
    }

    fn outcome(end: Option<TerminationReason>) -> StepOutcome {
        StepOutcome {
            obs: vec![0.0],
            reward: 0.0,
            end,
            info: StepInfo::default(),
        }
    }

    #[test]
    fn running_episode_is_neither_done_nor_truncated() {
        let o = outcome(None);
        assert!(!o.done());
        assert!(!o.truncated());
    }

    #[test]
    fn failure_is_done_but_not_truncated() {
        // The distinction the whole change turns on: a failure must not bootstrap.
        let o = outcome(Some(TerminationReason::Failure));
        assert!(o.done());
        assert!(!o.truncated());
    }

    #[test]
    fn timeout_is_both_done_and_truncated() {
        let o = outcome(Some(TerminationReason::Timeout));
        assert!(o.done());
        assert!(o.truncated());
    }
}
