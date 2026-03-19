use bevy::math::{Quat, Vec3};
use std::collections::HashMap;

#[derive(Debug, Clone, Default)]
pub struct SpawnSpec {
    pub position: Option<Vec3>,
    pub velocity: Option<Vec3>,
    pub attitude: Option<Quat>,
    pub angular_velocity: Option<Vec3>,
}

pub type Observation = Vec<f32>;

#[derive(Debug, Clone, Default)]
pub struct StepInfo {
    pub episode_step: u32,
    pub extra: HashMap<String, f32>,
}

pub trait TrainingEnv: Send + Sync + 'static {
    /// Start a new episode. Returns the initial observation and spawn spec.
    fn reset(&mut self) -> (Observation, SpawnSpec);

    /// Advance one step. Returns (obs, reward, done, info).
    fn step(&mut self, action: &[f32]) -> (Observation, f32, bool, StepInfo);

    fn observation_dim(&self) -> usize;
    fn action_dim(&self) -> usize;
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
}
