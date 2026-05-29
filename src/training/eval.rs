use crate::training::{Observation, TrainingEnv};

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct EvaluationSummary {
    pub episodes: usize,
    pub mean_return: f32,
    pub mean_length: f32,
    pub success_rate: f32,
}

pub fn evaluate_policy<E, F>(
    mut env: E,
    episodes: usize,
    max_steps: u32,
    mut policy: F,
) -> EvaluationSummary
where
    E: TrainingEnv,
    F: FnMut(&Observation) -> Vec<f32>,
{
    if episodes == 0 || max_steps == 0 {
        return EvaluationSummary::default();
    }

    let mut total_return = 0.0_f32;
    let mut total_length = 0_u64;
    let mut successes = 0_usize;

    for _ in 0..episodes {
        let (mut obs, _) = env.reset();
        let mut ep_return = 0.0_f32;
        let mut ep_len = 0_u32;
        let mut done = false;

        while !done && ep_len < max_steps {
            let action = policy(&obs);
            let (next_obs, reward, next_done, _info) = env.step(&action);
            obs = next_obs;
            ep_return += reward;
            ep_len += 1;
            done = next_done;
        }

        if ep_len >= max_steps {
            successes += 1;
        }
        total_return += ep_return;
        total_length += ep_len as u64;
    }

    EvaluationSummary {
        episodes,
        mean_return: total_return / episodes as f32,
        mean_length: total_length as f32 / episodes as f32,
        success_rate: successes as f32 / episodes as f32,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::training::{SpawnSpec, StepInfo};

    struct FixedLengthEnv {
        step: u32,
        episode_steps: u32,
        reward_per_step: f32,
    }

    impl TrainingEnv for FixedLengthEnv {
        fn reset(&mut self) -> (Observation, SpawnSpec) {
            self.step = 0;
            (vec![0.0], SpawnSpec::default())
        }

        fn step(&mut self, _action: &[f32]) -> (Observation, f32, bool, StepInfo) {
            self.step += 1;
            let done = self.step >= self.episode_steps;
            (
                vec![self.step as f32],
                self.reward_per_step,
                done,
                StepInfo {
                    episode_step: self.step,
                    ..Default::default()
                },
            )
        }

        fn observation_dim(&self) -> usize {
            1
        }

        fn action_dim(&self) -> usize {
            1
        }
    }

    #[test]
    fn evaluate_policy_aggregates_episode_metrics() {
        let env = FixedLengthEnv {
            step: 0,
            episode_steps: 3,
            reward_per_step: 2.0,
        };

        let summary = evaluate_policy(env, 4, 3, |_obs| vec![0.0]);

        assert_eq!(summary.episodes, 4);
        assert!((summary.mean_return - 6.0).abs() < 1e-6);
        assert!((summary.mean_length - 3.0).abs() < 1e-6);
        assert!((summary.success_rate - 1.0).abs() < 1e-6);
    }
}
