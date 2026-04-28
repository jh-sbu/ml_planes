//! Rollout buffer and Generalized Advantage Estimation (GAE).

use crate::training::Observation;

/// One step of environment interaction collected during rollout.
pub struct RolloutStep {
    pub obs:      Observation,
    pub action:   [f32; 4],  // tanh-squashed, in [-1,1]
    pub log_prob: f32,
    pub reward:   f32,
    pub value:    f32,
    pub done:     bool,
}

/// Buffer of rollout steps with pre-computed GAE advantages and TD-lambda returns.
pub struct RolloutBuffer {
    pub steps:      Vec<RolloutStep>,
    pub advantages: Vec<f32>,
    pub returns:    Vec<f32>,
}

impl RolloutBuffer {
    pub fn new() -> Self {
        Self::with_capacity(0)
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            steps:      Vec::with_capacity(capacity),
            advantages: Vec::with_capacity(capacity),
            returns:    Vec::with_capacity(capacity),
        }
    }

    pub fn push(&mut self, step: RolloutStep) {
        self.steps.push(step);
    }

    /// Compute GAE advantages and TD-lambda returns.
    ///
    /// Must be called after the rollout is complete.  `last_value` is the
    /// bootstrap value estimate at the step after the final step.
    pub fn compute_gae(&mut self, last_value: f32, gamma: f32, gae_lambda: f32) {
        let n = self.steps.len();
        self.advantages = vec![0.0; n];
        self.returns    = vec![0.0; n];

        let mut gae = 0.0_f32;
        for t in (0..n).rev() {
            let next_value = if self.steps[t].done {
                0.0
            } else if t + 1 < n {
                self.steps[t + 1].value
            } else {
                last_value
            };
            let delta = self.steps[t].reward + gamma * next_value - self.steps[t].value;
            // done resets the carry so episodes don't bleed into each other.
            gae = delta + gamma * gae_lambda * if self.steps[t].done { 0.0 } else { gae };
            self.advantages[t] = gae;
            self.returns[t]    = gae + self.steps[t].value;
        }
    }

    /// Normalize advantages to zero mean, unit variance (in-place).
    pub fn normalize_advantages(&mut self) {
        let n = self.advantages.len() as f32;
        let mean: f32 = self.advantages.iter().sum::<f32>() / n;
        let var:  f32 = self.advantages.iter().map(|x| (x - mean).powi(2)).sum::<f32>() / n;
        let std = var.sqrt() + 1e-8;
        for a in &mut self.advantages {
            *a = (*a - mean) / std;
        }
    }

    pub fn len(&self) -> usize {
        self.steps.len()
    }

    pub fn clear(&mut self) {
        self.steps.clear();
        self.advantages.clear();
        self.returns.clear();
    }
}

impl Default for RolloutBuffer {
    fn default() -> Self { Self::new() }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_step(reward: f32, value: f32, done: bool) -> RolloutStep {
        RolloutStep {
            obs:      [0.0; 10],
            action:   [0.0; 4],
            log_prob: 0.0,
            reward,
            value,
            done,
        }
    }

    #[test]
    fn gae_single_episode() {
        // 3-step episode: rewards [1, 1, 1], values [0.5, 0.5, 0.5], last_value = 0.0
        // gamma=1.0, lambda=1.0 → advantages should equal MC returns - values
        let mut buf = RolloutBuffer::new();
        buf.push(make_step(1.0, 0.5, false));
        buf.push(make_step(1.0, 0.5, false));
        buf.push(make_step(1.0, 0.5, true));

        buf.compute_gae(0.0, 1.0, 1.0);

        // With done=true on last step: last delta = 1 + 0 - 0.5 = 0.5, gae = 0.5
        // Step 1: delta = 1 + 0.5 - 0.5 = 1.0, gae = 1.0 + 0.5 = 1.5
        // Step 0: delta = 1 + 0.5 - 0.5 = 1.0, gae = 1.0 + 1.5 = 2.5
        assert!((buf.advantages[2] - 0.5).abs() < 1e-5, "adv[2]={}", buf.advantages[2]);
        assert!((buf.advantages[1] - 1.5).abs() < 1e-5, "adv[1]={}", buf.advantages[1]);
        assert!((buf.advantages[0] - 2.5).abs() < 1e-5, "adv[0]={}", buf.advantages[0]);

        // returns = advantages + values
        assert!((buf.returns[0] - 3.0).abs() < 1e-5, "ret[0]={}", buf.returns[0]);
    }

    #[test]
    fn gae_done_resets_carry() {
        // Two 1-step episodes back to back.
        // Episode 1: reward=1, value=0, done=true → adv = 1-0 = 1
        // Episode 2: reward=2, value=0, done=true → adv = 2-0 = 2
        // done=true must prevent episode 2's carry from bleeding into episode 1.
        let mut buf = RolloutBuffer::new();
        buf.push(make_step(2.0, 0.0, true));  // t=0 (latter episode, processed first in rev)
        buf.push(make_step(1.0, 0.0, true));  // t=1 ... wait, indices matter

        // Actually let me reason about order: t=0 then t=1
        // Processing in reverse: t=1 first, t=0 second
        // t=1: done=true, delta = 1 - 0 = 1, gae = 1
        // t=0: done=true, next_value=0, delta = 2 - 0 = 2, gae_carry = 0 (because done), gae = 2
        buf.compute_gae(0.0, 0.99, 0.95);

        assert!((buf.advantages[1] - 1.0).abs() < 1e-5, "adv[1]={}", buf.advantages[1]);
        assert!((buf.advantages[0] - 2.0).abs() < 1e-5, "adv[0]={}", buf.advantages[0]);
    }

    #[test]
    fn normalize_zero_mean_unit_std() {
        let mut buf = RolloutBuffer::new();
        for r in [1.0_f32, 2.0, 3.0, 4.0, 5.0] {
            buf.push(make_step(r, 0.0, false));
        }
        buf.compute_gae(0.0, 0.99, 0.95);
        buf.normalize_advantages();
        let mean: f32 = buf.advantages.iter().sum::<f32>() / buf.advantages.len() as f32;
        let std:  f32 = (buf.advantages.iter().map(|x| (x - mean).powi(2)).sum::<f32>()
            / buf.advantages.len() as f32).sqrt();
        assert!(mean.abs() < 1e-5, "mean not zero: {mean}");
        assert!((std - 1.0).abs() < 0.01, "std not 1: {std}");
    }
}
