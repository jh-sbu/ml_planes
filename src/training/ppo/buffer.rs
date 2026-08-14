//! Rollout buffer and Generalized Advantage Estimation (GAE).

use crate::training::Observation;

/// One step of environment interaction collected during rollout.
#[derive(Clone)]
pub struct RolloutStep {
    pub obs: Observation,
    pub action: [f32; 4], // tanh-squashed, in [-1,1]
    pub log_prob: f32,
    pub reward: f32,
    pub value: f32,
    pub done: bool,
    /// Set only on a **truncated** step (the episode hit its time limit while the
    /// plane was still flying), holding `V(s_{t+1})` evaluated at the terminal
    /// observation. `None` means either "episode still running" or "real terminal
    /// state", both of which bootstrap 0.
    ///
    /// Without this, a timeout trains the critic toward a return of exactly `r_T`
    /// when the correct target is `r_T + gamma * V(s_{t+1})` — and since the
    /// observation carries no time feature, that mislabels a state indistinguishable
    /// from any other in-flight state.
    pub bootstrap_value: Option<f32>,
}

/// Buffer of rollout steps with pre-computed GAE advantages and TD-lambda returns.
pub struct RolloutBuffer {
    pub steps: Vec<RolloutStep>,
    pub advantages: Vec<f32>,
    pub returns: Vec<f32>,
}

impl RolloutBuffer {
    pub fn new() -> Self {
        Self::with_capacity(0)
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            steps: Vec::with_capacity(capacity),
            advantages: Vec::with_capacity(capacity),
            returns: Vec::with_capacity(capacity),
        }
    }

    pub fn push(&mut self, step: RolloutStep) {
        self.steps.push(step);
    }

    /// Move all of `other`'s steps/advantages/returns onto the end of `self`,
    /// preserving order. Uses `Vec::append` (an O(1) pointer move of `other`'s
    /// backing storage), so merging per-env rollout buffers doesn't clone every
    /// `RolloutStep` the way a `for step in &other.steps { self.push(step.clone()) }`
    /// loop would.
    pub fn append(&mut self, mut other: RolloutBuffer) {
        self.steps.append(&mut other.steps);
        self.advantages.append(&mut other.advantages);
        self.returns.append(&mut other.returns);
    }

    /// Compute GAE advantages and TD-lambda returns.
    ///
    /// Must be called after the rollout is complete.  `last_value` is the
    /// bootstrap value estimate at the step after the final step.
    pub fn compute_gae(&mut self, last_value: f32, gamma: f32, gae_lambda: f32) {
        let n = self.steps.len();
        // Resize in place rather than `self.advantages = vec![0.0; n]`, which
        // would discard whatever capacity `with_capacity` already reserved and
        // allocate fresh every call.
        self.advantages.clear();
        self.advantages.resize(n, 0.0);
        self.returns.clear();
        self.returns.resize(n, 0.0);

        let mut gae = 0.0_f32;
        for t in (0..n).rev() {
            let next_value = if let Some(v) = self.steps[t].bootstrap_value {
                // Truncated: the episode was cut short by its time limit, not by a
                // failure, so the continuation value is real and must be bootstrapped.
                v
            } else if self.steps[t].done {
                0.0
            } else if t + 1 < n {
                self.steps[t + 1].value
            } else {
                last_value
            };
            let delta = self.steps[t].reward + gamma * next_value - self.steps[t].value;
            // done resets the carry so episodes don't bleed into each other. This stays
            // keyed on `done`, not on the bootstrap: a truncation is still a real episode
            // boundary, only its value target differs.
            gae = delta + gamma * gae_lambda * if self.steps[t].done { 0.0 } else { gae };
            self.advantages[t] = gae;
            self.returns[t] = gae + self.steps[t].value;
        }
    }

    /// Normalize advantages to zero mean, unit variance (in-place).
    pub fn normalize_advantages(&mut self) {
        let n = self.advantages.len() as f32;
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
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_step(reward: f32, value: f32, done: bool) -> RolloutStep {
        RolloutStep {
            obs: vec![0.0; 10],
            action: [0.0; 4],
            log_prob: 0.0,
            reward,
            value,
            done,
            bootstrap_value: None,
        }
    }

    /// A step that ended its episode by hitting the time limit, carrying the
    /// value estimate of the state the episode was cut short at.
    fn make_truncated_step(reward: f32, value: f32, bootstrap: f32) -> RolloutStep {
        RolloutStep {
            bootstrap_value: Some(bootstrap),
            ..make_step(reward, value, true)
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
        assert!(
            (buf.advantages[2] - 0.5).abs() < 1e-5,
            "adv[2]={}",
            buf.advantages[2]
        );
        assert!(
            (buf.advantages[1] - 1.5).abs() < 1e-5,
            "adv[1]={}",
            buf.advantages[1]
        );
        assert!(
            (buf.advantages[0] - 2.5).abs() < 1e-5,
            "adv[0]={}",
            buf.advantages[0]
        );

        // returns = advantages + values
        assert!(
            (buf.returns[0] - 3.0).abs() < 1e-5,
            "ret[0]={}",
            buf.returns[0]
        );
    }

    #[test]
    fn gae_done_resets_carry() {
        // Two 1-step episodes back to back.
        // Episode 1: reward=1, value=0, done=true → adv = 1-0 = 1
        // Episode 2: reward=2, value=0, done=true → adv = 2-0 = 2
        // done=true must prevent episode 2's carry from bleeding into episode 1.
        let mut buf = RolloutBuffer::new();
        buf.push(make_step(2.0, 0.0, true)); // t=0 (latter episode, processed first in rev)
        buf.push(make_step(1.0, 0.0, true)); // t=1 ... wait, indices matter

        // Actually let me reason about order: t=0 then t=1
        // Processing in reverse: t=1 first, t=0 second
        // t=1: done=true, delta = 1 - 0 = 1, gae = 1
        // t=0: done=true, next_value=0, delta = 2 - 0 = 2, gae_carry = 0 (because done), gae = 2
        buf.compute_gae(0.0, 0.99, 0.95);

        assert!(
            (buf.advantages[1] - 1.0).abs() < 1e-5,
            "adv[1]={}",
            buf.advantages[1]
        );
        assert!(
            (buf.advantages[0] - 2.0).abs() < 1e-5,
            "adv[0]={}",
            buf.advantages[0]
        );
    }

    #[test]
    fn gae_bootstraps_on_truncation() {
        // A single step whose episode ended only because it hit the time limit.
        // The plane was still flying, so the value target must include the
        // continuation `gamma * V(s')` rather than treating the state as absorbing.
        let gamma = 0.99_f32;
        let mut buf = RolloutBuffer::new();
        buf.push(make_truncated_step(1.0, 0.5, 4.0));

        buf.compute_gae(0.0, gamma, 0.95);

        let expected = 1.0 + gamma * 4.0 - 0.5;
        assert!(
            (buf.advantages[0] - expected).abs() < 1e-5,
            "truncated step must bootstrap V(s'): adv={} expected={expected}",
            buf.advantages[0]
        );
    }

    #[test]
    fn gae_zero_bootstrap_on_failure() {
        // Same shape, but the episode ended because the plane failed. That state
        // really is absorbing, so no continuation value may leak in.
        let mut buf = RolloutBuffer::new();
        buf.push(make_step(1.0, 0.5, true));

        buf.compute_gae(0.0, 0.99, 0.95);

        let expected = 1.0 - 0.5;
        assert!(
            (buf.advantages[0] - expected).abs() < 1e-5,
            "failure step must bootstrap 0: adv={} expected={expected}",
            buf.advantages[0]
        );
    }

    #[test]
    fn truncation_still_resets_the_gae_carry() {
        // Bootstrapping across a truncation must not also let the *next* episode's
        // advantage carry bleed backwards — the episode boundary is real either way.
        let gamma = 0.99_f32;
        let lambda = 0.95_f32;
        let mut buf = RolloutBuffer::new();
        buf.push(make_truncated_step(1.0, 0.5, 4.0)); // t=0, end of episode 1
        buf.push(make_step(100.0, 0.0, false)); // t=1, start of episode 2

        buf.compute_gae(0.0, gamma, lambda);

        // t=0 sees only its own delta, not episode 2's large reward.
        let expected = 1.0 + gamma * 4.0 - 0.5;
        assert!(
            (buf.advantages[0] - expected).abs() < 1e-5,
            "carry bled across a truncation: adv={} expected={expected}",
            buf.advantages[0]
        );
    }

    #[test]
    fn append_moves_steps_in_order_without_dropping_data() {
        let mut a = RolloutBuffer::new();
        a.push(make_step(1.0, 0.0, false));
        a.push(make_step(2.0, 0.0, false));
        a.advantages = vec![0.1, 0.2];
        a.returns = vec![0.1, 0.2];

        let mut b = RolloutBuffer::new();
        b.push(make_step(3.0, 0.0, false));
        b.push(make_step(4.0, 0.0, false));
        b.advantages = vec![0.3, 0.4];
        b.returns = vec![0.3, 0.4];

        a.append(b);

        assert_eq!(a.steps.len(), 4);
        let rewards: Vec<f32> = a.steps.iter().map(|s| s.reward).collect();
        assert_eq!(rewards, vec![1.0, 2.0, 3.0, 4.0]);
        assert_eq!(a.advantages, vec![0.1, 0.2, 0.3, 0.4]);
        assert_eq!(a.returns, vec![0.1, 0.2, 0.3, 0.4]);
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
