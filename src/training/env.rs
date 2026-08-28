use bevy::math::{Quat, Vec3};

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

    /// The env's current deterministic-variation seed.
    fn rng_seed(&self) -> u64;

    /// Reset deterministic episode variation to an absolute seed.
    ///
    /// This is the primitive a Gymnasium-style `reset(seed=...)` needs: the same
    /// `seed` must reproduce the same episode no matter how many resets came
    /// before it. [`Self::offset_rng_seed`] cannot express that on its own,
    /// because every `reset()` advances the seed an offset would be relative to.
    ///
    /// Note that `reset()` advances the seed *before* drawing, so `set_rng_seed(s)`
    /// followed by `reset()` runs the episode drawn from `s + 1`. The guarantee is
    /// reproducibility — the same `s` always yields the same episode sequence — not
    /// that the RNG stream literally begins at `s`.
    fn set_rng_seed(&mut self, seed: u64);

    /// Shift deterministic episode variation for cloned vectorized envs.
    fn offset_rng_seed(&mut self, offset: u64) {
        self.set_rng_seed(self.rng_seed().wrapping_add(offset));
    }
}

/// Type erasure for callers that must hold one env type over several concrete
/// ones — the FFI bindings, where a `#[pyclass]` cannot be generic. The trait is
/// object-safe (no generics, no `Self` returns) and `Send + Sync + 'static` are
/// supertraits, so `Box<dyn TrainingEnv>` carries them and drops straight into
/// [`VecEnv`](crate::training::VecEnv) with no extra bounds.
///
/// `offset_rng_seed` is forwarded explicitly even though it has a default body.
/// Without the arm, the default runs *on the box* and routes back through the
/// box's own `rng_seed`/`set_rng_seed`, silently discarding whatever override the
/// inner env supplies. No shipped env overrides it; the point is that the bug
/// would be invisible on the day one does.
///
/// Deliberately no `clone_box`/`dyn-clone` shim: the Rust trainers require
/// `E: TrainingEnv + Clone` (`ppo::PpoTrainer`) and stay monomorphized over
/// concrete env types. Erasure exists for the bindings, and a vtable call in the
/// PPO rollout loop buys nothing.
impl TrainingEnv for Box<dyn TrainingEnv> {
    fn reset(&mut self) -> (Observation, SpawnSpec) {
        (**self).reset()
    }

    fn step(&mut self, action: &[f32]) -> StepOutcome {
        (**self).step(action)
    }

    fn observation_dim(&self) -> usize {
        (**self).observation_dim()
    }

    fn action_dim(&self) -> usize {
        (**self).action_dim()
    }

    fn rng_seed(&self) -> u64 {
        (**self).rng_seed()
    }

    fn set_rng_seed(&mut self, seed: u64) {
        (**self).set_rng_seed(seed);
    }

    fn offset_rng_seed(&mut self, offset: u64) {
        (**self).offset_rng_seed(offset);
    }
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

    /// Records what it was asked to do so the box's forwarding can be observed
    /// from outside. `obs_dim` is per-instance so a heterogeneous pool can be
    /// told apart.
    struct FakeEnv {
        obs_dim: usize,
        resets: u32,
        steps: u32,
        last_action: Vec<f32>,
        seed: u64,
        /// Set by the `offset_rng_seed`-overriding variant only.
        offset_calls: u32,
        override_offset: bool,
    }

    impl FakeEnv {
        fn new(obs_dim: usize) -> Self {
            Self {
                obs_dim,
                resets: 0,
                steps: 0,
                last_action: Vec::new(),
                seed: 0,
                offset_calls: 0,
                override_offset: false,
            }
        }

        fn overriding_offset() -> Self {
            Self {
                override_offset: true,
                ..Self::new(1)
            }
        }
    }

    impl TrainingEnv for FakeEnv {
        fn reset(&mut self) -> (Observation, SpawnSpec) {
            self.resets += 1;
            (vec![self.resets as f32; self.obs_dim], SpawnSpec::default())
        }

        fn step(&mut self, action: &[f32]) -> StepOutcome {
            self.steps += 1;
            self.last_action = action.to_vec();
            StepOutcome {
                obs: vec![self.steps as f32; self.obs_dim],
                reward: self.steps as f32,
                end: None,
                info: StepInfo {
                    episode_step: self.steps,
                },
            }
        }

        fn observation_dim(&self) -> usize {
            self.obs_dim
        }

        fn action_dim(&self) -> usize {
            2
        }

        fn rng_seed(&self) -> u64 {
            self.seed
        }

        fn set_rng_seed(&mut self, seed: u64) {
            self.seed = seed;
        }

        fn offset_rng_seed(&mut self, offset: u64) {
            if self.override_offset {
                // Deliberately *not* seed + offset, so a box that runs the default
                // body instead of this one is distinguishable.
                self.offset_calls += 1;
                self.seed = offset * 10;
            } else {
                self.set_rng_seed(self.rng_seed().wrapping_add(offset));
            }
        }
    }

    #[test]
    fn boxed_env_forwards_every_method() {
        let mut boxed: Box<dyn TrainingEnv> = Box::new(FakeEnv::new(3));

        assert_eq!(boxed.observation_dim(), 3);
        assert_eq!(boxed.action_dim(), 2);

        let (obs, spawn) = boxed.reset();
        assert_eq!(obs, vec![1.0, 1.0, 1.0]);
        assert!(spawn.position.is_none());

        let outcome = boxed.step(&[0.25, -0.5]);
        assert_eq!(outcome.obs, vec![1.0, 1.0, 1.0]);
        assert_eq!(outcome.reward, 1.0);
        assert_eq!(outcome.info.episode_step, 1);

        boxed.set_rng_seed(31);
        assert_eq!(boxed.rng_seed(), 31);
    }

    /// The reason `offset_rng_seed` is forwarded explicitly: without the arm the
    /// default body runs on the box and the inner override never fires.
    #[test]
    fn boxed_env_honors_an_inner_offset_rng_seed_override() {
        let mut boxed: Box<dyn TrainingEnv> = Box::new(FakeEnv::overriding_offset());
        boxed.set_rng_seed(7);
        boxed.offset_rng_seed(4);
        assert_eq!(
            boxed.rng_seed(),
            40,
            "the box ran the default body instead of the inner override"
        );
    }
}
