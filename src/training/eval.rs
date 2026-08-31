//! The shared evaluation accumulator.
//!
//! Turning a rollout into an evaluation report takes more than the obs-index →
//! physical-units mapping in [`crate::training::eval_metrics`]: it also needs
//! the settled-tail window, the success criterion, and the core aggregation.
//! Those three rules used to live privately inside `src/bin/evaluate_policy.rs`
//! and were then re-implemented by hand, in Python, in
//! `bindings/python/examples/train_ppo_torch.py`. The copies drifted — most
//! visibly into two different episode-sampling schemes, which put ~15% between
//! the two evaluators' `mean_return` for one and the same policy.
//!
//! [`EvalRun`] is the single implementation both stacks now drive, so the rules
//! cannot diverge again. It deliberately does **not** own the environments or
//! the loop: the Rust binary steps one env sequentially and Python steps a set
//! of them with one batched forward, and neither shape needs to change for the
//! measurement to be shared.
//!
//! A **slot** is one in-flight episode's bookkeeping. `slots = 1` reproduces the
//! binary's sequential walk; `slots = episodes` gives each episode its own.
//! Because the accumulator owns each slot's step counter, the caller never
//! computes `in_tail` or "was this a success" for itself — which is precisely
//! the arithmetic the Python copy got to re-derive before.

use crate::training::eval_metrics::{MetricFamily, MetricRow, TaskMetrics};

/// Fraction of an episode's step budget treated as the settled "tail window"
/// for the `mean_tail_abs_*` metrics.
///
/// The window is a fraction of the *budget*, not of the episode's realised
/// length: an episode that ends early never settled, so it contributes no tail
/// samples at all.
pub const TAIL_FRACTION: f32 = 0.2;

/// Misuse of an [`EvalRun`]. Every variant is something a caller can hit from
/// Python, so none of them may be a panic — pyo3 turns a panic into an
/// uncatchable `PanicException` rather than a normal exception.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvalError {
    /// Slot index beyond the pool the run was built with.
    SlotOutOfRange(usize),
    /// Every declared episode has already been finished.
    AllEpisodesFinished,
    /// `finish` on a slot that has recorded no steps. A zero-length episode is
    /// a caller bug rather than a datapoint: it would drag `mean_length_steps`
    /// down and score as a non-success.
    EmptyEpisode(usize),
    /// The observation is too short for the metric family's highest read index.
    ObservationTooShort { got: usize, need: usize },
    /// `report` before every declared episode finished.
    Incomplete { finished: usize, expected: usize },
}

impl std::fmt::Display for EvalError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SlotOutOfRange(i) => write!(f, "slot {i} is out of range for this run"),
            Self::AllEpisodesFinished => {
                write!(f, "every declared episode has already been finished")
            }
            Self::EmptyEpisode(i) => {
                write!(f, "slot {i} finished an episode with no recorded steps")
            }
            Self::ObservationTooShort { got, need } => write!(
                f,
                "observation of length {got} is too short for this task; need at least {need}"
            ),
            Self::Incomplete { finished, expected } => write!(
                f,
                "only {finished} of {expected} episodes finished; the report is not ready"
            ),
        }
    }
}

impl std::error::Error for EvalError {}

/// A finished evaluation: the common core every task reports, plus the
/// task-specific tracking-error rows in stable order.
///
/// Values are plain floats. Precision is a presentation concern — each
/// [`MetricRow`] already carries the number of decimals it should print at, and
/// a JSON or dict consumer wants a number rather than a pre-rounded string.
#[derive(Debug, Clone, PartialEq)]
pub struct EvalReport {
    pub episodes: usize,
    pub success_rate: f32,
    pub mean_return: f32,
    pub mean_length_steps: f32,
    pub rows: Vec<MetricRow>,
}

/// One in-flight episode.
#[derive(Debug, Clone, Copy, Default)]
struct Slot {
    steps: u32,
    ep_return: f32,
}

/// Accumulates an evaluation across episodes, owning every rule that turns
/// observations into a report. See the module docs for the slot model.
pub struct EvalRun {
    metrics: TaskMetrics,
    family: MetricFamily,
    max_steps: u32,
    /// Last step index *not* in the tail window; `steps > tail_start` is in it.
    tail_start: u32,
    expected_episodes: usize,
    slots: Vec<Slot>,
    finished_episodes: usize,
    successes: usize,
    total_return: f32,
    total_steps: u64,
}

impl EvalRun {
    /// `episodes` is how many episodes the report will cover, `max_steps` the
    /// per-episode budget (which the caller must also pin on the environment,
    /// so the loop bound and the env's own `Timeout` coincide), and `slots` how
    /// many episodes are in flight at once.
    pub fn new(family: MetricFamily, episodes: usize, max_steps: u32, slots: usize) -> Self {
        // Mirrors the integer truncation the binary has always used:
        // `max_steps - (max_steps as f32 * TAIL_FRACTION) as u32`.
        let tail_start = max_steps.saturating_sub((max_steps as f32 * TAIL_FRACTION) as u32);
        Self {
            metrics: TaskMetrics::new(family),
            family,
            max_steps,
            tail_start,
            expected_episodes: episodes,
            slots: vec![Slot::default(); slots.max(1)],
            finished_episodes: 0,
            successes: 0,
            total_return: 0.0,
            total_steps: 0,
        }
    }

    /// Highest observation index this run's metric family reads, as a length.
    fn required_obs_len(&self) -> usize {
        self.family.max_obs_index() + 1
    }

    fn check(&self, slot: usize, obs: &[f32]) -> Result<(), EvalError> {
        if slot >= self.slots.len() {
            return Err(EvalError::SlotOutOfRange(slot));
        }
        if self.finished_episodes >= self.expected_episodes {
            return Err(EvalError::AllEpisodesFinished);
        }
        let need = self.required_obs_len();
        if obs.len() < need {
            return Err(EvalError::ObservationTooShort {
                got: obs.len(),
                need,
            });
        }
        Ok(())
    }

    /// Record one environment step for `slot`: the post-step observation and
    /// the reward it earned.
    ///
    /// Whether the step falls in the settled tail window is decided here, from
    /// the slot's own step counter — never by the caller.
    pub fn record(&mut self, slot: usize, obs: &[f32], reward: f32) -> Result<(), EvalError> {
        self.check(slot, obs)?;
        let s = &mut self.slots[slot];
        s.steps += 1;
        s.ep_return += reward;
        let in_tail = s.steps > self.tail_start;
        self.metrics.step(obs, in_tail);
        Ok(())
    }

    /// Close the episode in flight on `slot`, with the terminal observation.
    ///
    /// An episode that ran the whole `max_steps` budget counts as a success;
    /// one cut short by a crash or divergence does not.
    pub fn finish(&mut self, slot: usize, obs: &[f32]) -> Result<(), EvalError> {
        self.check(slot, obs)?;
        let s = self.slots[slot];
        if s.steps == 0 {
            return Err(EvalError::EmptyEpisode(slot));
        }
        self.metrics.finish_episode(obs);
        if s.steps >= self.max_steps {
            self.successes += 1;
        }
        self.total_return += s.ep_return;
        self.total_steps += s.steps as u64;
        self.finished_episodes += 1;
        self.slots[slot] = Slot::default();
        Ok(())
    }

    /// How many episodes have been closed so far.
    pub fn finished_episodes(&self) -> usize {
        self.finished_episodes
    }

    /// Whether every declared episode has finished.
    pub fn is_complete(&self) -> bool {
        self.finished_episodes >= self.expected_episodes
    }

    /// Produce the report, refusing if episodes are still outstanding — a
    /// partial report would silently divide by the declared episode count and
    /// understate every mean.
    pub fn try_report(self) -> Result<EvalReport, EvalError> {
        if !self.is_complete() {
            return Err(EvalError::Incomplete {
                finished: self.finished_episodes,
                expected: self.expected_episodes,
            });
        }
        Ok(self.into_report())
    }

    /// Produce the report.
    ///
    /// # Panics
    /// If episodes are still outstanding. Callers that cannot guarantee
    /// completion should use [`Self::try_report`].
    pub fn report(self) -> EvalReport {
        self.try_report().expect("evaluation is complete")
    }

    fn into_report(self) -> EvalReport {
        let episodes = self.expected_episodes.max(1) as f32;
        EvalReport {
            episodes: self.expected_episodes,
            success_rate: self.successes as f32 / episodes,
            mean_return: self.total_return / episodes,
            mean_length_steps: self.total_steps as f32 / episodes,
            rows: self.metrics.into_rows(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn obs13(alt: f32) -> Vec<f32> {
        let mut o = vec![0.0_f32; 13];
        o[0] = alt;
        o
    }

    /// The tail boundary is exclusive on the low side: with a budget of 10 the
    /// window is steps 9 and 10, not 8 through 10. Off by one here quietly
    /// changes what every `mean_tail_*` number in the project means.
    #[test]
    fn tail_window_is_the_final_fifth_of_the_budget() {
        let mut run = EvalRun::new(MetricFamily::LevelHold, 1, 10, 1);
        assert_eq!(run.tail_start, 8);
        for step in 1..=10 {
            // Only tail steps carry a non-zero error, so the tail mean is 1.0
            // exactly if and only if the boundary is right.
            let alt = if step > 8 { 1.0 } else { 0.0 };
            run.record(0, &obs13(alt), 0.0).unwrap();
        }
        run.finish(0, &obs13(0.0)).unwrap();
        let report = run.report();
        let tail = report
            .rows
            .iter()
            .find(|r| r.key == "mean_tail_abs_altitude_m")
            .unwrap()
            .value;
        assert_eq!(tail, 200.0);
    }

    /// A zero budget would make the tail window meaningless; saturating_sub
    /// keeps it from underflowing rather than panicking in release.
    #[test]
    fn a_zero_budget_does_not_underflow() {
        let run = EvalRun::new(MetricFamily::LevelHold, 1, 0, 1);
        assert_eq!(run.tail_start, 0);
    }
}
