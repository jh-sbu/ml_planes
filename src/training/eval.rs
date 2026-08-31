//! Evaluation aggregation shared by the Rust and Python evaluators.
//!
//! [`EvalRun`] tracks one in-flight episode per slot and owns the tail-window,
//! success, and metric aggregation rules.

use crate::training::eval_metrics::{MetricFamily, MetricRow, TaskMetrics};

/// Fraction of the step budget included in settled-tail metrics.
/// Early episodes that do not reach this window contribute no tail samples.
pub const TAIL_FRACTION: f32 = 0.2;

/// Invalid [`EvalRun`] operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvalError {
    /// The slot index is outside the configured pool.
    SlotOutOfRange(usize),
    /// All declared episodes are already finished.
    AllEpisodesFinished,
    /// The slot has no recorded steps.
    EmptyEpisode(usize),
    /// The observation is too short for the metric family's highest read index.
    ObservationTooShort { got: usize, need: usize },
    /// Not all declared episodes are finished.
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

/// Core and task-specific metrics for a completed evaluation.
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

/// Accumulates evaluation metrics across episodes.
pub struct EvalRun {
    metrics: TaskMetrics,
    family: MetricFamily,
    max_steps: u32,
    /// Last step excluded from the tail window.
    tail_start: u32,
    expected_episodes: usize,
    slots: Vec<Slot>,
    finished_episodes: usize,
    successes: usize,
    total_return: f32,
    total_steps: u64,
}

impl EvalRun {
    /// Create an evaluation for `episodes`, with up to `slots` in flight.
    /// `max_steps` must match the environment's episode limit.
    pub fn new(family: MetricFamily, episodes: usize, max_steps: u32, slots: usize) -> Self {
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

    /// Record a post-step observation and reward for `slot`.
    pub fn record(&mut self, slot: usize, obs: &[f32], reward: f32) -> Result<(), EvalError> {
        self.check(slot, obs)?;
        let s = &mut self.slots[slot];
        s.steps += 1;
        s.ep_return += reward;
        let in_tail = s.steps > self.tail_start;
        self.metrics.step(obs, in_tail);
        Ok(())
    }

    /// Finish `slot`; reaching `max_steps` counts as success.
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

    /// Produce a report once all episodes are finished.
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

    #[test]
    fn tail_window_is_the_final_fifth_of_the_budget() {
        let mut run = EvalRun::new(MetricFamily::LevelHold, 1, 10, 1);
        assert_eq!(run.tail_start, 8);
        for step in 1..=10 {
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

    #[test]
    fn a_zero_budget_does_not_underflow() {
        let run = EvalRun::new(MetricFamily::LevelHold, 1, 0, 1);
        assert_eq!(run.tail_start, 0);
    }
}
