//! Task-specific evaluation metric extraction.
//!
//! The deterministic policy evaluator (`src/bin/evaluate_policy.rs`) reports a
//! stable common core (`success_rate`, `mean_return`, `mean_length_steps`) for
//! every task, plus a handful of task-specific tracking-error metrics whose
//! names and obs-index mapping depend on the observation layout of the task's
//! environment.
//!
//! This module isolates that obs-index → physical-units mapping so it can be
//! unit-tested headlessly (the binary itself is `--features training` only and
//! not directly testable). The two observation layouts handled:
//!
//! - **Orbit family** (`orbit`, `residual_orbit`, `lstm_orbit`; 13-dim
//!   `build_orbit_observation`): `[radial/500, heading/0.5, bank_ff, alt/200,
//!   speed/50, ...]`.
//! - **Level-hold family** (13-dim `level_hold_observation`): `[alt_err/200,
//!   speed_err/50, alpha/0.5, pitch_rate, roll/0.5, roll_rate, beta/0.5, ...,
//!   fuel_fraction, density_ratio, airspeed/100]`. Only indices 0/1/4/6 are read.
//! - **Heading-hold family** (16-dim `heading_hold_observation`): the
//!   level-hold layout above (indices 0/1/4/6 reused) plus
//!   `[sin(heading_err)/0.5, cos(heading_err), turn_rate/0.2]` at 13/14/15.
//!   The heading error is read back via [`Source::CircularError`], **not**
//!   `Source::Scaled` — `|sin(e)|` folds above π/2, so a naive
//!   `|obs[13]| * scale` read would misreport a ~170° error as ~8°.

use crate::training::heading_hold_env::HEADING_ERROR_OBS_SCALE as HEADING_ERROR_SIN_SCALE;

/// Which observation layout a checkpoint's task uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MetricFamily {
    /// 13-dim orbit observation (orbit / residual_orbit / lstm_orbit).
    Orbit,
    /// 13-dim level-hold observation.
    LevelHold,
    /// 16-dim heading-hold observation (level-hold prefix + sin/cos heading error + turn rate).
    HeadingHold,
}

/// Decimal precision for metrics in physical distance/speed units (metres,
/// m/s). Millimetre / mm-per-second resolution is ample.
const DECIMALS_LINEAR: usize = 3;
/// Decimal precision for radian-valued metrics. Heading/roll/beta errors are
/// small in magnitude, so finer resolution avoids masking changes.
const DECIMALS_RADIAN: usize = 6;

/// Where a metric's raw value comes from in the observation vector.
enum Source {
    /// `|obs[obs_index]| * scale` — the layout every pre-existing family uses.
    Scaled { obs_index: usize, scale: f32 },
    /// Recovers the unfolded signed circular error from a `(sin/sin_scale,
    /// cos)` pair via `atan2(obs[sin_index] * sin_scale, obs[cos_index])`,
    /// then takes the absolute value. Required for any angle encoded as
    /// sin/cos (see the module doc) — `Scaled` would fold it above π/2.
    CircularError {
        sin_index: usize,
        sin_scale: f32,
        cos_index: usize,
    },
}

impl Source {
    fn value(&self, obs: &[f32]) -> f32 {
        match *self {
            Source::Scaled { obs_index, scale } => obs[obs_index].abs() * scale,
            Source::CircularError {
                sin_index,
                sin_scale,
                cos_index,
            } => (obs[sin_index] * sin_scale).atan2(obs[cos_index]).abs(),
        }
    }
}

/// One extracted metric: output key, value source, and the decimal precision
/// to print it at.
struct MetricSpec {
    key: &'static str,
    source: Source,
    decimals: usize,
}

impl MetricFamily {
    /// Per-step (mean-over-all-steps) metrics, in stable output order.
    fn step_specs(self) -> &'static [MetricSpec] {
        match self {
            MetricFamily::Orbit => &[
                MetricSpec {
                    key: "mean_abs_radial_m",
                    source: Source::Scaled {
                        obs_index: 0,
                        scale: 500.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_abs_heading_rad",
                    source: Source::Scaled {
                        obs_index: 1,
                        scale: 0.5,
                    },
                    decimals: DECIMALS_RADIAN,
                },
                MetricSpec {
                    key: "mean_abs_altitude_m",
                    source: Source::Scaled {
                        obs_index: 3,
                        scale: 200.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_abs_speed_mps",
                    source: Source::Scaled {
                        obs_index: 4,
                        scale: 50.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
            ],
            MetricFamily::LevelHold => &[
                MetricSpec {
                    key: "mean_abs_altitude_m",
                    source: Source::Scaled {
                        obs_index: 0,
                        scale: 200.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_abs_speed_mps",
                    source: Source::Scaled {
                        obs_index: 1,
                        scale: 50.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_abs_roll_rad",
                    source: Source::Scaled {
                        obs_index: 4,
                        scale: 0.5,
                    },
                    decimals: DECIMALS_RADIAN,
                },
                MetricSpec {
                    key: "mean_abs_beta_rad",
                    source: Source::Scaled {
                        obs_index: 6,
                        scale: 0.5,
                    },
                    decimals: DECIMALS_RADIAN,
                },
            ],
            MetricFamily::HeadingHold => &[
                MetricSpec {
                    key: "mean_abs_heading_rad",
                    source: Source::CircularError {
                        sin_index: 13,
                        sin_scale: HEADING_ERROR_SIN_SCALE,
                        cos_index: 14,
                    },
                    decimals: DECIMALS_RADIAN,
                },
                MetricSpec {
                    key: "mean_abs_altitude_m",
                    source: Source::Scaled {
                        obs_index: 0,
                        scale: 200.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_abs_speed_mps",
                    source: Source::Scaled {
                        obs_index: 1,
                        scale: 50.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_abs_roll_rad",
                    source: Source::Scaled {
                        obs_index: 4,
                        scale: 0.5,
                    },
                    decimals: DECIMALS_RADIAN,
                },
                MetricSpec {
                    key: "mean_abs_beta_rad",
                    source: Source::Scaled {
                        obs_index: 6,
                        scale: 0.5,
                    },
                    decimals: DECIMALS_RADIAN,
                },
            ],
        }
    }

    /// Tail-window (mean-over-final-steps-of-each-episode) metrics, in stable
    /// output order. Same obs indices/scales as `step_specs`, but the caller
    /// only feeds steps from the final portion of each episode — this is the
    /// "has it settled into a stable hold" signal, distinct from the
    /// whole-episode mean (dominated by the spawn-offset transient) and the
    /// single-frame `final_specs` snapshot (noisy at one instant).
    fn tail_specs(self) -> &'static [MetricSpec] {
        match self {
            MetricFamily::Orbit => &[
                MetricSpec {
                    key: "mean_tail_abs_radial_m",
                    source: Source::Scaled {
                        obs_index: 0,
                        scale: 500.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_tail_abs_heading_rad",
                    source: Source::Scaled {
                        obs_index: 1,
                        scale: 0.5,
                    },
                    decimals: DECIMALS_RADIAN,
                },
                MetricSpec {
                    key: "mean_tail_abs_altitude_m",
                    source: Source::Scaled {
                        obs_index: 3,
                        scale: 200.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_tail_abs_speed_mps",
                    source: Source::Scaled {
                        obs_index: 4,
                        scale: 50.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
            ],
            MetricFamily::LevelHold => &[
                MetricSpec {
                    key: "mean_tail_abs_altitude_m",
                    source: Source::Scaled {
                        obs_index: 0,
                        scale: 200.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_tail_abs_speed_mps",
                    source: Source::Scaled {
                        obs_index: 1,
                        scale: 50.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_tail_abs_roll_rad",
                    source: Source::Scaled {
                        obs_index: 4,
                        scale: 0.5,
                    },
                    decimals: DECIMALS_RADIAN,
                },
                MetricSpec {
                    key: "mean_tail_abs_beta_rad",
                    source: Source::Scaled {
                        obs_index: 6,
                        scale: 0.5,
                    },
                    decimals: DECIMALS_RADIAN,
                },
            ],
            MetricFamily::HeadingHold => &[
                MetricSpec {
                    key: "mean_tail_abs_heading_rad",
                    source: Source::CircularError {
                        sin_index: 13,
                        sin_scale: HEADING_ERROR_SIN_SCALE,
                        cos_index: 14,
                    },
                    decimals: DECIMALS_RADIAN,
                },
                MetricSpec {
                    key: "mean_tail_abs_altitude_m",
                    source: Source::Scaled {
                        obs_index: 0,
                        scale: 200.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_tail_abs_speed_mps",
                    source: Source::Scaled {
                        obs_index: 1,
                        scale: 50.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_tail_abs_roll_rad",
                    source: Source::Scaled {
                        obs_index: 4,
                        scale: 0.5,
                    },
                    decimals: DECIMALS_RADIAN,
                },
                MetricSpec {
                    key: "mean_tail_abs_beta_rad",
                    source: Source::Scaled {
                        obs_index: 6,
                        scale: 0.5,
                    },
                    decimals: DECIMALS_RADIAN,
                },
            ],
        }
    }

    /// Final-state (mean-over-episodes) metrics, in stable output order.
    fn final_specs(self) -> &'static [MetricSpec] {
        match self {
            MetricFamily::Orbit => &[
                MetricSpec {
                    key: "mean_final_abs_radial_m",
                    source: Source::Scaled {
                        obs_index: 0,
                        scale: 500.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_final_abs_altitude_m",
                    source: Source::Scaled {
                        obs_index: 3,
                        scale: 200.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
            ],
            MetricFamily::LevelHold => &[MetricSpec {
                key: "mean_final_abs_altitude_m",
                source: Source::Scaled {
                    obs_index: 0,
                    scale: 200.0,
                },
                decimals: DECIMALS_LINEAR,
            }],
            MetricFamily::HeadingHold => &[
                MetricSpec {
                    key: "mean_final_abs_heading_rad",
                    source: Source::CircularError {
                        sin_index: 13,
                        sin_scale: HEADING_ERROR_SIN_SCALE,
                        cos_index: 14,
                    },
                    decimals: DECIMALS_RADIAN,
                },
                MetricSpec {
                    key: "mean_final_abs_altitude_m",
                    source: Source::Scaled {
                        obs_index: 0,
                        scale: 200.0,
                    },
                    decimals: DECIMALS_LINEAR,
                },
            ],
        }
    }
}

/// One finished metric ready for output: stable key, mean value, and the
/// decimal precision to print it at.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MetricRow {
    pub key: &'static str,
    pub value: f32,
    pub decimals: usize,
}

/// Accumulates task-specific tracking-error metrics across an evaluation run.
///
/// Usage: call [`step`](Self::step) once per environment step with the
/// post-step observation, [`finish_episode`](Self::finish_episode) once per
/// episode with the terminal observation, then [`into_rows`](Self::into_rows)
/// to produce `(key, value)` pairs in stable order.
pub struct TaskMetrics {
    family: MetricFamily,
    step_sums: Vec<f32>,
    samples: u64,
    tail_sums: Vec<f32>,
    tail_samples: u64,
    final_sums: Vec<f32>,
    episodes: u64,
}

impl TaskMetrics {
    pub fn new(family: MetricFamily) -> Self {
        Self {
            family,
            step_sums: vec![0.0; family.step_specs().len()],
            samples: 0,
            tail_sums: vec![0.0; family.tail_specs().len()],
            tail_samples: 0,
            final_sums: vec![0.0; family.final_specs().len()],
            episodes: 0,
        }
    }

    /// Accumulate per-step tracking errors from a post-step observation.
    /// `in_tail` marks steps the caller considers part of the settled,
    /// steady-state window at the end of the episode (e.g. its final 20%);
    /// those also feed the `tail_specs` accumulators.
    pub fn step(&mut self, obs: &[f32], in_tail: bool) {
        for (sum, spec) in self.step_sums.iter_mut().zip(self.family.step_specs()) {
            *sum += spec.source.value(obs);
        }
        self.samples += 1;
        if in_tail {
            for (sum, spec) in self.tail_sums.iter_mut().zip(self.family.tail_specs()) {
                *sum += spec.source.value(obs);
            }
            self.tail_samples += 1;
        }
    }

    /// Accumulate final-state tracking errors from the terminal observation.
    pub fn finish_episode(&mut self, final_obs: &[f32]) {
        for (sum, spec) in self.final_sums.iter_mut().zip(self.family.final_specs()) {
            *sum += spec.source.value(final_obs);
        }
        self.episodes += 1;
    }

    /// Produce the metric rows in stable output order: all per-step means,
    /// then all tail-window means, then all final-state means. Each row
    /// carries its print precision.
    pub fn into_rows(self) -> Vec<MetricRow> {
        let samples = self.samples.max(1) as f32;
        let tail_samples = self.tail_samples.max(1) as f32;
        let episodes = self.episodes.max(1) as f32;
        let mut rows =
            Vec::with_capacity(self.step_sums.len() + self.tail_sums.len() + self.final_sums.len());
        for (sum, spec) in self.step_sums.iter().zip(self.family.step_specs()) {
            rows.push(MetricRow {
                key: spec.key,
                value: sum / samples,
                decimals: spec.decimals,
            });
        }
        for (sum, spec) in self.tail_sums.iter().zip(self.family.tail_specs()) {
            rows.push(MetricRow {
                key: spec.key,
                value: sum / tail_samples,
                decimals: spec.decimals,
            });
        }
        for (sum, spec) in self.final_sums.iter().zip(self.family.final_specs()) {
            rows.push(MetricRow {
                key: spec.key,
                value: sum / episodes,
                decimals: spec.decimals,
            });
        }
        rows
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orbit_rows_match_legacy_keys_and_scales() {
        let mut m = TaskMetrics::new(MetricFamily::Orbit);
        // Two steps; second is the terminal obs used for final metrics too.
        // obs[0]=radial/500, obs[1]=heading/0.5, obs[3]=alt/200, obs[4]=speed/50.
        m.step(
            &[
                0.2, -0.4, 0.0, 0.5, -0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ],
            false,
        );
        m.step(
            &[
                0.4, 0.2, 0.0, -0.25, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            ],
            true,
        );
        m.finish_episode(&[
            0.4, 0.2, 0.0, -0.25, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]);

        let rows = m.into_rows();
        let keys: Vec<&str> = rows.iter().map(|r| r.key).collect();
        assert_eq!(
            keys,
            vec![
                "mean_abs_radial_m",
                "mean_abs_heading_rad",
                "mean_abs_altitude_m",
                "mean_abs_speed_mps",
                "mean_tail_abs_radial_m",
                "mean_tail_abs_heading_rad",
                "mean_tail_abs_altitude_m",
                "mean_tail_abs_speed_mps",
                "mean_final_abs_radial_m",
                "mean_final_abs_altitude_m",
            ]
        );
        let row = |k: &str| *rows.iter().find(|r| r.key == k).unwrap();
        let get = |k: &str| row(k).value;
        // radial: (0.2*500 + 0.4*500)/2 = (100 + 200)/2 = 150
        assert!((get("mean_abs_radial_m") - 150.0).abs() < 1e-3);
        // heading: (0.4*0.5 + 0.2*0.5)/2 = (0.2 + 0.1)/2 = 0.15
        assert!((get("mean_abs_heading_rad") - 0.15).abs() < 1e-6);
        // altitude: (0.5*200 + 0.25*200)/2 = (100 + 50)/2 = 75
        assert!((get("mean_abs_altitude_m") - 75.0).abs() < 1e-3);
        // speed: (0.6*50 + 0.2*50)/2 = (30 + 10)/2 = 20
        assert!((get("mean_abs_speed_mps") - 20.0).abs() < 1e-3);
        // tail: only the second step (in_tail=true) contributes.
        assert!((get("mean_tail_abs_radial_m") - 200.0).abs() < 1e-3); // 0.4*500
        assert!((get("mean_tail_abs_heading_rad") - 0.1).abs() < 1e-6); // 0.2*0.5
        assert!((get("mean_tail_abs_altitude_m") - 50.0).abs() < 1e-3); // 0.25*200
        assert!((get("mean_tail_abs_speed_mps") - 10.0).abs() < 1e-3); // 0.2*50
                                                                       // final radial over 1 episode: 0.4*500 = 200
        assert!((get("mean_final_abs_radial_m") - 200.0).abs() < 1e-3);
        // final altitude over 1 episode: 0.25*200 = 50
        assert!((get("mean_final_abs_altitude_m") - 50.0).abs() < 1e-3);
        // Radians use 6 decimals; linear metrics use 3.
        assert_eq!(row("mean_abs_heading_rad").decimals, 6);
        assert_eq!(row("mean_abs_radial_m").decimals, 3);
        assert_eq!(row("mean_abs_altitude_m").decimals, 3);
        assert_eq!(row("mean_abs_speed_mps").decimals, 3);
        assert_eq!(row("mean_tail_abs_heading_rad").decimals, 6);
        assert_eq!(row("mean_tail_abs_radial_m").decimals, 3);
        assert_eq!(row("mean_tail_abs_altitude_m").decimals, 3);
        assert_eq!(row("mean_tail_abs_speed_mps").decimals, 3);
        assert_eq!(row("mean_final_abs_radial_m").decimals, 3);
        assert_eq!(row("mean_final_abs_altitude_m").decimals, 3);
    }

    #[test]
    fn level_hold_rows_have_altitude_speed_attitude_no_radial_heading() {
        let mut m = TaskMetrics::new(MetricFamily::LevelHold);
        // obs[0]=alt/200, obs[1]=speed/50, obs[4]=roll/0.5, obs[6]=beta/0.5.
        m.step(&[0.5, -0.4, 0.0, 0.0, 0.2, 0.0, -0.6, 0.0, 0.0, 0.0], true);
        m.finish_episode(&[0.5, -0.4, 0.0, 0.0, 0.2, 0.0, -0.6, 0.0, 0.0, 0.0]);

        let rows = m.into_rows();
        let keys: Vec<&str> = rows.iter().map(|r| r.key).collect();
        assert_eq!(
            keys,
            vec![
                "mean_abs_altitude_m",
                "mean_abs_speed_mps",
                "mean_abs_roll_rad",
                "mean_abs_beta_rad",
                "mean_tail_abs_altitude_m",
                "mean_tail_abs_speed_mps",
                "mean_tail_abs_roll_rad",
                "mean_tail_abs_beta_rad",
                "mean_final_abs_altitude_m",
            ]
        );
        assert!(!keys.contains(&"mean_abs_radial_m"));
        assert!(!keys.contains(&"mean_abs_heading_rad"));
        let row = |k: &str| *rows.iter().find(|r| r.key == k).unwrap();
        let get = |k: &str| row(k).value;
        assert!((get("mean_abs_altitude_m") - 100.0).abs() < 1e-3); // 0.5*200
        assert!((get("mean_abs_speed_mps") - 20.0).abs() < 1e-3); // 0.4*50
        assert!((get("mean_abs_roll_rad") - 0.1).abs() < 1e-6); // 0.2*0.5
        assert!((get("mean_abs_beta_rad") - 0.3).abs() < 1e-6); // 0.6*0.5
                                                                // Single step, in_tail=true, so tail == step here.
        assert!((get("mean_tail_abs_altitude_m") - 100.0).abs() < 1e-3);
        assert!((get("mean_tail_abs_speed_mps") - 20.0).abs() < 1e-3);
        assert!((get("mean_tail_abs_roll_rad") - 0.1).abs() < 1e-6);
        assert!((get("mean_tail_abs_beta_rad") - 0.3).abs() < 1e-6);
        assert!((get("mean_final_abs_altitude_m") - 100.0).abs() < 1e-3);
        // Radian-valued attitude metrics keep 6 decimals; linear metres at 3.
        assert_eq!(row("mean_abs_roll_rad").decimals, 6);
        assert_eq!(row("mean_abs_beta_rad").decimals, 6);
        assert_eq!(row("mean_abs_altitude_m").decimals, 3);
        assert_eq!(row("mean_abs_speed_mps").decimals, 3);
        assert_eq!(row("mean_tail_abs_roll_rad").decimals, 6);
        assert_eq!(row("mean_tail_abs_beta_rad").decimals, 6);
        assert_eq!(row("mean_tail_abs_altitude_m").decimals, 3);
        assert_eq!(row("mean_tail_abs_speed_mps").decimals, 3);
        assert_eq!(row("mean_final_abs_altitude_m").decimals, 3);
    }

    #[test]
    fn tail_metrics_ignore_steps_outside_the_tail_window() {
        let mut m = TaskMetrics::new(MetricFamily::LevelHold);
        // Large transient error, outside the tail window.
        m.step(&[1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], false);
        // Small settled error, inside the tail window.
        m.step(&[0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], true);
        m.step(&[0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], true);
        m.finish_episode(&[0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

        let rows = m.into_rows();
        let get = |k: &str| rows.iter().find(|r| r.key == k).unwrap().value;
        // Whole-episode mean is dragged up by the transient step.
        assert!((get("mean_abs_altitude_m") - (200.0 + 20.0 + 20.0) / 3.0).abs() < 1e-3);
        // Tail mean only sees the two settled steps: 0.1*200 = 20.
        assert!((get("mean_tail_abs_altitude_m") - 20.0).abs() < 1e-3);
        assert!((get("mean_tail_abs_speed_mps") - 5.0).abs() < 1e-3); // 0.1*50
    }

    #[test]
    fn empty_run_yields_zero_rows_without_dividing_by_zero() {
        let rows = TaskMetrics::new(MetricFamily::Orbit).into_rows();
        assert_eq!(rows.len(), 10);
        assert!(rows.iter().all(|r| r.value == 0.0));
    }

    /// The critical correctness property `Source::CircularError` exists for:
    /// a sin-encoded angle must NOT be read via `|obs[i]| * scale`, because
    /// `|sin(e)|` folds above π/2 — a near-worst-case ~2.97 rad (170°) error
    /// would misreport as ~0.14 rad (8°) under the naive `Scaled` formula,
    /// making a failing policy look excellent.
    #[test]
    fn circular_error_recovers_unfolded_signed_error() {
        let e: f32 = 2.97;
        let (sin_e, cos_e) = e.sin_cos();
        // obs[13] = sin(e)/0.5, obs[14] = cos(e), matching heading_hold_observation.
        let mut obs = vec![0.0; 16];
        obs[13] = sin_e / HEADING_ERROR_SIN_SCALE;
        obs[14] = cos_e;

        let mut m = TaskMetrics::new(MetricFamily::HeadingHold);
        m.step(&obs, true);
        m.finish_episode(&obs);
        let rows = m.into_rows();
        let get = |k: &str| rows.iter().find(|r| r.key == k).unwrap().value;

        // Must recover ~2.97, never the folded ~0.14 a naive |sin|*scale read would give.
        assert!(
            (get("mean_abs_heading_rad") - e).abs() < 1e-4,
            "expected ~{e}, got {}",
            get("mean_abs_heading_rad")
        );
        assert!(
            get("mean_abs_heading_rad") > 1.0,
            "must not be folded near zero"
        );
    }

    #[test]
    fn heading_hold_rows_include_heading_and_reuse_level_hold_layout() {
        let mut obs = vec![0.0; 16];
        // obs[0]=alt/200, obs[1]=speed/50, obs[4]=roll/0.5, obs[6]=beta/0.5 (shared prefix).
        obs[0] = 0.5;
        obs[1] = -0.4;
        obs[4] = 0.2;
        obs[6] = -0.6;
        let e = 0.3_f32;
        let (sin_e, cos_e) = e.sin_cos();
        obs[13] = sin_e / HEADING_ERROR_SIN_SCALE;
        obs[14] = cos_e;

        let mut m = TaskMetrics::new(MetricFamily::HeadingHold);
        m.step(&obs, true);
        m.finish_episode(&obs);
        let rows = m.into_rows();
        let keys: Vec<&str> = rows.iter().map(|r| r.key).collect();
        assert_eq!(
            keys,
            vec![
                "mean_abs_heading_rad",
                "mean_abs_altitude_m",
                "mean_abs_speed_mps",
                "mean_abs_roll_rad",
                "mean_abs_beta_rad",
                "mean_tail_abs_heading_rad",
                "mean_tail_abs_altitude_m",
                "mean_tail_abs_speed_mps",
                "mean_tail_abs_roll_rad",
                "mean_tail_abs_beta_rad",
                "mean_final_abs_heading_rad",
                "mean_final_abs_altitude_m",
            ]
        );
        let row = |k: &str| *rows.iter().find(|r| r.key == k).unwrap();
        let get = |k: &str| row(k).value;
        assert!((get("mean_abs_heading_rad") - e).abs() < 1e-4);
        assert!((get("mean_abs_altitude_m") - 100.0).abs() < 1e-3);
        assert!((get("mean_abs_speed_mps") - 20.0).abs() < 1e-3);
        assert!((get("mean_abs_roll_rad") - 0.1).abs() < 1e-6);
        assert!((get("mean_abs_beta_rad") - 0.3).abs() < 1e-6);
        assert_eq!(row("mean_abs_heading_rad").decimals, 6);
    }
}
