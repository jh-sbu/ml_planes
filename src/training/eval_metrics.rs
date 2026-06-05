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
//! - **Level-hold family** (10-dim): `[alt_err/200, speed_err/50, alpha/0.5,
//!   pitch_rate, roll/0.5, roll_rate, beta/0.5, ...]`.

/// Which observation layout a checkpoint's task uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MetricFamily {
    /// 13-dim orbit observation (orbit / residual_orbit / lstm_orbit).
    Orbit,
    /// 10-dim level-hold observation.
    LevelHold,
}

/// Decimal precision for metrics in physical distance/speed units (metres,
/// m/s). Millimetre / mm-per-second resolution is ample.
const DECIMALS_LINEAR: usize = 3;
/// Decimal precision for radian-valued metrics. Heading/roll/beta errors are
/// small in magnitude, so finer resolution is kept to avoid masking changes the
/// downstream skills compare on (matches the legacy `{:.6}` heading output).
const DECIMALS_RADIAN: usize = 6;

/// One extracted metric: output key, source obs index, the de-normalising scale
/// that converts the normalised observation back to physical units, and the
/// decimal precision to print it at.
struct MetricSpec {
    key: &'static str,
    obs_index: usize,
    scale: f32,
    decimals: usize,
}

impl MetricFamily {
    /// Per-step (mean-over-all-steps) metrics, in stable output order.
    fn step_specs(self) -> &'static [MetricSpec] {
        match self {
            MetricFamily::Orbit => &[
                MetricSpec {
                    key: "mean_abs_radial_m",
                    obs_index: 0,
                    scale: 500.0,
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_abs_heading_rad",
                    obs_index: 1,
                    scale: 0.5,
                    decimals: DECIMALS_RADIAN,
                },
                MetricSpec {
                    key: "mean_abs_altitude_m",
                    obs_index: 3,
                    scale: 200.0,
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_abs_speed_mps",
                    obs_index: 4,
                    scale: 50.0,
                    decimals: DECIMALS_LINEAR,
                },
            ],
            MetricFamily::LevelHold => &[
                MetricSpec {
                    key: "mean_abs_altitude_m",
                    obs_index: 0,
                    scale: 200.0,
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_abs_speed_mps",
                    obs_index: 1,
                    scale: 50.0,
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_abs_roll_rad",
                    obs_index: 4,
                    scale: 0.5,
                    decimals: DECIMALS_RADIAN,
                },
                MetricSpec {
                    key: "mean_abs_beta_rad",
                    obs_index: 6,
                    scale: 0.5,
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
                    obs_index: 0,
                    scale: 500.0,
                    decimals: DECIMALS_LINEAR,
                },
                MetricSpec {
                    key: "mean_final_abs_altitude_m",
                    obs_index: 3,
                    scale: 200.0,
                    decimals: DECIMALS_LINEAR,
                },
            ],
            MetricFamily::LevelHold => &[MetricSpec {
                key: "mean_final_abs_altitude_m",
                obs_index: 0,
                scale: 200.0,
                decimals: DECIMALS_LINEAR,
            }],
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
    final_sums: Vec<f32>,
    episodes: u64,
}

impl TaskMetrics {
    pub fn new(family: MetricFamily) -> Self {
        Self {
            family,
            step_sums: vec![0.0; family.step_specs().len()],
            samples: 0,
            final_sums: vec![0.0; family.final_specs().len()],
            episodes: 0,
        }
    }

    /// Accumulate per-step tracking errors from a post-step observation.
    pub fn step(&mut self, obs: &[f32]) {
        for (sum, spec) in self.step_sums.iter_mut().zip(self.family.step_specs()) {
            *sum += obs[spec.obs_index].abs() * spec.scale;
        }
        self.samples += 1;
    }

    /// Accumulate final-state tracking errors from the terminal observation.
    pub fn finish_episode(&mut self, final_obs: &[f32]) {
        for (sum, spec) in self.final_sums.iter_mut().zip(self.family.final_specs()) {
            *sum += final_obs[spec.obs_index].abs() * spec.scale;
        }
        self.episodes += 1;
    }

    /// Produce the metric rows in stable output order: all per-step means
    /// first, then all final-state means. Each row carries its print precision.
    pub fn into_rows(self) -> Vec<MetricRow> {
        let samples = self.samples.max(1) as f32;
        let episodes = self.episodes.max(1) as f32;
        let mut rows = Vec::with_capacity(self.step_sums.len() + self.final_sums.len());
        for (sum, spec) in self.step_sums.iter().zip(self.family.step_specs()) {
            rows.push(MetricRow {
                key: spec.key,
                value: sum / samples,
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
        m.step(&[
            0.2, -0.4, 0.0, 0.5, -0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]);
        m.step(&[
            0.4, 0.2, 0.0, -0.25, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        ]);
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
        // final radial over 1 episode: 0.4*500 = 200
        assert!((get("mean_final_abs_radial_m") - 200.0).abs() < 1e-3);
        // final altitude over 1 episode: 0.25*200 = 50
        assert!((get("mean_final_abs_altitude_m") - 50.0).abs() < 1e-3);
        // Precision: the radian-valued heading metric keeps 6 decimals (matching
        // the legacy output); linear metres/speed metrics print at 3.
        assert_eq!(row("mean_abs_heading_rad").decimals, 6);
        assert_eq!(row("mean_abs_radial_m").decimals, 3);
        assert_eq!(row("mean_abs_altitude_m").decimals, 3);
        assert_eq!(row("mean_abs_speed_mps").decimals, 3);
        assert_eq!(row("mean_final_abs_radial_m").decimals, 3);
        assert_eq!(row("mean_final_abs_altitude_m").decimals, 3);
    }

    #[test]
    fn level_hold_rows_have_altitude_speed_attitude_no_radial_heading() {
        let mut m = TaskMetrics::new(MetricFamily::LevelHold);
        // obs[0]=alt/200, obs[1]=speed/50, obs[4]=roll/0.5, obs[6]=beta/0.5.
        m.step(&[0.5, -0.4, 0.0, 0.0, 0.2, 0.0, -0.6, 0.0, 0.0, 0.0]);
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
        assert!((get("mean_final_abs_altitude_m") - 100.0).abs() < 1e-3);
        // Radian-valued attitude metrics keep 6 decimals; linear metres at 3.
        assert_eq!(row("mean_abs_roll_rad").decimals, 6);
        assert_eq!(row("mean_abs_beta_rad").decimals, 6);
        assert_eq!(row("mean_abs_altitude_m").decimals, 3);
        assert_eq!(row("mean_abs_speed_mps").decimals, 3);
        assert_eq!(row("mean_final_abs_altitude_m").decimals, 3);
    }

    #[test]
    fn empty_run_yields_zero_rows_without_dividing_by_zero() {
        let rows = TaskMetrics::new(MetricFamily::Orbit).into_rows();
        assert_eq!(rows.len(), 6);
        assert!(rows.iter().all(|r| r.value == 0.0));
    }
}
