//! Behavior pins for the shared evaluation accumulator (`ml_planes::training::eval`).
//!
//! `EvalRun` is the single implementation of the rules that turn a rollout into
//! an evaluation report: the settled-tail window, the success criterion, core
//! aggregation, and (via `TaskMetrics`) the obs-index → physical-units mapping.
//! It exists because those rules were written twice — once privately inside
//! `src/bin/evaluate_policy.rs`, once by hand in Python in
//! `bindings/python/examples/train_ppo_torch.py::evaluate_checkpoint` — and the
//! two copies disagreed. Both now drive this type, so a divergence has nowhere
//! left to hide.
//!
//! The numbers below are hand-computed from the fed observations rather than
//! captured from a run. That is the point: a captured value would happily pin
//! whatever the code does today, including a wrong tail boundary.

use ml_planes::training::eval::{EvalError, EvalRun};
use ml_planes::training::MetricFamily;

/// A level-hold-shaped observation. Only indices 0/1/4/6 are read by
/// `MetricFamily::LevelHold` (altitude, speed, roll, beta), so the rest is
/// padding that must not be touched.
fn level_hold_obs(alt: f32, speed: f32, roll: f32, beta: f32) -> Vec<f32> {
    let mut obs = vec![0.0_f32; 13];
    obs[0] = alt;
    obs[1] = speed;
    obs[4] = roll;
    obs[6] = beta;
    obs
}

fn row(rows: &[ml_planes::training::eval_metrics::MetricRow], key: &str) -> f32 {
    rows.iter()
        .find(|r| r.key == key)
        .unwrap_or_else(|| panic!("no metric row {key}"))
        .value
}

/// One full-length episode: every core metric and every level-hold row is
/// computed from values chosen so the expected answer is exact in f32.
#[test]
fn a_single_full_length_episode_aggregates_exactly() {
    // max_steps = 10 ⇒ tail_start = 10 - (10 * 0.2) = 8, so steps 9 and 10 are
    // the tail (the rule is `len > tail_start`, not `>=`).
    let mut run = EvalRun::new(MetricFamily::LevelHold, 1, 10, 1);

    // Steps 1..=8 sit outside the tail; steps 9..=10 inside it, at half the
    // error, so the tail mean and the all-step mean must differ.
    for step in 1..=10 {
        let scale = if step > 8 { 0.5 } else { 1.0 };
        let obs = level_hold_obs(scale, scale, scale, scale);
        run.record(0, &obs, 1.0).expect("record");
    }
    run.finish(0, &level_hold_obs(0.25, 0.0, 0.0, 0.0))
        .expect("finish");

    assert!(run.is_complete(), "one episode of one was recorded");
    let report = run.report();

    assert_eq!(report.episodes, 1);
    // Ran the full budget, so it counts as a success.
    assert_eq!(report.success_rate, 1.0);
    assert_eq!(report.mean_return, 10.0);
    assert_eq!(report.mean_length_steps, 10.0);

    // All-step mean of |obs[0]| is (8 * 1.0 + 2 * 0.5) / 10 = 0.9, scaled by 200.
    assert_eq!(row(&report.rows, "mean_abs_altitude_m"), 0.9 * 200.0);
    assert_eq!(row(&report.rows, "mean_abs_speed_mps"), 0.9 * 50.0);
    assert_eq!(row(&report.rows, "mean_abs_roll_rad"), 0.9 * 0.5);
    assert_eq!(row(&report.rows, "mean_abs_beta_rad"), 0.9 * 0.5);

    // Tail mean is the two 0.5 steps only.
    assert_eq!(row(&report.rows, "mean_tail_abs_altitude_m"), 0.5 * 200.0);
    assert_eq!(row(&report.rows, "mean_tail_abs_speed_mps"), 0.5 * 50.0);

    // Final-state metric reads the terminal observation handed to `finish`.
    assert_eq!(row(&report.rows, "mean_final_abs_altitude_m"), 0.25 * 200.0);
}

/// The success criterion is "ran the full step budget", so an episode cut short
/// is a failure however good its tracking was.
#[test]
fn an_episode_short_of_the_budget_is_not_a_success() {
    let mut run = EvalRun::new(MetricFamily::LevelHold, 2, 10, 1);

    // Episode 0 runs the full 10 steps.
    for _ in 0..10 {
        run.record(0, &level_hold_obs(0.0, 0.0, 0.0, 0.0), 1.0)
            .expect("record");
    }
    run.finish(0, &level_hold_obs(0.0, 0.0, 0.0, 0.0))
        .expect("finish");

    // Episode 1 dies after 4.
    for _ in 0..4 {
        run.record(0, &level_hold_obs(0.0, 0.0, 0.0, 0.0), 1.0)
            .expect("record");
    }
    run.finish(0, &level_hold_obs(0.0, 0.0, 0.0, 0.0))
        .expect("finish");

    let report = run.report();
    assert_eq!(report.episodes, 2);
    assert_eq!(report.success_rate, 0.5);
    assert_eq!(report.mean_length_steps, 7.0);
    assert_eq!(report.mean_return, 7.0);
}

/// The tail window is the final 20% of the *step budget*, not of the episode's
/// realised length — an episode that dies early contributes no tail samples at
/// all. This is what makes `mean_tail_*` mean "settled", and it is the rule the
/// hand-written Python copy had to reproduce.
#[test]
fn a_short_episode_contributes_no_tail_samples() {
    let mut run = EvalRun::new(MetricFamily::LevelHold, 1, 100, 1);
    // tail_start = 80; this episode ends at 50.
    for _ in 0..50 {
        run.record(0, &level_hold_obs(1.0, 0.0, 0.0, 0.0), 0.0)
            .expect("record");
    }
    run.finish(0, &level_hold_obs(1.0, 0.0, 0.0, 0.0))
        .expect("finish");

    let report = run.report();
    // No tail samples were taken, so the tail mean must be a clean zero rather
    // than a divide-by-zero or a silent reuse of the all-step mean.
    assert_eq!(row(&report.rows, "mean_tail_abs_altitude_m"), 0.0);
    assert_eq!(row(&report.rows, "mean_abs_altitude_m"), 200.0);
}

/// Every family the registry can hand back must produce its own row set; the
/// orbit family reads a different observation layout entirely.
#[test]
fn the_orbit_family_reports_its_own_rows() {
    let mut run = EvalRun::new(MetricFamily::Orbit, 1, 10, 1);
    let mut obs = vec![0.0_f32; 14];
    obs[0] = 1.0; // radial / 500
    obs[3] = 1.0; // altitude / 200
    for _ in 0..10 {
        run.record(0, &obs, 0.0).expect("record");
    }
    run.finish(0, &obs).expect("finish");

    let report = run.report();
    assert_eq!(row(&report.rows, "mean_abs_radial_m"), 500.0);
    assert_eq!(row(&report.rows, "mean_abs_altitude_m"), 200.0);
    assert_eq!(row(&report.rows, "mean_final_abs_radial_m"), 500.0);
    // Keys must be the orbit set, not the level-hold set.
    assert!(
        report.rows.iter().all(|r| r.key != "mean_abs_beta_rad"),
        "orbit family must not report level-hold rows"
    );
}

/// The Rust binary drives one slot sequentially; the Python evaluator drives one
/// slot per episode. Both must produce the same report for the same episodes —
/// only the order in which the sums accumulate differs, which is a last-ULP
/// effect, not a difference in the rules.
#[test]
fn slot_count_does_not_change_the_report() {
    fn feed(slots: usize) -> ml_planes::training::eval::EvalReport {
        let mut run = EvalRun::new(MetricFamily::LevelHold, 4, 20, slots);
        // Episode e gets slot (e % slots) and a distinct error level.
        for episode in 0..4 {
            let slot = episode % slots;
            let level = 0.1 * (episode + 1) as f32;
            for _ in 0..20 {
                run.record(slot, &level_hold_obs(level, level, 0.0, 0.0), level)
                    .expect("record");
            }
            run.finish(slot, &level_hold_obs(level, 0.0, 0.0, 0.0))
                .expect("finish");
        }
        run.report()
    }

    let sequential = feed(1);
    let parallel = feed(4);

    assert_eq!(sequential.episodes, parallel.episodes);
    assert_eq!(sequential.success_rate, parallel.success_rate);
    assert!(
        (sequential.mean_return - parallel.mean_return).abs() < 1e-4,
        "mean_return {} vs {}",
        sequential.mean_return,
        parallel.mean_return
    );
    for a in &sequential.rows {
        let b = row(&parallel.rows, a.key);
        assert!(
            (a.value - b).abs() < 1e-3,
            "{}: {} vs {}",
            a.key,
            a.value,
            b
        );
    }
}

// ---------------------------------------------------------------------------
// Misuse must be an error, never a panic — the bindings turn a Rust panic into
// an uncatchable PanicException, so every one of these has to be reachable as a
// normal Python exception.
// ---------------------------------------------------------------------------

#[test]
fn an_out_of_range_slot_is_an_error() {
    let mut run = EvalRun::new(MetricFamily::LevelHold, 1, 10, 2);
    let obs = level_hold_obs(0.0, 0.0, 0.0, 0.0);
    assert_eq!(run.record(2, &obs, 0.0), Err(EvalError::SlotOutOfRange(2)));
    assert_eq!(run.finish(2, &obs), Err(EvalError::SlotOutOfRange(2)));
}

#[test]
fn recording_more_episodes_than_declared_is_an_error() {
    let mut run = EvalRun::new(MetricFamily::LevelHold, 1, 10, 1);
    let obs = level_hold_obs(0.0, 0.0, 0.0, 0.0);
    run.record(0, &obs, 0.0).expect("record");
    run.finish(0, &obs).expect("finish");
    // The one declared episode is already done.
    assert_eq!(run.finish(0, &obs), Err(EvalError::AllEpisodesFinished));
    assert_eq!(
        run.record(0, &obs, 0.0),
        Err(EvalError::AllEpisodesFinished)
    );
}

#[test]
fn finishing_a_slot_that_recorded_nothing_is_an_error() {
    let mut run = EvalRun::new(MetricFamily::LevelHold, 2, 10, 1);
    let obs = level_hold_obs(0.0, 0.0, 0.0, 0.0);
    // A zero-length episode is a caller bug, not a legitimate datapoint: it
    // would drag mean_length down and count as a non-success.
    assert_eq!(run.finish(0, &obs), Err(EvalError::EmptyEpisode(0)));
}

#[test]
fn a_short_observation_is_an_error_not_a_panic() {
    let mut run = EvalRun::new(MetricFamily::LevelHold, 1, 10, 1);
    // LevelHold reads index 6; a 4-element observation would index out of
    // bounds inside TaskMetrics.
    let short = vec![0.0_f32; 4];
    assert_eq!(
        run.record(0, &short, 0.0),
        Err(EvalError::ObservationTooShort { got: 4, need: 7 })
    );
}

#[test]
fn reporting_before_every_episode_finished_is_refused() {
    let mut run = EvalRun::new(MetricFamily::LevelHold, 2, 10, 1);
    let obs = level_hold_obs(0.0, 0.0, 0.0, 0.0);
    run.record(0, &obs, 0.0).expect("record");
    run.finish(0, &obs).expect("finish");
    assert!(!run.is_complete(), "one of two episodes is done");
    assert_eq!(
        run.try_report().err(),
        Some(EvalError::Incomplete {
            finished: 1,
            expected: 2
        })
    );
}
