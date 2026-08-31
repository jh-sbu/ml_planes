//! Integration tests for the shared evaluation accumulator.

use ml_planes::training::eval::{EvalError, EvalRun};
use ml_planes::training::MetricFamily;

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

#[test]
fn a_single_full_length_episode_aggregates_exactly() {
    // With a budget of 10, steps 9 and 10 are in the tail window.
    let mut run = EvalRun::new(MetricFamily::LevelHold, 1, 10, 1);

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
    assert_eq!(report.success_rate, 1.0);
    assert_eq!(report.mean_return, 10.0);
    assert_eq!(report.mean_length_steps, 10.0);

    // (8 * 1.0 + 2 * 0.5) / 10 = 0.9.
    assert_eq!(row(&report.rows, "mean_abs_altitude_m"), 0.9 * 200.0);
    assert_eq!(row(&report.rows, "mean_abs_speed_mps"), 0.9 * 50.0);
    assert_eq!(row(&report.rows, "mean_abs_roll_rad"), 0.9 * 0.5);
    assert_eq!(row(&report.rows, "mean_abs_beta_rad"), 0.9 * 0.5);

    assert_eq!(row(&report.rows, "mean_tail_abs_altitude_m"), 0.5 * 200.0);
    assert_eq!(row(&report.rows, "mean_tail_abs_speed_mps"), 0.5 * 50.0);

    assert_eq!(row(&report.rows, "mean_final_abs_altitude_m"), 0.25 * 200.0);
}

#[test]
fn an_episode_short_of_the_budget_is_not_a_success() {
    let mut run = EvalRun::new(MetricFamily::LevelHold, 2, 10, 1);

    for _ in 0..10 {
        run.record(0, &level_hold_obs(0.0, 0.0, 0.0, 0.0), 1.0)
            .expect("record");
    }
    run.finish(0, &level_hold_obs(0.0, 0.0, 0.0, 0.0))
        .expect("finish");

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

#[test]
fn a_short_episode_contributes_no_tail_samples() {
    let mut run = EvalRun::new(MetricFamily::LevelHold, 1, 100, 1);
    for _ in 0..50 {
        run.record(0, &level_hold_obs(1.0, 0.0, 0.0, 0.0), 0.0)
            .expect("record");
    }
    run.finish(0, &level_hold_obs(1.0, 0.0, 0.0, 0.0))
        .expect("finish");

    let report = run.report();
    assert_eq!(row(&report.rows, "mean_tail_abs_altitude_m"), 0.0);
    assert_eq!(row(&report.rows, "mean_abs_altitude_m"), 200.0);
}

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
    assert!(
        report.rows.iter().all(|r| r.key != "mean_abs_beta_rad"),
        "orbit family must not report level-hold rows"
    );
}

#[test]
fn slot_count_does_not_change_the_report() {
    fn feed(slots: usize) -> ml_planes::training::eval::EvalReport {
        let mut run = EvalRun::new(MetricFamily::LevelHold, 4, 20, slots);
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
    assert_eq!(run.finish(0, &obs), Err(EvalError::EmptyEpisode(0)));
}

#[test]
fn a_short_observation_is_an_error_not_a_panic() {
    let mut run = EvalRun::new(MetricFamily::LevelHold, 1, 10, 1);
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
