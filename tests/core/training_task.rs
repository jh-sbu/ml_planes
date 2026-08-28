//! Behavior-preservation pins for the task registry (`ml_planes::training::task`).
//!
//! The registry replaced three hand-maintained copies of the same task table —
//! one in `train_ppo`, one in `train_bc`, one as string matching in
//! `evaluate_policy`. Nothing in the type system says the values that landed
//! here are the values those binaries used, and a default that shifted during
//! the lift would change what every future run trains on while everything still
//! compiled, ran, and reported success. It would surface months later as a
//! checkpoint that underperforms for no visible reason.
//!
//! So the numbers below are asserted against literals on purpose. If one of
//! them needs to change, change it deliberately and update the literal in the
//! same commit.

use ml_planes::training::task::{self, EnvSpec, OrbitGeometry, Task};
use ml_planes::training::{MetricFamily, TrainingEnv};

use crate::common::generic_jet_config;

#[test]
fn every_task_name_round_trips() {
    for task in Task::ALL {
        assert_eq!(
            Task::parse(task.as_str()),
            Ok(task),
            "{} should parse back to itself",
            task.as_str()
        );
    }
    assert_eq!(
        Task::ALL.map(Task::as_str),
        [
            "level_hold",
            "heading_hold",
            "orbit",
            "residual_orbit",
            "lstm_orbit"
        ]
    );
}

#[test]
fn an_unknown_task_name_errors() {
    let err = Task::parse("barrel_roll").expect_err("unknown task should not parse");
    assert!(
        err.contains("barrel_roll"),
        "error should name the input: {err}"
    );
    assert!(
        err.contains("level_hold") && err.contains("lstm_orbit"),
        "error should list the supported tasks: {err}"
    );
}

/// Also catches a shipped profile renamed out from under the registry.
#[test]
fn every_shipped_config_profile_exists_on_disk() {
    for task in Task::ALL {
        let reward = task.reward_config_path();
        assert!(
            std::path::Path::new(reward).is_file(),
            "{}: reward profile {reward} is missing",
            task.as_str()
        );
        if let Some(ppo) = task.default_ppo_config_path() {
            assert!(
                std::path::Path::new(ppo).is_file(),
                "{}: PPO profile {ppo} is missing",
                task.as_str()
            );
        }
    }
}

#[test]
fn reward_profile_paths_match_the_pre_refactor_binaries() {
    assert_eq!(
        Task::LevelHold.reward_config_path(),
        "assets/training/level_hold.reward.ron"
    );
    assert_eq!(
        Task::HeadingHold.reward_config_path(),
        "assets/training/heading_hold.reward.ron"
    );
    assert_eq!(
        Task::Orbit.reward_config_path(),
        "assets/training/orbit.reward.ron"
    );
    assert_eq!(
        Task::ResidualOrbit.reward_config_path(),
        "assets/training/orbit.reward.ron"
    );
    assert_eq!(
        Task::LstmOrbit.reward_config_path(),
        "assets/training/wu_orbit.reward.ron"
    );

    assert_eq!(
        Task::LevelHold.default_ppo_config_path(),
        Some("assets/training/level_hold.ppo.ron")
    );
    assert_eq!(
        Task::HeadingHold.default_ppo_config_path(),
        Some("assets/training/heading_hold.ppo.ron")
    );
    for task in [Task::Orbit, Task::ResidualOrbit, Task::LstmOrbit] {
        assert_eq!(
            task.default_ppo_config_path(),
            None,
            "{} had no tuned PPO default before the refactor",
            task.as_str()
        );
    }
}

#[test]
fn model_dirs_and_stems_match_the_pre_refactor_binaries() {
    let expected = [
        (Task::LevelHold, "level_hold", "ppo_level_hold"),
        (Task::HeadingHold, "heading_hold", "ppo_heading_hold"),
        (Task::Orbit, "orbit", "ppo_orbit"),
        (Task::ResidualOrbit, "orbit_residual", "ppo_orbit_residual"),
        (Task::LstmOrbit, "lstm_orbit", "ppo_lstm_orbit"),
    ];
    for (task, dir, stem) in expected {
        assert_eq!(task.model_dir(), dir, "{}", task.as_str());
        assert_eq!(task.ppo_default_stem(), stem, "{}", task.as_str());
    }

    // train_bc only ever supported these three; the other two have no
    // `DemonstrationEnv` to roll a PID expert out of.
    assert_eq!(Task::LevelHold.bc_default_stem(), Some("bc_level_hold"));
    assert_eq!(Task::HeadingHold.bc_default_stem(), Some("bc_heading_hold"));
    assert_eq!(Task::Orbit.bc_default_stem(), Some("bc_orbit"));
    assert_eq!(Task::ResidualOrbit.bc_default_stem(), None);
    assert_eq!(Task::LstmOrbit.bc_default_stem(), None);
}

#[test]
fn default_target_envelopes_match_the_pre_refactor_binaries() {
    for task in Task::ALL {
        assert_eq!(
            task.default_target_alt_range(),
            500.0..=5000.0,
            "{}",
            task.as_str()
        );
        assert_eq!(
            task.default_target_speed_range(),
            90.0..=140.0,
            "{}",
            task.as_str()
        );
        assert_eq!(
            task.default_target_heading_range_deg(),
            -180.0..=180.0,
            "{}",
            task.as_str()
        );
    }
}

#[test]
fn default_orbit_geometry_matches_the_pre_refactor_binaries() {
    let standard = OrbitGeometry {
        altitude: 1000.0,
        airspeed: 100.0,
        radius: 1000.0,
    };
    assert_eq!(Task::Orbit.default_orbit_geometry(), standard);
    assert_eq!(Task::ResidualOrbit.default_orbit_geometry(), standard);
    // The one that differs, and the whole reason this is a struct now.
    assert_eq!(
        Task::LstmOrbit.default_orbit_geometry(),
        OrbitGeometry {
            altitude: 1000.0,
            airspeed: 100.0,
            radius: 3000.0,
        }
    );
}

/// `EnvSpec` stores radians; the accessor is degrees. Getting this wrong trains
/// across an envelope 57x too wide and reports nothing.
#[test]
fn env_spec_converts_the_heading_envelope_to_radians() {
    let spec = EnvSpec::defaults_for(Task::HeadingHold, generic_jet_config());
    assert!(
        (*spec.target_heading_range.start() + std::f32::consts::PI).abs() < 1e-5,
        "expected -pi rad, got {}",
        spec.target_heading_range.start()
    );
    assert!(
        (*spec.target_heading_range.end() - std::f32::consts::PI).abs() < 1e-5,
        "expected +pi rad, got {}",
        spec.target_heading_range.end()
    );
}

#[test]
fn make_env_builds_every_task_at_its_documented_dimensions() {
    let expected_obs = [
        (Task::LevelHold, 13),
        (Task::HeadingHold, 16),
        (Task::Orbit, 14),
        (Task::ResidualOrbit, 14),
        (Task::LstmOrbit, 14),
    ];
    for (task, obs_dim) in expected_obs {
        let spec = EnvSpec::defaults_for(task, generic_jet_config());
        let env = task::make_env(task, &spec, None)
            .unwrap_or_else(|e| panic!("{} should build: {e}", task.as_str()));
        assert_eq!(env.observation_dim(), obs_dim, "{}", task.as_str());
        assert_eq!(env.action_dim(), 4, "{}", task.as_str());
    }
}

/// The binaries warn and fall back; a library call must not silently hand back
/// an env trained against a reward profile the caller did not ask for.
#[test]
fn make_env_errors_on_an_unreadable_reward_profile() {
    let spec = EnvSpec::defaults_for(Task::Orbit, generic_jet_config());
    let err = task::make_env(Task::Orbit, &spec, Some("assets/training/nope.reward.ron"))
        .err()
        .expect("a missing reward profile should not fall back silently");
    assert!(
        err.contains("nope.reward.ron"),
        "error should name the path: {err}"
    );
}

#[test]
fn metric_family_and_recurrence_match_evaluate_policy() {
    assert_eq!(Task::LevelHold.metric_family(), MetricFamily::LevelHold);
    assert_eq!(Task::HeadingHold.metric_family(), MetricFamily::HeadingHold);
    for task in [Task::Orbit, Task::ResidualOrbit, Task::LstmOrbit] {
        assert_eq!(
            task.metric_family(),
            MetricFamily::Orbit,
            "{}",
            task.as_str()
        );
    }
    for task in Task::ALL {
        assert_eq!(
            task.is_recurrent(),
            task == Task::LstmOrbit,
            "{} recurrence",
            task.as_str()
        );
    }
}
