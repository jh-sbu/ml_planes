//! The task registry: task name → environment, shipped config profiles, model
//! output location, and default training envelope.
//!
//! This mapping used to exist three times — once in `train_ppo`, again in
//! `train_bc`, and a third time as bare string matching in `evaluate_policy` —
//! with the orbit geometry spelled as positional literals inside each `match`
//! arm. It lives here instead because it is not CLI presentation: it is the
//! definition of what "the orbit task" *is*, and every consumer must agree on it
//! or checkpoints stop being comparable.
//!
//! [`make_env`] is the erased entry point the FFI bindings use. It delegates to
//! the same five concrete constructors the binaries call, so a Python-built env
//! cannot drift from a Rust-built one — the divergence CLAUDE.md §3's constraint
//! (b) exists to prevent.
//!
//! What deliberately stays in the binaries: CLI parsing, every `println!`, the
//! warn-then-fall-back-to-defaults handling of a bad `--reward-config`, log-file
//! plumbing, and `save_path_for`'s output-stem search.

use std::ops::RangeInclusive;

use crate::plane::PlaneConfig;
use crate::training::eval_metrics::MetricFamily;
use crate::training::heading_hold_env::{self, HeadingHoldEnv};
use crate::training::level_hold_env::{self, LevelHoldEnv};
use crate::training::reward_config::{
    load_ron_config, HeadingHoldRewardConfig, LevelHoldRewardConfig, OrbitRewardConfig,
};
use crate::training::wu_orbit_reward::WuOrbitRewardConfig;
use crate::training::{OrbitEnv, ResidualOrbitEnv, TrainingEnv, WuOrbitEnv};

/// A learnable task, and the key into everything the training stack needs to
/// build one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Task {
    LevelHold,
    HeadingHold,
    Orbit,
    ResidualOrbit,
    LstmOrbit,
}

/// The circle an orbit-family episode flies.
///
/// A struct rather than the `(f32, f32, f32)` tuple the binaries used, because
/// three bare floats in a row is exactly how `(1000.0, 100.0, 3000.0)` and
/// `(1000.0, 100.0, 1000.0)` came to differ in one position without anyone
/// noticing which.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrbitGeometry {
    pub altitude: f32,
    pub airspeed: f32,
    pub radius: f32,
}

impl Task {
    pub const ALL: [Task; 5] = [
        Task::LevelHold,
        Task::HeadingHold,
        Task::Orbit,
        Task::ResidualOrbit,
        Task::LstmOrbit,
    ];

    pub fn parse(value: &str) -> Result<Self, String> {
        match value {
            "level_hold" => Ok(Self::LevelHold),
            "heading_hold" => Ok(Self::HeadingHold),
            "orbit" => Ok(Self::Orbit),
            "residual_orbit" => Ok(Self::ResidualOrbit),
            "lstm_orbit" => Ok(Self::LstmOrbit),
            other => Err(format!(
                "Unsupported --task '{other}'. Use 'level_hold', 'heading_hold', 'orbit', 'residual_orbit', or 'lstm_orbit'."
            )),
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::LevelHold => "level_hold",
            Self::HeadingHold => "heading_hold",
            Self::Orbit => "orbit",
            Self::ResidualOrbit => "residual_orbit",
            Self::LstmOrbit => "lstm_orbit",
        }
    }

    /// Shipped reward/termination profile, used when no `--reward-config` is given.
    pub fn reward_config_path(self) -> &'static str {
        match self {
            Self::LevelHold => "assets/training/level_hold.reward.ron",
            Self::HeadingHold => "assets/training/heading_hold.reward.ron",
            Self::Orbit | Self::ResidualOrbit => "assets/training/orbit.reward.ron",
            Self::LstmOrbit => "assets/training/wu_orbit.reward.ron",
        }
    }

    /// Per-task default `--ppo-config` path. `None` means the task has no tuned
    /// default and falls back to `PpoHyperparams::default()` (which mirrors
    /// `assets/training/default.ppo.ron`).
    pub fn default_ppo_config_path(self) -> Option<&'static str> {
        match self {
            Self::LevelHold => Some("assets/training/level_hold.ppo.ron"),
            Self::HeadingHold => Some("assets/training/heading_hold.ppo.ron"),
            Self::Orbit | Self::ResidualOrbit | Self::LstmOrbit => None,
        }
    }

    /// Subdirectory of `models/` this task's checkpoints are written to. Matches
    /// `ControllerKind::model_dir` for the corresponding RL controller.
    pub fn model_dir(self) -> &'static str {
        match self {
            Self::LevelHold => "level_hold",
            Self::HeadingHold => "heading_hold",
            Self::Orbit => "orbit",
            Self::ResidualOrbit => "orbit_residual",
            Self::LstmOrbit => "lstm_orbit",
        }
    }

    /// Default output stem for a PPO run (`train_ppo`).
    pub fn ppo_default_stem(self) -> &'static str {
        match self {
            Self::LevelHold => "ppo_level_hold",
            Self::HeadingHold => "ppo_heading_hold",
            Self::Orbit => "ppo_orbit",
            Self::ResidualOrbit => "ppo_orbit_residual",
            Self::LstmOrbit => "ppo_lstm_orbit",
        }
    }

    /// Default output stem for a behavior-cloning run, or `None` for the two
    /// tasks `train_bc` does not support — `ResidualOrbit` and `LstmOrbit` have
    /// no `DemonstrationEnv` impl to roll a PID expert out of.
    pub fn bc_default_stem(self) -> Option<&'static str> {
        match self {
            Self::LevelHold => Some("bc_level_hold"),
            Self::HeadingHold => Some("bc_heading_hold"),
            Self::Orbit => Some("bc_orbit"),
            Self::ResidualOrbit | Self::LstmOrbit => None,
        }
    }

    /// Which observation layout `evaluate_policy` should read this task's
    /// rollouts back through.
    pub fn metric_family(self) -> MetricFamily {
        match self {
            Self::LevelHold => MetricFamily::LevelHold,
            Self::HeadingHold => MetricFamily::HeadingHold,
            Self::Orbit | Self::ResidualOrbit | Self::LstmOrbit => MetricFamily::Orbit,
        }
    }

    /// Whether the task's policy is recurrent (`LstmActorCritic`) rather than
    /// feed-forward (`ActorCritic`).
    pub fn is_recurrent(self) -> bool {
        matches!(self, Self::LstmOrbit)
    }

    /// Per-episode target-altitude envelope when `--target-alt-range` is absent.
    pub fn default_target_alt_range(self) -> RangeInclusive<f32> {
        level_hold_env::DEFAULT_TARGET_ALT_MIN..=level_hold_env::DEFAULT_TARGET_ALT_MAX
    }

    /// Per-episode target-airspeed envelope when `--target-speed-range` is absent.
    ///
    /// `HeadingHold` reads its own constants. They currently hold the same
    /// numbers as level hold's, and the two are kept separate on purpose so
    /// re-opening heading hold's envelope (it used to floor at 110 m/s) does not
    /// silently move level hold's.
    pub fn default_target_speed_range(self) -> RangeInclusive<f32> {
        match self {
            Self::HeadingHold => {
                heading_hold_env::DEFAULT_TARGET_AIRSPEED_MIN
                    ..=heading_hold_env::DEFAULT_TARGET_AIRSPEED_MAX
            }
            _ => {
                level_hold_env::DEFAULT_TARGET_AIRSPEED_MIN
                    ..=level_hold_env::DEFAULT_TARGET_AIRSPEED_MAX
            }
        }
    }

    /// Per-episode target heading *change* envelope, in **degrees**.
    ///
    /// The `_deg` suffix is load-bearing. The CLI and these constants are in
    /// degrees; `HeadingHoldEnv` and [`EnvSpec::target_heading_range`] are in
    /// radians. A caller that skips the conversion trains across an envelope
    /// 57× too wide and gets no error for it.
    pub fn default_target_heading_range_deg(self) -> RangeInclusive<f32> {
        heading_hold_env::DEFAULT_TARGET_HEADING_DEG_MIN
            ..=heading_hold_env::DEFAULT_TARGET_HEADING_DEG_MAX
    }

    /// The circle the orbit family flies. Meaningless for the two non-orbit
    /// tasks, which ignore [`EnvSpec::orbit`] entirely.
    pub fn default_orbit_geometry(self) -> OrbitGeometry {
        match self {
            // The Wu curriculum trains on a wider circle than the direct and
            // residual orbit tasks.
            Self::LstmOrbit => OrbitGeometry {
                altitude: 1000.0,
                airspeed: 100.0,
                radius: 3000.0,
            },
            _ => OrbitGeometry {
                altitude: 1000.0,
                airspeed: 100.0,
                radius: 1000.0,
            },
        }
    }
}

/// Everything needed to build a task's environment except its reward profile.
///
/// Holds *resolved values*, not paths: which airframe, which target envelope,
/// which circle. The reward config is passed alongside because its type varies
/// per task, and because loading it is where the binaries do their own printing
/// and warn-then-default handling.
#[derive(Debug, Clone)]
pub struct EnvSpec {
    pub plane_config: PlaneConfig,
    pub target_alt_range: RangeInclusive<f32>,
    pub target_speed_range: RangeInclusive<f32>,
    /// **Radians.** See [`Task::default_target_heading_range_deg`].
    pub target_heading_range: RangeInclusive<f32>,
    pub orbit: OrbitGeometry,
}

impl EnvSpec {
    /// The envelope a bare `train_ppo --task <t>` trains across.
    pub fn defaults_for(task: Task, plane_config: PlaneConfig) -> Self {
        let heading_deg = task.default_target_heading_range_deg();
        Self {
            plane_config,
            target_alt_range: task.default_target_alt_range(),
            target_speed_range: task.default_target_speed_range(),
            target_heading_range: heading_deg.start().to_radians()..=heading_deg.end().to_radians(),
            orbit: task.default_orbit_geometry(),
        }
    }
}

pub fn level_hold_env(spec: &EnvSpec, reward: LevelHoldRewardConfig) -> LevelHoldEnv {
    LevelHoldEnv::with_target_ranges(
        spec.target_alt_range.clone(),
        spec.target_speed_range.clone(),
        spec.plane_config.clone(),
        reward,
    )
}

pub fn heading_hold_env(spec: &EnvSpec, reward: HeadingHoldRewardConfig) -> HeadingHoldEnv {
    HeadingHoldEnv::with_target_ranges(
        spec.target_heading_range.clone(),
        spec.target_alt_range.clone(),
        spec.target_speed_range.clone(),
        spec.plane_config.clone(),
        reward,
    )
}

pub fn orbit_env(spec: &EnvSpec, reward: OrbitRewardConfig) -> OrbitEnv {
    OrbitEnv::with_reward_config(
        spec.orbit.altitude,
        spec.orbit.airspeed,
        spec.orbit.radius,
        spec.plane_config.clone(),
        reward,
    )
}

pub fn residual_orbit_env(spec: &EnvSpec, reward: OrbitRewardConfig) -> ResidualOrbitEnv {
    ResidualOrbitEnv::with_reward_config(
        spec.orbit.altitude,
        spec.orbit.airspeed,
        spec.orbit.radius,
        spec.plane_config.clone(),
        reward,
    )
}

pub fn wu_orbit_env(spec: &EnvSpec, reward: WuOrbitRewardConfig) -> WuOrbitEnv {
    WuOrbitEnv::with_reward_config(
        spec.orbit.altitude,
        spec.orbit.airspeed,
        spec.orbit.radius,
        spec.plane_config.clone(),
        reward,
    )
}

/// Build any task's environment behind one type — the entry point for callers
/// that cannot be generic over the env type, i.e. the FFI bindings.
///
/// `reward_path` defaults to [`Task::reward_config_path`], matching what a bare
/// `train_ppo --task <t>` actually trains against. Unlike the binaries, an
/// unreadable profile is an **error** rather than a warning: a binary has an
/// operator watching its stderr, and a library call does not.
///
/// The returned box erases [`CurriculumEnv`](crate::training::CurriculumEnv), so
/// `LstmOrbit` arrives fixed at its first curriculum stage. Build a
/// [`wu_orbit_env`] directly if the stage needs to move.
pub fn make_env(
    task: Task,
    spec: &EnvSpec,
    reward_path: Option<&str>,
) -> Result<Box<dyn TrainingEnv>, String> {
    let path = reward_path.unwrap_or_else(|| task.reward_config_path());
    fn load<T: serde::de::DeserializeOwned>(path: &str) -> Result<T, String> {
        load_ron_config(path).map_err(|e| format!("could not load reward config {path}: {e}"))
    }
    Ok(match task {
        Task::LevelHold => Box::new(level_hold_env(spec, load(path)?)),
        Task::HeadingHold => Box::new(heading_hold_env(spec, load(path)?)),
        Task::Orbit => Box::new(orbit_env(spec, load(path)?)),
        Task::ResidualOrbit => Box::new(residual_orbit_env(spec, load(path)?)),
        Task::LstmOrbit => Box::new(wu_orbit_env(spec, load(path)?)),
    })
}
