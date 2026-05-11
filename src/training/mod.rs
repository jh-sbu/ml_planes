pub mod env;
mod flight_env;
pub mod level_hold_env;
pub mod orbit_env;
pub mod orbit_residual_env;
#[cfg(feature = "training")]
pub mod ppo;
pub mod reward_config;
pub mod vec_env;
pub mod wu_orbit_env;
pub mod wu_orbit_reward;

pub use env::{Observation, SpawnSpec, StepInfo, TrainingEnv};

/// Optional trait for training environments that support a curriculum.
///
/// Implemented by `WuOrbitEnv`; `LstmPpoTrainer` requires this bound
/// to call `advance_curriculum` automatically based on return thresholds.
pub trait CurriculumEnv: TrainingEnv {
    /// Advance to the next curriculum stage (idempotent at the final stage).
    fn advance_curriculum(&mut self);
    /// Human-readable name of the current stage (for logging).
    fn curriculum_stage_name(&self) -> &'static str;
    /// Mean episode return threshold to advance from the current stage to the next.
    /// Returns `f32::INFINITY` when already at the final stage.
    fn next_stage_threshold(&self) -> f32;
}
pub use level_hold_env::LevelHoldEnv;
pub use orbit_env::OrbitEnv;
pub use orbit_residual_env::ResidualOrbitEnv;
#[cfg(feature = "training")]
pub use ppo::{
    ActorCritic, LstmActorCritic, LstmHiddenState, LstmPpoTrainer, LstmRolloutBuffer,
    LstmRolloutStep, LstmSequence, PpoTrainer, RolloutBuffer, LSTM_HIDDEN,
};
pub use reward_config::{LevelHoldRewardConfig, OrbitRewardConfig};
pub use vec_env::VecEnv;
pub use wu_orbit_env::WuOrbitEnv;
pub use wu_orbit_reward::{CurriculumStage, WuOrbitRewardConfig};
