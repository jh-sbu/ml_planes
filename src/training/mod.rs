pub mod bc;
pub mod env;
mod flight_env;
pub mod level_hold_env;
pub mod orbit_env;
pub mod orbit_residual_env;
#[cfg(any(feature = "inference", feature = "training"))]
pub mod ppo;
pub mod reward_config;
pub mod vec_env;
pub mod wu_orbit_env;
pub mod wu_orbit_reward;

pub use bc::{collect_demonstrations, BcDataset, DemonstrationEnv};
pub use env::{Observation, SpawnSpec, StepInfo, TrainingEnv};

#[cfg(any(feature = "inference", feature = "training"))]
pub(crate) use flight_env::direct_action_to_inputs;

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
#[cfg(any(feature = "inference", feature = "training"))]
pub use ppo::{ActorCritic, LstmActorCritic, LstmHiddenState, LSTM_HIDDEN};

#[cfg(feature = "training")]
pub use ppo::{
    LstmPpoTrainer, LstmRolloutBuffer, LstmRolloutStep, LstmSequence, PpoTrainer, RolloutBuffer,
};
pub use reward_config::{LevelHoldRewardConfig, OrbitRewardConfig};
pub use vec_env::VecEnv;
pub use wu_orbit_env::WuOrbitEnv;
pub use wu_orbit_reward::{CurriculumStage, WuOrbitRewardConfig};
