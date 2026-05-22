// Inference-safe: model definitions only (burn::module, burn::nn, burn::tensor).
pub mod lstm_model;
pub mod model;

// Training-only: require autodiff, optimizer, CSV logging, filesystem I/O.
#[cfg(feature = "training")]
pub mod buffer;
#[cfg(feature = "training")]
pub mod csv_log;
#[cfg(feature = "training")]
pub mod lstm_buffer;
#[cfg(feature = "training")]
pub mod lstm_trainer;
#[cfg(feature = "training")]
pub mod trainer;

pub use lstm_model::{LstmActorCritic, LstmHiddenState, LSTM_HIDDEN};
pub use model::ActorCritic;

#[cfg(feature = "training")]
pub use buffer::RolloutBuffer;
#[cfg(feature = "training")]
pub use csv_log::CsvLog;
#[cfg(feature = "training")]
pub use lstm_buffer::{LstmRolloutBuffer, LstmRolloutStep, LstmSequence};
#[cfg(feature = "training")]
pub use lstm_trainer::LstmPpoTrainer;
#[cfg(feature = "training")]
pub use trainer::PpoTrainer;
