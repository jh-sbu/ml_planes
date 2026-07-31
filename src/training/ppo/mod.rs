// Inference-safe: model definitions only (burn::module, burn::nn, burn::tensor).
pub mod lstm_model;
pub mod model;

/// Serializes access to the shared backend RNG (`Backend::seed`, weight init
/// via `LinearConfig`/`LstmConfig::init`, and `Tensor::random` sampling).
///
/// Burn's `ndarray` backend keeps this RNG behind a single **process-global**
/// mutex (`burn-ndarray`'s internal `SEED`), so under `cargo test`'s default
/// multi-threaded harness, two unrelated tests that each construct or sample
/// a model can interleave their draws and silently break a
/// `Backend::seed`-based determinism guarantee — even though each individual
/// burn call is itself thread-safe. `ActorCritic`/`LstmActorCritic`'s `new`,
/// `new_seeded`, and sampling methods all take this lock for the duration of
/// the single RNG-touching operation, so "same seed → same weights/output"
/// assertions hold under parallel test execution. Recovers from poisoning
/// (a prior panic while holding the lock) since the guarded state is just
/// `()` — there is nothing to roll back.
pub(crate) static RNG_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

/// Acquire [`RNG_LOCK`], recovering from poison.
pub(crate) fn rng_lock() -> std::sync::MutexGuard<'static, ()> {
    RNG_LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

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
