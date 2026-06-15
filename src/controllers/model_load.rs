//! Error type for loading trained RL policies.
//!
//! A checkpoint's `.mpk` weights deserialize successfully even when their
//! observation dimension no longer matches the current model — burn's
//! `load_record`/`load_file` adopt the file's tensor shapes without validating
//! them. The mismatch would otherwise only surface as a matmul-shape panic on
//! the first forward pass. The RL controllers' `load`/`load_bytes` methods
//! validate the loaded input dimension and return [`ModelLoadError`] so callers
//! can skip the model gracefully (log + fall back) instead of crashing.

use std::fmt;

/// Failure modes when loading a trained policy checkpoint.
#[derive(Debug)]
pub enum ModelLoadError {
    /// The underlying recorder failed to read/deserialize the checkpoint.
    Recorder(burn::record::RecorderError),
    /// The checkpoint loaded but its observation dimension does not match the
    /// dimension this controller feeds — almost always a stale (pre-fuel)
    /// checkpoint. Loading it would panic at the first forward pass.
    DimensionMismatch { expected: usize, found: usize },
}

impl From<burn::record::RecorderError> for ModelLoadError {
    fn from(e: burn::record::RecorderError) -> Self {
        ModelLoadError::Recorder(e)
    }
}

impl fmt::Display for ModelLoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ModelLoadError::Recorder(e) => write!(f, "{e}"),
            ModelLoadError::DimensionMismatch { expected, found } => write!(
                f,
                "incompatible model (expects obs dim {found}, controller needs {expected}; \
                 likely a stale pre-fuel checkpoint) — retrain to use",
            ),
        }
    }
}

impl std::error::Error for ModelLoadError {}
