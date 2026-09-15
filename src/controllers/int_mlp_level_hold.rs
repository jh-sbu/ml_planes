//! IntMLP level hold: a trained network whose memory is two error integrators.
//!
//! Architecture and training pipeline (DAgger from `InversionLevelHoldController`,
//! then evolution strategies) are promoted from `experiments/nn_arch`; checkpoints are
//! produced by the `train_int_mlp` binary into `models/int_mlp_level_hold/`.
//!
//! The checkpoint is loaded once through burn and then run on the flat
//! [`IntMlpWeights`] kernel — no tensor backend, and so no `Mutex`, per tick.

use std::any::Any;

use burn::backend::NdArray;
use burn::module::Module;
#[cfg(not(target_arch = "wasm32"))]
use burn::record::DefaultFileRecorder;
use burn::record::{FullPrecisionSettings, NamedMpkBytesRecorder, Recorder, RecorderError};

use crate::controllers::model_load::ModelLoadError;
use crate::controllers::{ControllerTargets, FlightController};
use crate::plane::{ControlInputs, ControllerContext, FlightState};
use crate::training::direct_action_to_inputs;
use crate::training::int_mlp::{
    int_mlp_features, IntMlpWeights, IntegratorState, INT_MLP_INPUT_DIM,
};
use crate::training::int_mlp_model::IntMlpPolicy;
use crate::training::level_hold_env::level_hold_observation;

/// Trained IntMLP level-hold policy. Create a fresh instance per episode: the
/// integrators start at zero.
#[derive(Debug, Clone)]
pub struct IntMlpLevelHoldController {
    weights: IntMlpWeights,
    integrators: IntegratorState,
    pub target_altitude: f32,
    pub target_airspeed: f32,
}

impl IntMlpLevelHoldController {
    pub fn from_weights(
        weights: IntMlpWeights,
        target_altitude: f32,
        target_airspeed: f32,
    ) -> Self {
        Self {
            weights,
            integrators: IntegratorState::default(),
            target_altitude,
            target_airspeed,
        }
    }

    /// Validate a loaded record and move it onto the flat kernel. burn adopts a
    /// file's tensor shapes without checking them, so a mis-shaped checkpoint would
    /// otherwise surface as an out-of-bounds panic on the first tick.
    fn from_model(
        model: IntMlpPolicy<NdArray>,
        target_altitude: f32,
        target_airspeed: f32,
    ) -> Result<Self, ModelLoadError> {
        let found = model.input_dim();
        if found != INT_MLP_INPUT_DIM {
            return Err(ModelLoadError::DimensionMismatch {
                expected: INT_MLP_INPUT_DIM,
                found,
            });
        }
        let weights = model
            .to_weights()
            .map_err(|e| ModelLoadError::Recorder(RecorderError::Unknown(e)))?;
        Ok(Self::from_weights(
            weights,
            target_altitude,
            target_airspeed,
        ))
    }

    /// Load a `.mpk` checkpoint from `path` (without the extension).
    #[cfg(not(target_arch = "wasm32"))]
    pub fn load(
        path: &str,
        target_altitude: f32,
        target_airspeed: f32,
    ) -> Result<Self, ModelLoadError> {
        let device = Default::default();
        let model = IntMlpPolicy::<NdArray>::new(&device).load_file(
            path,
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
            &device,
        )?;
        Self::from_model(model, target_altitude, target_airspeed)
    }

    /// Load a checkpoint from embedded bytes (for builds without `std::fs`).
    pub fn load_bytes(
        bytes: &[u8],
        target_altitude: f32,
        target_airspeed: f32,
    ) -> Result<Self, ModelLoadError> {
        let device = Default::default();
        let record = NamedMpkBytesRecorder::<FullPrecisionSettings>::default()
            .load(bytes.to_vec(), &device)?;
        let model = IntMlpPolicy::<NdArray>::new(&device).load_record(record);
        Self::from_model(model, target_altitude, target_airspeed)
    }

    pub fn weights(&self) -> &IntMlpWeights {
        &self.weights
    }

    pub fn integrators(&self) -> IntegratorState {
        self.integrators
    }
}

impl FlightController for IntMlpLevelHoldController {
    fn update(&mut self, state: &FlightState, _ctx: &ControllerContext, dt: f32) -> ControlInputs {
        let obs = level_hold_observation(state, self.target_altitude, self.target_airspeed);
        self.integrators.step(&obs, dt);
        let action = self
            .weights
            .action(&int_mlp_features(&obs, &self.integrators));
        direct_action_to_inputs(&action)
    }

    fn name(&self) -> &'static str {
        "IntMlpLevelHold"
    }

    fn targets(&self) -> ControllerTargets {
        ControllerTargets::LevelHold {
            altitude: self.target_altitude,
            airspeed: self.target_airspeed,
        }
    }

    /// A changed target restarts that target's integral, mirroring
    /// `InversionLevelHoldController`: the integral trims against one setpoint.
    fn apply_targets(&mut self, targets: &ControllerTargets, _state: &FlightState) {
        if let ControllerTargets::LevelHold { altitude, airspeed } = *targets {
            if altitude != self.target_altitude {
                self.integrators.altitude = 0.0;
            }
            if airspeed != self.target_airspeed {
                self.integrators.speed = 0.0;
            }
            self.target_altitude = altitude;
            self.target_airspeed = airspeed;
        }
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}
