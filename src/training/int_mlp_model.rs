//! burn module for the IntMLP level-hold network (see [`crate::training::int_mlp`]).
//!
//! This is the *persistence and gradient* form of the network: DAgger fits it with
//! autodiff, and checkpoints are its `.mpk` records. Inference and ES rollouts use
//! the flat [`IntMlpWeights`] kernel instead, reached through [`IntMlpPolicy::to_weights`].
//! The two forward passes are pinned against each other by `tests/rl/int_mlp.rs`.

use burn::{
    module::{Initializer, Module, Param},
    nn::Linear,
    tensor::{backend::Backend, Tensor, TensorData},
};

use crate::training::int_mlp::{
    IntMlpBlock, IntMlpWeights, INT_MLP_ACTION_DIM, INT_MLP_HIDDEN, INT_MLP_INPUT_DIM,
};

/// `pre = out(tanh(fc2(tanh(fc1(x))))) + skip(x)` over the 15 scaled IntMLP features.
///
/// Fields are public so tests can assemble deliberately mis-shaped checkpoints.
#[derive(Module, Debug)]
pub struct IntMlpPolicy<B: Backend> {
    pub fc1: Linear<B>,
    pub fc2: Linear<B>,
    pub out: Linear<B>,
    pub skip: Linear<B>,
}

impl<B: Backend> IntMlpPolicy<B> {
    /// Fresh network with the experiment's init: orthogonal weights (gain √2 on the
    /// hidden layers, 0.01 on the head), zero biases, and an all-zero skip path.
    pub fn new(device: &B::Device) -> Self {
        let _guard = crate::training::ppo::rng_lock();
        Self::new_locked(device)
    }

    /// [`Self::new`] with the backend RNG seeded first, under `ppo::rng_lock()`.
    pub fn new_seeded(device: &B::Device, seed: u64) -> Self {
        let _guard = crate::training::ppo::rng_lock();
        B::seed(device, seed);
        Self::new_locked(device)
    }

    /// Shared body of `new`/`new_seeded`; the caller holds `rng_lock()`.
    fn new_locked(device: &B::Device) -> Self {
        let sqrt2 = std::f64::consts::SQRT_2;
        let model = Self {
            fc1: orthogonal_linear(INT_MLP_INPUT_DIM, INT_MLP_HIDDEN, sqrt2, device),
            fc2: orthogonal_linear(INT_MLP_HIDDEN, INT_MLP_HIDDEN, sqrt2, device),
            out: orthogonal_linear(INT_MLP_HIDDEN, INT_MLP_ACTION_DIM, 0.01, device),
            skip: zero_linear(INT_MLP_INPUT_DIM, INT_MLP_ACTION_DIM, device),
        };
        // burn defers a `Param`'s random draw to first access; force it now, while
        // the lock (and any seed) is still in effect. See `ActorCritic::new_locked`.
        let _ = model.forward(Tensor::zeros([1, INT_MLP_INPUT_DIM], device));
        model
    }

    /// Pre-tanh output for a `[batch, 15]` batch of scaled features.
    pub fn forward(&self, x: Tensor<B, 2>) -> Tensor<B, 2> {
        let h = self.fc1.forward(x.clone()).tanh();
        let h = self.fc2.forward(h).tanh();
        self.out.forward(h) + self.skip.forward(x)
    }

    /// Input width this network expects, read from `fc1` (burn stores `[in, out]`).
    /// After loading a record this is the *checkpoint's* width.
    pub fn input_dim(&self) -> usize {
        self.fc1.weight.val().dims()[0]
    }

    /// Copy the parameters into the flat inference kernel. Errors if any tensor's
    /// shape does not match the IntMLP layout (a mis-shaped checkpoint).
    pub fn to_weights(&self) -> Result<IntMlpWeights, String> {
        let mut w = IntMlpWeights::zeros();
        let layers = [
            (
                &self.fc1,
                IntMlpBlock::Fc1Weight,
                IntMlpBlock::Fc1Bias,
                "fc1",
            ),
            (
                &self.fc2,
                IntMlpBlock::Fc2Weight,
                IntMlpBlock::Fc2Bias,
                "fc2",
            ),
            (
                &self.out,
                IntMlpBlock::OutWeight,
                IntMlpBlock::OutBias,
                "out",
            ),
            (
                &self.skip,
                IntMlpBlock::SkipWeight,
                IntMlpBlock::SkipBias,
                "skip",
            ),
        ];
        for (layer, wb, bb, name) in layers {
            let (rows, cols) = wb.shape();
            let weight = layer.weight.val();
            if weight.dims() != [cols, rows] {
                return Err(format!(
                    "{name}.weight has shape {:?}, IntMLP needs [{cols}, {rows}]",
                    weight.dims()
                ));
            }
            // burn [in, out] -> flat row-major [out, in]
            let src = weight
                .into_data()
                .to_vec::<f32>()
                .map_err(|e| format!("{e:?}"))?;
            let dst = w.block_mut(wb);
            for r in 0..rows {
                for c in 0..cols {
                    dst[r * cols + c] = src[c * rows + r];
                }
            }
            let bias = layer
                .bias
                .as_ref()
                .ok_or_else(|| format!("{name} has no bias"))?
                .val()
                .into_data()
                .to_vec::<f32>()
                .map_err(|e| format!("{e:?}"))?;
            if bias.len() != rows {
                return Err(format!(
                    "{name}.bias has {} entries, IntMLP needs {rows}",
                    bias.len()
                ));
            }
            w.block_mut(bb).copy_from_slice(&bias);
        }
        Ok(w)
    }

    /// Build a module holding exactly `weights`.
    pub fn from_weights(weights: &IntMlpWeights, device: &B::Device) -> Self {
        let layer = |wb: IntMlpBlock, bb: IntMlpBlock| {
            let (rows, cols) = wb.shape();
            let src = weights.block(wb);
            // flat row-major [out, in] -> burn [in, out]
            let mut burn_order = vec![0.0; rows * cols];
            for r in 0..rows {
                for c in 0..cols {
                    burn_order[c * rows + r] = src[r * cols + c];
                }
            }
            Linear {
                weight: Param::from_tensor(Tensor::from_data(
                    TensorData::new(burn_order, vec![cols, rows]),
                    device,
                )),
                bias: Some(Param::from_tensor(Tensor::from_data(
                    TensorData::new(weights.block(bb).to_vec(), vec![rows]),
                    device,
                ))),
            }
        };
        Self {
            fc1: layer(IntMlpBlock::Fc1Weight, IntMlpBlock::Fc1Bias),
            fc2: layer(IntMlpBlock::Fc2Weight, IntMlpBlock::Fc2Bias),
            out: layer(IntMlpBlock::OutWeight, IntMlpBlock::OutBias),
            skip: layer(IntMlpBlock::SkipWeight, IntMlpBlock::SkipBias),
        }
    }
}

/// A `Linear` with orthogonal weights of the given gain and a zero bias. Built by
/// hand because `LinearConfig` would also run the initializer on the 1-D bias.
fn orthogonal_linear<B: Backend>(
    d_input: usize,
    d_output: usize,
    gain: f64,
    device: &B::Device,
) -> Linear<B> {
    Linear {
        weight: Initializer::Orthogonal { gain }.init_with(
            [d_input, d_output],
            Some(d_input),
            Some(d_output),
            device,
        ),
        bias: Some(Param::from_tensor(Tensor::zeros([d_output], device))),
    }
}

fn zero_linear<B: Backend>(d_input: usize, d_output: usize, device: &B::Device) -> Linear<B> {
    Linear {
        weight: Param::from_tensor(Tensor::zeros([d_input, d_output], device)),
        bias: Some(Param::from_tensor(Tensor::zeros([d_output], device))),
    }
}
