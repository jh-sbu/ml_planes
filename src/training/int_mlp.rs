//! IntMLP: the level-hold network whose memory is two clamped error integrators.
//!
//! Promoted from `experiments/nn_arch`. A memoryless policy on the 13-element
//! level-hold observation converges to a trim *bias* it cannot null. IntMLP carries
//! pure integrators of the altitude and airspeed errors and feeds them to the network,
//! so any stable equilibrium of the closed loop must sit at zero error: the
//! integrators are only stationary there, however imprecisely the network fits.
//!
//! This module is the single definition of that contract — integrator update,
//! feature vector, and the flat forward pass — shared by the controller, the
//! DAgger/ES trainer and `evaluate_policy`, the same way `level_hold_observation`
//! is shared between `LevelHoldEnv` and `RlLevelHoldController`. It deliberately
//! has no `burn` dependency: the ES stage perturbs [`IntMlpWeights`] directly and
//! rolls out thousands of forward passes per step, where tensor overhead dominates.
//!
//! Network: `pre = out(tanh(fc2(tanh(fc1(x))))) + skip(x)`, `action = tanh(pre)`, with
//! `x = [obs(13), alt_integral, speed_integral] * INPUT_SCALE`.

use crate::training::level_hold_env::LEVEL_HOLD_OBS_DIM;

/// Network input width: the level-hold observation plus the two integrators.
pub const INT_MLP_INPUT_DIM: usize = LEVEL_HOLD_OBS_DIM + 2;
/// Width of both hidden layers.
pub const INT_MLP_HIDDEN: usize = 64;
/// `[elevator, throttle_norm, aileron, rudder]`.
pub const INT_MLP_ACTION_DIM: usize = 4;

/// Anti-windup clamp on the altitude-error integral [m·s].
pub const ALT_INTEGRAL_CLAMP: f32 = 30.0;
/// Anti-windup clamp on the airspeed-error integral [m].
pub const SPEED_INTEGRAL_CLAMP: f32 = 20.0;

/// Fixed diagonal input scale. The two error channels are boosted ×10 because at
/// steady state `obs[0] = alt_err / 200` is ~1e-5; the integrators are shrunk to
/// keep their clamped range near unit scale.
pub const INPUT_SCALE: [f32; INT_MLP_INPUT_DIM] = [
    10.0, 10.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.1, 0.2,
];

/// The policy's memory: integrals of the altitude and airspeed errors.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct IntegratorState {
    /// ∫ (altitude − target) dt, clamped to ±[`ALT_INTEGRAL_CLAMP`].
    pub altitude: f32,
    /// ∫ (airspeed − target) dt, clamped to ±[`SPEED_INTEGRAL_CLAMP`].
    pub speed: f32,
}

impl IntegratorState {
    /// Advance both integrals by one step, reading the errors out of a level-hold
    /// observation. Called *before* the forward pass on the same observation.
    pub fn step(&mut self, obs: &[f32], dt: f32) {
        let alt_err = obs[0] * 200.0;
        let speed_err = obs[1] * 50.0;
        self.altitude =
            (self.altitude + alt_err * dt).clamp(-ALT_INTEGRAL_CLAMP, ALT_INTEGRAL_CLAMP);
        self.speed =
            (self.speed + speed_err * dt).clamp(-SPEED_INTEGRAL_CLAMP, SPEED_INTEGRAL_CLAMP);
    }
}

/// Assemble the scaled network input from an observation and the (already stepped)
/// integrators.
pub fn int_mlp_features(obs: &[f32], state: &IntegratorState) -> [f32; INT_MLP_INPUT_DIM] {
    let mut x = [0.0; INT_MLP_INPUT_DIM];
    x[..LEVEL_HOLD_OBS_DIM].copy_from_slice(&obs[..LEVEL_HOLD_OBS_DIM]);
    x[LEVEL_HOLD_OBS_DIM] = state.altitude;
    x[LEVEL_HOLD_OBS_DIM + 1] = state.speed;
    for (v, s) in x.iter_mut().zip(INPUT_SCALE) {
        *v *= s;
    }
    x
}

/// One parameter tensor of the flat layout, in storage order.
///
/// Weights are stored row-major as `[out, in]` (PyTorch's convention, so the order
/// matches `experiments/nn_arch`'s `named_parameters()`); burn stores `Linear`
/// weights as `[in, out]`, so conversions transpose.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntMlpBlock {
    Fc1Weight,
    Fc1Bias,
    Fc2Weight,
    Fc2Bias,
    OutWeight,
    OutBias,
    SkipWeight,
    SkipBias,
}

impl IntMlpBlock {
    pub const ALL: [IntMlpBlock; 8] = [
        Self::Fc1Weight,
        Self::Fc1Bias,
        Self::Fc2Weight,
        Self::Fc2Bias,
        Self::OutWeight,
        Self::OutBias,
        Self::SkipWeight,
        Self::SkipBias,
    ];

    /// `(rows, cols)`; biases are `(n, 1)`.
    pub fn shape(self) -> (usize, usize) {
        match self {
            Self::Fc1Weight => (INT_MLP_HIDDEN, INT_MLP_INPUT_DIM),
            Self::Fc1Bias | Self::Fc2Bias => (INT_MLP_HIDDEN, 1),
            Self::Fc2Weight => (INT_MLP_HIDDEN, INT_MLP_HIDDEN),
            Self::OutWeight => (INT_MLP_ACTION_DIM, INT_MLP_HIDDEN),
            Self::OutBias | Self::SkipBias => (INT_MLP_ACTION_DIM, 1),
            Self::SkipWeight => (INT_MLP_ACTION_DIM, INT_MLP_INPUT_DIM),
        }
    }

    pub fn len(self) -> usize {
        let (r, c) = self.shape();
        r * c
    }

    /// Index range of this block inside the flat vector.
    pub fn range(self) -> std::ops::Range<usize> {
        let start: usize = Self::ALL
            .iter()
            .take_while(|b| **b != self)
            .map(|b| b.len())
            .sum();
        start..start + self.len()
    }
}

/// All 5,508 IntMLP parameters as one flat vector.
#[derive(Debug, Clone, PartialEq)]
pub struct IntMlpWeights {
    params: Vec<f32>,
}

impl IntMlpWeights {
    /// Total parameter count of the flat layout.
    pub const N_PARAMS: usize = INT_MLP_HIDDEN * INT_MLP_INPUT_DIM
        + INT_MLP_HIDDEN
        + INT_MLP_HIDDEN * INT_MLP_HIDDEN
        + INT_MLP_HIDDEN
        + INT_MLP_ACTION_DIM * INT_MLP_HIDDEN
        + INT_MLP_ACTION_DIM
        + INT_MLP_ACTION_DIM * INT_MLP_INPUT_DIM
        + INT_MLP_ACTION_DIM;

    pub fn zeros() -> Self {
        Self {
            params: vec![0.0; Self::N_PARAMS],
        }
    }

    /// Wrap a flat vector, rejecting one of the wrong length.
    pub fn from_vec(params: Vec<f32>) -> Result<Self, String> {
        if params.len() != Self::N_PARAMS {
            return Err(format!(
                "IntMLP expects {} parameters, got {}",
                Self::N_PARAMS,
                params.len()
            ));
        }
        Ok(Self { params })
    }

    pub fn as_slice(&self) -> &[f32] {
        &self.params
    }

    pub fn as_mut_slice(&mut self) -> &mut [f32] {
        &mut self.params
    }

    pub fn into_vec(self) -> Vec<f32> {
        self.params
    }

    pub fn block(&self, block: IntMlpBlock) -> &[f32] {
        &self.params[block.range()]
    }

    pub fn block_mut(&mut self, block: IntMlpBlock) -> &mut [f32] {
        &mut self.params[block.range()]
    }

    /// Pre-tanh network output for one scaled feature vector.
    pub fn forward_pre_tanh(&self, x: &[f32; INT_MLP_INPUT_DIM]) -> [f32; INT_MLP_ACTION_DIM] {
        let mut h1 = [0.0; INT_MLP_HIDDEN];
        affine(
            self.block(IntMlpBlock::Fc1Weight),
            self.block(IntMlpBlock::Fc1Bias),
            x,
            &mut h1,
        );
        h1.iter_mut().for_each(|v| *v = v.tanh());
        let mut h2 = [0.0; INT_MLP_HIDDEN];
        affine(
            self.block(IntMlpBlock::Fc2Weight),
            self.block(IntMlpBlock::Fc2Bias),
            &h1,
            &mut h2,
        );
        h2.iter_mut().for_each(|v| *v = v.tanh());
        let mut out = [0.0; INT_MLP_ACTION_DIM];
        affine(
            self.block(IntMlpBlock::OutWeight),
            self.block(IntMlpBlock::OutBias),
            &h2,
            &mut out,
        );
        let mut skip = [0.0; INT_MLP_ACTION_DIM];
        affine(
            self.block(IntMlpBlock::SkipWeight),
            self.block(IntMlpBlock::SkipBias),
            x,
            &mut skip,
        );
        for (o, s) in out.iter_mut().zip(skip) {
            *o += s;
        }
        out
    }

    /// Deterministic action `tanh(pre)` in `[-1, 1]`.
    pub fn action(&self, x: &[f32; INT_MLP_INPUT_DIM]) -> [f32; INT_MLP_ACTION_DIM] {
        self.forward_pre_tanh(x).map(f32::tanh)
    }
}

/// `y = W·x + b` with `W` row-major `[y.len(), x.len()]`.
fn affine(w: &[f32], b: &[f32], x: &[f32], y: &mut [f32]) {
    let cols = x.len();
    for (r, out) in y.iter_mut().enumerate() {
        let row = &w[r * cols..(r + 1) * cols];
        *out = row.iter().zip(x).map(|(wi, xi)| wi * xi).sum::<f32>() + b[r];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plane::PHYSICS_DT;

    fn obs_with(alt_err_norm: f32, speed_err_norm: f32) -> Vec<f32> {
        let mut obs = vec![0.0; LEVEL_HOLD_OBS_DIM];
        obs[0] = alt_err_norm;
        obs[1] = speed_err_norm;
        obs
    }

    #[test]
    fn integrator_step_integrates_denormalised_errors() {
        let mut s = IntegratorState::default();
        // obs[0] = 0.5 ⇒ +100 m; obs[1] = -0.2 ⇒ -10 m/s.
        s.step(&obs_with(0.5, -0.2), PHYSICS_DT);
        assert!((s.altitude - 100.0 * PHYSICS_DT).abs() < 1e-6, "{s:?}");
        assert!((s.speed - -10.0 * PHYSICS_DT).abs() < 1e-6, "{s:?}");
        s.step(&obs_with(0.5, -0.2), PHYSICS_DT);
        assert!((s.altitude - 200.0 * PHYSICS_DT).abs() < 1e-5, "{s:?}");
    }

    #[test]
    fn integrators_clamp_against_windup() {
        let mut s = IntegratorState::default();
        for _ in 0..10_000 {
            s.step(&obs_with(2.0, -2.0), PHYSICS_DT);
        }
        assert_eq!(s.altitude, ALT_INTEGRAL_CLAMP);
        assert_eq!(s.speed, -SPEED_INTEGRAL_CLAMP);
        for _ in 0..10_000 {
            s.step(&obs_with(-2.0, 2.0), PHYSICS_DT);
        }
        assert_eq!(s.altitude, -ALT_INTEGRAL_CLAMP);
        assert_eq!(s.speed, SPEED_INTEGRAL_CLAMP);
    }

    #[test]
    fn features_append_integrators_and_apply_the_input_scale() {
        let obs: Vec<f32> = (0..LEVEL_HOLD_OBS_DIM).map(|i| i as f32 + 1.0).collect();
        let state = IntegratorState {
            altitude: 5.0,
            speed: -2.0,
        };
        let x = int_mlp_features(&obs, &state);
        for i in 0..LEVEL_HOLD_OBS_DIM {
            assert_eq!(x[i], obs[i] * INPUT_SCALE[i], "feature {i}");
        }
        assert_eq!(x[13], 5.0 * 0.1);
        assert_eq!(x[14], -2.0 * 0.2);
    }

    #[test]
    fn flat_layout_holds_5508_parameters() {
        assert_eq!(IntMlpWeights::N_PARAMS, 5508);
        let total: usize = IntMlpBlock::ALL
            .iter()
            .map(|b| b.shape().0 * b.shape().1)
            .sum();
        assert_eq!(total, IntMlpWeights::N_PARAMS);
        assert_eq!(IntMlpWeights::zeros().as_slice().len(), 5508);
        assert!(IntMlpWeights::from_vec(vec![0.0; 5507]).is_err());
        assert!(IntMlpWeights::from_vec(vec![0.0; 5508]).is_ok());
    }

    #[test]
    fn zero_network_commands_zero() {
        let w = IntMlpWeights::zeros();
        let x = [0.7; INT_MLP_INPUT_DIM];
        assert_eq!(w.action(&x), [0.0; 4]);
    }

    #[test]
    fn skip_path_is_row_major_out_by_in() {
        let mut w = IntMlpWeights::zeros();
        // skip.weight[row 2 (aileron), col 13 (alt integral)] = 0.5; skip.bias[3] = -0.25
        w.block_mut(IntMlpBlock::SkipWeight)[2 * INT_MLP_INPUT_DIM + 13] = 0.5;
        w.block_mut(IntMlpBlock::SkipBias)[3] = -0.25;
        let mut x = [0.0; INT_MLP_INPUT_DIM];
        x[13] = 0.4;
        let pre = w.forward_pre_tanh(&x);
        assert_eq!(pre, [0.0, 0.0, 0.2, -0.25]);
        assert_eq!(w.action(&x), [0.0, 0.0, 0.2f32.tanh(), (-0.25f32).tanh()]);
    }

    #[test]
    fn trunk_path_applies_tanh_between_layers() {
        let mut w = IntMlpWeights::zeros();
        w.block_mut(IntMlpBlock::Fc1Bias)[5] = 0.3; // h1[5] = tanh(0.3)
        w.block_mut(IntMlpBlock::Fc2Weight)[7 * INT_MLP_HIDDEN + 5] = 1.5; // h2[7] = tanh(1.5·h1[5])
        w.block_mut(IntMlpBlock::OutWeight)[1 * INT_MLP_HIDDEN + 7] = 2.0;
        w.block_mut(IntMlpBlock::OutBias)[1] = 0.1;
        let pre = w.forward_pre_tanh(&[0.0; INT_MLP_INPUT_DIM]);
        let expected = 2.0 * (1.5 * 0.3f32.tanh()).tanh() + 0.1;
        assert!((pre[1] - expected).abs() < 1e-7, "{pre:?} vs {expected}");
        assert_eq!([pre[0], pre[2], pre[3]], [0.0; 3]);
    }
}
