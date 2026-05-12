//! Wu et al. (2025) FC-LSTM-FC actor-critic for recurrent PPO.
//!
//! Architecture (actor and critic each have independent trunks):
//!   fc1:  Linear(obs_dim → 128), tanh
//!   fc2:  Linear(128 → 128),     tanh
//!   lstm: Lstm(128 → 128)
//!   fc3:  Linear(128 → 128),     tanh
//!   out:  Linear(128 → 4)  [actor] / Linear(128 → 1) [critic]
//!
//! Training: process full sequences [batch, seq_len, obs_dim] with BPTT.
//! Inference: single step [1, obs_dim] with carried LstmState.

use burn::{
    module::{Module, Param},
    nn::{Linear, LinearConfig, Lstm, LstmConfig, LstmState},
    tensor::{backend::Backend, Distribution, Shape, Tensor, TensorData},
};

pub const LSTM_HIDDEN: usize = 128;
pub const LSTM_FC: usize = 128;

// ---------------------------------------------------------------------------
// CPU-only hidden state transfer type (no B generic)
// ---------------------------------------------------------------------------

/// LSTM hidden state stored on the CPU between rollout steps.
#[derive(Clone, Debug)]
pub struct LstmHiddenState {
    pub h: Vec<f32>,
    pub c: Vec<f32>,
}

impl Default for LstmHiddenState {
    fn default() -> Self {
        Self {
            h: vec![0.0; LSTM_HIDDEN],
            c: vec![0.0; LSTM_HIDDEN],
        }
    }
}

impl LstmHiddenState {
    /// Convert to a burn `LstmState<B, 2>` with shape `[batch=1, hidden_size]`.
    pub fn to_burn_state<B: Backend>(&self, device: &B::Device) -> LstmState<B, 2> {
        let h_t = Tensor::<B, 2>::from_data(
            TensorData::new(self.h.clone(), vec![1, LSTM_HIDDEN]),
            device,
        );
        let c_t = Tensor::<B, 2>::from_data(
            TensorData::new(self.c.clone(), vec![1, LSTM_HIDDEN]),
            device,
        );
        LstmState::new(c_t, h_t)
    }

    /// Convert from a burn `LstmState<B, 2>` — readback to CPU.
    pub fn from_burn_state<B: Backend>(state: LstmState<B, 2>) -> Self {
        let h = state
            .hidden
            .into_data()
            .to_vec::<f32>()
            .expect("lstm h readback");
        let c = state
            .cell
            .into_data()
            .to_vec::<f32>()
            .expect("lstm c readback");
        Self { h, c }
    }

    /// Build a batched `LstmState<B, 2>` (shape `[n, LSTM_HIDDEN]`) from a slice of per-env states.
    pub fn batch_to_burn<B: Backend>(
        states: &[LstmHiddenState],
        device: &B::Device,
    ) -> LstmState<B, 2> {
        let n = states.len();
        let h_flat: Vec<f32> = states.iter().flat_map(|s| s.h.iter().copied()).collect();
        let c_flat: Vec<f32> = states.iter().flat_map(|s| s.c.iter().copied()).collect();
        let h_t = Tensor::<B, 2>::from_data(TensorData::new(h_flat, vec![n, LSTM_HIDDEN]), device);
        let c_t = Tensor::<B, 2>::from_data(TensorData::new(c_flat, vec![n, LSTM_HIDDEN]), device);
        LstmState::new(c_t, h_t)
    }

    /// Unpack a batched `LstmState<B, 2>` (shape `[n, LSTM_HIDDEN]`) into per-env states.
    pub fn unbatch_from_burn<B: Backend>(state: LstmState<B, 2>, n: usize) -> Vec<LstmHiddenState> {
        let h_all = state
            .hidden
            .into_data()
            .to_vec::<f32>()
            .expect("lstm h readback");
        let c_all = state
            .cell
            .into_data()
            .to_vec::<f32>()
            .expect("lstm c readback");
        (0..n)
            .map(|i| LstmHiddenState {
                h: h_all[i * LSTM_HIDDEN..(i + 1) * LSTM_HIDDEN].to_vec(),
                c: c_all[i * LSTM_HIDDEN..(i + 1) * LSTM_HIDDEN].to_vec(),
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Policy network (actor trunk)
// ---------------------------------------------------------------------------

#[derive(Module, Debug)]
pub struct LstmPolicyNet<B: Backend> {
    fc1: Linear<B>,
    fc2: Linear<B>,
    lstm: Lstm<B>,
    fc3: Linear<B>,
    out: Linear<B>,
}

impl<B: Backend> LstmPolicyNet<B> {
    pub fn new(device: &B::Device, obs_dim: usize) -> Self {
        Self {
            fc1: LinearConfig::new(obs_dim, LSTM_FC).init(device),
            fc2: LinearConfig::new(LSTM_FC, LSTM_FC).init(device),
            lstm: LstmConfig::new(LSTM_FC, LSTM_HIDDEN, true).init(device),
            fc3: LinearConfig::new(LSTM_HIDDEN, LSTM_FC).init(device),
            out: LinearConfig::new(LSTM_FC, 4).init(device),
        }
    }

    /// Forward over a full sequence.
    ///
    /// `obs`: `[batch, seq_len, obs_dim]`
    /// Returns `(pre_tanh_means [batch, seq_len, 4], final_state)`.
    pub fn forward_seq(
        &self,
        obs: Tensor<B, 3>,
        state: Option<LstmState<B, 2>>,
    ) -> (Tensor<B, 3>, LstmState<B, 2>) {
        let [batch, seq_len, obs_dim] = obs.dims();
        let flat = obs.reshape([batch * seq_len, obs_dim]);
        let x = self.fc1.forward(flat).tanh();
        let x = self.fc2.forward(x).tanh();
        let x = x.reshape([batch, seq_len, LSTM_FC]);
        let (lstm_out, final_state) = self.lstm.forward(x, state);
        let flat2 = lstm_out.reshape([batch * seq_len, LSTM_HIDDEN]);
        let x = self.fc3.forward(flat2).tanh();
        let out = self.out.forward(x);
        (out.reshape([batch, seq_len, 4]), final_state)
    }

    /// Single-step forward for inference.
    ///
    /// `obs`: `[batch, obs_dim]`
    /// Returns `(pre_tanh_mean [batch, 4], new_state)`.
    pub fn forward_step(
        &self,
        obs: Tensor<B, 2>,
        state: Option<LstmState<B, 2>>,
    ) -> (Tensor<B, 2>, LstmState<B, 2>) {
        let x = self.fc1.forward(obs).tanh();
        let x = self.fc2.forward(x).tanh();
        let x = x.unsqueeze_dim::<3>(1); // [batch, 1, LSTM_FC]
        let (lstm_out, new_state) = self.lstm.forward(x, state);
        let lstm_out = lstm_out.squeeze_dims::<2>(&[1]); // [batch, LSTM_HIDDEN]
        let x = self.fc3.forward(lstm_out).tanh();
        let out = self.out.forward(x);
        (out, new_state)
    }
}

// ---------------------------------------------------------------------------
// Value network (critic trunk)
// ---------------------------------------------------------------------------

#[derive(Module, Debug)]
pub struct LstmValueNet<B: Backend> {
    fc1: Linear<B>,
    fc2: Linear<B>,
    lstm: Lstm<B>,
    fc3: Linear<B>,
    out: Linear<B>,
}

impl<B: Backend> LstmValueNet<B> {
    pub fn new(device: &B::Device, obs_dim: usize) -> Self {
        Self {
            fc1: LinearConfig::new(obs_dim, LSTM_FC).init(device),
            fc2: LinearConfig::new(LSTM_FC, LSTM_FC).init(device),
            lstm: LstmConfig::new(LSTM_FC, LSTM_HIDDEN, true).init(device),
            fc3: LinearConfig::new(LSTM_HIDDEN, LSTM_FC).init(device),
            out: LinearConfig::new(LSTM_FC, 1).init(device),
        }
    }

    /// Full-sequence forward.
    ///
    /// Returns `(values [batch, seq_len, 1], final_state)`.
    pub fn forward_seq(
        &self,
        obs: Tensor<B, 3>,
        state: Option<LstmState<B, 2>>,
    ) -> (Tensor<B, 3>, LstmState<B, 2>) {
        let [batch, seq_len, obs_dim] = obs.dims();
        let flat = obs.reshape([batch * seq_len, obs_dim]);
        let x = self.fc1.forward(flat).tanh();
        let x = self.fc2.forward(x).tanh();
        let x = x.reshape([batch, seq_len, LSTM_FC]);
        let (lstm_out, final_state) = self.lstm.forward(x, state);
        let flat2 = lstm_out.reshape([batch * seq_len, LSTM_HIDDEN]);
        let x = self.fc3.forward(flat2).tanh();
        let out = self.out.forward(x);
        (out.reshape([batch, seq_len, 1]), final_state)
    }

    /// Single-step forward.
    ///
    /// Returns `(value [batch, 1], new_state)`.
    pub fn forward_step(
        &self,
        obs: Tensor<B, 2>,
        state: Option<LstmState<B, 2>>,
    ) -> (Tensor<B, 2>, LstmState<B, 2>) {
        let x = self.fc1.forward(obs).tanh();
        let x = self.fc2.forward(x).tanh();
        let x = x.unsqueeze_dim::<3>(1);
        let (lstm_out, new_state) = self.lstm.forward(x, state);
        let lstm_out = lstm_out.squeeze_dims::<2>(&[1]);
        let x = self.fc3.forward(lstm_out).tanh();
        let out = self.out.forward(x);
        (out, new_state)
    }
}

// ---------------------------------------------------------------------------
// Actor-Critic
// ---------------------------------------------------------------------------

#[derive(Module, Debug)]
pub struct LstmActorCritic<B: Backend> {
    pub policy: LstmPolicyNet<B>,
    pub value: LstmValueNet<B>,
    /// Learnable log-std, shape [4], init = −0.5.
    pub log_std: Param<Tensor<B, 1>>,
}

impl<B: Backend> LstmActorCritic<B> {
    pub fn new(device: &B::Device, obs_dim: usize) -> Self {
        let log_std_init = Tensor::<B, 1>::full([4], -0.5, device);
        Self {
            policy: LstmPolicyNet::new(device, obs_dim),
            value: LstmValueNet::new(device, obs_dim),
            log_std: Param::from_tensor(log_std_init),
        }
    }

    // -----------------------------------------------------------------------
    // Training: full-sequence methods
    // -----------------------------------------------------------------------

    /// Process full sequences for the PPO update.
    ///
    /// `obs`: `[batch, seq_len, obs_dim]`
    /// Returns `(policy_means [B,T,4], values [B,T,1], final_policy_state, final_value_state)`.
    pub fn forward_sequence(
        &self,
        obs: Tensor<B, 3>,
        policy_state: Option<LstmState<B, 2>>,
        value_state: Option<LstmState<B, 2>>,
    ) -> (Tensor<B, 3>, Tensor<B, 3>, LstmState<B, 2>, LstmState<B, 2>) {
        let (policy_means, p_state) = self.policy.forward_seq(obs.clone(), policy_state);
        let (values, v_state) = self.value.forward_seq(obs, value_state);
        (policy_means, values, p_state, v_state)
    }

    /// Re-evaluate stored actions for PPO loss computation.
    ///
    /// `old_actions`: `[batch, seq_len, 4]` tanh-squashed actions from rollout.
    /// Returns `(log_probs [B*T], entropy [B*T], values [B,T,1])`.
    pub fn evaluate_sequence(
        &self,
        obs: Tensor<B, 3>,
        old_actions: Tensor<B, 3>,
        policy_state: Option<LstmState<B, 2>>,
        value_state: Option<LstmState<B, 2>>,
    ) -> (Tensor<B, 1>, Tensor<B, 1>, Tensor<B, 3>) {
        let [batch, seq_len, _] = obs.dims();
        let bt = batch * seq_len;

        let (pre_tanh_means, values, _, _) = self.forward_sequence(obs, policy_state, value_state);

        let flat_means = pre_tanh_means.reshape([bt, 4]);
        let std = self.log_std.val().unsqueeze::<2>().expand([bt, 4]).exp();

        let flat_actions = old_actions.reshape([bt, 4]);
        let a_clamped = flat_actions.clamp(-1.0_f32 + 1e-6, 1.0_f32 - 1e-6);
        let one = Tensor::<B, 2>::ones_like(&a_clamped);
        let pre_tanh =
            ((one.clone() + a_clamped.clone()) / (one - a_clamped.clone())).log() * 0.5_f32;

        let log_prob = gaussian_log_prob_squashed(pre_tanh, flat_means, std.clone(), a_clamped);

        let log_std_val = self.log_std.val().unsqueeze::<2>().expand([bt, 4]);
        let ln_2pi_half = 0.5_f32 * (1.0 + (2.0 * std::f32::consts::PI).ln());
        let entropy = (log_std_val + ln_2pi_half)
            .sum_dim(1)
            .squeeze_dims::<1>(&[1]);

        (log_prob, entropy, values)
    }

    // -----------------------------------------------------------------------
    // Inference / rollout: single-step methods
    // -----------------------------------------------------------------------

    /// Deterministic action for inference (no sampling noise).
    ///
    /// `obs`: `[batch, obs_dim]`
    /// Returns `(action [batch, 4], new_policy_state)`.
    pub fn mean_action_step(
        &self,
        obs: Tensor<B, 2>,
        state: Option<LstmState<B, 2>>,
    ) -> (Tensor<B, 2>, LstmState<B, 2>) {
        let (pre_tanh, new_state) = self.policy.forward_step(obs, state);
        (pre_tanh.tanh(), new_state)
    }

    /// Stochastic action with reparameterisation for rollout collection.
    ///
    /// Returns `(action [batch, 4], log_prob [batch], new_policy_state)`.
    pub fn sample_action_step(
        &self,
        obs: Tensor<B, 2>,
        state: Option<LstmState<B, 2>>,
    ) -> (Tensor<B, 2>, Tensor<B, 1>, LstmState<B, 2>) {
        let batch = obs.dims()[0];
        let device = self.log_std.val().device();
        let (pre_tanh_mean, new_state) = self.policy.forward_step(obs, state);
        let std = self.log_std.val().unsqueeze::<2>().expand([batch, 4]).exp();
        let noise = Tensor::<B, 2>::random(
            Shape::new([batch, 4]),
            Distribution::Normal(0.0, 1.0),
            &device,
        );
        let pre_tanh_sample = pre_tanh_mean.clone() + noise * std.clone();
        let action = pre_tanh_sample.clone().tanh();
        let log_prob =
            gaussian_log_prob_squashed(pre_tanh_sample, pre_tanh_mean, std, action.clone());
        (action, log_prob, new_state)
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn gaussian_log_prob_squashed<B: Backend>(
    pre_tanh_sample: Tensor<B, 2>,
    pre_tanh_mean: Tensor<B, 2>,
    std: Tensor<B, 2>,
    action: Tensor<B, 2>,
) -> Tensor<B, 1> {
    let diff = (pre_tanh_sample - pre_tanh_mean) / std.clone();
    let log_density = diff.powf_scalar(2.0) * (-0.5_f32)
        - std.log()
        - 0.5_f32 * (2.0 * std::f32::consts::PI).ln();
    // Subtract log|det J| of tanh squashing: log p(a) = log p(u) - log(1 - a²).
    let correction =
        (Tensor::<B, 2>::ones_like(&action) - action.powf_scalar(2.0) + 1e-6_f32).log();
    (log_density - correction)
        .sum_dim(1)
        .squeeze_dims::<1>(&[1])
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use burn::backend::NdArray;

    type B = NdArray;

    fn device() -> <B as Backend>::Device {
        Default::default()
    }

    #[test]
    fn hidden_state_round_trip() {
        let h: Vec<f32> = (0..LSTM_HIDDEN).map(|i| i as f32 * 0.01).collect();
        let c: Vec<f32> = (0..LSTM_HIDDEN).map(|i| -(i as f32) * 0.01).collect();
        let hs = LstmHiddenState {
            h: h.clone(),
            c: c.clone(),
        };
        let burn_state = hs.to_burn_state::<B>(&device());
        let back = LstmHiddenState::from_burn_state(burn_state);
        for (a, b) in h.iter().zip(back.h.iter()) {
            assert!((a - b).abs() < 1e-5, "h mismatch");
        }
        for (a, b) in c.iter().zip(back.c.iter()) {
            assert!((a - b).abs() < 1e-5, "c mismatch");
        }
    }

    #[test]
    fn batch_hidden_state_round_trip() {
        let states: Vec<LstmHiddenState> = (0..4)
            .map(|i| LstmHiddenState {
                h: vec![i as f32; LSTM_HIDDEN],
                c: vec![-(i as f32); LSTM_HIDDEN],
            })
            .collect();
        let batched = LstmHiddenState::batch_to_burn::<B>(&states, &device());
        let unbatched = LstmHiddenState::unbatch_from_burn(batched, 4);
        for (orig, back) in states.iter().zip(unbatched.iter()) {
            assert!((orig.h[0] - back.h[0]).abs() < 1e-5, "h mismatch");
            assert!((orig.c[0] - back.c[0]).abs() < 1e-5, "c mismatch");
        }
    }

    #[test]
    fn forward_sequence_output_shapes() {
        let obs_dim = 13;
        let model = LstmActorCritic::<B>::new(&device(), obs_dim);
        let obs = Tensor::<B, 3>::zeros([2, 8, obs_dim], &device());
        let (means, values, _ps, _vs) = model.forward_sequence(obs, None, None);
        assert_eq!(means.dims(), [2, 8, 4]);
        assert_eq!(values.dims(), [2, 8, 1]);
    }

    #[test]
    fn mean_action_step_shape_and_range() {
        let obs_dim = 13;
        let model = LstmActorCritic::<B>::new(&device(), obs_dim);
        let obs = Tensor::<B, 2>::zeros([1, obs_dim], &device());
        let (action, _state) = model.mean_action_step(obs, None);
        assert_eq!(action.dims(), [1, 4]);
        for v in action.into_data().to_vec::<f32>().unwrap() {
            assert!(v.abs() <= 1.0 + 1e-5, "action outside [-1,1]: {v}");
            assert!(v.is_finite(), "action not finite: {v}");
        }
    }

    #[test]
    fn sample_action_step_shapes() {
        let obs_dim = 13;
        let model = LstmActorCritic::<B>::new(&device(), obs_dim);
        let obs = Tensor::<B, 2>::zeros([4, obs_dim], &device());
        let (action, log_prob, _state) = model.sample_action_step(obs, None);
        assert_eq!(action.dims(), [4, 4]);
        assert_eq!(log_prob.dims(), [4]);
        for v in log_prob.into_data().to_vec::<f32>().unwrap() {
            assert!(v.is_finite(), "log_prob not finite: {v}");
        }
    }

    #[test]
    fn lstm_state_carries_across_steps() {
        let obs_dim = 13;
        let model = LstmActorCritic::<B>::new(&device(), obs_dim);
        let obs = Tensor::<B, 2>::random(
            Shape::new([1, obs_dim]),
            Distribution::Normal(0.0, 1.0),
            &device(),
        );
        let (_, state1) = model.mean_action_step(obs.clone(), None);
        let (a2_with_state, _) = model.mean_action_step(obs.clone(), Some(state1));
        let (a2_zero_state, _) = model.mean_action_step(obs, None);
        let v_with = a2_with_state.into_data().to_vec::<f32>().unwrap();
        let v_zero = a2_zero_state.into_data().to_vec::<f32>().unwrap();
        let any_diff = v_with
            .iter()
            .zip(v_zero.iter())
            .any(|(a, b)| (a - b).abs() > 1e-6);
        assert!(any_diff, "LSTM state should change the output");
    }

    #[test]
    fn evaluate_sequence_shapes() {
        let obs_dim = 13;
        let model = LstmActorCritic::<B>::new(&device(), obs_dim);
        let obs = Tensor::<B, 3>::zeros([2, 8, obs_dim], &device());
        let actions = Tensor::<B, 3>::random(
            Shape::new([2, 8, 4]),
            Distribution::Uniform(-0.9, 0.9),
            &device(),
        );
        let (log_probs, entropy, values) = model.evaluate_sequence(obs, actions, None, None);
        assert_eq!(log_probs.dims(), [16]); // B*T = 2*8
        assert_eq!(entropy.dims(), [16]);
        assert_eq!(values.dims(), [2, 8, 1]);
    }

    #[test]
    fn evaluate_sequence_values_finite() {
        let obs_dim = 13;
        let model = LstmActorCritic::<B>::new(&device(), obs_dim);
        let obs = Tensor::<B, 3>::random(
            Shape::new([2, 4, obs_dim]),
            Distribution::Normal(0.0, 1.0),
            &device(),
        );
        let actions = Tensor::<B, 3>::random(
            Shape::new([2, 4, 4]),
            Distribution::Uniform(-0.9, 0.9),
            &device(),
        );
        let (lp, ent, vals) = model.evaluate_sequence(obs, actions, None, None);
        for v in lp.into_data().to_vec::<f32>().unwrap() {
            assert!(v.is_finite(), "log_prob not finite: {v}");
        }
        for v in ent.into_data().to_vec::<f32>().unwrap() {
            assert!(v.is_finite(), "entropy not finite: {v}");
        }
        for v in vals.into_data().to_vec::<f32>().unwrap() {
            assert!(v.is_finite(), "value not finite: {v}");
        }
    }

    #[test]
    fn log_std_initialised_to_neg_half() {
        let model = LstmActorCritic::<B>::new(&device(), 13);
        let vals = model.log_std.val().into_data().to_vec::<f32>().unwrap();
        for v in vals {
            assert!((v - (-0.5_f32)).abs() < 1e-5, "log_std init wrong: {v}");
        }
    }

    #[test]
    fn value_step_shape() {
        let obs_dim = 13;
        let model = LstmActorCritic::<B>::new(&device(), obs_dim);
        let obs = Tensor::<B, 2>::zeros([3, obs_dim], &device());
        let (val, _) = model.value.forward_step(obs, None);
        assert_eq!(val.dims(), [3, 1]);
    }
}
