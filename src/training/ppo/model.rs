//! Actor-Critic neural network for PPO.
//!
//! Both networks are 2-hidden-layer MLPs with tanh activations.
//! The policy outputs pre-tanh mean values; actions are squashed via tanh
//! when needed. log_std is a learnable [4] parameter shared across the batch.

use burn::{
    module::{Module, Param},
    nn::{Linear, LinearConfig},
    tensor::{backend::Backend, Distribution, Shape, Tensor},
};

// ---------------------------------------------------------------------------
// Policy network
// ---------------------------------------------------------------------------

#[derive(Module, Debug)]
pub struct PolicyNet<B: Backend> {
    fc1: Linear<B>,
    fc2: Linear<B>,
    out: Linear<B>,
}

impl<B: Backend> PolicyNet<B> {
    pub fn new(device: &B::Device, obs_dim: usize) -> Self {
        Self {
            fc1: LinearConfig::new(obs_dim, 64).init(device),
            fc2: LinearConfig::new(64, 64).init(device),
            out: LinearConfig::new(64, 4).init(device),
        }
    }

    /// Returns raw (pre-tanh) mean — unbounded output.
    pub fn forward(&self, obs: Tensor<B, 2>) -> Tensor<B, 2> {
        let x = self.fc1.forward(obs).tanh();
        let x = self.fc2.forward(x).tanh();
        self.out.forward(x)
    }

    /// Observation dimension this network expects, read from the first layer's
    /// weight shape `[obs_dim, 64]`. After `load_record`/`load_file` this reflects
    /// the *loaded* checkpoint's dimension, so it can be validated against the
    /// dimension the caller will actually feed.
    pub fn input_dim(&self) -> usize {
        self.fc1.weight.val().dims()[0]
    }
}

// ---------------------------------------------------------------------------
// Value network
// ---------------------------------------------------------------------------

#[derive(Module, Debug)]
pub struct ValueNet<B: Backend> {
    fc1: Linear<B>,
    fc2: Linear<B>,
    out: Linear<B>,
}

impl<B: Backend> ValueNet<B> {
    pub fn new(device: &B::Device, obs_dim: usize) -> Self {
        Self {
            fc1: LinearConfig::new(obs_dim, 64).init(device),
            fc2: LinearConfig::new(64, 64).init(device),
            out: LinearConfig::new(64, 1).init(device),
        }
    }

    /// Returns value estimate, shape [batch, 1].
    pub fn forward(&self, obs: Tensor<B, 2>) -> Tensor<B, 2> {
        let x = self.fc1.forward(obs).tanh();
        let x = self.fc2.forward(x).tanh();
        self.out.forward(x)
    }
}

// ---------------------------------------------------------------------------
// Actor-Critic
// ---------------------------------------------------------------------------

#[derive(Module, Debug)]
pub struct ActorCritic<B: Backend> {
    pub policy: PolicyNet<B>,
    pub value: ValueNet<B>,
    /// Learnable log-std shared across batch, shape [4], init = -0.5.
    pub log_std: Param<Tensor<B, 1>>,
}

impl<B: Backend> ActorCritic<B> {
    pub fn new(device: &B::Device, obs_dim: usize) -> Self {
        let log_std_init = Tensor::<B, 1>::full([4], -0.5, device);
        Self {
            policy: PolicyNet::new(device, obs_dim),
            value: ValueNet::new(device, obs_dim),
            log_std: Param::from_tensor(log_std_init),
        }
    }

    /// Observation dimension the policy network expects (see [`PolicyNet::input_dim`]).
    pub fn input_dim(&self) -> usize {
        self.policy.input_dim()
    }

    /// Deterministic action = tanh(policy_mean). Used for inference.
    pub fn mean_action(&self, obs: Tensor<B, 2>) -> Tensor<B, 2> {
        self.policy.forward(obs).tanh()
    }

    /// Sample action with reparameterisation and compute its log-prob.
    ///
    /// Returns (action [batch,4], log_prob [batch]).
    pub fn sample_action(&self, obs: Tensor<B, 2>) -> (Tensor<B, 2>, Tensor<B, 1>) {
        let batch = obs.dims()[0];
        let device = self.log_std.val().device();

        let pre_tanh_mean = self.policy.forward(obs);
        let std = self
            .log_std
            .val()
            .unsqueeze::<2>() // [1, 4]
            .expand([batch, 4])
            .exp();

        let noise = Tensor::<B, 2>::random(
            Shape::new([batch, 4]),
            Distribution::Normal(0.0, 1.0),
            &device,
        );
        let pre_tanh_sample = pre_tanh_mean.clone() + noise * std.clone();
        let action = pre_tanh_sample.clone().tanh();
        let log_prob =
            gaussian_log_prob_squashed(pre_tanh_sample, pre_tanh_mean, std, action.clone());
        (action, log_prob)
    }

    /// Evaluate stored actions: compute (log_prob [batch], entropy [batch]).
    ///
    /// `old_actions` are tanh-squashed values in [-1, 1] as stored in the rollout.
    pub fn evaluate_actions(
        &self,
        obs: Tensor<B, 2>,
        old_actions: Tensor<B, 2>,
    ) -> (Tensor<B, 1>, Tensor<B, 1>) {
        let batch = obs.dims()[0];
        let pre_tanh_mean = self.policy.forward(obs);
        let std = self.log_std.val().unsqueeze::<2>().expand([batch, 4]).exp();

        // Recover pre-tanh value from stored actions via atanh.
        // Clamp to avoid atanh(±1) = ±∞.
        let a_clamped = old_actions.clamp(-1.0_f32 + 1e-6, 1.0_f32 - 1e-6);
        // atanh(a) = 0.5 * ln((1+a)/(1-a))
        let one = Tensor::<B, 2>::ones_like(&a_clamped);
        let pre_tanh =
            ((one.clone() + a_clamped.clone()) / (one - a_clamped.clone())).log() * 0.5_f32;

        let log_prob = gaussian_log_prob_squashed(pre_tanh, pre_tanh_mean, std.clone(), a_clamped);

        // Entropy of the pre-squash Gaussian (closed form):
        //   H = sum_i [ 0.5 * (1 + ln(2π)) + ln(σ_i) ]
        let log_std_val = self.log_std.val().unsqueeze::<2>().expand([batch, 4]);
        let ln_2pi_half = 0.5_f32 * (1.0 + (2.0 * std::f32::consts::PI).ln());
        let entropy = (log_std_val + ln_2pi_half)
            .sum_dim(1)
            .squeeze_dims::<1>(&[1]);

        (log_prob, entropy)
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Log-prob of tanh-squashed Gaussian.
///
/// log π(a|s) = Σ_i [ -0.5·((u_i - μ_i)/σ_i)² - ln(σ_i) - 0.5·ln(2π)
///                    - ln(1 - a_i² + ε) ]
///
/// where u = pre_tanh_sample, a = tanh(u).
fn gaussian_log_prob_squashed<B: Backend>(
    pre_tanh_sample: Tensor<B, 2>,
    pre_tanh_mean: Tensor<B, 2>,
    std: Tensor<B, 2>,
    action: Tensor<B, 2>,
) -> Tensor<B, 1> {
    // Gaussian log-density (per element)
    let diff = (pre_tanh_sample - pre_tanh_mean) / std.clone();
    let log_density = diff.powf_scalar(2.0) * (-0.5_f32)
        - std.log()
        - 0.5_f32 * (2.0 * std::f32::consts::PI).ln();

    // Tanh-squash correction (per element): -ln(1 - a² + ε)
    let correction =
        (Tensor::<B, 2>::ones_like(&action) - action.powf_scalar(2.0) + 1e-6_f32).log();

    // Sum over action dim → [batch]
    (log_density + correction)
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
    fn actor_critic_output_shapes() {
        let model = ActorCritic::<B>::new(&device(), 10);
        let obs = Tensor::<B, 2>::zeros([4, 10], &device());
        let (action, log_prob) = model.sample_action(obs.clone());
        assert_eq!(action.dims(), [4, 4]);
        assert_eq!(log_prob.dims(), [4]);
        let value = model.value.forward(obs).squeeze_dims::<1>(&[1]);
        assert_eq!(value.dims(), [4]);
    }

    #[test]
    fn actor_critic_accepts_dynamic_obs_dim() {
        let model = ActorCritic::<B>::new(&device(), 12);
        let obs = Tensor::<B, 2>::zeros([3, 12], &device());
        let (action, log_prob) = model.sample_action(obs.clone());
        assert_eq!(action.dims(), [3, 4]);
        assert_eq!(log_prob.dims(), [3]);
        let value = model.value.forward(obs).squeeze_dims::<1>(&[1]);
        assert_eq!(value.dims(), [3]);
    }

    #[test]
    fn policy_output_in_range() {
        let model = ActorCritic::<B>::new(&device(), 10);
        let obs = Tensor::<B, 2>::random([8, 10], Distribution::Normal(0.0, 1.0), &device());
        let (action, _) = model.sample_action(obs);
        let data = action.into_data().to_vec::<f32>().unwrap();
        for v in &data {
            assert!(v.abs() <= 1.0 + 1e-5, "action out of [-1,1]: {v}");
            assert!(v.is_finite(), "action is NaN/inf: {v}");
        }
    }

    #[test]
    fn input_dim_reports_construction_dim() {
        assert_eq!(ActorCritic::<B>::new(&device(), 13).input_dim(), 13);
        assert_eq!(ActorCritic::<B>::new(&device(), 14).input_dim(), 14);
    }

    #[test]
    fn input_dim_reflects_loaded_record_dim() {
        use burn::record::{FullPrecisionSettings, NamedMpkBytesRecorder, Recorder};

        // Save a 13-dim model, then load it into a freshly-built 14-dim model.
        // `load_record` adopts the saved shape, so `input_dim` must report 13.
        let saved = ActorCritic::<B>::new(&device(), 13);
        let recorder = NamedMpkBytesRecorder::<FullPrecisionSettings>::default();
        let bytes = recorder
            .record(saved.into_record(), ())
            .expect("record to bytes");
        let record = recorder.load(bytes, &device()).expect("load record");
        let loaded = ActorCritic::<B>::new(&device(), 14).load_record(record);
        assert_eq!(loaded.input_dim(), 13);
    }

    #[test]
    fn log_std_init() {
        let model = ActorCritic::<B>::new(&device(), 10);
        let vals = model.log_std.val().into_data().to_vec::<f32>().unwrap();
        assert_eq!(vals.len(), 4);
        for v in vals {
            assert!((v - (-0.5_f32)).abs() < 1e-5, "log_std init wrong: {v}");
        }
    }

    #[test]
    fn evaluate_actions_shapes() {
        let model = ActorCritic::<B>::new(&device(), 10);
        let obs = Tensor::<B, 2>::zeros([4, 10], &device());
        let (action, _) = model.sample_action(obs.clone());
        let (log_prob, entropy) = model.evaluate_actions(obs, action);
        assert_eq!(log_prob.dims(), [4]);
        assert_eq!(entropy.dims(), [4]);
    }

    #[test]
    fn log_prob_is_finite() {
        let model = ActorCritic::<B>::new(&device(), 10);
        let obs = Tensor::<B, 2>::random([4, 10], Distribution::Normal(0.0, 1.0), &device());
        let (action, log_prob) = model.sample_action(obs.clone());
        let (log_prob2, entropy) = model.evaluate_actions(obs, action);
        for v in log_prob.into_data().to_vec::<f32>().unwrap() {
            assert!(v.is_finite(), "sample log_prob not finite: {v}");
        }
        for v in log_prob2.into_data().to_vec::<f32>().unwrap() {
            assert!(v.is_finite(), "evaluate log_prob not finite: {v}");
        }
        for v in entropy.into_data().to_vec::<f32>().unwrap() {
            assert!(v.is_finite(), "entropy not finite: {v}");
        }
    }
}
