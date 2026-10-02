//! Batched RL policy inference: one forward pass per loaded model per tick.
//!
//! **Why.** burn's ndarray backend is built for batched throughput. A batch-of-1
//! forward through the 14→64→64→4 actor costs ~9 µs, almost all of it fixed
//! per-op overhead (allocation, dispatch, and `matrixmultiply` re-packing the
//! weight matrix on every call); the same network costs ~1.7 µs per row once the
//! rows share a forward. Stepping each RL plane's policy inside its own
//! `FlightController::update` paid the batch-of-1 price every tick.
//!
//! **How.** An RL controller exposes a [`BatchedPolicy`] through
//! `FlightController::batched()`, splitting its tick into [`BatchedPolicy::observe`]
//! (write the observation, do any pre-policy work such as the residual
//! controller's PID baseline) and [`BatchedPolicy::finish`] (map the action, and
//! for the LSTM carry the new hidden state). `run_flight_controllers` pushes every
//! such controller into a [`PolicyBatch`], which groups them by the model they
//! share and runs each group as one forward pass.
//!
//! **The batch is rebuilt from scratch every tick** and holds nothing between
//! ticks, so spawning, removing, switching, demoting, and model hot-swapping a
//! plane need no bookkeeping here: the next tick's query simply sees a different
//! population. Keep it that way — a persistent registry would have to mirror the
//! ECS through every lifecycle path, including the `PendingPlaneSpawn` window.
//!
//! **Sharing.** Grouping needs planes flying the same checkpoint to hold the
//! same model, so the controllers' loaders go through [`load_mlp_bytes`] /
//! [`load_mlp_file`] (and the LSTM pair), which dedupe by checkpoint *content*.
//! Keying on bytes rather than path means a checkpoint retrained in place under
//! the same name is a new model, never a stale cache hit. The cache holds only
//! `Weak` references, so a model is freed once no controller uses it.
//!
//! **Determinism.** Each controller's own `update()` runs through this same code
//! as a batch of one ([`run_single`]), so batched and unbatched ticks share every
//! line except the batch size. Results are bit-identical across batch sizes
//! because burn's ndarray matmul computes each output row with the same
//! summation order whatever the row count — a property of the backend, not a
//! promise burn makes, which is why `tests/rl/rl_batching.rs` pins it. If it
//! ever breaks, a plane's trajectory would start depending on how many *other*
//! planes share its model.

use std::sync::{Arc, Mutex, Weak};

use burn::{
    backend::NdArray,
    module::Module,
    record::{FullPrecisionSettings, NamedMpkBytesRecorder, Recorder, RecorderError},
    tensor::{backend::Backend, Tensor, TensorData},
};

use crate::plane::{ControlInputs, ControllerContext, FlightState};
use crate::training::ppo::lstm_model::{LstmActorCritic, LstmHiddenState};
use crate::training::ppo::model::ActorCritic;

type InfB = NdArray;

/// A loaded feed-forward actor-critic, shared by every controller flying it.
///
/// `Mutex` because burn's `Param` is not `Sync`; the lock is taken once per
/// group forward, not once per plane.
pub type SharedMlp = Arc<Mutex<ActorCritic<InfB>>>;
/// A loaded recurrent actor-critic, shared by every controller flying it.
pub type SharedLstm = Arc<Mutex<LstmActorCritic<InfB>>>;

/// The model a [`BatchedPolicy`] runs. Identity (the `Arc`), not weights, is what
/// groups controllers into one forward.
#[derive(Clone)]
pub enum SharedPolicy {
    Mlp(SharedMlp),
    Lstm(SharedLstm),
}

impl SharedPolicy {
    /// True when `self` and `other` are the same loaded model.
    pub fn same_model(&self, other: &SharedPolicy) -> bool {
        match (self, other) {
            (SharedPolicy::Mlp(a), SharedPolicy::Mlp(b)) => Arc::ptr_eq(a, b),
            (SharedPolicy::Lstm(a), SharedPolicy::Lstm(b)) => Arc::ptr_eq(a, b),
            _ => false,
        }
    }
}

/// The split form of an RL controller's tick, so its forward pass can be shared.
pub trait BatchedPolicy {
    /// The model this controller runs.
    fn policy(&self) -> &SharedPolicy;

    /// Append this tick's observation to `obs` (exactly the model's input
    /// dimension), doing any work that must precede the policy.
    fn observe(
        &mut self,
        state: &FlightState,
        ctx: &ControllerContext,
        dt: f32,
        obs: &mut Vec<f32>,
    );

    /// The recurrent state to feed this tick. `Some` exactly for LSTM policies.
    fn hidden(&self) -> Option<&LstmHiddenState> {
        None
    }

    /// Turn this controller's row of the batch's output into control inputs.
    /// `hidden` is the updated recurrent state (`Some` exactly for LSTM policies).
    fn finish(&mut self, action: &[f32], hidden: Option<LstmHiddenState>) -> ControlInputs;
}

/// One model's rows for this tick.
struct Group<K> {
    policy: SharedPolicy,
    members: Vec<K>,
    obs: Vec<f32>,
    hidden: Vec<LstmHiddenState>,
}

/// Collects this tick's batched controllers, grouped by shared model.
///
/// `K` identifies a member when its action comes back (an `Entity` in the live
/// sim, an index in tests).
pub struct PolicyBatch<K> {
    groups: Vec<Group<K>>,
}

impl<K> Default for PolicyBatch<K> {
    fn default() -> Self {
        Self { groups: Vec::new() }
    }
}

impl<K: Copy> PolicyBatch<K> {
    /// Observe `policy` for this tick and queue it under `key`.
    pub fn push(
        &mut self,
        key: K,
        policy: &mut dyn BatchedPolicy,
        state: &FlightState,
        ctx: &ControllerContext,
        dt: f32,
    ) {
        // A linear scan: a tick has a handful of distinct models, not hundreds.
        let idx = match self
            .groups
            .iter()
            .position(|g| g.policy.same_model(policy.policy()))
        {
            Some(idx) => idx,
            None => {
                self.groups.push(Group {
                    policy: policy.policy().clone(),
                    members: Vec::new(),
                    obs: Vec::new(),
                    hidden: Vec::new(),
                });
                self.groups.len() - 1
            }
        };
        let group = &mut self.groups[idx];
        group.members.push(key);
        policy.observe(state, ctx, dt, &mut group.obs);
        if let Some(hidden) = policy.hidden() {
            group.hidden.push(hidden.clone());
        }
    }

    /// True when nothing is queued.
    pub fn is_empty(&self) -> bool {
        self.groups.is_empty()
    }

    /// Run one forward per group and hand each member its action (and, for an
    /// LSTM, its new hidden state). Leaves the batch empty, dropping its model
    /// references, ready for the next tick.
    pub fn run(&mut self, mut finish: impl FnMut(K, &[f32], Option<LstmHiddenState>)) {
        let device: <InfB as Backend>::Device = Default::default();
        for group in self.groups.drain(..) {
            let n = group.members.len();
            let obs_dim = group.obs.len() / n;
            assert_eq!(
                obs_dim * n,
                group.obs.len(),
                "every member of a policy group must write the same observation width"
            );
            let obs =
                Tensor::<InfB, 2>::from_data(TensorData::new(group.obs, vec![n, obs_dim]), &device);
            match &group.policy {
                SharedPolicy::Mlp(model) => {
                    // Deterministic inference: mean action, no sampling noise.
                    let actions = model.lock().unwrap().mean_action(obs);
                    let actions = actions
                        .into_data()
                        .to_vec::<f32>()
                        .expect("policy action data");
                    let width = actions.len() / n;
                    for (key, action) in group.members.into_iter().zip(actions.chunks_exact(width))
                    {
                        finish(key, action, None);
                    }
                }
                SharedPolicy::Lstm(model) => {
                    assert_eq!(
                        group.hidden.len(),
                        n,
                        "every LSTM member must supply its hidden state"
                    );
                    let state = LstmHiddenState::batch_to_burn::<InfB>(&group.hidden, &device);
                    let (actions, new_state) =
                        model.lock().unwrap().mean_action_step(obs, Some(state));
                    let hidden = LstmHiddenState::unbatch_from_burn(new_state, n);
                    let actions = actions
                        .into_data()
                        .to_vec::<f32>()
                        .expect("lstm action data");
                    let width = actions.len() / n;
                    for ((key, action), h) in group
                        .members
                        .into_iter()
                        .zip(actions.chunks_exact(width))
                        .zip(hidden)
                    {
                        finish(key, action, Some(h));
                    }
                }
            }
        }
    }
}

/// Step one controller on its own — a batch of one through the same path the
/// live sim batches through, so the two cannot drift.
pub fn run_single(
    policy: &mut dyn BatchedPolicy,
    state: &FlightState,
    ctx: &ControllerContext,
    dt: f32,
) -> ControlInputs {
    let mut batch = PolicyBatch::<()>::default();
    batch.push((), policy, state, ctx, dt);
    let mut result = None;
    batch.run(|(), action, hidden| result = Some((action.to_vec(), hidden)));
    let (action, hidden) = result.expect("a batch of one yields one action");
    policy.finish(&action, hidden)
}

// ---------------------------------------------------------------------------
// Shared-model cache

/// One cached checkpoint: its bytes (compared in full on a hash hit, so a
/// collision can never alias two models) and the model, held weakly.
struct CacheEntry<M> {
    hash: u64,
    bytes: Vec<u8>,
    model: Weak<Mutex<M>>,
}

static MLP_CACHE: Mutex<Vec<CacheEntry<ActorCritic<InfB>>>> = Mutex::new(Vec::new());
static LSTM_CACHE: Mutex<Vec<CacheEntry<LstmActorCritic<InfB>>>> = Mutex::new(Vec::new());

fn hash_bytes(bytes: &[u8]) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    bytes.hash(&mut h);
    h.finish()
}

/// Return the live model for `bytes`, or build one with `load` and remember it.
///
/// The cache lock is held across `load` so two threads loading the same new
/// checkpoint cannot each build a copy; loads are rare (spawn, switch, hot-swap).
fn cached<M>(
    cache: &Mutex<Vec<CacheEntry<M>>>,
    bytes: &[u8],
    load: impl FnOnce() -> Result<M, RecorderError>,
) -> Result<Arc<Mutex<M>>, RecorderError> {
    let hash = hash_bytes(bytes);
    let mut entries = cache.lock().unwrap();
    entries.retain(|e| e.model.strong_count() > 0);
    if let Some(model) = entries
        .iter()
        .find(|e| e.hash == hash && e.bytes == bytes)
        .and_then(|e| e.model.upgrade())
    {
        return Ok(model);
    }
    let model = Arc::new(Mutex::new(load()?));
    entries.push(CacheEntry {
        hash,
        bytes: bytes.to_vec(),
        model: Arc::downgrade(&model),
    });
    Ok(model)
}

/// Load (or reuse) the feed-forward model whose checkpoint is `bytes`.
///
/// The model's dimensions come from the checkpoint; callers still validate the
/// observation width against what they will feed (`check_obs_dim`).
pub fn load_mlp_bytes(bytes: &[u8]) -> Result<SharedMlp, RecorderError> {
    cached(&MLP_CACHE, bytes, || {
        let device: <InfB as Backend>::Device = Default::default();
        let record = NamedMpkBytesRecorder::<FullPrecisionSettings>::default()
            .load(bytes.to_vec(), &device)?;
        // The skeleton's input width is irrelevant: `load_record` adopts the
        // checkpoint's tensor shapes.
        Ok(ActorCritic::<InfB>::new(&device, 1).load_record(record))
    })
}

/// Load (or reuse) the recurrent model whose checkpoint is `bytes`.
pub fn load_lstm_bytes(bytes: &[u8]) -> Result<SharedLstm, RecorderError> {
    cached(&LSTM_CACHE, bytes, || {
        let device: <InfB as Backend>::Device = Default::default();
        let record = NamedMpkBytesRecorder::<FullPrecisionSettings>::default()
            .load(bytes.to_vec(), &device)?;
        Ok(LstmActorCritic::<InfB>::new(&device, 1).load_record(record))
    })
}

/// Read the `.mpk` at `path` the way burn's `DefaultFileRecorder` does: the
/// extension is *set* (not appended) and I/O errors map to the same variants, so
/// switching a loader from `Module::load_file` to this changes no observable
/// behavior. The named-MessagePack file and bytes formats are identical.
#[cfg(not(target_arch = "wasm32"))]
fn read_mpk(path: &str) -> Result<Vec<u8>, RecorderError> {
    let mut path = std::path::PathBuf::from(path);
    path.set_extension("mpk");
    std::fs::read(&path).map_err(|err| match err.kind() {
        std::io::ErrorKind::NotFound => RecorderError::FileNotFound(err.to_string()),
        _ => RecorderError::Unknown(err.to_string()),
    })
}

/// [`load_mlp_bytes`] on the `.mpk` at `path` (given without its extension, as
/// `Module::load_file` takes it).
#[cfg(not(target_arch = "wasm32"))]
pub fn load_mlp_file(path: &str) -> Result<SharedMlp, RecorderError> {
    load_mlp_bytes(&read_mpk(path)?)
}

/// [`load_lstm_bytes`] on the `.mpk` at `path` (given without its extension).
#[cfg(not(target_arch = "wasm32"))]
pub fn load_lstm_file(path: &str) -> Result<SharedLstm, RecorderError> {
    load_lstm_bytes(&read_mpk(path)?)
}
