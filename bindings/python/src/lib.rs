//! PyO3 extension module — bindings only, no simulation logic.
//!
//! Everything here marshals to and from `ml_planes`'s existing public API; a
//! reward, termination, or physics rule implemented on this side would make a
//! Python rollout diverge from the Rust one (see CLAUDE.md §3, Python bindings).
//! `bindings/python/tests/test_parity.py` is what turns that from a promise into
//! a test: it replays a Rust rollout's own actions through [`Env`] and [`VecEnv`]
//! and demands bit-identical observations and rewards.
//!
//! Two conventions run through the whole file:
//!
//! * `terminated` and `truncated` are never collapsed into one `done`. A failure
//!   is absorbing (bootstrap 0); a timeout cut a still-flying episode short and
//!   must bootstrap `gamma * V(s')`. Collapsing them is the PPO value-target bug
//!   CLAUDE.md warns about, reintroduced at the language boundary.
//! * Nothing auto-resets, exactly as on the Rust side, so a caller can read the
//!   terminal observation off the step before calling `reset`.

use numpy::ndarray::Array2;
use numpy::{AllowTypeChange, IntoPyArray, PyArray1, PyArray2, PyArrayLike1, PyArrayLike2};
use pyo3::exceptions::{PyIndexError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyDict;
use std::ops::RangeInclusive;

use ml_planes::training::task::{self, EnvSpec, Task};
use ml_planes::training::{SpawnSpec, StepOutcome, TerminationReason, TrainingEnv, VecEnv};

/// Fixed physics timestep in seconds, from `ml_planes::plane::PHYSICS_DT`.
#[pyfunction]
fn physics_dt() -> f32 {
    ml_planes::plane::PHYSICS_DT
}

fn parse_task(name: &str) -> PyResult<Task> {
    Task::parse(name).map_err(PyValueError::new_err)
}

fn range_or(
    given: Option<(f32, f32)>,
    default: RangeInclusive<f32>,
) -> PyResult<RangeInclusive<f32>> {
    match given {
        None => Ok(default),
        Some((lo, hi)) if lo <= hi => Ok(lo..=hi),
        Some((lo, hi)) => Err(PyValueError::new_err(format!(
            "range ({lo}, {hi}) is inverted; expected (min, max)"
        ))),
    }
}

/// The single place an [`EnvSpec`] is built, so [`Env`] and [`VecEnv`] cannot
/// drift into training across different envelopes.
#[allow(clippy::too_many_arguments)]
fn build_spec(
    task: Task,
    plane_config: Option<&str>,
    target_alt_range: Option<(f32, f32)>,
    target_speed_range: Option<(f32, f32)>,
    target_heading_range_deg: Option<(f32, f32)>,
    orbit_altitude: Option<f32>,
    orbit_airspeed: Option<f32>,
    orbit_radius: Option<f32>,
    max_episode_steps: Option<u32>,
) -> PyResult<EnvSpec> {
    let plane_path = plane_config.unwrap_or(ml_planes::training::DEFAULT_PLANE_CONFIG_PATH);
    // The fallible loader, never `load_plane_config_or_exit` — that one calls
    // `std::process::exit(2)` and would take the interpreter down with it.
    let cfg = ml_planes::training::load_plane_config(plane_path)
        .map_err(|e| PyValueError::new_err(format!("cannot load {plane_path}: {e}")))?;

    let mut spec = EnvSpec::defaults_for(task, cfg);
    spec.target_alt_range = range_or(target_alt_range, spec.target_alt_range.clone())?;
    spec.target_speed_range = range_or(target_speed_range, spec.target_speed_range.clone())?;
    // The kwarg is DEGREES; `EnvSpec::target_heading_range` is RADIANS. Convert
    // once, here — a missed conversion is silent and only shows up as a policy
    // trained on the wrong envelope.
    if let Some(given) = target_heading_range_deg {
        let deg = range_or(Some(given), 0.0..=0.0)?;
        spec.target_heading_range = deg.start().to_radians()..=deg.end().to_radians();
    }
    if let Some(v) = orbit_altitude {
        spec.orbit.altitude = v;
    }
    if let Some(v) = orbit_airspeed {
        spec.orbit.airspeed = v;
    }
    if let Some(v) = orbit_radius {
        spec.orbit.radius = v;
    }
    // A zero budget would make every `step` a truncation before the plane has
    // moved, which reads as a working evaluation returning nonsense rather than
    // as an error. `EnvSpec` itself permits it, so reject it at the boundary.
    if let Some(n) = max_episode_steps {
        if n == 0 {
            return Err(PyValueError::new_err(
                "max_episode_steps must be at least 1",
            ));
        }
        spec.max_episode_steps = Some(n);
    }
    Ok(spec)
}

fn make_env(task: Task, spec: &EnvSpec, reward_config: Option<&str>) -> PyResult<Box<dyn TrainingEnv>> {
    task::make_env(task, spec, reward_config).map_err(PyValueError::new_err)
}

/// Copy an action row out of a numpy view into an owned buffer.
///
/// `as_slice` is `None` for a non-contiguous view (`arr[::2]`), so the fallback
/// is not optional. Owning the data also lets the batched step release the GIL
/// without holding a borrow into a live numpy array.
fn action_vec(view: numpy::ndarray::ArrayView1<'_, f32>) -> Vec<f32> {
    match view.as_slice() {
        Some(s) => s.to_vec(),
        None => view.iter().copied().collect(),
    }
}

fn spawn_dict<'py>(py: Python<'py>, s: &SpawnSpec) -> PyResult<Bound<'py, PyDict>> {
    let d = PyDict::new(py);
    d.set_item("position", s.position.map(|v| [v.x, v.y, v.z]))?;
    d.set_item("velocity", s.velocity.map(|v| [v.x, v.y, v.z]))?;
    // glam stores a quaternion as (x, y, z, w), not (w, x, y, z).
    d.set_item("attitude", s.attitude.map(|q| [q.x, q.y, q.z, q.w]))?;
    d.set_item(
        "angular_velocity",
        s.angular_velocity.map(|v| [v.x, v.y, v.z]),
    )?;
    d.set_item("fuel_fraction", s.fuel_fraction)?;
    Ok(d)
}

fn step_info<'py>(py: Python<'py>, outcome: &StepOutcome) -> PyResult<Bound<'py, PyDict>> {
    let d = PyDict::new(py);
    d.set_item("episode_step", outcome.info.episode_step)?;
    Ok(d)
}

fn terminated_of(outcome: &StepOutcome) -> bool {
    outcome.end == Some(TerminationReason::Failure)
}

/// Stack a per-env column of fixed-width vectors into an `(N, W)` array.
///
/// `None` for a field every env left empty, and a Python list with holes for the
/// mixed case — inventing a sentinel would be a physics claim this layer has no
/// business making. Every shipped env fills all five, but the trait permits not.
fn stack_column<'py, const W: usize>(
    py: Python<'py>,
    vals: &[Option<[f32; W]>],
) -> PyResult<Bound<'py, PyAny>> {
    if vals.iter().all(Option::is_none) {
        return Ok(py.None().into_bound(py));
    }
    match vals.iter().copied().collect::<Option<Vec<[f32; W]>>>() {
        Some(rows) => {
            let flat: Vec<f32> = rows.into_iter().flatten().collect();
            Ok(Array2::from_shape_vec((vals.len(), W), flat)
                .map_err(|e| PyValueError::new_err(e.to_string()))?
                .into_pyarray(py)
                .into_any())
        }
        None => Ok(vals.to_vec().into_pyobject(py)?.into_any()),
    }
}

fn stacked_spawn_dict<'py>(py: Python<'py>, specs: &[SpawnSpec]) -> PyResult<Bound<'py, PyDict>> {
    let d = PyDict::new(py);
    let pos: Vec<_> = specs.iter().map(|s| s.position.map(|v| [v.x, v.y, v.z])).collect();
    let vel: Vec<_> = specs.iter().map(|s| s.velocity.map(|v| [v.x, v.y, v.z])).collect();
    let att: Vec<_> = specs
        .iter()
        .map(|s| s.attitude.map(|q| [q.x, q.y, q.z, q.w]))
        .collect();
    let ang: Vec<_> = specs
        .iter()
        .map(|s| s.angular_velocity.map(|v| [v.x, v.y, v.z]))
        .collect();
    let fuel: Vec<_> = specs.iter().map(|s| s.fuel_fraction).collect();

    d.set_item("position", stack_column(py, &pos)?)?;
    d.set_item("velocity", stack_column(py, &vel)?)?;
    d.set_item("attitude", stack_column(py, &att)?)?;
    d.set_item("angular_velocity", stack_column(py, &ang)?)?;
    d.set_item(
        "fuel_fraction",
        match fuel.iter().copied().collect::<Option<Vec<f32>>>() {
            Some(all) => all.into_pyarray(py).into_any(),
            None => fuel.into_pyobject(py)?.into_any(),
        },
    )?;
    Ok(d)
}

/// One `ml_planes::training` environment, in the Gymnasium shape.
#[pyclass(name = "Env", module = "ml_planes")]
struct PyEnv {
    inner: Box<dyn TrainingEnv>,
    task: Task,
}

#[pymethods]
impl PyEnv {
    /// `task` is one of `ml_planes.TASKS`. `plane_config` and `reward_config`
    /// are filesystem paths **relative to the repo root**, defaulting to the same
    /// airframe and shipped reward profile a bare `train_ppo --task <task>` uses.
    ///
    /// The target-range kwargs override the default training envelope;
    /// `target_heading_range_deg` is in degrees. Pass the same envelope a
    /// checkpoint was trained with, for the same reason the Rust CLI flags exist.
    ///
    /// `max_episode_steps` overrides the reward profile's own episode budget,
    /// the counterpart of what `evaluate_policy` does by assigning the field
    /// directly. An evaluator that caps its loop without it scores episodes
    /// against a budget the env does not share.
    #[new]
    #[pyo3(signature = (
        task,
        *,
        plane_config = None,
        reward_config = None,
        target_alt_range = None,
        target_speed_range = None,
        target_heading_range_deg = None,
        orbit_altitude = None,
        orbit_airspeed = None,
        orbit_radius = None,
        max_episode_steps = None,
        seed = None,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        task: &str,
        plane_config: Option<&str>,
        reward_config: Option<&str>,
        target_alt_range: Option<(f32, f32)>,
        target_speed_range: Option<(f32, f32)>,
        target_heading_range_deg: Option<(f32, f32)>,
        orbit_altitude: Option<f32>,
        orbit_airspeed: Option<f32>,
        orbit_radius: Option<f32>,
        max_episode_steps: Option<u32>,
        seed: Option<u64>,
    ) -> PyResult<Self> {
        let task = parse_task(task)?;
        let spec = build_spec(
            task,
            plane_config,
            target_alt_range,
            target_speed_range,
            target_heading_range_deg,
            orbit_altitude,
            orbit_airspeed,
            orbit_radius,
            max_episode_steps,
        )?;
        let mut inner = make_env(task, &spec, reward_config)?;
        if let Some(seed) = seed {
            inner.set_rng_seed(seed);
        }
        Ok(Self { inner, task })
    }

    /// Start a new episode. Returns `(obs, info)`; `info["spawn"]` carries the
    /// episode's initial position/velocity/attitude/fuel.
    ///
    /// `reset()` advances the seed *before* drawing, so `reset(seed=s)` runs the
    /// episode drawn from `s + 1`. The guarantee is that the same `s` always
    /// reproduces the same episode, not that the stream literally begins at `s`.
    #[pyo3(signature = (seed = None))]
    fn reset<'py>(
        &mut self,
        py: Python<'py>,
        seed: Option<u64>,
    ) -> PyResult<(Bound<'py, PyArray1<f32>>, Bound<'py, PyDict>)> {
        if let Some(seed) = seed {
            self.inner.set_rng_seed(seed);
        }
        let (obs, spec) = self.inner.reset();
        let info = PyDict::new(py);
        info.set_item("spawn", spawn_dict(py, &spec)?)?;
        Ok((obs.into_pyarray(py), info))
    }

    /// `(obs, reward, terminated, truncated, info)`.
    ///
    /// `obs` is the state reached by this step, including at an episode end —
    /// nothing auto-resets, which is what keeps the terminal observation
    /// available for truncation bootstrapping.
    fn step<'py>(
        &mut self,
        py: Python<'py>,
        action: PyArrayLike1<'py, f32, AllowTypeChange>,
    ) -> PyResult<(
        Bound<'py, PyArray1<f32>>,
        f32,
        bool,
        bool,
        Bound<'py, PyDict>,
    )> {
        let expected = self.inner.action_dim();
        let action = action_vec(action.as_array());
        if action.len() != expected {
            return Err(PyValueError::new_err(format!(
                "expected {expected} actions, got {}",
                action.len()
            )));
        }
        let outcome = self.inner.step(&action);
        Ok((
            outcome.obs.clone().into_pyarray(py),
            outcome.reward,
            terminated_of(&outcome),
            outcome.truncated(),
            step_info(py, &outcome)?,
        ))
    }

    /// Reset deterministic episode variation to an absolute seed.
    fn seed(&mut self, seed: u64) {
        self.inner.set_rng_seed(seed);
    }

    #[getter]
    fn observation_dim(&self) -> usize {
        self.inner.observation_dim()
    }

    #[getter]
    fn action_dim(&self) -> usize {
        self.inner.action_dim()
    }

    #[getter]
    fn task(&self) -> &'static str {
        self.task.as_str()
    }

    #[getter]
    fn rng_seed(&self) -> u64 {
        self.inner.rng_seed()
    }

    fn __repr__(&self) -> String {
        format!(
            "Env(task='{}', observation_dim={}, action_dim={})",
            self.task.as_str(),
            self.inner.observation_dim(),
            self.inner.action_dim()
        )
    }
}

/// N independent episodes of one task, stepped together.
///
/// Unlike Gymnasium's vector envs this does **not** auto-reset a finished
/// sub-env: the caller resets it explicitly with `reset_at`, or a batch of them
/// with `reset_done`. That is deliberate — it is what lets a trainer read the
/// terminal observation off the step and bootstrap a truncated episode's value
/// before restarting it.
///
/// `step` runs the pool sequentially, so one `VecEnv` uses one core however
/// large `num_envs` is. It does release the GIL, though, so **several `VecEnv`
/// objects driven from Python threads do scale across cores** — no `rayon` or
/// other Rust-side change required. The detached section has to be long enough
/// to amortize GIL handoff for that to pay: measured on a 16-core machine,
/// `num_envs=256` scaled ~4.4x at 4 threads, while `num_envs=16` (~5 us per
/// step) showed no gain at all, because the threads spend their time convoying
/// on GIL reacquisition rather than integrating.
#[pyclass(name = "VecEnv", module = "ml_planes")]
struct PyVecEnv {
    inner: VecEnv<Box<dyn TrainingEnv>>,
    task: Task,
    obs_dim: usize,
    action_dim: usize,
}

impl PyVecEnv {
    /// Normalize a possibly-negative index, Python-style.
    ///
    /// `VecEnv`'s indexed accessors panic out of range, and pyo3 turns a panic
    /// into `PanicException` — not catchable as `IndexError`, and it prints a
    /// Rust backtrace. So bounds-check before delegating, always.
    fn resolve(&self, i: isize) -> PyResult<usize> {
        let n = self.inner.n() as isize;
        let k = if i < 0 { i + n } else { i };
        if k < 0 || k >= n {
            return Err(PyIndexError::new_err(format!(
                "env index {i} out of range for {n} envs"
            )));
        }
        Ok(k as usize)
    }
}

#[pymethods]
impl PyVecEnv {
    /// Builds `num_envs` independent envs of one task. A `seed` spaces the pool
    /// `ENV_SEED_STRIDE` (1000) apart, so sub-env `i` matches a standalone `Env`
    /// seeded `seed + i * 1000`.
    #[new]
    #[pyo3(signature = (
        task,
        num_envs,
        *,
        plane_config = None,
        reward_config = None,
        target_alt_range = None,
        target_speed_range = None,
        target_heading_range_deg = None,
        orbit_altitude = None,
        orbit_airspeed = None,
        orbit_radius = None,
        max_episode_steps = None,
        seed = None,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        task: &str,
        num_envs: usize,
        plane_config: Option<&str>,
        reward_config: Option<&str>,
        target_alt_range: Option<(f32, f32)>,
        target_speed_range: Option<(f32, f32)>,
        target_heading_range_deg: Option<(f32, f32)>,
        orbit_altitude: Option<f32>,
        orbit_airspeed: Option<f32>,
        orbit_radius: Option<f32>,
        max_episode_steps: Option<u32>,
        seed: Option<u64>,
    ) -> PyResult<Self> {
        // `VecEnv::new` asserts on an empty pool; a panic here would surface as
        // an uncatchable PanicException rather than a normal Python error.
        if num_envs == 0 {
            return Err(PyValueError::new_err("VecEnv needs at least one environment"));
        }
        let task = parse_task(task)?;
        let spec = build_spec(
            task,
            plane_config,
            target_alt_range,
            target_speed_range,
            target_heading_range_deg,
            orbit_altitude,
            orbit_airspeed,
            orbit_radius,
            max_episode_steps,
        )?;
        let envs = (0..num_envs)
            .map(|_| make_env(task, &spec, reward_config))
            .collect::<PyResult<Vec<_>>>()?;

        let obs_dim = envs[0].observation_dim();
        let action_dim = envs[0].action_dim();
        let mut inner = VecEnv::new(envs);
        if let Some(seed) = seed {
            inner.set_rng_seed(seed);
        }
        Ok(Self {
            inner,
            task,
            obs_dim,
            action_dim,
        })
    }

    /// Reset every env. Returns `((N, obs_dim) obs, info)`, where `info["spawn"]`
    /// stacks each env's spawn state.
    #[pyo3(signature = (seed = None))]
    fn reset<'py>(
        &mut self,
        py: Python<'py>,
        seed: Option<u64>,
    ) -> PyResult<(Bound<'py, PyArray2<f32>>, Bound<'py, PyDict>)> {
        if let Some(seed) = seed {
            self.inner.set_rng_seed(seed);
        }
        let out = self.inner.reset_all_with_specs();
        let specs: Vec<SpawnSpec> = out.iter().map(|(_, s)| s.clone()).collect();
        let flat: Vec<f32> = out.iter().flat_map(|(o, _)| o.iter().copied()).collect();

        let info = PyDict::new(py);
        info.set_item("spawn", stacked_spawn_dict(py, &specs)?)?;
        Ok((self.batch_array(py, flat)?, info))
    }

    /// Reset one env, leaving the rest mid-episode. Returns `(obs, info)`.
    #[pyo3(signature = (index, seed = None))]
    fn reset_at<'py>(
        &mut self,
        py: Python<'py>,
        index: isize,
        seed: Option<u64>,
    ) -> PyResult<(Bound<'py, PyArray1<f32>>, Bound<'py, PyDict>)> {
        let i = self.resolve(index)?;
        if let Some(seed) = seed {
            self.inner.set_rng_seed_at(i, seed);
        }
        let (obs, spec) = self.inner.reset_at_with_spec(i);
        let info = PyDict::new(py);
        info.set_item("spawn", spawn_dict(py, &spec)?)?;
        Ok((obs.into_pyarray(py), info))
    }

    /// Restart every sub-env the `mask` selects, leaving the rest untouched.
    ///
    /// The batched form of [`Self::reset_at`], for the caller the no-auto-reset
    /// contract leaves holding a `done` array. Takes the observation batch the
    /// step returned and gives back a copy with the masked rows replaced by
    /// fresh ones, so a caller never scatters rows by hand. It steps nothing.
    ///
    /// `info["index"]` lists the envs that were reset. `info["spawn"]` keeps
    /// row `i` meaning env `i` — as every array on this class does — so an
    /// untouched env is a `None` hole rather than a shifted row.
    #[pyo3(signature = (obs, mask))]
    fn reset_done<'py>(
        &mut self,
        py: Python<'py>,
        obs: PyArrayLike2<'py, f32, AllowTypeChange>,
        mask: PyArrayLike1<'py, bool>,
    ) -> PyResult<(Bound<'py, PyArray2<f32>>, Bound<'py, PyDict>)> {
        let n = self.inner.n();
        let obs_view = obs.as_array();
        let (rows, cols) = (obs_view.shape()[0], obs_view.shape()[1]);
        if rows != n || cols != self.obs_dim {
            return Err(PyValueError::new_err(format!(
                "expected observations of shape ({n}, {}), got ({rows}, {cols})",
                self.obs_dim
            )));
        }
        let mask_view = mask.as_array();
        if mask_view.len() != n {
            return Err(PyValueError::new_err(format!(
                "expected a mask of length {n}, got {}",
                mask_view.len()
            )));
        }

        // `iter()` walks in logical row-major order whatever the memory layout,
        // matching `batch_array`'s expectation.
        let mut flat: Vec<f32> = obs_view.iter().copied().collect();
        let selected: Vec<bool> = mask_view.iter().copied().collect();
        drop(obs);
        drop(mask);

        // A default spec is the hole `stack_column` renders as `None`; every
        // shipped env fills all five fields, so it cannot be confused with a
        // real spawn.
        let mut specs = vec![SpawnSpec::default(); n];
        let mut index: Vec<u32> = Vec::new();
        for (i, reset) in selected.iter().enumerate() {
            if !reset {
                continue;
            }
            let (fresh, spec) = self.inner.reset_at_with_spec(i);
            flat[i * self.obs_dim..(i + 1) * self.obs_dim].copy_from_slice(&fresh);
            specs[i] = spec;
            index.push(i as u32);
        }

        let info = PyDict::new(py);
        info.set_item("index", index.into_pyarray(py))?;
        info.set_item("spawn", stacked_spawn_dict(py, &specs)?)?;
        Ok((self.batch_array(py, flat)?, info))
    }

    /// Step every env with an `(N, action_dim)` batch.
    ///
    /// Returns `(obs (N, D), rewards (N,), terminated (N,), truncated (N,), info)`
    /// where `info["episode_step"]` is a `(N,)` uint32 array — a dict of arrays,
    /// the Gymnasium vector convention, rather than N separate dicts.
    fn step<'py>(
        &mut self,
        py: Python<'py>,
        actions: PyArrayLike2<'py, f32, AllowTypeChange>,
    ) -> PyResult<(
        Bound<'py, PyArray2<f32>>,
        Bound<'py, PyArray1<f32>>,
        Bound<'py, PyArray1<bool>>,
        Bound<'py, PyArray1<bool>>,
        Bound<'py, PyDict>,
    )> {
        let n = self.inner.n();
        let view = actions.as_array();
        let (rows, cols) = (view.shape()[0], view.shape()[1]);
        if rows != n || cols != self.action_dim {
            return Err(PyValueError::new_err(format!(
                "expected actions of shape ({n}, {}), got ({rows}, {cols})",
                self.action_dim
            )));
        }
        // Copy out of the numpy buffer *before* detaching: a borrowed view into a
        // live array is `Send` on stable and would compile, but another thread
        // could mutate the array while the GIL is released. `iter()` walks in
        // logical row-major order whatever the memory layout, which is env-major.
        let flat: Vec<f32> = view.iter().copied().collect();
        drop(actions);

        // Pure Rust compute over N envs; nothing here touches Python.
        let envs = &mut self.inner;
        let outcomes = py.detach(move || envs.step_batch(&flat));

        let obs: Vec<f32> = outcomes.iter().flat_map(|o| o.obs.iter().copied()).collect();
        let rewards: Vec<f32> = outcomes.iter().map(|o| o.reward).collect();
        let terminated: Vec<bool> = outcomes.iter().map(terminated_of).collect();
        let truncated: Vec<bool> = outcomes.iter().map(StepOutcome::truncated).collect();
        let steps: Vec<u32> = outcomes.iter().map(|o| o.info.episode_step).collect();

        let info = PyDict::new(py);
        info.set_item("episode_step", steps.into_pyarray(py))?;

        Ok((
            self.batch_array(py, obs)?,
            rewards.into_pyarray(py),
            terminated.into_pyarray(py),
            truncated.into_pyarray(py),
            info,
        ))
    }

    /// Seed the whole pool from one base, spacing sub-envs `ENV_SEED_STRIDE` apart.
    fn seed(&mut self, seed: u64) {
        self.inner.set_rng_seed(seed);
    }

    /// Seed one sub-env to an absolute seed, leaving its neighbours alone.
    fn seed_at(&mut self, index: isize, seed: u64) -> PyResult<()> {
        let i = self.resolve(index)?;
        self.inner.set_rng_seed_at(i, seed);
        Ok(())
    }

    #[getter]
    fn num_envs(&self) -> usize {
        self.inner.n()
    }

    #[getter]
    fn observation_dim(&self) -> usize {
        self.obs_dim
    }

    #[getter]
    fn action_dim(&self) -> usize {
        self.action_dim
    }

    #[getter]
    fn task(&self) -> &'static str {
        self.task.as_str()
    }

    fn __len__(&self) -> usize {
        self.inner.n()
    }

    fn __repr__(&self) -> String {
        format!(
            "VecEnv(task='{}', num_envs={}, observation_dim={}, action_dim={})",
            self.task.as_str(),
            self.inner.n(),
            self.obs_dim,
            self.action_dim
        )
    }
}

impl PyVecEnv {
    fn batch_array<'py>(
        &self,
        py: Python<'py>,
        flat: Vec<f32>,
    ) -> PyResult<Bound<'py, PyArray2<f32>>> {
        // A pool built from one task is homogeneous, but `VecEnv` itself permits
        // mixed observation widths, so report a ragged batch instead of panicking.
        Array2::from_shape_vec((self.inner.n(), self.obs_dim), flat)
            .map(|a| a.into_pyarray(py))
            .map_err(|e| PyValueError::new_err(format!("ragged observation batch: {e}")))
    }
}

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(physics_dt, m)?)?;
    m.add_class::<PyEnv>()?;
    m.add_class::<PyVecEnv>()?;
    // From `Task::ALL`, so Python never hardcodes the five names.
    m.add(
        "TASKS",
        Task::ALL.iter().map(|t| t.as_str()).collect::<Vec<_>>(),
    )?;
    Ok(())
}
