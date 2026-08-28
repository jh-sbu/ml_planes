//! PyO3 extension module — bindings only, no simulation logic.
//!
//! Everything here marshals to and from `ml_planes`'s existing public API; a
//! reward, termination, or physics rule implemented on this side would make a
//! Python rollout diverge from the Rust one (see CLAUDE.md §3, Python bindings).

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use ml_planes::training::task::{self, EnvSpec, Task};
use ml_planes::training::{TerminationReason, TrainingEnv};

/// Fixed physics timestep in seconds, from `ml_planes::plane::PHYSICS_DT`.
#[pyfunction]
fn physics_dt() -> f32 {
    ml_planes::plane::PHYSICS_DT
}

/// Parity harness — **not** the binding surface.
///
/// This exists to prove, in a test, that a Python-driven rollout of a
/// `ml_planes::training` environment is bit-identical to the Rust-driven one. It
/// is deliberately not re-exported from the `ml_planes` package, and its
/// signatures may change without notice: the real env wrapper (all five tasks, a
/// `VecEnv`, a Gymnasium adapter, and the question of what `reset()` should
/// return alongside the observation) is a separate piece of work, and shipping a
/// public shape here would settle those questions by accident.
///
/// `lstm_orbit` is read-only-staged: the erased `Box<dyn TrainingEnv>` does not
/// carry `CurriculumEnv`, so the curriculum cannot be advanced from here.
#[pyclass(name = "_Env")]
struct PyEnv {
    inner: Box<dyn TrainingEnv>,
}

#[pymethods]
impl PyEnv {
    /// `task` is one of `level_hold`, `heading_hold`, `orbit`, `residual_orbit`,
    /// `lstm_orbit`. `plane_config` and `reward_config` are filesystem paths
    /// relative to the repo root, defaulting to the same airframe and shipped
    /// reward profile a bare `train_ppo --task <task>` uses.
    #[new]
    #[pyo3(signature = (task, plane_config = None, reward_config = None))]
    fn new(task: &str, plane_config: Option<&str>, reward_config: Option<&str>) -> PyResult<Self> {
        let task = Task::parse(task).map_err(PyValueError::new_err)?;
        let plane_path = plane_config.unwrap_or(ml_planes::training::DEFAULT_PLANE_CONFIG_PATH);
        // The fallible loader, never `load_plane_config_or_exit` — that one calls
        // `std::process::exit(2)` and would take the interpreter down with it.
        let cfg = ml_planes::training::load_plane_config(plane_path)
            .map_err(|e| PyValueError::new_err(format!("cannot load {plane_path}: {e}")))?;
        let spec = EnvSpec::defaults_for(task, cfg);
        let inner = task::make_env(task, &spec, reward_config).map_err(PyValueError::new_err)?;
        Ok(Self { inner })
    }

    /// Start a new episode. Observation only — see the class docstring.
    fn reset(&mut self) -> Vec<f32> {
        self.inner.reset().0
    }

    /// `(obs, reward, terminated, truncated, info)`.
    ///
    /// `terminated` and `truncated` are kept apart, as Gymnasium wants them and
    /// as `StepOutcome.end` already encodes: a failure is absorbing, a timeout
    /// cut a still-flying episode short and its value must be bootstrapped.
    /// Collapsing them into one "done" flag is the value-target bug CLAUDE.md
    /// warns about, reintroduced at the language boundary.
    fn step(
        &mut self,
        action: Vec<f32>,
    ) -> PyResult<(
        Vec<f32>,
        f32,
        bool,
        bool,
        std::collections::HashMap<&'static str, u32>,
    )> {
        let expected = self.inner.action_dim();
        if action.len() != expected {
            return Err(PyValueError::new_err(format!(
                "expected {expected} actions, got {}",
                action.len()
            )));
        }
        let outcome = self.inner.step(&action);
        let terminated = outcome.end == Some(TerminationReason::Failure);
        let truncated = outcome.truncated();
        let info = std::collections::HashMap::from([("episode_step", outcome.info.episode_step)]);
        Ok((outcome.obs, outcome.reward, terminated, truncated, info))
    }

    fn observation_dim(&self) -> usize {
        self.inner.observation_dim()
    }

    fn action_dim(&self) -> usize {
        self.inner.action_dim()
    }

    /// Absolute seed: the same value always reproduces the same episode sequence.
    /// Note that `reset()` advances the seed before drawing, so `seed(s)` then
    /// `reset()` runs the episode drawn from `s + 1`.
    fn seed(&mut self, seed: u64) {
        self.inner.set_rng_seed(seed);
    }

    fn rng_seed(&self) -> u64 {
        self.inner.rng_seed()
    }
}

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(physics_dt, m)?)?;
    m.add_class::<PyEnv>()?;
    Ok(())
}
