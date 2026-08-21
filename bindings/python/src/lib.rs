//! PyO3 extension module — bindings only, no simulation logic.
//!
//! Everything here marshals to and from `ml_planes`'s existing public API; a
//! reward, termination, or physics rule implemented on this side would make a
//! Python rollout diverge from the Rust one (see CLAUDE.md §3, Python bindings).

use pyo3::prelude::*;

/// Fixed physics timestep in seconds, from `ml_planes::plane::PHYSICS_DT`.
#[pyfunction]
fn physics_dt() -> f32 {
    ml_planes::plane::PHYSICS_DT
}

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(physics_dt, m)?)?;
    Ok(())
}
