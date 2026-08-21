"""Scaffolding checks: the extension module builds, imports, and links the Rust crate.

These do not test env behavior — they pin the wiring (mixed maturin layout, the
`ml_planes._core` module name, and the `ml_planes` path dependency) so a broken
build fails here rather than deep in a training script.
"""

import ml_planes


def test_package_exposes_extension_module():
    assert ml_planes._core is not None


def test_physics_dt_comes_from_the_rust_crate():
    # ml_planes::plane::PHYSICS_DT — the single source of the 64 Hz timestep.
    assert ml_planes.physics_dt() == 1.0 / 64.0
