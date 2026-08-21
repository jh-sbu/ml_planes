"""Python bindings for the ml_planes 6-DOF flight training environments.

The Rust extension module is `ml_planes._core`; names are re-exported here so
callers import from `ml_planes` and never touch the private module directly.
"""

from . import _core
from ._core import physics_dt

__all__ = ["physics_dt"]
