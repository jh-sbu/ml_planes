Vibe coded flight sim using Rapier/Bevy.

AI Summary:

# ml_planes

6-DOF flight simulation sandbox for traditional and ML-based autopilot research.

Planes are physically simulated rigid bodies (Bevy Rapier 3D) with a coefficient-based linear stability aerodynamic model. Flight controllers — PID autopilots or future ML policies — are hot-swappable at runtime via a trait interface. All ML runs in pure Rust with the `burn` crate; no Python, no IPC.

## Quick Start

```bash
# Interactive visual sim (default)
cargo run

# Headless (no rendering)
cargo run --no-default-features

# All tests (always headless)
cargo test --no-default-features
```

## Feature Flags

| Flag | Effect |
|---|---|
| `visual` (default) | Full Bevy rendering pipeline, egui HUD, Rapier debug render |
| `training` | Enables the `burn` ML framework |
| *(none — `--no-default-features`)* | Headless physics only; used for tests and training loops |

## Architecture

```
src/
  aerodynamics/   # coefficient model: (FlightState, ControlInputs, PlaneConfig) → (F, τ)
  plane/          # PlaneConfig asset, FlightState, ControlInputs, physics systems
  controllers/    # FlightController trait + PID, Manual, LevelHold, Ascent, Wingman
  environment/    # infinite ground collider, plane spawner, episode lifecycle
  camera/         # FreeLook and Follow camera modes  [visual only]
  ui/             # egui HUD                          [visual only]
  training/       # TrainingEnv trait, episode management, LevelHoldEnv
```

### Physics Stack

```
FlightController  →  Aero Model  →  Rapier RigidBody
```

Each plane is a full Rapier `RigidBody`. The aero model computes forces and torques in body frame from current `FlightState`, `ControlInputs`, and the plane's `PlaneConfig` asset:

| Force/Moment | Equation |
|---|---|
| Lift | `L = q·S·(CL0 + CLα·α + CLδe·δe)` |
| Drag | `D = q·S·(CD0 + CDi·CL²)` |
| Pitching moment | `M = q·S·c̄·(Cm0 + Cmα·α + Cmq·(q·c̄/2V) + Cmδe·δe)` |
| Roll / Yaw | Lateral-directional coefficients + stability derivatives |

## Plane Assets

All aerodynamic data lives in `.plane.ron` files under `assets/planes/`. The asset loader parses these at runtime — no compile-time plane data.

`assets/planes/generic_jet.plane.ron` is the reference configuration (5000 kg, 60 kN thrust).

Controller tuning profiles are stored separately in `.tuning.ron` files and loaded the same way.

## Controllers

| Controller | Description |
|---|---|
| `ManualController` | Keyboard input; direct control-surface commands |
| `LevelHoldController` | Pitch + throttle PID to hold target altitude and airspeed |
| `AscentController` | Climbs to a target altitude then hands off |
| `WingmanController` | Formation flight; holds a fixed offset behind a leader entity |

Add a new controller by implementing `FlightController`:

```rust
pub trait FlightController: Send + Sync {
    fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs;
}
```

## Maneuver Roadmap

1. **Level hold** — pitch/throttle PID to maintain altitude and airspeed (complete)
2. **Formation flight** — follow a lead plane at a fixed relative offset (next)
3. **Aerial refueling** — approach lead from the rear to a docking position (planned)

## Scope / Non-Goals

- No takeoff or landing. Planes spawn mid-air; ground contact ends the episode.
- No terrain, no compressibility, no structural limits.
- ML runtime is pure Rust (`burn`). No Python, no IPC, no C extensions.
