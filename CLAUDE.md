# ml_planes — Developer Reference

## 1. Project Overview

**Goal:** A physically simulated training sandbox flight simulation for exploring traditional and ML-based flight control schemes. Autopilot agents perform individual maneuvers (level hold, formation flight, aerial refueling) against a realistic 6-DOF aerodynamic model.

**Tech stack:**
- Physics: `bevy_rapier3d` (Rapier rigid-body dynamics)
- Rendering: `bevy` + `bevy_egui` HUD (feature-flagged off for training)
- Aerodynamics: custom coefficient-based model (coefficient tables in `.plane.ron` assets)
- ML: `burn` (pure Rust; no Python, no IPC)
- Asset format: RON (Rusty Object Notation, native Bevy reflection)

**Development philosophy:** Environment and tests first; maneuvers second. The environment, aerodynamic model, and test suite must be solid before any controller or ML work begins.

---

## 2. Architecture

### Crate / Module Structure

```
src/
  aerodynamics/   # coefficient model, force/torque computation
  plane/          # PlaneConfig asset, FlightState, ControlInputs, physics systems
  controllers/    # FlightController trait, PID, LevelHold, Manual
  environment/    # infinite ground collider + shader, plane spawner
  camera/         # FreeLook and Follow camera modes
  ui/             # egui HUD, extensible info panel
  training/       # TrainingEnv trait, episode management
```

### Key Types

| Type | Kind | Description |
|---|---|---|
| `PlaneConfig` | RON asset | Geometry, mass/inertia, aero coefficients, engine params, control limits |
| `FlightState` | ECS component | Position, velocity, attitude (quat), angular velocity, α, β, airspeed, altitude |
| `ControlInputs` | ECS component | Aileron/elevator/rudder/throttle **or** rate commands (see Action Spaces) |
| `FlightController` | trait | `fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs` |
| `PidController<T>` | generic struct | PID with integral wind-up clamp and output limits |
| `TrainingEnv` | trait | `reset()`, `step(action) -> (obs, reward, done, info)` |

### Physics Layering

```
┌─────────────────────────────────────────┐
│  FlightController  (Box<dyn trait>)     │  top: outputs ControlInputs
├─────────────────────────────────────────┤
│  Aerodynamic Model                      │  middle: (FlightState, ControlInputs,
│  (FlightState, ControlInputs,           │          PlaneConfig) → (F_body, τ_body)
│   PlaneConfig) → (F_body, τ_body)       │
├─────────────────────────────────────────┤
│  Rapier RigidBody                       │  bottom: net force + torque in body frame
└─────────────────────────────────────────┘
```

### Aerodynamic Model (Linear Stability, Body Frame)

Dynamic pressure: `q̄ = ½·ρ·V²`

| Force/Moment | Equation |
|---|---|
| Lift | `L = q̄·S·(CL0 + CLα·α + CLδe·δe)` |
| Drag | `D = q̄·S·(CD0 + CDi·CL²)` |
| Pitching moment | `M = q̄·S·c̄·(Cm0 + Cmα·α + Cmq·(q·c̄/2V) + Cmδe·δe)` |
| Roll / Yaw | Lateral-directional coefficients + stability derivatives (see `PlaneConfig`) |

All coefficients are defined per-asset in `.plane.ron` files. No compile-time aero data.

### Action Spaces

Each `FlightController` declares which action space it uses (configurable per controller):

| Space | Channels | Range |
|---|---|---|
| Control surfaces | `[aileron, elevator, rudder, throttle]` | `[-1, 1]` |
| Angular rate commands | `[roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd, thrust]` | `[-1, 1]` |

The aerodynamic model maps whichever representation to net force and torque before applying to Rapier.

### Feature Flags

```toml
[features]
default = ["visual"]
visual = ["bevy/default", "bevy_egui"]
training = []
```

- `visual` (default): full Bevy rendering pipeline + egui HUD
- `training`: no rendering, max-speed simulation (no `visual`)
- All tests run with `--no-default-features` (headless); no rendering in CI

---

## 3. Scope Decisions

| Topic | Decision |
|---|---|
| Takeoff / Landing | **Out of scope.** Planes spawn mid-air. Ground = death plane (contact → episode reset). No landing gear, no ground effect, no runway. |
| Spawn system | Accepts arbitrary `(position, velocity, attitude, angular_velocity)` for ML episode resets. |
| Stall modeling | Simple: CL caps at `CLmax`. No deep stall, no spin dynamics. |
| Compressibility | Ignored. Low-Mach assumption throughout. |
| Structural limits | Not modeled. |
| ML runtime | Pure Rust (`burn`). No Python, no IPC, no C extensions. |
| Multi-agent | Architecture must support one `Box<dyn FlightController>` per plane entity. Exact multi-agent training strategy deferred. |

---

## 4. Maneuver Roadmap

1. **Level flight hold** — pitch/throttle PID to maintain target altitude and airspeed
2. **Formation flight** — follow a scripted lead plane at a fixed relative offset
3. **Aerial refueling** — approach a lead plane from the rear to a docking position
4. *(extensible — add new `TrainingEnv` impls without changing core architecture)*

---

## 5. Key Dependencies

```toml
[dependencies]
bevy = "0.15"
bevy_rapier3d = "0.28"
ron = "0.8"
serde = { version = "1", features = ["derive"] }

[dependencies.bevy_egui]
version = "0.31"
optional = true

[dependencies.burn]
version = "0.16"
optional = true   # pulled in by the `training` feature

[features]
default = ["visual"]
visual = ["bevy/default", "bevy_egui"]
training = ["burn"]
```

---

## 6. Test Strategy

### Unit tests (`src/` modules)
- `PidController`: step response, integral wind-up clamp, output clamping
- Aerodynamic force computation: known α/V inputs → expected lift/drag/moment
- `FlightState` kinematics: attitude integration, angle-of-attack calculation

### Integration tests (`tests/`)
- Physics consistency: energy conservation over N steps in level flight
- Controller convergence: level-hold reaches target altitude within tolerance in N steps
- Episode reset/spawn: `TrainingEnv::reset()` produces correct initial `FlightState`

### Rules
- All tests must pass with `cargo test --no-default-features`
- No rendering, no Bevy `App` window, no GPU resources in tests
- Tests are deterministic (fixed seed where randomness is needed)

---

## 7. Invariants / Non-Negotiables

- Every plane is a full 6-DOF Rapier `RigidBody` — no simplified kinematics.
- `FlightController` is always `Box<dyn FlightController>` per entity — never hard-wired to a concrete type.
- `PlaneConfig` is loaded at runtime via Bevy's asset server — no compile-time plane data.
- All physics runs at Rapier's fixed timestep; rendering interpolates between steps.
- The ground is a flat infinite collider acting as a death plane — no terrain, no landing.
