# ml_planes — Developer Reference

## 1. Project Overview

**Goal:** A physically simulated training sandbox flight simulation for exploring traditional and ML-based flight control schemes. Autopilot agents perform individual maneuvers (level hold, formation flight, aerial refueling) against a realistic 6-DOF aerodynamic model.

**Tech stack:**
- Physics: `bevy_rapier3d` (Rapier rigid-body dynamics)
- Rendering: `bevy` + `bevy_egui` HUD (feature-flagged off for training)
- Aerodynamics: custom coefficient-based model (coefficient tables in `.plane.ron` assets)
- ML: `burn` (pure Rust; no Python, no IPC)
- Asset format: RON (Rusty Object Notation); aerodynamic configs use Bevy's asset loader; reward/termination configs (`*.reward.ron` in `assets/training/`) are loaded directly via `ron::de::from_reader` — no Bevy asset server required

**Development philosophy:** Test-Driven Development (TDD) is mandatory. Write a failing test before writing any implementation code. The Red-Green-Refactor cycle governs all new features: red (failing test), green (minimal implementation to pass), refactor (clean up). The environment, aerodynamic model, and test suite must be solid before any controller or ML work begins.

---

## 2. Architecture

### Crate / Module Structure

```
src/
  aerodynamics/   # coefficient model, force/torque computation
  plane/          # PlaneConfig asset, FlightState, ControlInputs, physics systems
  controllers/    # FlightController trait, PID, LevelHold, Ascent, Orbit, HeadingHold, Wingman, L1 flight-plan, RL variants, ControllerKind
                  #   guidance.rs — shared L1 + orbit bank-command primitives; flight_plan.rs — FlightPlan asset; l1.rs — L1Controller
  environment/    # infinite ground collider + shader, plane spawner
  camera/         # FreeLook and Follow camera modes
  ui/             # egui HUD, extensible info panel
  training/
    ppo/            # PPO trainer, rollout buffer, ActorCritic model
    flight_env.rs   # shared 6-DOF Euler physics integration (integrate_state)
    level_hold_env.rs
    orbit_env.rs
```

### Key Types

| Type | Kind | Description |
|---|---|---|
| `PlaneConfig` | RON asset | Geometry, mass/inertia, aero coefficients, engine params, control limits |
| `FlightPlan` | RON asset | Ordered legs (`Waypoint`/`Orbit`) + L1 period/damping; loaded from `assets/plans/*.plan.ron` via the Bevy asset loader |
| `FlightState` | ECS component | Position, velocity, attitude (quat), angular velocity, α, β, airspeed, altitude |
| `ControlInputs` | ECS component | Aileron/elevator/rudder/throttle **or** rate commands (see Action Spaces) |
| `FlightController` | trait | `fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs` |
| `PidController<T>` | generic struct | PID with integral wind-up clamp and output limits |
| `TrainingEnv` | trait | `reset()`, `step(action) -> (obs, reward, done, info)` |
| `PlaneConfigHandle` | ECS component | Newtype wrapping `Handle<PlaneConfig>` — required because `Handle<T>` is not a `Component` in Bevy 0.18 |
| `ControllerKind` | enum | Factory selector for all controller types; `build()` does bumpless integral seeding |
| `OrbitController` | struct | 3-level cascade PID orbit around a fixed world-frame point |
| `RlOrbitController` | struct | Burn ActorCritic policy for orbit (obs dim=13); training-gated |
| `LevelHoldRewardConfig` / `OrbitRewardConfig` | plain structs | Reward weights, scales, alive bonus, failure penalty, and termination thresholds; loaded from `assets/training/*.reward.ron` at training startup |
| `WingmanController` | struct | Formation flight; holds a fixed offset in the leader's body frame |
| `AscentController` | struct | Climbs to target altitude then latches to level hold |
| `L1Controller` | struct | Follows a preset `FlightPlan` (waypoint sequences + orbit circles) via L1 nonlinear lateral guidance; built from the plan asset by `apply_flight_plan` |

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
visual = ["bevy/default", "bevy_egui", "bevy_rapier3d/debug-render-3d"]
training = ["burn"]
```

- `visual` (default): full Bevy rendering pipeline + egui HUD + Rapier debug renderer
- `training`: pulls in `burn`; no rendering, max-speed simulation (no `visual`)
- All tests run with `--no-default-features` (headless); no rendering in CI

### Orbit Controller Architecture

3-level cascade for circular orbit around a fixed world-frame point:

1. **Radial guidance** — position error in world frame → heading offset
2. **Heading guidance** — heading error → bank angle correction
3. **Bank feedforward** — `atan(V² / (g·R)) · direction_sign` (gravity-based centripetal law)
4. **Inner stabilization** — delegates to `LevelHoldController` with overridden targets

`from_state()` auto-centers the orbit perpendicular to current velocity for bumpless engagement.

The radial+heading+feedforward bank computation is factored into
`controllers/guidance.rs::orbit_bank_command()` and shared with `L1Controller`'s orbit legs.

### L1 Flight-Plan Controller

`L1Controller` (`controllers/l1.rs`) follows a preset `FlightPlan` — an ordered list of
`FlightPlanLeg`s — sequencing automatically between legs and delegating stabilization to an
inner `LevelHoldController` (same cascade pattern as Orbit/Wingman).

- **Waypoint legs** — straight-line **L1 nonlinear lateral guidance**
  (`guidance.rs::l1_straight_bank`): `L1 = (1/π)·damping·period·V`, lateral accel
  `2·V²/L1·sin(η)` → `bank = atan(a/g)`. Fly-by capture advances when horizontal distance
  `< capture_radius`.
- **Orbit legs** — reuse `orbit_bank_command`; advance after `turns` full revolutions
  (signed swept-angle accumulator). `turns: None` loiters forever (terminal hold).
- **End of plan** — a `turns: None` final orbit loiters indefinitely; otherwise the
  controller enters `L1Phase::Finished` and holds wings-level at the last setpoints.

`ControllerKind::FlightPlan::build()` cannot access the plan asset, so it returns a PID-orbit
fallback; the visual-mode `apply_flight_plan` system swaps in the real `L1Controller` once the
`FlightPlanHandle`'s `.plan.ron` asset finishes loading (mirrors the RL-load pattern). It has
no tuning pool — the inner loop uses `LevelHoldController` defaults; L1 period/damping live in
the plan asset.

### Training Physics (Self-Contained)

Training environments (`LevelHoldEnv`, `OrbitEnv`) do **not** use Bevy or Rapier. Instead:

- `training/flight_env.rs::integrate_state()` provides 6-DOF Euler integration
- Aerodynamics: shared `compute_aero_forces()` from `aerodynamics/`
- Result: deterministic rollouts, fast vectorized training, no ECS overhead
- `VecEnv` wraps any `TrainingEnv` to run N parallel episodes (seeds offset via `offset_rng_seed()`)
- Integration step is **60 Hz** (`dt = 1/60 s`) by design — this is the self-contained Euler
  integrator and is intentionally distinct from the 64 Hz Rapier fixed schedule used by the
  live sim (`main.rs`), the `observe_state` example, and the Bevy/Rapier tests.

### RL Inference Pattern

Both `RlLevelHoldController` and `RlOrbitController` follow the same pattern:

- Backend: `burn`'s `ActorCritic<NdArray>` (CPU; no GPU required at inference time)
- `Param` is not `Sync` → wrap model in `std::sync::Mutex`
- Deterministic inference: `model.mean_action()` (no sampling noise, reproducible)
- Action mapping: `throttle = (action[1] + 1.0) / 2.0` converts `[-1, 1]` network output to `[0, 1]`

### Adding a New `ControllerKind` (Checklist)

`ControllerKind::build()` always returns a valid PID fallback. This means missing visual-mode wiring compiles cleanly and fails silently at runtime — there is no compile-time guard.

**PID / non-RL controller:**

1. `kind.rs` — add variant to `ControllerKind` enum
2. `kind.rs` — `name()`: human-readable label
3. `kind.rs` — `ALL`: add to the cycle list under the correct feature gate
4. `kind.rs` — `build()`: factory arm
5. `main.rs` — `apply_controller_switch`: add to the correct tuning family match (`Orbit` or `LevelHold` pattern)
6. `main.rs` — `cycle_tune_profile`: same tuning family pattern

**RL controller — all of the above, plus:**

7. `kind.rs` — `model_dir()`: return the `models/` subdirectory name (e.g. `"orbit_residual"`)
8. `main.rs` — `#[cfg(feature = "training")]` import block: import `XxxController` and `XxxConfig`
9. `main.rs` — `apply_rl_controller_switch` guard: add variant to the `matches!()` pattern
10. `main.rs` — `apply_rl_controller_switch` match arm: call `::load()` with error fallback
11. `main.rs` — `apply_model_switch` match arm: same `::load()` call for HUD model cycling

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
| Reward/termination tuning | Constants live in `assets/training/*.reward.ron`, loaded by `train_ppo` at startup. Edit the RON to retune without recompiling. `Default` impls mirror the file values so tests never need file I/O. Each task loads its baseline profile (`level_hold`/`orbit`/`wu_orbit`) by default; pass `--reward-config <path>` to `train_ppo`, `train_bc`, or `evaluate_policy` to load an alternate profile (a missing file falls back to the compiled defaults with a warning). |
| Multi-agent | Architecture must support one `Box<dyn FlightController>` per plane entity. Exact multi-agent training strategy deferred. |

---

## 4. Maneuver Roadmap

1. **Level flight hold** — COMPLETE. Cascade PID: altitude outer → pitch inner, airspeed, roll, yaw. RL policy trained (`RlLevelHoldController`, obs dim=10).
2. **Ascent** — COMPLETE. Climbs to target altitude then hands off to level hold.
3. **Formation flight (wingman)** — COMPLETE. Follows leader at fixed body-frame offset (`WingmanController`).
4. **Circular orbit** — COMPLETE. 3-level cascade PID around world-frame point. RL policy trained (`RlOrbitController`, obs dim=13).
5. **Flight-plan following** — COMPLETE. `L1Controller` follows a preset `FlightPlan` (waypoint sequences + orbit circles) via L1 nonlinear lateral guidance. Replaces the former single-target `WaypointController`.
6. **Aerial refueling** — NEXT. Approach lead plane from the rear to a docking position.
7. *(extensible — add new `TrainingEnv` impls without changing core architecture)*

---

## 5. Key Dependencies

```toml
[dependencies]
bevy = { version = "0.18", default-features = false, features = ["bevy_asset"] }
bevy_rapier3d = { version = "0.33", default-features = false, features = ["dim3"] }
ron = "0.8"
serde = { version = "1", features = ["derive"] }
naga = { version = "26", features = ["termcolor"] }

[dependencies.bevy_egui]
version = "0.39"
optional = true

[dependencies.burn]
version = "0.20"
optional = true
features = ["wgpu", "ndarray", "autodiff", "train", "tui"]

[features]
default = ["visual"]
visual = ["bevy/default", "bevy_egui", "bevy_rapier3d/debug-render-3d"]
training = ["burn"]

[[bin]]
name = "train_ppo"
path = "src/bin/train_ppo.rs"
required-features = ["training"]
```

> `burn` features: `ndarray` = CPU backend for inference (no GPU required in production); `wgpu` = GPU backend for training; `tui` = training progress display.

> **Bevy feature flag note:** `default-features = false` disables all optional
> subsystems. `bevy_asset` **is** an optional feature of the `bevy` meta-crate
> and must be explicitly enabled — it is NOT automatically present. Always include
> `features = ["bevy_asset"]` in the bevy dependency so asset loading works in
> headless and training builds. Do not list rendering/audio/UI crates as features
> unless you intend to enable them.

### Bevy 0.18 API Notes (confirmed against installed crates)

**Event system — observer-based, no EventWriter/EventReader:**
- `#[derive(Event)]` still derives events.
- Fire: `commands.trigger(MyEvent(data))` — no `App::add_event` needed.
- Listen: `app.add_observer(|on: On<MyEvent>| { … })`.
- `EventWriter`, `EventReader`, and `App::add_event` do **not** exist in 0.18.

**Collider::halfspace returns Option:**
```rust
Collider::halfspace(Vec3::Y).unwrap()  // Vec3::Y is always valid
```

**RapierContext access:**
```rust
// System param (not Res<RapierContext>):
rapier_context: ReadRapierContext,
// Usage:
let Ok(ctx) = rapier_context.single() else { return };
ctx.contact_pair(e1, e2)
```

**MassProperties (dim3) — four fields required:**
```rust
MassProperties {
    local_center_of_mass: Vec3::ZERO,
    mass: cfg.mass,
    principal_inertia: cfg.inertia,
    principal_inertia_local_frame: Quat::IDENTITY,
}
```

**Fog — `DistanceFog`, not `FogSettings`:**
```rust
use bevy::pbr::{DistanceFog, FogFalloff};
commands.spawn(DistanceFog { color: …, falloff: FogFalloff::Linear { start, end }, ..default() });
```

**Mouse input — `AccumulatedMouseMotion`, not `EventReader`:**
- `EventReader` does not exist in 0.18 — even for engine/built-in events like `MouseMotion`.
- Mouse delta per frame: `Res<AccumulatedMouseMotion>` (from `bevy::input::mouse`).
  Access via `accumulated.delta: Vec2`.
- Mouse button state: `Res<ButtonInput<MouseButton>>` (unchanged).

**`EguiContexts::ctx_mut()` returns `Result`:**
```rust
let Ok(ctx) = contexts.ctx_mut() else { return };
// ctx is &mut egui::Context
```

**`EguiPlugin` — use `::default()`, not struct literal:**
```rust
// Correct — struct fields change between patch versions, default() is stable
app.add_plugins(EguiPlugin::default());
// Wrong — struct initializer requires all fields and some are deprecated
app.add_plugins(EguiPlugin { enable_multipass_for_primary_context: false });
```

---

## 6. Test Strategy

### TDD Workflow (mandatory)

1. **Red** — Write a failing test that captures the intended behavior. Run `cargo test --no-default-features` and confirm it fails for the right reason (not a compile error).
2. **Green** — Write the minimum implementation to make the test pass. No premature abstraction.
3. **Refactor** — Clean up duplication and structure without changing behavior. Tests must still pass.

**By component:**
- **New controller** — Unit test the control law (given `FlightState` → assert `ControlInputs` values) before writing the `FlightController` impl. Add an integration test in `tests/` before wiring into `ControllerKind`.
- **New `TrainingEnv`** — Test `reset()` initial state and `step()` reward/obs values before implementing `TrainingEnv`. Add a termination-condition test.
- **Aerodynamic changes** — Test the force/torque equations with known inputs before editing `compute_aero_forces()`.
- **New `ControllerKind` variant** — Test `build()` produces a non-panicking controller and that `name()` is non-empty before wiring into `main.rs`.

### Unit tests (`src/` modules)
- `PidController`: step response, integral wind-up clamp, output clamping
- Aerodynamic force computation: known α/V inputs → expected lift/drag/moment
- `FlightState` kinematics: attitude integration, angle-of-attack calculation

### Integration tests (`tests/`)
- `aero_physics.rs` — energy conservation over N steps in level flight
- `pid_convergence.rs` — pure PID closed-loop step response
- `spawn_reset.rs` — `TrainingEnv::reset()` produces correct initial `FlightState`
- `level_hold.rs` — level-hold cascade convergence to target altitude
- `wingman.rs` — formation flight relative-position tracking
- `ppo_training.rs` — RL trainer instantiation (training-gated; run with `--features training`)

### Rules
- All tests must pass with `cargo test --no-default-features`
- No rendering, no Bevy `App` window, no GPU resources in tests
- Tests are deterministic (fixed seed where randomness is needed)
- Run `cargo fmt` at the end of every editing session before committing
- Tests are written **before** the implementation they cover — never after
- A PR that adds implementation without a prior failing test is not accepted
- `cargo test --no-default-features` must be the first and last step of every change cycle

---

## 7. Invariants / Non-Negotiables

- Every plane is a full 6-DOF Rapier `RigidBody` — no simplified kinematics.
- `FlightController` is always `Box<dyn FlightController>` per entity — never hard-wired to a concrete type.
- `PlaneConfig` is loaded at runtime via Bevy's asset server — no compile-time plane data.
- All physics runs at Rapier's fixed timestep (64 Hz, `dt = 1/64 s`, set via `TimestepMode::Fixed` + `RapierPhysicsPlugin::in_fixed_schedule()` in `main.rs`); rendering interpolates between steps. The headless `observe_state` example and `tests/common/mod.rs` mirror this exact 64 Hz fixed-schedule setup.
- The ground is a flat infinite collider acting as a death plane — no terrain, no landing.

---

## 8. Implementation Roadmap

> Full plan: `plans/roadmap.md`
> WASM feasibility: `plans/wasm_feasibility.md`

M0–M12 (environment phase + level hold + formation flight + orbit + RL training) are complete. M13 (aerial refueling) is next.
