# ml_planes — Developer Reference

## 1. Project Overview

**Goal:** A physically simulated training sandbox flight simulation for exploring traditional and ML-based flight control schemes. Autopilot agents perform individual maneuvers (level hold, formation flight, aerial refueling) against a realistic 6-DOF aerodynamic model.

**Tech stack:**
- Physics: `bevy_rapier3d` (Rapier rigid-body dynamics)
- Rendering: `bevy` + `bevy_egui` HUD (feature-flagged off for training)
- Aerodynamics: custom coefficient-based model (coefficient tables in `.plane.ron` assets)
- ML: `burn` (pure Rust; no Python, no IPC). CPU `ndarray` backend at inference (the `inference` feature); GPU `wgpu` + `autodiff` for training (the `training` feature)
- Asset format: RON (Rusty Object Notation). `.plane.ron` (aero config) and `.plan.ron` (flight plan) use Bevy's asset loader; `.tuning.ron` (PID gain pools), `*.reward.ron` (reward/termination, in `assets/training/`), `*.ppo.ron` (PPO hyperparameters, in `assets/training/`), and multi-plane `*.scenario.ron` (in `assets/scenarios/`) are loaded directly via `ron::de` (`implicit_some` enabled for scenarios) — no Bevy asset server required

**Development philosophy:** Test-Driven Development (TDD) is mandatory. Write a failing test before writing any implementation code. The Red-Green-Refactor cycle governs all new features: red (failing test), green (minimal implementation to pass), refactor (clean up). The environment, aerodynamic model, and test suite must be solid before any controller or ML work begins.

---

## 2. Architecture

### Crate / Module Structure

```
src/
  aerodynamics/   # coefficient model (model.rs), force/torque computation
                  #   atmosphere.rs — ISA air_density(altitude)/density_ratio + standard constants
  plane/          # PlaneConfig asset, FlightState, ControlInputs, physics systems
                  #   context.rs — ControllerContext (shared leader state for Wingman)
  controllers/    # FlightController trait + ControllerKind factory. Controllers:
                  #   manual.rs, level_hold.rs, heading_hold.rs, ascent.rs, orbit.rs,
                  #   wingman.rs, l1.rs (L1 flight-plan), + 4 RL variants
                  #   (rl_level_hold, rl_orbit, rl_orbit_residual, rl_lstm_orbit)
                  #   pid.rs — PidController<T> utility struct (NOT a FlightController)
                  #   guidance.rs — shared L1 + orbit bank-command primitives
                  #   flight_plan.rs — FlightPlan asset; tuning.rs — per-plane gain pools
                  #   selected_model.rs — model hot-swap; orbit_marker.rs; component.rs
  environment/    # infinite ground collider + shader, plane spawner
                  #   spawner.rs — spawn_plane (auto-assigns PlaneId + PlaneIndex)
                  #   lifecycle.rs — LifecyclePlugin: Spawn/RemovePlaneCommand observers
                  #     + cleanup_orphaned_wingmen (headless-safe; no rendering deps)
  camera/         # FreeLook and Follow camera modes
                  #   recover_camera_on_target_loss — Follow(dead) → FreeLook
  ui/             # egui HUD, map panel, time-acceleration control, file-load dialog
                  #   lifecycle_panel.rs — "Planes" roster/spawn panel + N/Delete hotkeys
  scenario.rs     # multi-plane .scenario.ron model + controller factory (drives examples/observe_state.rs)
  training/
    env.rs          # TrainingEnv + CurriculumEnv traits, Observation/SpawnSpec/StepInfo
    flight_env.rs   # shared 6-DOF Euler integrator (integrate_state); pub(crate)-private
    level_hold_env.rs, orbit_env.rs, orbit_residual_env.rs, wu_orbit_env.rs
    vec_env.rs      # VecEnv<E> — N parallel episodes
    reward_config.rs, wu_orbit_reward.rs, ppo_config.rs   # RON-backed configs
    bc.rs           # behavior cloning (DemonstrationEnv, collect_demonstrations, BcDataset)
    eval.rs, eval_metrics.rs   # evaluate_policy, EvaluationSummary, TaskMetrics
    ppo/            # MLP track: model.rs/trainer.rs/buffer.rs (ActorCritic, PpoTrainer)
                    # LSTM track: lstm_model.rs/lstm_trainer.rs/lstm_buffer.rs
                    # csv_log.rs — training-metric CSV logging
  bin/              # train_ppo, train_bc, evaluate_policy (all required-features = training)
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
| `ControllerKind` | enum | Factory selector for all controller types; `build()` does bumpless integral seeding. `ALL` cycle list is feature-gated (RL variants only under `inference`/`training`) |
| `ManualController` | struct | Passthrough — applies raw keyboard/stick inputs, no autopilot |
| `OrbitController` | struct | 3-level cascade PID orbit around a fixed world-frame point |
| `HeadingHoldController` | struct | Holds a configurable heading via an inner level-hold cascade |
| `RlLevelHoldController` | struct | Burn `ActorCritic` policy for level hold (obs dim=10); `inference`/`training`-gated |
| `RlOrbitController` | struct | Burn `ActorCritic` policy for orbit (obs dim=13); `inference`/`training`-gated |
| `RlOrbitResidualController` | struct | Burn `ActorCritic` policy emitting residual deltas added to the PID orbit baseline (obs dim=13); paired with `ResidualOrbitEnv` |
| `RlLstmOrbitController` | struct | Recurrent `LstmActorCritic` orbit policy (Wu et al. FC-LSTM-FC); carries `LstmHiddenState` across steps; paired with `WuOrbitEnv` |
| `LevelHoldRewardConfig` / `OrbitRewardConfig` | plain structs | Reward weights, scales, alive bonus, failure penalty, and termination thresholds; loaded from `assets/training/*.reward.ron` at training startup |
| `WuOrbitRewardConfig` / `CurriculumStage` | plain structs/enum | Wu et al. multiplicative-Gaussian orbit reward (`R^TT × R^PS × R^RS`) + 3-stage curriculum; from `wu_orbit.reward.ron` |
| `PpoHyperparams` | plain struct | PPO training-loop config (gamma, gae_lambda, clip, lr, …); from `assets/training/*.ppo.ron` |
| `WingmanController` | struct | Formation flight; holds a fixed offset in the leader's body frame via a heading-damped lateral cascade (cross-track → heading → bank) |
| `AscentController` | struct | Climbs to target altitude then latches to level hold |
| `L1Controller` | struct | Follows a preset `FlightPlan` (waypoint sequences + orbit circles) via L1 nonlinear lateral guidance; built from the plan asset by `apply_flight_plan` |
| `VecEnv<E>` | struct | Wraps any `TrainingEnv` to run N parallel episodes (seeds offset per env) |
| `DemonstrationEnv` / `BcDataset` | trait / struct | Behavior cloning: `collect_demonstrations()` rolls out a PID expert into a supervised dataset for `train_bc` pretraining |
| `EvaluationSummary` / `TaskMetrics` | structs | Policy-evaluation output from `evaluate_policy` (success rate + per-metric families) |
| `Scenario` / `ResolvedScenario` | RON model (`src/scenario.rs`) | Multi-plane `.scenario.ron`: per-plane initial state, `.plane.ron` config, and a `ControllerSpec` (incl. `Wingman` peer references by name, optional inline tuning, cfg-gated RL specs). `resolve()` assigns `PlaneId`s and computes initial states; `build_controller()` builds the boxed controller. Drives `examples/observe_state.rs` via `--scenario`. |

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

Dynamic pressure: `q̄ = ½·ρ(h)·V²`

Air density `ρ` varies with altitude via the **International Standard Atmosphere**
(`aerodynamics/atmosphere.rs::air_density`): troposphere (0–11 km) barometric power
law over a −6.5 K/km lapse, then isothermal stratosphere (11–20 km). `air_density`
reads `FlightState.altitude`, so every `q̄`-derived force (lift, drag, moments) thins
with altitude automatically — no call-site changes. Air-breathing **thrust** also
scales by the density ratio `ρ(h)/ρ₀` (`density_ratio`), so engines lose thrust at
altitude. This single `compute_aero_forces` path is shared by the live Rapier sim and
the self-contained training integrator, so both see the same altitude physics.

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
| Residual (RL) | `[Δelevator, Δthrottle, Δaileron, Δrudder]` | `[-1, 1]` |

The aerodynamic model maps whichever representation to net force and torque before
applying to Rapier. The **residual** space (`RlOrbitResidualController` / `ResidualOrbitEnv`)
adds each clamped delta on top of the PID orbit controller's output, so the policy only
learns corrections to a working baseline. Training environments emit direct actions in
`[elevator, throttle, aileron, rudder]` order (`direct_action_to_inputs`).

### Feature Flags

```toml
[features]
default = ["visual"]
visual = ["bevy/default", "bevy_egui", "rfd"]
wasm = ["visual", "inference"]
inference = ["burn/std", "burn/ndarray"]
training = ["inference", "burn/wgpu", "burn/autodiff", "burn/train", "burn/tui"]
```

- `visual` (default): full Bevy rendering pipeline + egui HUD + `rfd` native file dialogs
- `inference`: `burn` CPU (`ndarray`) backend only — loads/runs trained RL policies
  headlessly, no training stack. Layered into both `visual` (via `wasm`) and `training`.
- `training`: builds on `inference`, adds `burn` GPU (`wgpu`), `autodiff`, `train`, and
  `tui`; no rendering, max-speed simulation
- `wasm`: `visual` + `inference` — browser build (CPU inference in the renderer)
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

### Wingman Controller Architecture

`WingmanController` (`controllers/wingman.rs`) holds a fixed formation slot in the
leader's body frame, reading the leader's live state from `ControllerContext` each
tick. Three channels feed an inner `LevelHoldController` (same cascade pattern as
Orbit/L1):

1. **Altitude** — inner `target_altitude` ← world Y of the desired slot.
2. **Range** (fore-aft along leader heading) — `range_pid` → Δairspeed → inner `target_airspeed`.
3. **Lateral** (cross-track along the leader's right-wing axis) — a **heading-damped
   two-stage cascade** mirroring the orbit/L1 lateral guidance:
   - Stage 1: cross-track error → `lateral_pid` → demanded heading offset (crab angle).
     Gains match the orbit `radial_pid` (`kp=0.002, kd=0.01`, ±0.5 rad).
   - Stage 2: heading error (demanded heading vs. own ground track, same
     `cross.atan2(dot)` math as `guidance::orbit_bank_command`, reusing
     `ground_heading`) → `heading_pid` → bank, with `target_roll = -heading_pid(...)`
     (same sign convention as orbit). `heading_pid` mirrors orbit (`kp=0.7, kd=0.1`,
     ±π/3); its `kd` supplies the **heading-rate damping**.

Driving bank from heading error (rather than mapping cross-track straight to bank)
makes the on-slot equilibrium stable over multi-minute holds and lets the wingman
re-capture the slot from an offset. The earlier direct position→bank loop lacked
heading damping and **also used the divergent feedback sign**, so it spiraled away;
both are guarded by `tests/wingman.rs::wingman_holds_slot_over_three_minutes` (180 s
on-slot hold) and the `lateral_error_commands_restoring_bank_direction` /
`heading_misalignment_commands_corrective_bank` unit tests. A too-aggressive
`lateral_pid.kp` (≫0.002) re-introduces a sustained lateral oscillation, since the
inner heading/roll loop is comparatively slow.

### Runtime Plane Lifecycle

Planes can be added/removed at runtime via observer commands (`environment/lifecycle.rs`,
`LifecyclePlugin`, registered headless + visual since it has no rendering deps):

- `SpawnPlaneCommand { spec, kind, config_path }` / `RemovePlaneCommand(Entity)` — fired
  with `commands.trigger(..)` (Bevy 0.18 observer events). The spawn observer builds the
  controller from the spawn-state via `ControllerKind::build()` (valid PID fallback for
  Wingman/FlightPlan/RL) and calls `spawn_plane`; the remove observer despawns (tolerant of
  stale entities). Ground-contact auto-removal is **out of scope** — `PlaneGroundContactEvent`
  still fires but nothing observes it.
- **Automatic indexing:** `spawn_plane` now takes a `config_path: &str` (per-plane
  `.plane.ron`, no longer hardcoded) and inserts `PlaneIndex(plane_id.0)` itself — every
  spawned plane is automatically visible to camera cycling, the map, and the HUD. Callers no
  longer hand-insert `PlaneIndex`. `generic_jet_spawn_config()` supplies the shared spawn-time
  mass/inertia.
- **Removal cleanup:** `cleanup_orphaned_wingmen` (Update, headless-safe) flips a wingman whose
  `leader_id` is no longer live to `ControllerKind::LevelHold`; `recover_camera_on_target_loss`
  (visual) drops the camera from `Follow(dead)` back to `FreeLook` so it/the HUD don't freeze.
- **UI (visual):** the bottom-left **Planes** panel (`ui/lifecycle_panel.rs`) lists live planes
  with Remove buttons and a spawn form (kind dropdown + config path), plus hotkeys **`N`**
  (spawn ahead of camera) and **`Delete`** (remove followed); both suppressed while egui has
  keyboard focus.

### Training Physics (Self-Contained)

Training environments (`LevelHoldEnv`, `OrbitEnv`, `ResidualOrbitEnv`, `WuOrbitEnv`) do
**not** use Bevy or Rapier. Instead:

- `training/flight_env.rs::integrate_state()` provides 6-DOF Euler integration
- Aerodynamics: shared `compute_aero_forces()` from `aerodynamics/`
- Result: deterministic rollouts, fast vectorized training, no ECS overhead
- `VecEnv` wraps any `TrainingEnv` to run N parallel episodes (seeds offset via `offset_rng_seed()`)
- Integration step is **60 Hz** (`dt = 1/60 s`) by design — this is the self-contained Euler
  integrator and is intentionally distinct from the 64 Hz Rapier fixed schedule used by the
  live sim (`main.rs`), the `observe_state` example, and the Bevy/Rapier tests.

### RL Inference Pattern

All four RL controllers (`RlLevelHoldController`, `RlOrbitController`,
`RlOrbitResidualController`, `RlLstmOrbitController`) follow the same pattern:

- Backend: `burn`'s `ActorCritic<NdArray>` (CPU; no GPU required at inference time).
  `RlLstmOrbitController` uses the recurrent `LstmActorCritic` instead.
- `Param` is not `Sync` → wrap model in `std::sync::Mutex`
- Deterministic inference: `model.mean_action()` (no sampling noise, reproducible)
- Action mapping: `throttle = (action[1] + 1.0) / 2.0` converts `[-1, 1]` network output to `[0, 1]`
- `RlLstmOrbit` additionally threads `LstmHiddenState` from one step into the next, so the
  policy must be stepped sequentially within an episode.
- All are gated behind `inference` (loaded in the renderer) or `training`; the non-ML build
  excludes them from `ControllerKind::ALL` entirely.

### LSTM Recurrent RL

A second PPO track trains recurrent policies for partially-observed orbit control:

- `LstmActorCritic` (`ppo/lstm_model.rs`) — FC → LSTM → FC (Wu et al. 2025 architecture).
- `LstmPpoTrainer` (`ppo/lstm_trainer.rs`) — trains over `LstmSequence`s held in
  `LstmRolloutBuffer`; requires a `CurriculumEnv` so it can auto-advance the curriculum.
- Driven by the `lstm_orbit` task in `train_ppo`; inference via `RlLstmOrbitController`.

### Curriculum Training

`WuOrbitEnv` (`training/wu_orbit_env.rs`) implements the optional `CurriculumEnv` trait
(`advance_curriculum`, `curriculum_stage_name`, `next_stage_threshold`). `LstmPpoTrainer`
advances stages automatically once the mean episode return crosses the stage threshold.
Stages and the multiplicative-Gaussian reward (`R^TT × R^PS × R^RS`, `wu_orbit_reward.rs`)
are configured in `assets/training/wu_orbit.reward.ron`. The env shares `OrbitEnv`'s
spawn/termination logic and 13-dim observation (see the module header for documented
deviations from the paper).

### Behavior Cloning (Supervised Pretraining)

`train_bc` pretrains a policy by imitating a PID expert before any PPO:

- `DemonstrationEnv` + `collect_demonstrations()` roll out the expert into a `BcDataset`
  (`training/bc.rs`).
- The supervised model is saved under `models/<task>/<stem>.mpk`, then handed to
  `train_ppo --init-from <path>` as a warm start. Supported for `level_hold` / `orbit`.

### Policy Evaluation

`evaluate_policy` (`src/bin/evaluate_policy.rs`, ndarray/CPU only) rolls a checkpoint
out over N episodes and reports an `EvaluationSummary` (success rate + `TaskMetrics`
families). Supports `--task {level_hold|orbit|residual_orbit|lstm_orbit}` and, for
`lstm_orbit`, `--curriculum-stage {coarse|heading_fine|full}`.

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
8. `main.rs` — `#[cfg(any(feature = "inference", feature = "training"))]` import block: import `XxxController` and `XxxConfig`
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
| Reward/termination tuning | Constants live in `assets/training/*.reward.ron`, loaded by `train_ppo` at startup. Edit the RON to retune without recompiling. `Default` impls mirror the file values so tests never need file I/O. Each task loads its baseline profile (`level_hold`/`orbit`/`wu_orbit`) by default; pass `--reward-config <path>` to `train_ppo`, `train_bc`, or `evaluate_policy` to load an alternate profile (a missing file falls back to the compiled defaults with a warning). PPO loop hyperparameters are similarly overridable via `--ppo-config <path>` (`assets/training/*.ppo.ron`). |
| Multi-agent | Architecture must support one `Box<dyn FlightController>` per plane entity. Exact multi-agent training strategy deferred. |

---

## 4. Maneuver Roadmap

1. **Level flight hold** — COMPLETE. Cascade PID: altitude outer → pitch inner, airspeed, roll, yaw. RL policy trained (`RlLevelHoldController`, obs dim=10).
2. **Ascent** — COMPLETE. Climbs to target altitude then hands off to level hold.
3. **Formation flight (wingman)** — COMPLETE. Follows leader at fixed body-frame offset (`WingmanController`).
4. **Circular orbit** — COMPLETE. 3-level cascade PID around world-frame point. Three RL variants: `RlOrbitController` (direct, obs dim=13), `RlOrbitResidualController` (residual over PID), and `RlLstmOrbitController` (recurrent, Wu-curriculum). Policies also reachable via behavior-cloning warm start.
5. **Flight-plan following** — COMPLETE. `L1Controller` follows a preset `FlightPlan` (waypoint sequences + orbit circles) via L1 nonlinear lateral guidance. Replaces the former single-target `WaypointController`.
6. **Aerial refueling** — NEXT. Approach lead plane from the rear to a docking position.
7. *(extensible — add new `TrainingEnv` impls without changing core architecture)*

---

## 5. Key Dependencies

```toml
[features]
default = ["visual"]
visual = ["bevy/default", "bevy_egui", "rfd"]
wasm = ["visual", "inference"]
inference = ["burn/std", "burn/ndarray"]
training = ["inference", "burn/wgpu", "burn/autodiff", "burn/train", "burn/tui"]

[dependencies]
bevy = { version = "0.18", default-features = false, features = ["bevy_asset"] }
bevy_rapier3d = { version = "0.33", default-features = false, features = ["dim3"] }
ron = "0.8"
serde = { version = "1", features = ["derive"] }
bevy_egui = { version = "0.39", optional = true }
rfd = { version = "0.15", optional = true }
burn = { version = "0.20", optional = true, default-features = false }
naga = { version = "26", features = ["termcolor"] }

[[bin]]
name = "train_ppo"        # required-features = ["training"]
path = "src/bin/train_ppo.rs"
[[bin]]
name = "train_bc"         # required-features = ["training"] — BC pretraining
path = "src/bin/train_bc.rs"
[[bin]]
name = "evaluate_policy"  # required-features = ["training"] — policy rollout/metrics
path = "src/bin/evaluate_policy.rs"
```

> `burn` features are selected per crate-feature (`default-features = false`): `ndarray` =
> CPU backend (enabled by `inference`, used in production and WASM); `wgpu` = GPU backend +
> `autodiff`/`train` for training; `tui` = training progress display. `train_ppo` tasks:
> `level_hold`, `orbit`, `residual_orbit`, `lstm_orbit`.

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
- `heading_hold.rs` — heading-hold convergence to a commanded heading
- `wingman.rs` — formation flight relative-position tracking
- `flight_plan.rs` — L1 flight-plan leg sequencing / waypoint capture
- `orbit_tune_sync.rs` — orbit tuning-pool / gain-sync invariants
- `scenario.rs` — `.scenario.ron` parse/resolve/build + CSV header pinning (`ml_planes::scenario::CSV_HEADER`)
- `lifecycle.rs` — runtime spawn/remove commands, auto-indexing, orphaned-wingman + camera cleanup (camera case is `visual`-gated)
- `rl_inference.rs` — RL controller load + deterministic inference (`inference`/`training`-gated)
- `ppo_training.rs` — RL trainer instantiation (training-gated; run with `--features training`)

### Rules
- All tests must pass with `cargo test --no-default-features`
- No rendering, no Bevy `App` window, no GPU resources in tests
- Tests are deterministic (fixed seed where randomness is needed)
- Run `cargo fmt` at the end of every editing session before committing — always run it, even if you believe the code is already correctly formatted. Never skip it based on visual inspection.
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
