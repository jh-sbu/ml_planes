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
                  #   sim_control.rs — SimControlPlugin: headless controller-rebuild
                  #     systems (apply_initial_tuning/apply_controller_switch/
                  #     apply_flight_plan + RL load arms), shared by visual client & server
  environment/    # infinite ground collider + shader, plane spawner
                  #   spawner.rs — spawn_plane (auto-assigns PlaneId + PlaneIndex)
                  #   lifecycle.rs — LifecyclePlugin: Spawn/RemovePlaneCommand observers
                  #     + cleanup_orphaned_wingmen (headless-safe; no rendering deps)
  camera/         # FreeLook and Follow camera modes
                  #   recover_camera_on_target_loss — Follow(dead) → FreeLook
  ui/             # egui HUD, map panel, time-acceleration control, file-load dialog
                  #   lifecycle_panel.rs — "Planes" roster/spawn panel + N/Delete hotkeys
                  #   menu.rs — AppState (MainMenu/ScenarioSelect/InGame), main menu +
                  #     scenario-select screens, scenario spawn-on-enter / despawn-on-exit
  net/            # shared client/server protocol (feature = "net"), compiled into both ends
                  #   protocol.rs — NetProtocolPlugin: replicated components + client→server
                  #     command events, registered in identical order on both peers
                  #     (PROTOCOL_ID, DEFAULT_PORT)
                  #   server.rs — ServerSimPlugin (scenario spawn, `Replicated` marker,
                  #     sim-speed + FromClient command handlers; transport-free for tests) +
                  #     start_renet_server (feature = "server")
                  #   client.rs — ClientNetPlugin (NetInterpolation two-snapshot pose buffer →
                  #     interpolated Transform, ~2-tick render delay) + renet client transport +
                  #     ServerProcess (child-server kill-on-drop)
  sim_speed.rs    # SimSpeed — authoritative sim playback speed (pause/1x/5x/10x), replicated
  notifications.rs # Notifications resource — transient menu banner messages
  scenario.rs     # multi-plane .scenario.ron model + controller factory (drives
                  #   examples/observe_state.rs AND the visual menu's Start Scenario flow)
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
  mcp/              # MCP control client (feature = "mcp"): headless replicon client + rmcp
                    #   stdio server exposing the live sim to an LLM agent
                    #   snapshot.rs — SimSnapshot read-path mirror (collect_snapshot)
                    #   bridge.rs — ControlRequest write-path channel (drain_control_requests)
                    #   lifecycle.rs — poll_reconnect (auto-reconnect) + check_shutdown
                    #   service.rs — rmcp ServerHandler + #[tool] methods (rmcp quarantined here)
                    #   args.rs — --connect / --connect-timeout / --quiet
  bin/              # train_ppo, train_bc (required-features = training); evaluate_policy
                    #   (required-features = inference); ml_planes_server (server);
                    #   ml_planes_mcp (mcp)
```

### Key Types

| Type | Kind | Description |
|---|---|---|
| `PlaneConfig` | RON asset | Geometry, mass/inertia, aero coefficients, engine params, **`powerplant`**, control limits |
| `Powerplant` | enum (in `PlaneConfig`) | `JetFuel { capacity_kg, tsfc, fuel_type }` (burns mass, lightens, flames out) or `Electric { capacity, consumption }` (constant mass). Helpers: `capacity()`, `contributes_mass()`, `effective_mass(empty, remaining)`, `burn_rate(thrust)`. `#[serde(default)]` ⇒ generic-jet default |
| `FuelType` | enum | `JetA`/`Jp8`/`Jp5` + `properties()` (density, specific energy) + `label()`. For HUD/display; kerosene grades are ~identical so the grade does **not** enter the burn math |
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
| `RlLevelHoldController` | struct | Burn `ActorCritic` policy for level hold (obs dim=11); `inference`/`training`-gated |
| `RlOrbitController` | struct | Burn `ActorCritic` policy for orbit (obs dim=14); `inference`/`training`-gated |
| `RlOrbitResidualController` | struct | Burn `ActorCritic` policy emitting residual deltas added to the PID orbit baseline (obs dim=14); paired with `ResidualOrbitEnv` |
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
| `ControllerTelemetry` | ECS component (enum) | Read-only per-controller status (`Orbit { radial_error }`, `FlightPlan { leg, status }`, `Ascent`, `Wingman`, `None`, …) the server snapshots off the active controller each tick and **replicates**, so the thin client — which never steps a controller — can show it on the HUD. Default `None` (`controllers/telemetry.rs`) |
| `SimSpeed` | resource (enum) | Authoritative sim playback speed (pause/1×/5×/10×) in `src/sim_speed.rs`; replicated so the client displays server-side time acceleration rather than scaling its own clock. Set via `SetSimSpeedCommand` |
| `Scenario` / `ResolvedScenario` | RON model (`src/scenario.rs`) | Multi-plane `.scenario.ron`: per-plane initial state, optional `fuel_fraction` (0–1; default full tank → loaded mass), `.plane.ron` config, and a `ControllerSpec` (incl. `Wingman` peer references by name, optional inline tuning, RL specs). `resolve()` assigns `PlaneId`s and computes initial states; `build_controller()` builds the boxed controller; `ControllerSpec::kind()` maps a spec to its `ControllerKind`. RL specs **always parse** (so `default.scenario.ron` loads in every build) but only *build* on a native `--features inference` build; otherwise `build_controller` returns `Err` and the live spawner skips that plane. Drives `examples/observe_state.rs` via `--scenario` and the visual menu's Start Scenario flow (`environment::spawn_resolved_scenario`). CSV output ends with a `fuel_remaining` column. |

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

**Stated simplifications** (accepted scope — full list in the `aerodynamics/model.rs` header):
no side force (CY ≡ 0; sideslip makes moments, not forces); lift/drag rotated wind→body by α
only (β not in the force rotation); thrust body-fixed along +X; gyroscopic ω×(Iω) omitted on
*both* sides (Rapier's `gyroscopic_forces_enabled` defaults off, matching `integrate_state`);
dynamics use g = 9.81 while the ISA density model uses 9.80665 internally. Fuel-burn ordering
differs by one tick at flameout only: the live sim burns before applying forces
(`consume_fuel` → `apply_aerodynamic_forces`), training burns after.

**Sign conventions:** the body frame (+X fwd, +Y right, +Z up) mirrors NED about the roll and
pitch axes — positive roll torque = +Y wing **up**, positive pitch torque = nose **down**.
Derivatives copied from NED references must flip: `cm_alpha` and `cl_beta` are **positive**
here (stable), `cl_r` **negative**. Dampings (`cl_p`, `cm_q`, `cn_r`) and `cn_beta` keep their
usual signs. `tests/core/plane_assets.rs` pins these for every shipped airframe.

### Fuel & Charge (Powerplant)

Each `PlaneConfig` carries a `powerplant` (`plane/config.rs`). `FlightState.consumable_remaining`
tracks the remaining fuel (kg) or charge (kWh); it **defaults to `f32::INFINITY`** = an
unmodelled / unlimited tank, so ad-hoc states and most tests neither burn nor change mass
(`effective_mass` and `engine_thrust` treat non-finite as full). Spawn/reset opt into the model
by assigning a finite capacity. The live `spawn_plane`, the training reset, **and the
`observe_state`/`.scenario.ron` runner** all load a finite tank (`capacity × fuel_fraction`,
default full) and fly the resulting loaded mass — so `observe_state` (and the `/tune`,
`/observe-state` skills it drives) see the real loaded weight, not the dry `mass` field. Set a
per-plane `fuel_fraction` (0–1) in the scenario to fly a partial tank.

- **Thrust** comes from the shared `aerodynamics::engine_thrust(state, inputs, cfg)`: the usual
  `throttle · thrust_max · density_ratio(altitude)`, but **0 when empty** (flameout / dead battery).
  Used by both `compute_aero_forces` and the burn accounting so the live sim and training agree.
- **Burn** is thrust-specific: `Powerplant::burn_rate(thrust)` = `tsfc · thrust` (jet) or
  `consumption · thrust` (electric), consumed each tick. In training this lives in
  `integrate_state`; in the live sim, two FixedUpdate systems (`plane/systems.rs`) run before the
  physics step: `consume_fuel` (decrement) and `update_plane_mass` (rewrite Rapier
  `AdditionalMassProperties.mass` for mass-contributing powerplants).
- **Mass**: `effective_mass(empty_mass, remaining)` = `empty + remaining` for jets (airframe
  lightens as it burns), `empty` for electric (constant). `PlaneConfig.mass` is the **dry/empty**
  mass — the generic jet's 5000 kg dry + 2000 kg fuel ⇒ 7000 kg loaded. Inertia is held constant
  w.r.t. fuel (scope decision).
- **Fuel types** (`FuelType::{JetA,Jp8,Jp5}`) carry density + specific energy but, because
  kerosene grades differ <1%, are display-only — capacity is stored directly in kg. Volume-limited
  tanks / energy-based range are deferred.
- **HUD** (`ui/hud.rs`, visual): a fuel/charge readout labelled by powerplant kind
  ("Fuel … kg [Jet A]" vs "Charge … kWh") with a fraction bar.
- Example assets: `assets/planes/generic_jet.plane.ron` (JetFuel) and
  `electric_trainer.plane.ron` (Electric); `assets/scenarios/mixed_powerplant.scenario.ron` flies both.

**RL impact:** appending the fuel fraction grew the observation (level-hold 10→11, orbit
family 13→14, via `FlightState::fuel_fraction_obs` / `FUEL_OBS_SCALE`). This **invalidated all
prior checkpoints** — a 13/10-dim `.mpk` loads but panics at the matmul on the new obs (guarded
by `rl_inference::loaded_orbit_policy_runs_forward_pass`). Current valid (but **short, smoke-only**)
checkpoints live at `models/<task>/fuel_smoke_<task>.mpk` (the orbit one is also the
`ppo_orbit_1.mpk` test fixture). Pre-existing `models/**` checkpoints from before this change are
dimensionally stale — retrain before use. **The 2026-07 flight-model audit fixes (lateral
derivative sign flips `cl_beta`/`cl_r` + body-fixed thrust) changed the physics again: all
checkpoints still *load* (obs dims unchanged) but are behaviorally stale — retrain before
production use.** To restore production quality, retrain each task and
re-tune the (now heavier, 7000 kg loaded) jet:

```
# per task ∈ {level_hold, orbit, residual_orbit, lstm_orbit}: use the train-evaluate-optimize skill,
# or directly:  cargo run --release --features training --bin train_ppo -- --task <t> --plain
# then re-tune PID gains for the heavier jet via the `tune` skill (writes generic_jet.tuning.ron).
```

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

See §5 for the authoritative feature/dependency list; the summary:

```toml
[features]
default = ["client"]                    # the networked renderer (no local physics)
client = ["visual", "net"]
visual = ["bevy/default", "bevy_egui", "rfd"]
server = ["net"]                        # headless authoritative sim
net = ["dep:bevy_replicon", "dep:bevy_replicon_renet", "bevy/serialize"]
mcp = ["net", ...]                      # MCP control client (feature parity with `server`)
wasm = ["visual", "inference"]          # browser build; no net (deferred)
inference = ["burn/std", "burn/ndarray", "bevy/bevy_log"]
training = ["inference", "burn/wgpu", "burn/autodiff", "burn/train", "burn/tui"]
```

- `client` (**default**): `visual` + `net`. The renderer is a **pure client** — it runs no
  physics; planes arrive via replication and all mutations go out as commands. Plain `cargo run`
  is the networked client.
- `server`: headless authoritative sim (`ml_planes_server` bin) — Rapier + controllers + fuel at
  64 Hz, broadcasting replicated state and applying client commands. No rendering.
- `net`: the shared `src/net/` protocol + `bevy_replicon`/renet transport, compiled into both
  client and server. `bevy/serialize` gives `Vec3`/`Quat`/`Transform` serde for replication.
- `visual`: full Bevy rendering pipeline + egui HUD + `rfd` native file dialogs
- `inference`: `burn` CPU (`ndarray`) backend only — loads/runs trained RL policies
  headlessly, no training stack. Layered into both `visual` (via `wasm`) and `training`.
- `training`: builds on `inference`, adds `burn` GPU (`wgpu`), `autodiff`, `train`, and
  `tui`; no rendering, max-speed simulation
- `wasm`: `visual` + `inference` — browser build (CPU inference in the renderer); no `net`
- All tests run with `--no-default-features` (headless); no rendering in CI. Net/server tests
  need `--features "mcp server"` (see §6)

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
- **Networked build (`net`):** the client never spawns locally. The panel/hotkeys instead send
  `SpawnPlaneNetCommand` / `RemovePlaneNetCommand` (see Client/Server Networking); the server's
  `FromClient` handlers re-issue the *same* local `SpawnPlaneCommand` / `RemovePlaneCommand`
  triggers, so the observer logic above is the single authoritative spawn/remove path shared by
  local and networked play.

### Client/Server Networking (`net`)

The visual app is a **pure client**: it runs no physics and never steps a controller. The
authoritative 64 Hz Rapier sim, all `FlightController`s, and fuel burn live in the headless
`ml_planes_server` (`--features server`). Planes arrive by replication; every client-side
mutation goes out as a command. Shared code (`aerodynamics/`, `controllers/`, `plane/`,
`environment/` core, `scenario.rs`) is unchanged and compiled into both. The protocol lives in
`src/net/` and is registered identically on both peers by `NetProtocolPlugin` (same order, or
replicon rejects the connection); `PROTOCOL_ID` (currently **2** — v2 added `ControllerTelemetry`)
gates version-mismatched peers.

- **Replicated (server → client), in registration order:** `Transform`, `FlightState`,
  `ControlInputs`, `PlaneId`, `PlaneIndex`, `ControllerKind`, `SelectedTuningProfile`,
  `PlaneTuningPath`, `ControllerTelemetry`, and (`inference`-gated) `SelectedModel`. The client
  HUD/map/camera read these read-only. `PlaneTuningPath` lets the client rebuild a
  `PlaneTuningHandle` and reuse the existing profile enumeration for its dropdown.
- **Commands (client → server)** — all `add_client_event`, `Channel::Ordered`, received on the
  server as `On<FromClient<…>>` observers: `SwitchControllerCommand`, `SetTuningProfileCommand`,
  `ManualInputCommand` (sent every client frame while manually flying; latest-wins),
  `SpawnPlaneNetCommand`, `RemovePlaneNetCommand`, `SetSimSpeedCommand`, and (`inference`-gated)
  `SetModelCommand`. Each handler just mutates the component that `SimControlPlugin` (controller
  rebuilds) or the `LifecyclePlugin` observers already react to, so there is **one authoritative
  application path** shared with local play — no divergent server-only logic.
- **Rendering:** the client never predicts (deferred — see the plan's Out of Scope). It only
  **interpolates**: `ClientNetPlugin` buffers the last two replicated `FlightState` poses
  (`NetInterpolation`) and blends `Transform` at `now − RENDER_DELAY` (≈ 2 server ticks at 64 Hz),
  so a prev/curr pair is always available. Reading `FlightState` and writing `Transform` avoids
  any replication self-write feedback.
- **Transport:** renet UDP (`bevy_replicon_renet`). `start_renet_server` / `start_renet_client`
  are added by the binaries, **not** the plugins, so tests (`ServerSimPlugin` is transport-free)
  never bind a socket. `ServerProcess` wraps a client-launched local server child and kills+reaps
  it on drop (covers window-close / Quit paths that skip `OnExit(InGame)`).

### Visual App Flow (Main Menu + Scenarios)

The visual app boots into a **main menu** instead of a hardcoded scene. State is a
Bevy `States` enum `AppState` (`ui/menu.rs`, registered by `MenuPlugin` via `UiPlugin`)
— the only app-level `States` in the codebase. The menu flow differs by build:

**Networked client build (`feature = "net"`, the default `client`):** `AppState { MainMenu,
ScenarioSelect, ConnectEntry, Connecting, InGame }`. The client never simulates — physics +
spawning live on the dedicated server (`plans/client_server.md` Phase 5).

- **Main menu** (`MainMenu`): **Start New Server** → `ScenarioSelect`; **Connect to Server** →
  `ConnectEntry`; **Train** (only under `feature = "training"`, no-op placeholder); **Quit**
  (`MessageWriter<AppExit>` — Bevy 0.18 uses `MessageWriter`, not `EventWriter`).
- **Scenario select** (`ScenarioSelect`, host path): picking a scenario launches a local
  `ml_planes_server` child process (`launch_local_server`, stored in the `LocalServer` resource)
  hosting that scenario on `DEFAULT_PORT`, inserts `ConnectTarget(127.0.0.1:DEFAULT_PORT)`, and
  enters `Connecting`.
- **Connect entry** (`ConnectEntry`, join path): a `host:port` text field (`ConnectForm`,
  default `127.0.0.1:5555`); **Connect** parses via `parse_addr`, inserts `ConnectTarget`, and
  enters `Connecting`. `LocalServer` stays `None` (remote join).
- **Connecting** (`Connecting`): `OnEnter` runs `start_renet_client` (opens the renet transport
  to `ConnectTarget`) + `arm_connect_deadline`. `poll_connecting` advances to `InGame` once
  replicon's `ClientState == Connected`, or tears the attempt down (`teardown_connection`) and
  returns to `MainMenu` with a `Notifications` banner after `CONNECT_TIMEOUT_SECS` (5 s).
  **Cancel** also tears down + returns.
- **In game** (`InGame`): the client renders replicated planes (no local spawn). `OnExit`
  (`despawn_in_game_planes`, net variant) calls `teardown_connection` (disconnect the renet
  client, kill any `LocalServer` child), despawns all `PlaneId` entities, and resets the camera
  to `FreeLook`. **`Esc`** / the in-game **Main Menu** button return to the menu (Esc suppressed
  while egui wants keyboard). A client-launched local server is *also* killed on full client
  exit (window close / **Quit**, which skip `OnExit(InGame)`): `LocalServer` wraps a
  `net::ServerProcess` whose `Drop` kills+reaps the child when the resource is dropped at app
  shutdown, so no orphaned server survives the client.

**Non-net visual build (`wasm` / local-sim, `not(feature = "net")`):** `AppState { MainMenu,
ScenarioSelect, InGame }` — the original local single-player flow, unchanged. **Start Scenario**
→ `ScenarioSelect` → picking one sets `SelectedScenario` and enters `InGame`, where
`spawn_selected_scenario` spawns the resolved scenario locally via `spawn_resolved_scenario`;
`OnExit` despawns `PlaneId` entities and resets the camera.

- All gameplay HUD/input/`apply_*` systems are gated `run_if(in_state(AppState::InGame))` (in
  `ui/plugin.rs` and `main.rs`), so the menu screens stay clean. `main.rs::setup` no longer
  exists — the former hardcoded demo scene is now `assets/scenarios/default.scenario.ron`.

`spawn_resolved_scenario` (`environment/scenario_spawn.rs`) is the live-app counterpart to
`observe_state`'s hand-spawning: per plane it calls `build_controller`, seeds the Rapier body
via `load_spawn_config`, spawns through `spawn_plane` with `ControllerSpec::kind()`, and attaches
per-kind extras (`FormationOffset` for wingmen, `SelectedModel` for RL, `FlightPlanHandle` for
flight plans so `apply_flight_plan` re-installs the `L1Controller` after the tuning rebuild).
Scenario `config` paths use the observe_state `assets/...` convention and are stripped to the
Bevy asset-relative form for the live spawner.

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
spawn/termination logic and 14-dim observation (see the module header for documented
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
| Multi-agent | Architecture must support one `Box<dyn FlightController>` per plane entity. Exact multi-agent training strategy deferred. Cross-plane state is read via the per-tick `ControllerContext` snapshot (`plane/context.rs`), whose `find`/`others` do a **linear scan** — deliberately, since `N` is small, the snapshot is rebuilt every tick, and the only per-tick peer lookup (`WingmanController`'s leader) is not hot. Massive scenarios (hundreds/thousands of agents each doing per-tick peer lookups) are **deferred but not out of scope**; if they land, build an `id → index` map once in phase 1 of `run_flight_controllers` and pass it alongside the slice. `find`/`others` encapsulate access, so that stays a local change — see the `ControllerContext` doc comment. |

---

## 4. Maneuver Roadmap

1. **Level flight hold** — COMPLETE. Cascade PID: altitude outer → pitch inner, airspeed, roll, yaw. RL policy trained (`RlLevelHoldController`, obs dim=11).
2. **Ascent** — COMPLETE. Climbs to target altitude then hands off to level hold.
3. **Formation flight (wingman)** — COMPLETE. Follows leader at fixed body-frame offset (`WingmanController`).
4. **Circular orbit** — COMPLETE. 3-level cascade PID around world-frame point. Three RL variants: `RlOrbitController` (direct, obs dim=14), `RlOrbitResidualController` (residual over PID), and `RlLstmOrbitController` (recurrent, Wu-curriculum). Policies also reachable via behavior-cloning warm start.
5. **Flight-plan following** — COMPLETE. `L1Controller` follows a preset `FlightPlan` (waypoint sequences + orbit circles) via L1 nonlinear lateral guidance. Replaces the former single-target `WaypointController`.
6. **Aerial refueling** — NEXT. Approach lead plane from the rear to a docking position.
7. *(extensible — add new `TrainingEnv` impls without changing core architecture)*

---

## 5. Key Dependencies

```toml
[features]
default = ["client"]                              # networked renderer (no local physics)
client = ["visual", "net"]
visual = ["bevy/default", "bevy_egui", "rfd"]
wasm = ["visual", "inference"]
inference = ["burn/std", "burn/ndarray", "bevy/bevy_log"]
training = ["inference", "burn/wgpu", "burn/autodiff", "burn/train", "burn/tui"]
net = ["dep:bevy_replicon", "dep:bevy_replicon_renet", "bevy/serialize"]
server = ["net"]                                  # headless authoritative sim
# MCP control client (headless replicon client + rmcp stdio server). Feature parity with
# the server is required: `mcp` ↔ `server`, `mcp,inference` ↔ `server,inference`.
mcp = ["net", "dep:rmcp", "dep:tokio", "dep:crossbeam-channel", "dep:tracing-subscriber"]

[dependencies]
bevy = { version = "0.18", default-features = false, features = ["bevy_asset"] }
bevy_rapier3d = { version = "0.33", default-features = false, features = ["dim3"] }
ron = "0.8"
serde = { version = "1", features = ["derive"] }
bevy_egui = { version = "0.39", optional = true }
rfd = { version = "0.15", optional = true }
burn = { version = "0.20", optional = true, default-features = false }
bevy_replicon = { version = "0.40", optional = true }
bevy_replicon_renet = { version = "0.16", optional = true }
rmcp = { version = "~2.0.0", optional = true, features = ["server", "transport-io", "macros"] }
tokio = { version = "1", optional = true }
crossbeam-channel = { version = "0.5", optional = true }
tracing-subscriber = { version = "0.3", optional = true, features = ["env-filter"] }
naga = { version = "26", features = ["termcolor"] }

[[bin]]
name = "train_ppo"         # required-features = ["training"]
path = "src/bin/train_ppo.rs"
[[bin]]
name = "train_bc"          # required-features = ["training"] — BC pretraining
path = "src/bin/train_bc.rs"
[[bin]]
name = "evaluate_policy"   # required-features = ["inference"] — policy rollout/metrics
path = "src/bin/evaluate_policy.rs"
[[bin]]
name = "ml_planes_server"  # required-features = ["server"] — headless authoritative sim
path = "src/bin/server.rs"
[[bin]]
name = "ml_planes_mcp"     # required-features = ["mcp"] — MCP control client
path = "src/bin/mcp.rs"
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

Consolidated into **three** test binaries — `tests/core/`, `tests/net/`, `tests/rl/` — each a
directory-with-`main.rs` whose sibling `*.rs` files are plain modules (not separately compiled;
see `plans/test_compile_speed.md` for why: fewer link steps). Run one former file's tests with
the binary + a module filter, e.g. `cargo test --no-default-features --test core wingman::`;
`net`/`rl` additionally need their feature flags.

**`tests/core/` (core sim, `--no-default-features`):**
- `pid_convergence` — pure PID closed-loop step response
- `spawn_reset` — `TrainingEnv::reset()` produces correct initial `FlightState`
- `orbit_tune_sync` — orbit tuning-pool / gain-sync invariants
- `scenario` — `.scenario.ron` parse/resolve/build, per-plane `fuel_fraction` carried through `resolve()`, `ControllerSpec::kind()` mapping, `spawn_resolved_scenario` live spawn, `default.scenario.ron` resolve, + CSV header pinning (`ml_planes::scenario::CSV_HEADER`, incl. trailing `fuel_remaining`)
- `lifecycle` — runtime spawn/remove commands, auto-indexing, orphaned-wingman + camera cleanup (camera case is `visual`-gated, so it runs only under `just test-visual`)
- `sim_control` — relocated `SimControlPlugin` controller-rebuild systems, headless (runs in core)
- `controller_telemetry` — each controller's `FlightController::telemetry()` accessor / `ControllerTelemetry` shape (runs in core)
- **module-gated `#[cfg(sim_enabled)]` in `tests/core/main.rs`** (compile out on net-without-server builds):
  - `aero_physics` — energy conservation over N steps in level flight
  - `level_hold` — level-hold cascade convergence to target altitude
  - `heading_hold` — heading-hold convergence to a commanded heading
  - `flight_plan` — L1 flight-plan leg sequencing / waypoint capture
  - `wingman` — formation flight relative-position tracking
  - `fuel` — live-sim fuel: spawn-time tank load (`fuel_fraction`), `consume_fuel` burn + `update_plane_mass`, shipped-asset powerplant parse

**`tests/net/` (`net`/`server`/`mcp`; the whole binary compiles out without `net`):**
- `net_serde` — serde round-trips for the replicated types + command events (`net`-gated)
- `net_protocol` — `NetProtocolPlugin` registration / replication-rule invariants (`net`-gated)
- `client_net` — client interpolation math (`NetInterpolation` buffer → interpolated `Transform`) (`net`-gated)
- `local_server` — in-process replicon client↔server command + replicated-state round-trip (`server`-gated)
- `server_sim` — `ServerSimPlugin` boot / scenario spawn / `FromClient` command handlers, transport-free (`server`-gated)
- `mcp_snapshot` — MCP read-path snapshot mirror, transport-free (`mcp`-gated)
- `mcp_bridge` — MCP write-path command bridge drain, transport-free (`mcp`-gated)
- `mcp_lifecycle` — MCP auto-reconnect (`poll_reconnect`) + clean shutdown (`check_shutdown`), transport-free (`mcp`-gated)
- `mcp_e2e` — MCP end-to-end over real UDP: boots an `ml_planes_server` child, inspect + spawn/remove round-trip (`mcp`+`server`-gated, `#[ignore]`; run with `--features "mcp server" --test net -- --ignored mcp_e2e`)

**`tests/rl/` (`inference`/`training`; the whole binary compiles out without an RL backend):**
- `rl_inference` — RL controller load + deterministic inference (`inference`/`training`-gated)
- `ppo_training` — RL trainer instantiation (training-gated; run with `--features training --test rl ppo_training::`)

**Visual/UI unit tests (`visual`-gated, in `src/`; run with `just test-visual`):** `src/ui/**` and
`src/camera/**` are `#[cfg(feature = "visual")]` modules (`src/lib.rs`), so their `mod tests` are
**invisible to every `--no-default-features` recipe** — `just test` and `just test-all` silently
skip them. `just test-visual` (`--features visual`) is the only recipe that compiles them:
- `ui/lifecycle_panel` — spawn-pose helpers + Planes-panel layout (`roster_max_height`, plus
  headless-`egui::Context` layout assertions that the 100-plane roster stays capped and clear of
  the HUD)
- `ui/map` — map projection / zoom / fit math; `ui/hud` — camera-follow index resolution;
  `ui/file_load` — asset-relative path munging; `ui/menu` — scenario discovery + `parse_addr`
- `camera/systems` — follow-camera orbit offset math
- also picks up the `visual`-gated `lifecycle::camera_recovers_to_free_look_when_followed_plane_removed`
  in `tests/core` and `src/main.rs`'s `cycle_index_wraps_both_directions` (a bin-target test no
  other recipe runs)

These are pure math / headless egui only — no window, no GPU (`visual` costs a `bevy/default`
wgpu+winit *link*, not a display at runtime). Since `test-visual` omits `net`, `sim_enabled` holds,
so it also re-runs the core sim suite and compile-checks the local-sim/`wasm` bin path.

### Rules
- All tests must pass with `cargo test --no-default-features`
- **The complete supported test matrix is `just test-all`** (justfile at repo root): core
  (`--no-default-features`), net parity (`--features "mcp server"`), and RL inference
  (`--features inference`). `just test-training` (heavy wgpu build, needs a GPU) and
  `just test-visual` cover the training- and `visual`-gated suites separately. Feature combos
  outside the justfile are **not supported test configurations** — a green ad-hoc run (e.g. bare
  `--features mcp`, where the sim tests compile out) is not coverage. Run the full matrix before
  committing when net/mcp/RL code was touched.
- **Run `just test-visual` after any UI/camera work.** `src/ui/**` and `src/camera/**` are
  `visual`-gated, so `test-all` does not compile them at all — a green `test-all` says **nothing**
  about the UI tests, and a broken one is invisible until someone runs `test-visual`. It is kept
  out of `test-all` only because `visual` pulls `bevy/default` (wgpu/winit/GTK link cost), not
  because the tests need a display.
- **Sim-dependent tests require the sim chain (`sim_enabled` cfg).** The 6-DOF FixedUpdate chain
  in `PlanePlugin` compiles in only under `any(not(feature = "net"), feature = "server")`. A
  `net`-without-`server` build (e.g. bare `--features mcp`, since `mcp` enables `net` but not
  `server`) runs no physics, leaving `FlightState` at its `Default`. `build.rs` derives a
  `sim_enabled` cfg from that same condition, and the physics/controller integration tests
  (`aero_physics`, `flight_plan`, `fuel`, `heading_hold`, `level_hold`, `wingman`) are
  module-gated `#[cfg(sim_enabled)]` on their `mod` declarations in `tests/core/main.rs`, so
  they **compile out** (not fail) on such a build. A spurious
  "altitude=0 / plane reached ground" is this gating, **not** a physics bug — never touch the
  aero model to chase it. To run these tests against a networked build add the server feature
  (keep `--no-default-features` — the default features build the *app binary*, whose rendering
  plugins panic headless; the `visual` **tests** themselves are headless-safe, see `test-visual`):
  `cargo test --no-default-features --features "mcp server"` (mirrors the MCP↔server
  feature-parity rule).
- No rendering, no Bevy `App` window, no GPU resources in tests — including the `visual` suite,
  which stays pure math / headless `egui::Context` (drive layout via `ctx.run(RawInput { .. })`,
  never a real window). A test that needs a display is out of scope.
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
- All physics runs at Rapier's fixed timestep (64 Hz, `dt = 1/64 s`, set via `TimestepMode::Fixed` + `RapierPhysicsPlugin::in_fixed_schedule()`). In the client/server split the authoritative sim runs in the **server** (`ml_planes_server`, `src/bin/server.rs`); the `net` client (`main.rs`) runs no physics and renders interpolated replicated state. The headless `observe_state` example and `tests/common/mod.rs` mirror this exact 64 Hz fixed-schedule setup.
- The ground is a flat infinite collider acting as a death plane — no terrain, no landing.

---

## 8. Implementation Roadmap

> Full plan: `plans/roadmap.md`
> WASM feasibility: `plans/wasm_feasibility.md`

M0–M12 (environment phase + level hold + formation flight + orbit + RL training) are complete. M13 (aerial refueling) is next.
