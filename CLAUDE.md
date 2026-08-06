# ml_planes — Developer Reference

## 1. Project Overview

**Goal:** A physically simulated training sandbox flight simulation for exploring traditional and ML-based flight control schemes. Autopilot agents perform individual maneuvers (level hold, formation flight, aerial refueling) against a realistic 6-DOF aerodynamic model.

**Tech stack:**
- Physics: `bevy_rapier3d` (Rapier rigid-body dynamics)
- Rendering: `bevy` + `bevy_egui` HUD (feature-flagged off for training)
- Aerodynamics: custom coefficient-based model (coefficient tables in `.plane.ron` assets)
- ML: `burn` (pure Rust; no Python, no IPC). CPU `ndarray` backend at inference (the `inference` feature) and, by default, at training too (the `training` feature adds `autodiff`); GPU `wgpu` training is opt-in via the `wgpu` feature
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
                  #   wingman.rs, l1.rs (L1 flight-plan), + 5 RL variants
                  #   (rl_level_hold, rl_heading_hold, rl_orbit, rl_orbit_residual,
                  #   rl_lstm_orbit)
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
    level_hold_env.rs, heading_hold_env.rs, orbit_env.rs, orbit_residual_env.rs, wu_orbit_env.rs
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
| `FlightController` | trait | `fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs`, plus `telemetry()`/`targets()` (read-only status / editable setpoints, both default to publishing nothing) and `apply_targets()` (applies an edit, ignoring a mismatched variant) |
| `PidController<T>` | generic struct | PID with integral wind-up clamp and output limits |
| `TrainingEnv` | trait | `reset()`, `step(action) -> (obs, reward, done, info)` |
| `PlaneConfigHandle` | ECS component | Newtype wrapping `Handle<PlaneConfig>` — required because `Handle<T>` is not a `Component` in Bevy 0.18 |
| `ControllerKind` | enum | Factory selector for all controller types; `build()` does bumpless integral seeding. `ALL` cycle list is feature-gated (RL variants only under `inference`/`training`) |
| `ManualController` | struct | Passthrough — applies raw keyboard/stick inputs, no autopilot |
| `OrbitController` | struct | 3-level cascade PID orbit around a fixed world-frame point |
| `HeadingHoldController` | struct | Holds a configurable heading via an inner level-hold cascade |
| `RlLevelHoldController` | struct | Burn `ActorCritic` policy for level hold (obs dim=13); `inference`/`training`-gated. Target altitude/airspeed are randomized per-episode in training (`LevelHoldEnv::with_target_ranges`, default 500–5000 m / 90–140 m/s) so one policy generalizes across the envelope; obs appends `density_ratio(altitude)` and raw airspeed so the network can actually distinguish operating points |
| `RlHeadingHoldController` | struct | Burn `ActorCritic` policy for heading hold (obs dim=16); `inference`/`training`-gated. Obs = the 13-dim `level_hold_observation` prefix + `[sin(heading_err)/0.5, cos(heading_err), world-vertical turn_rate/0.2]` (`heading_hold_observation`); target heading/altitude/airspeed randomized per-episode via `HeadingHoldEnv::with_target_ranges` (default heading ±180°, airspeed 90–140 m/s — same envelope as level hold; the tighter 110 floor was dropped because it made everyday 100 m/s flight out-of-distribution, accepting the bank-authority squeeze at the slow corner). Takes an `RlHeadingHoldConfig { target_heading, target_altitude, target_airspeed }` (mirrors `RlOrbitConfig`); reports through the shared `ControllerTargets::HeadingHold` variant, same as the PID controller |
| `RlOrbitController` | struct | Burn `ActorCritic` policy for orbit (obs dim=14); `inference`/`training`-gated |
| `RlOrbitResidualController` | struct | Burn `ActorCritic` policy emitting residual deltas added to the PID orbit baseline (obs dim=14); paired with `ResidualOrbitEnv` |
| `RlLstmOrbitController` | struct | Recurrent `LstmActorCritic` orbit policy (Wu et al. FC-LSTM-FC); carries `LstmHiddenState` across steps; paired with `WuOrbitEnv` |
| `LevelHoldRewardConfig` / `HeadingHoldRewardConfig` / `OrbitRewardConfig` | plain structs | Reward weights, scales, alive bonus, failure penalty, and termination thresholds; loaded from `assets/training/*.reward.ron` at training startup. `HeadingHoldRewardConfig` has no `\|roll\|` penalty (bank is the control authority for turning) — instead a roll-*rate* term (chatter guard), a bank-*excess* term past ±60° (matching the PID heading loop's own clamp), and an **α-excess** term past `alpha_soft_limit` (stall guard; **weight defaults to 0.0/inert**, opt in via a profile — see `heading_hold_stall_safe.reward.ron`). Its fields carry no `#[serde(default)]` by design, so adding one invalidates every shipped profile file — update them all, and `tests/core/training_assets.rs` will catch it if you don't (a parse failure is only a runtime *warning* that silently falls back to the compiled defaults) |
| `WuOrbitRewardConfig` / `CurriculumStage` | plain structs/enum | Wu et al. multiplicative-Gaussian orbit reward (`R^TT × R^PS × R^RS`) + 3-stage curriculum; from `wu_orbit.reward.ron` |
| `PpoHyperparams` | plain struct | PPO training-loop config (gamma, gae_lambda, clip, lr, …); from `assets/training/*.ppo.ron` |
| `WingmanController` | struct | Formation flight; holds a fixed offset in the leader's body frame via a heading-damped lateral cascade (cross-track → heading → bank) |
| `AscentController` | struct | Climbs to target altitude then latches to level hold |
| `L1Controller` | struct | Follows a preset `FlightPlan` (waypoint sequences + orbit circles) via L1 nonlinear lateral guidance; built from the plan asset by `apply_flight_plan` |
| `VecEnv<E>` | struct | Wraps any `TrainingEnv` to run N parallel episodes (seeds offset per env) |
| `DemonstrationEnv` / `BcDataset` | trait / struct | Behavior cloning: `collect_demonstrations()` rolls out a PID expert into a supervised dataset for `train_bc` pretraining |
| `EvaluationSummary` / `TaskMetrics` | structs | Policy-evaluation output from `evaluate_policy` (success rate + per-metric families) |
| `ControllerTelemetry` | ECS component (enum) | Read-only **derived** per-controller status (`Orbit { radial_error }`, `FlightPlan { leg, status }`, `Ascent`, `Wingman`, `None`, …) the server snapshots off the active controller each tick and **replicates**, so the thin client — which never steps a controller — can show it on the HUD. Default `None` (`controllers/telemetry.rs`). The *settable* half of controller state lives in `ControllerTargets`, not here — see its row below |
| `ControllerTargets` | ECS component (enum) | Editable controller setpoints (`LevelHold { altitude, airspeed }`, `Ascent { altitude }`, `HeadingHold { heading, altitude, airspeed }`, `Orbit(OrbitParams)`, `Wingman { leader }`, `None`) built by `FlightController::targets()`, snapshotted server-side each tick (`sync_controller_targets`), and **replicated** so a networked client's HUD can seed its target-editor widgets from it. Edits flow back via `SetControllerTargetsCommand`, applied through `FlightController::apply_targets()`. One variant per *widget set*, not per controller kind — e.g. all three RL orbit controllers report through the same `Orbit` variant as the PID `OrbitController`. Default `None` (`controllers/targets.rs`) |
| `SimSpeed` | resource (enum) | Authoritative sim playback speed (pause/1×/5×/10×) in `src/sim_speed.rs`; replicated so the client displays server-side time acceleration rather than scaling its own clock. Set via `SetSimSpeedCommand` |
| `Scenario` / `ResolvedScenario` | RON model (`src/scenario.rs`) | Multi-plane `.scenario.ron`: per-plane initial state, optional `fuel_fraction` (0–1; default full tank → loaded mass), `.plane.ron` config, and a `ControllerSpec` (incl. `Wingman` peer references by name, optional inline tuning, RL specs). `resolve()` assigns **scenario-local** `PlaneId`s (`idx + 1`) and computes initial states — the live spawner remaps these to runtime ids (see `spawn_resolved_scenario` below), since `NextPlaneId` is a separate, never-reset allocator; `build_controller()` builds the boxed controller; `ControllerSpec::kind()` maps a spec to its `ControllerKind`. A wingman naming itself as leader is rejected at `resolve()` time. RL specs **always parse** (so `default.scenario.ron` loads in every build) but only *build* on a native `--features inference` build; otherwise `build_controller` returns `Err` and the live spawner skips that plane. Drives `examples/observe_state.rs` via `--scenario` and the visual menu's Start Scenario flow (`environment::spawn_resolved_scenario`). CSV output ends with a `fuel_remaining` column. |

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
production use.** **A subsequent change grew the level-hold observation again, 11→13**
(`density_ratio(altitude)` + raw airspeed, appended so a policy trained over the now-randomized
500–5000 m / 90–140 m/s target envelope — see roadmap item 1 — can actually distinguish
operating points instead of only seeing normalized error terms); `models/level_hold/` was
retrained under the new 13-dim observation post-fuel-model (`ppo_level_hold.mpk` /
`ppo_level_hold_1.mpk` — checked as of 2026-08-04, both dimensionally current). To restore
production quality for the other tasks, retrain each task and re-tune the (now heavier, 7000 kg
loaded) jet:

```
# per task ∈ {level_hold, orbit, residual_orbit, lstm_orbit}: use the train-evaluate-optimize skill,
# or directly:  cargo run --release --features training --bin train_ppo -- --task <t> --plain
# then re-tune PID gains for the heavier jet via the `tune` skill (writes generic_jet.tuning.ron).
```

**`heading_hold` task** (added after the fuel-model rework, so it was never affected by the
history above — its 16-dim observation was current from the start). The current production
checkpoint is **`models/heading_hold/skill_heading_hold_goal3_alpha_guard.mpk`**, the first one
retrained after the 2026-08-06 physics/envelope changes (64 Hz training integrator, 90 m/s
airspeed floor) that left the earlier `skill_heading_hold_goal{,2}_*` checkpoints behaviorally
stale. 1000-episode eval at the full envelope: `success_rate 0.995`,
`mean_tail_abs_heading_rad 0.00856` (0.49°), `mean_tail_abs_altitude_m 1.431`,
`mean_tail_abs_speed_mps 0.323`; live Rapier rollouts at four nominal operating points track to
0.97–1.58 m / 0.37–0.39 m/s / 0.31–0.51°. Reward/PPO profiles:
`assets/training/heading_hold_stall_safe.{reward,ppo}.ron` (pass both explicitly; neither is the
task default). `smoke_heading_hold.mpk` remains as the pipeline smoke test only (100k steps,
`--target-heading-range -30:30`, and the pre-2026-08 `terminal_failure_penalty: -50.0`); it is
not flight-quality and its returns aren't comparable to anything trained since.

**Known envelope limit (heading_hold).** At the joint corner of ≥5000 m *and* ≤90 m/s *and* a
large commanded turn, the generic jet has almost no maneuvering margin — loaded (7000 kg) level
flight there already needs CL ≈ 1.15 of `cl_max` 1.4, capping sustained level turns near 35°
bank — and every checkpoint to date, RL or PID, degrades badly there. Live rollouts of the
current policy still depart at 5000 m / 90 m/s / 120° turn. This is an airframe-margin corner,
not a tracking-quality one: the PID expert loses ~111 m of altitude at that same point versus
~1.4 m envelope-wide (measure it with `examples/heading_hold_expert_baseline.rs`, which rolls
`HeadingHoldEnv::make_expert()` through the same env, tail window, and success definition
`evaluate_policy` uses). **Do not chase it by loosening `max_altitude_error`** — that hides the
failure in the accept gate instead of fixing it. The reward's `alpha_soft_limit` /
`alpha_excess_weight` guard (see `heading_hold_stall_safe.reward.ron`) is what keeps the policy
off `cl_max` elsewhere in the envelope; `compute_aero_forces` merely clamps CL, so without that
term nothing tells a policy that it is mushing.

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
training = ["inference", "burn/autodiff", "burn/train", "burn/tui"]
wgpu = ["training", "burn/wgpu"]         # opt-in GPU training backend
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
- `training`: builds on `inference`, adds `burn` `autodiff`, `train`, and `tui`; defaults to the
  CPU (`ndarray`) backend — no rendering, max-speed simulation
- `wgpu`: opt-in GPU training backend (builds on `training`, adds `burn/wgpu`); pass
  `--backend wgpu` to `train_ppo`/`train_bc` to use it
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

`ControllerKind::Wingman.build()` (`kind.rs`) returns a plain `LevelHoldController` —
the generic factory has no leader reference to construct a real `WingmanController`
with. Because of that, the tuning-rebuild systems in `sim_control.rs`
(`apply_initial_tuning`, `apply_controller_switch`) can't just call `kind.build()`
for a wingman the way they do for every other kind: doing so would silently replace
a live `WingmanController` with the `LevelHold` fallback the moment the plane's
`.tuning.ron` asset loaded (or a profile was switched), while `ControllerKind` kept
reporting `Wingman`. Instead they snapshot `(leader_id, offset, range/lateral/heading
PIDs)` via `extract_wingman_params` *before* rebuilding, then re-wrap the freshly
tuned inner `LevelHoldController` via `WingmanController::from_inner` afterward
(`restore_wingman`) — the wingman analogue of `extract_orbit_params` +
`OrbitController::apply_params`. The wingman draws its tuning from the **`level_hold`**
pool (there's no separate `wingman` tuning family). `cleanup_orphaned_wingmen`
(`environment/lifecycle.rs`) additionally demotes `ControllerKind::Wingman` to
`LevelHold` if the active controller ever isn't actually a `WingmanController` (e.g. a
fresh `SpawnPlaneCommand { kind: Wingman }`, whose generic `build()` is the same
fallback) — a safety net for the one case extract/restore can't help, since there's
nothing to preserve.

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
  mass/inertia. `spawn_plane` allocates the next id from `NextPlaneId`; `spawn_plane_with_id`
  is the underlying primitive for callers (the scenario spawner) that must pin an explicit,
  pre-reserved `PlaneId` instead.
- **Removal cleanup:** `cleanup_orphaned_wingmen` (Update, headless-safe) flips a wingman whose
  `leader_id` is no longer live to `ControllerKind::LevelHold`; the same system also demotes any
  `Wingman`-kind plane whose active controller isn't actually a `WingmanController` (see the
  Wingman Controller Architecture section above). `recover_camera_on_target_loss`
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
replicon rejects the connection); `PROTOCOL_ID` (currently **3** — v2 added `ControllerTelemetry`;
v3 added `ControllerTargets` + `SetControllerTargetsCommand`) gates version-mismatched peers.

- **Replicated (server → client), in registration order:** `Transform`, `FlightState`,
  `ControlInputs`, `PlaneId`, `PlaneIndex`, `ControllerKind`, `SelectedTuningProfile`,
  `PlaneTuningPath`, `ControllerTelemetry`, `ControllerTargets`, and (`inference`-gated)
  `SelectedModel`. The client HUD/map/camera read these read-only. `PlaneTuningPath` lets the
  client rebuild a `PlaneTuningHandle` and reuse the existing profile enumeration for its
  dropdown. `ControllerTargets` is the settable counterpart to `ControllerTelemetry` — see its
  Key Types row below — and is what the HUD's target-editor widgets (Target Alt/Spd/Hdg, orbit
  geometry, wingman leader) seed from on a client, since `ActiveController` itself never is.
- **Commands (client → server)** — all `add_client_event`, `Channel::Ordered`, received on the
  server as `On<FromClient<…>>` observers: `SwitchControllerCommand`, `SetTuningProfileCommand`,
  `ManualInputCommand` (sent every client frame while manually flying; latest-wins),
  `SpawnPlaneNetCommand`, `RemovePlaneNetCommand`, `SetSimSpeedCommand`,
  `SetControllerTargetsCommand` (sent on every target-editor widget edit, including mid-drag —
  high-rate and latest-wins, like `ManualInputCommand`; applied via
  `FlightController::apply_targets`, which ignores a variant mismatch rather than panicking), and
  (`inference`-gated) `SetModelCommand`. Each handler just mutates the component that
  `SimControlPlugin` (controller rebuilds) or the `LifecyclePlugin` observers already react to, so
  there is **one authoritative application path** shared with local play — no divergent
  server-only logic.
- **Rendering:** the client never predicts (deferred — see the plan's Out of Scope). It only
  **interpolates**: `ClientNetPlugin` buffers the last two replicated `FlightState` poses
  (`NetInterpolation`) and blends `Transform` at `now − RENDER_DELAY` (≈ 2 server ticks at 64 Hz),
  so a prev/curr pair is always available. Reading `FlightState` and writing `Transform` avoids
  any replication self-write feedback.
- **Transport:** renet UDP (`bevy_replicon_renet`). `start_renet_server` / `start_renet_client`
  are added by the binaries, **not** the plugins, so tests (`ServerSimPlugin` is transport-free)
  never bind a socket. `ServerProcess` wraps a client-launched local server child and kills+reaps
  it on drop (covers window-close / Quit paths that skip `OnExit(InGame)`).
- **Client and server are separate `cargo` targets with disjoint feature sets** (`client`+
  `training` for `ml_planes` vs `server`+`inference` for `ml_planes_server`), so a plain `cargo run
  --bin ml_planes` (the `planes` alias) never rebuilds the sibling `ml_planes_server` binary that
  "Start New Server" spawns (`launch_local_server`/`local_server_path`, `src/net/client.rs`) — the
  two silently drift apart under separate builds. `check_local_server_staleness` compares mtimes
  and pushes a `Notifications` banner when the local server binary predates the client executable.
  `just play`/`just play-debug` build both before running so the pair can't drift; `cargo planes`
  alone is still fine for iterating on the client, the banner just covers the gap.

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
via `load_spawn_config`, spawns through `spawn_plane_with_id` with `ControllerSpec::kind()`, and
attaches per-kind extras (`FormationOffset` for wingmen, `SelectedModel` for RL, `FlightPlanHandle`
for flight plans so `apply_flight_plan` re-installs the `L1Controller` after the tuning rebuild).
Scenario `config` paths use the observe_state `assets/...` convention and are stripped to the
Bevy asset-relative form for the live spawner.

`resolve()`'s `PlaneId`s are **scenario-local** (`idx + 1`) — not the ids planes actually spawn
with. `NextPlaneId` is a persistent, never-reset allocator, so the live/runtime ids diverge from
the resolved numbering whenever other planes already exist or a scenario plane is skipped (e.g. a
missing RL `.mpk` or flight-plan asset). Since a wingman's `leader_id` is baked in by
`build_controller` from the resolved numbering, `spawn_resolved_scenario` runs four passes: build
every controller (recording failures); transitively skip any wingman whose leader failed to build
(fixed point, since a wingman may itself lead another wingman); reserve a contiguous runtime-id
block for the survivors and remap each surviving wingman's `leader_id` from resolved to runtime;
then spawn under the reserved ids via `spawn_plane_with_id`. `spawn_plane` remains the thin
allocating wrapper used by every other spawn path (lifecycle commands, tests).

### Training Physics (Self-Contained)

Training environments (`LevelHoldEnv`, `OrbitEnv`, `ResidualOrbitEnv`, `WuOrbitEnv`) do
**not** use Bevy or Rapier. Instead:

- `training/flight_env.rs::integrate_state()` provides 6-DOF Euler integration
- Aerodynamics: shared `compute_aero_forces()` from `aerodynamics/`
- Result: deterministic rollouts, fast vectorized training, no ECS overhead
- `VecEnv` wraps any `TrainingEnv` to run N parallel episodes (seeds offset via `offset_rng_seed()`)
- Integration step is **64 Hz**, read from the shared `ml_planes::plane::PHYSICS_DT` — the *same*
  constant the live sim's Rapier fixed schedule, Bevy's `Time<Fixed>`, and the replicon server
  tick use. The Euler integrator and Rapier are still different integrators, but they no longer
  run at different rates.
  **This used to be 60 Hz "by design", and that was a mistake.** `evaluate_policy` drives the
  training envs and never touches Rapier, so a 60 Hz env meant the accept gate was structurally
  unable to see how a policy behaves in the sim it actually flies in. A heading-hold checkpoint
  accepted at `mean_tail_abs_altitude_m = 0.333` measured **1.47 m** live; re-running the eval
  pinned to the live operating point isolated ~0.8 m of that to an out-of-distribution airspeed
  and the rest to the timestep. Changing `PHYSICS_HZ` now moves both sides at once, and
  `tests/core/physics_timestep.rs` guards the one clock nothing sets explicitly (`Time<Fixed>`).
  A consequence worth knowing: at 64 Hz a fixed step budget buys less wall-clock, so
  `max_episode_steps` was rescaled to keep episode *duration* constant (3000 → 3200 level-hold,
  3600 → 3840 elsewhere). `gamma` is per-step, so no reward term needed re-deriving, but the
  maximum achievable return shifted ~6.7% — `mean_return` is not comparable across the change.
- **Level-hold observation contract:** `LevelHoldEnv` and `RlLevelHoldController` must agree
  bit-for-bit on the 13-dim observation vector; rather than keep two copies in sync by hand,
  both call the single free function `training::level_hold_env::level_hold_observation()`. Any
  change to the vector's length or element order must be made there once — `LEVEL_HOLD_OBS_DIM`
  (also in that module) and the tests pinning it (`rl_inference::level_hold_controller_obs_matches_env_obs`,
  the `loading_stale_dim_level_hold_model_errors` dimension guard) exist specifically to catch a
  future drift between training and inference.
- **Heading-hold observation contract:** same pattern, one layer up — `HeadingHoldEnv` and
  `RlHeadingHoldController` both call `training::heading_hold_env::heading_hold_observation()`,
  which itself calls `level_hold_observation()` for its first 13 elements and appends
  `[sin(heading_error)/0.5, cos(heading_error), turn_rate/0.2]`. The heading error is shared
  too — both the free function and the PID `HeadingHoldController::update` call
  `controllers::heading_hold::heading_error()`, so RL and PID can never disagree on its sign
  or its low-speed (`speed_xz > 1.0`) fallback. `HEADING_HOLD_OBS_DIM` (= `LEVEL_HOLD_OBS_DIM +
  3`) and `rl_inference::{heading_hold_controller_obs_matches_env_obs,
  loading_stale_dim_heading_hold_model_errors}` guard drift the same way. The heading term is
  sin/cos-encoded rather than a raw `error/π` scalar specifically because episodes are sampled
  across the full ±180° circle and a raw signed term would jump ±2.0 across the ±π boundary —
  see the `heading_hold_env` module header for the full reasoning. This encoding also means
  `eval_metrics::MetricFamily::HeadingHold`'s heading spec must read the value back via
  `Source::CircularError` (`atan2(sin, cos)`), not `Source::Scaled` — the latter would fold a
  ~170° error down to ~8°.

### RL Inference Pattern

All five RL controllers (`RlLevelHoldController`, `RlHeadingHoldController`, `RlOrbitController`,
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
  `train_ppo --init-from <path>` as a warm start. Supported for `level_hold` / `heading_hold` /
  `orbit`. `heading_hold`'s BC expert (`HeadingHoldEnv::make_expert`) re-applies the episode's
  actual (resampled) targets via `apply_targets` after construction — `HeadingHoldController::new`
  alone would seed its inner altitude/airspeed from the *spawn* state, which differs from the
  episode target by the reset's spawn offset.

### Policy Evaluation

`evaluate_policy` (`src/bin/evaluate_policy.rs`, ndarray/CPU only) rolls a checkpoint
out over N episodes and reports an `EvaluationSummary` (success rate + `TaskMetrics`
families). Supports `--task {level_hold|heading_hold|orbit|residual_orbit|lstm_orbit}` and, for
`lstm_orbit`, `--curriculum-stage {coarse|heading_fine|full}`.

### Adding a New `ControllerKind` (Checklist)

`ControllerKind::build()` always returns a valid PID fallback. This means missing visual-mode wiring compiles cleanly and fails silently at runtime — there is no compile-time guard.

**PID / non-RL controller:**

1. `kind.rs` — add variant to `ControllerKind` enum. Append it (don't insert in the middle) —
   `ControllerKind` derives `Serialize`/`Deserialize` under `net`, and a mid-enum insertion
   shifts every later variant's discriminant for a stale peer; bump `net::protocol::PROTOCOL_ID`
   either way so a mismatched client/server fails cleanly instead of misreading a kind.
2. `kind.rs` — `name()`: human-readable label
3. `kind.rs` — `ALL`: add to the cycle list under the correct feature gate
4. `kind.rs` — `build()`: factory arm
5. `kind.rs` — `is_heading_hold()`: add if the new kind uses the `heading_hold` tuning pool
   (e.g. an RL variant of an existing PID-tuned kind)
6. `controllers/sim_control.rs` — `apply_initial_tuning` **and** `apply_controller_switch`:
   add to the correct tuning-family match (`Orbit`/`LevelHold`/`HeadingHold` pattern) in *both*
   systems — they're two independent matches, easy to update one and miss the other (this is
   exactly the bug fixed alongside `RlHeadingHold`: `apply_initial_tuning` had no `HeadingHold`
   arm at all and silently fell through to `level_hold` gains)
7. `main.rs` — `tuning_profile_names` **and** `ui/hud.rs` — `net_tuning_names`: the same
   tuning-family match exists a third and fourth time, for the HUD's tune-profile dropdown /
   `cycle_tune_profile` hotkey and the networked-client HUD equivalent. Both are non-exhaustive
   (`_ => None`/`return None`), so a miss here is silent, not a compile error.
8. If the controller has editable setpoints, implement `FlightController::targets()`/
   `apply_targets()` — reuse an existing `ControllerTargets` variant when the new controller's
   widget set matches one already defined (e.g. any orbit-shaped controller reuses `Orbit`, any
   heading-shaped controller reuses `HeadingHold`; see `controllers/targets.rs`), and add a case
   to `tests/core/controller_targets.rs`. Skipping this step means the new controller silently
   ships with an un-editable HUD on the networked client — the exact bug class `ControllerTargets`
   exists to close (see the Client/Server Networking section above).
9. `ui/map.rs` — `kind_color`: the only *exhaustive* `match` on `ControllerKind` — the compiler
   forces you to place the new variant here, making it the reliable canary that you added one at
   all.
10. `ui/lifecycle_panel.rs` — `SPAWNABLE_KINDS` (the `inference`-gated list, if the new kind is
    RL) + its `spawnable_kinds_include_rl_when_ml_enabled` test.
11. `scenario.rs` — `ControllerSpec` variant (RL specs are deliberately **not**
    `inference`-gated so `.scenario.ron` files stay parseable everywhere), `kind()`,
    `rl_model_stem()` if RL, `build_controller`'s `inference`-gated arm, **and** its
    `#[cfg(not(inference))]` catch-all `Err` arm — miss that last one and only the
    non-inference build fails to compile, easy to not notice if you only build with
    `--features inference`.
12. `src/mcp/bridge.rs` — `spawnable_kind_names()` + `parse_spawnable_controller_kind()` (both
    `inference`-gated per-arm for RL kinds); `src/mcp/service.rs` — the kind lists baked into
    several `#[tool(description = "...")]` doc strings (grep the file for the sibling kind's
    name; nothing enforces these at compile time).

**RL controller — all of the above, plus:**

13. `kind.rs` — `model_dir()`: return the `models/` subdirectory name (e.g. `"heading_hold"`)
14. `src/controllers/mod.rs` — `#[cfg(feature = "inference")] pub mod rl_xxx;` + re-export
    `RlXxxController` (and `RlXxxConfig`, if the controller has more than one/two setpoints —
    mirror `RlOrbitConfig`/`RlHeadingHoldConfig` rather than bare positional args)
15. `controllers/sim_control.rs` — the `inference`-gated import block, `apply_model_switch`
    match arm, `rl_kind_needs_load_on_change` guard, `apply_rl_controller_switch`'s two match
    arms (the "no checkpoint" demotion **and** the load-and-demote-on-error arm — miss the
    demotion and a plane keeps the RL label while actually flying PID), and
    `preserve_rl_controller`'s arm. Extend the `rl_kind_load_gate` test's hand-maintained kind
    array.
16. `Task::HeadingHold`-shaped work in each RL binary: `src/bin/train_ppo.rs`,
    `src/bin/train_bc.rs`, `src/bin/evaluate_policy.rs` — task enum/parse/reward-config-path/
    model_dir/default_stem, plus whatever per-task CLI flags the new obs/targets need (see
    `--target-heading-range` as the template for a new per-task range flag).

**A kind whose `build()` can't reconstruct its full state** (like `Wingman`, which needs a
leader reference the generic factory doesn't have, or any RL kind, which needs a loaded model
the generic factory has no path for): the tuning-rebuild systems (`apply_initial_tuning`,
`apply_controller_switch` in `controllers/sim_control.rs`) will silently replace a live
instance with the fallback the next time the plane's tuning asset loads or its profile
switches. Add an extract/restore pair — snapshot the state that would be lost *before* calling
`kind.build()`, then re-wrap the freshly built fallback controller *after* — in **both**
systems, mirroring `extract_orbit_params`/`OrbitController::apply_params` (orbit geometry),
`extract_wingman_params`/`restore_wingman` (wingman formation state), or
`preserve_rl_controller` (all five RL kinds: a downcast check for the four pure-policy
controllers, `RlOrbitResidualController::retune` for the one RL kind that owns real PID
gains). Unlike the other two, `preserve_rl_controller` doesn't need a restore step — it runs
*instead of* `kind.build()` (returning `true`) rather than wrapping its output, since there's
nothing for the generic factory to contribute once a policy is loaded.

**A kind whose `build()` *can* reconstruct the controller but re-seeds its setpoints from the
live `FlightState`** (`LevelHold`, `HeadingHold`, `Ascent` — `LevelHoldController::with_tuning`
captures `state.altitude`/`state.airspeed`, `HeadingHoldController::from_state` captures
`ground_track_heading(state)`, `AscentController::new` re-targets `state.altitude + 1000.0`): a
tuning rebuild would silently cancel a scenario- or pilot-commanded target the moment the asset
loads, even though the controller *type* survives. This doesn't need a bespoke extract/restore
pair like the ones above — reuse the setpoints the controller already publishes through
`FlightController::targets()`/`apply_targets()` instead: snapshot `targets()` before
`kind.build()`, replay it via `apply_targets()` after (`rebuild_preserves_targets`,
`extract_targets`, `restore_targets` in `sim_control.rs`). Keep `Orbit`/`Wingman` out of this
list — they already have their own pairs above, and their `apply_targets` has side effects
(PID resets, auto-centering) a blind replay must not trigger. Add the new kind to
`rebuild_preserves_targets` and a case to both `tests/core/sim_control.rs` and (for a scenario
that ships a non-default target) `tests/core/scenario.rs`.

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

1. **Level flight hold** — COMPLETE. Cascade PID: altitude outer → pitch inner, airspeed, roll, yaw. RL policy trained (`RlLevelHoldController`, obs dim=13) over a randomized 500–5000 m / 90–140 m/s target envelope, configurable via `--target-alt-range`/`--target-speed-range` on `train_ppo`/`train_bc`/`evaluate_policy`. **Heading hold** (outer heading PID over the level-hold cascade, `HeadingHoldController`) is likewise COMPLETE, with an RL variant (`RlHeadingHoldController`, obs dim=16) added over a randomized ±180° target-heading-change / 500–5000 m / 90–140 m/s envelope, configurable via `--target-heading-range`/`--target-alt-range`/`--target-speed-range`. Production checkpoint: `models/heading_hold/skill_heading_hold_goal3_alpha_guard.mpk` (0.49° / 1.43 m / 0.32 m/s tail error at `success_rate` 0.995 over 1000 episodes) with `assets/training/heading_hold_stall_safe.{reward,ppo}.ron`; see the heading_hold checkpoint notes in §2 for its one known envelope limit. `smoke_heading_hold.mpk` is a pipeline smoke checkpoint only.
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
training = ["inference", "burn/autodiff", "burn/train", "burn/tui"]
wgpu = ["training", "burn/wgpu"]         # opt-in GPU training backend
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
name = "train_ppo"         # required-features = ["training"]; --backend wgpu needs feature "wgpu"
path = "src/bin/train_ppo.rs"
[[bin]]
name = "train_bc"          # required-features = ["training"] — BC pretraining; wgpu as above
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
> CPU backend (enabled by `inference`, used in production, WASM, and as the default training
> backend); `autodiff`/`train` (from `training`) run PPO/BC on top of it; `wgpu` (its own opt-in
> crate feature) = GPU backend for training, selected at runtime via `--backend wgpu`
> (`ml_planes::training::Backend`); `tui` = training progress display. `train_ppo` tasks:
> `level_hold`, `heading_hold`, `orbit`, `residual_orbit`, `lstm_orbit`. Both `train_ppo` and
> `train_bc` accept `--seed <u64>` to fix weight init, minibatch shuffling, and (per-env) episode
> resets for a reproducible run's starting point — see the "Known nondeterminism" note in §6 for
> what it does and does not guarantee. For `level_hold`/`heading_hold`, `train_ppo`, `train_bc`,
> and `evaluate_policy` all accept `--target-alt-range <MIN:MAX|VALUE>` / `--target-speed-range
> <MIN:MAX|VALUE>` to set the per-episode target-altitude/airspeed envelope the policy is
> trained/evaluated across (default `500:5000` / `90:140` for **both** `level_hold` and
> `heading_hold` — heading_hold's floor used to be a tighter `110`, dropped because it made
> everyday 100 m/s flight out-of-distribution; pass `--target-speed-range 110:140` to recover the
> easier envelope); a bare `VALUE` pins a single fixed target, reproducing
> the pre-randomization behavior. `heading_hold` additionally accepts `--target-heading-range
> <MIN:MAX|VALUE>` in **degrees** (default `-180:180`) for the per-episode target heading *change*
> (spawns always start on ground track 0, so the sampled value is the required turn). All three
> are parsed by the shared `training::parse_f32_range`; pass the same ranges to `evaluate_policy`
> that a checkpoint was trained with for a comparable report.

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

**Audio — `AudioPlugin` disabled, no audio assets yet:**
- `bevy_audio` comes in transitively via `visual = ["bevy/default", ...]`, but no
  code uses `AudioPlayer`/`AudioSource` today.
- `main.rs`'s `DefaultPlugins` registration disables it
  (`.disable::<bevy::audio::AudioPlugin>()`) to avoid ALSA/JACK/OSS device-probe
  errors at startup on machines without a configured sound server.
- To add real audio later: drop the `.disable::<...>()` call and add
  `AudioPlayer`/`AudioSource` usage as normal — `bevy_audio` is already compiled in.

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
- `training_assets` — every shipped `assets/training/*.reward.ron` profile parses into its
  config struct (the structs have no `#[serde(default)]`, and the training binaries downgrade
  a parse failure to a warning + compiled defaults, so a stale profile otherwise trains the
  wrong reward and still reports success)
- `orbit_tune_sync` — orbit tuning-pool / gain-sync invariants
- `scenario` — `.scenario.ron` parse/resolve/build, per-plane `fuel_fraction` carried through `resolve()`, `ControllerSpec::kind()` mapping, `spawn_resolved_scenario` live spawn, `default.scenario.ron` resolve, + CSV header pinning (`ml_planes::scenario::CSV_HEADER`, incl. trailing `fuel_remaining`); self-referential wingman-leader rejection; resolved-vs-runtime `PlaneId` remap, leader-skip cascade, and a scenario wingman's `WingmanController` surviving the tuning rebuild end-to-end
- `lifecycle` — runtime spawn/remove commands, auto-indexing, orphaned-wingman + camera cleanup (camera case is `visual`-gated, so it runs only under `just test-visual`), + a `Wingman`-kind plane with no `WingmanController` installed falling back to `LevelHold`
- `sim_control` — relocated `SimControlPlugin` controller-rebuild systems, headless (runs in core); + a `WingmanController` surviving both the initial-tuning rebuild and a later profile switch
- `controller_telemetry` — each controller's `FlightController::telemetry()` accessor / `ControllerTelemetry` shape (runs in core)
- `controller_targets` — each controller's `FlightController::targets()`/`apply_targets()` accessors: read/write round trip, mismatched-variant no-op, and the ascent-complete-latch / orbit-PID-reset side effects `apply_targets` owns (runs in core; RL variants' `targets()` covered in `tests/rl/rl_inference.rs`)
- **module-gated `#[cfg(sim_enabled)]` in `tests/core/main.rs`** (compile out on net-without-server builds):
  - `aero_physics` — energy conservation over N steps in level flight
  - `level_hold` — level-hold cascade convergence to target altitude
  - `heading_hold` — heading-hold convergence to a commanded heading
  - `flight_plan` — L1 flight-plan leg sequencing / waypoint capture
  - `wingman` — formation flight relative-position tracking
  - `fuel` — live-sim fuel: spawn-time tank load (`fuel_fraction`), `consume_fuel` burn + `update_plane_mass`, shipped-asset powerplant parse

**`tests/net/` (`net`/`server`/`mcp`; the whole binary compiles out without `net`):**
- `net_serde` — serde round-trips for the replicated types + command events (`net`-gated), incl. every `ControllerTargets` variant and `SetControllerTargetsCommand`
- `net_protocol` — `NetProtocolPlugin` registration / replication-rule invariants (`net`-gated)
- `client_net` — client interpolation math (`NetInterpolation` buffer → interpolated `Transform`) (`net`-gated)
- `local_server` — `ServerProcess` drop-kills-child (a client-launched local server dies with the client); **not** an in-process replicon round trip despite the name (`server`-gated)
- `server_sim` — `ServerSimPlugin` boot / scenario spawn / `FromClient` command handlers, transport-free (`server`-gated), incl. `ControllerTargets` replication and `SetControllerTargetsCommand` (apply + echo-back, variant-mismatch no-op, unknown-plane no-op)
- `mcp_snapshot` — MCP read-path snapshot mirror, transport-free (`mcp`-gated), incl. `ControllerTargets` mirrored into `PlaneSnapshot.targets`
- `mcp_bridge` — MCP write-path command bridge drain, transport-free (`mcp`-gated), incl. `ControlRequest::SetControllerTargets` and the `merge_controller_targets`/`parse_orbit_direction` helpers backing the `set_controller_targets` tool
- `mcp_lifecycle` — MCP auto-reconnect (`poll_reconnect`) + clean shutdown (`check_shutdown`), transport-free (`mcp`-gated)
- `mcp_e2e` — MCP end-to-end over real UDP: boots an `ml_planes_server` child, inspect + spawn/remove round-trip (`mcp`+`server`-gated, `#[ignore]`; run with `--features "mcp server" --test net -- --ignored mcp_e2e`)

**`tests/rl/` (`inference`/`training`; the whole binary compiles out without an RL backend):**
- `rl_inference` — RL controller load + deterministic inference (`inference`/`training`-gated),
  incl. the level-hold and heading-hold obs-matches-env and stale-dimension guards, and each RL
  controller's `targets()` round trip through its shared `ControllerTargets` variant
- `rl_sim_control` — `SimControlPlugin`'s tuning-rebuild systems preserve a loaded RL policy
  instead of silently replacing it with the PID fallback `ControllerKind::build()` produces
  (`RlLevelHold`/`RlHeadingHold`/`RlOrbit`/`RlOrbitResidual`), plus the load-failure-demotes-kind
  guard for `RlLevelHold`/`RlHeadingHold` (`inference`-gated)
- `ppo_training` — short PPO rollout/update loops (level_hold + orbit), asserting no NaN in
  policy output after training; **not** a convergence/reward-trend check (see the "Known
  nondeterminism" note below) (training-gated; run with `--features training --test rl ppo_training::`)

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
  (`--features inference`). `just test-training` (CPU/ndarray, the default training backend;
  `just test-training-gpu` covers the opt-in `wgpu` backend, heavy build, needs a GPU) and
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
- Tests are deterministic (fixed seed where randomness is needed). **Known exception:** `burn`
  module init (`LinearConfig::init` etc., used by `ActorCritic::new`/`LstmActorCritic::new`) draws
  from burn's default backend RNG. A fixed seed is now wired up end-to-end —
  `ActorCritic::new_seeded`/`LstmActorCritic::new_seeded` seed the backend RNG before constructing
  the model (and force-realize burn 0.20's lazily-initialized `Param`s via a throwaway forward
  pass while the seed is still in effect — otherwise the actual random draw is deferred to
  whatever later moment/thread first calls `forward`); `PpoTrainer::with_n_envs_seeded` /
  `LstmPpoTrainer::with_n_envs_seeded` additionally seed `rng_seed` (minibatch shuffling) and
  every env's `offset_rng_seed` (episode resets) from the same seed. Plumbed through
  `train_ppo`/`train_bc`'s `--seed` flag and `PpoHyperparams.seed` (`*.ppo.ron`, CLI wins if both
  given). Any code path that still calls the **unseeded** `new()`/`with_n_envs()` (`seed: None`,
  the default) is exactly as nondeterministic as before — that includes most existing tests, e.g.
  `tests/rl/ppo_training`, which must still only assert seed-independent invariants (no NaN,
  correct shapes/dims, no panics), never a specific reward value or trend (a prior version of
  `ppo_50_iterations_no_nan` asserted the reward didn't regress over 50 iterations and failed
  reproducibly on unmodified `main` purely from this, not from any actual bug).
  **Even with a seed supplied, only the run's *starting point* is guaranteed bit-identical**
  (verified: `ActorCritic::new_seeded`'s weight init, checked via exact `mean_action` output
  equality) — not necessarily a full multi-iteration training run's final checkpoint. burn's
  `ndarray` matmul backend (`matrixmultiply`) has its own tiny (~1 ULP/iteration) floating-point
  non-associativity in the forward/backward computation that compounds across PPO updates,
  independent of this seed and not fixable from application code; two `--seed`-matched real
  `train_ppo` runs start identically and track closely but were observed to diverge by a few ULP
  after a single gradient update and further over subsequent iterations. (Separately: burn's
  `Param` IDs, serialized into every `.mpk`, are randomly generated per run regardless of
  seeding — comparing checkpoints for reproducibility must load and compare tensor *values*, e.g.
  via `mean_action`, never raw file bytes / `cmp`.) A test-suite-specific hazard: burn-ndarray
  keeps its RNG behind one **process-global** mutex, so under `cargo test`'s default
  multi-threaded harness, an unrelated concurrent test that also constructs/samples a model can
  perturb a seeded-determinism assertion even though each individual burn call is thread-safe —
  `ppo::rng_lock()` (`src/training/ppo/mod.rs`) serializes every RNG-touching call (production
  and test) against this; any new test that seeds and reads model output must acquire it too (see
  `ActorCritic::new_seeded`'s tests for the pattern) or it can flake under parallel execution.
  Convergence/reward-quality checks belong in the slower `evaluate_policy` /
  `train-evaluate-optimize` workflows, which run full-length training and are not part of
  `cargo test`.
- Run `cargo fmt` at the end of every editing session before committing — always run it, even if you believe the code is already correctly formatted. Never skip it based on visual inspection.
- Tests are written **before** the implementation they cover — never after
- A PR that adds implementation without a prior failing test is not accepted
- `cargo test --no-default-features` must be the first and last step of every change cycle

---

## 7. Invariants / Non-Negotiables

- Every plane is a full 6-DOF Rapier `RigidBody` — no simplified kinematics.
- `FlightController` is always `Box<dyn FlightController>` per entity — never hard-wired to a concrete type.
- `PlaneConfig` is loaded at runtime via Bevy's asset server — no compile-time plane data.
- All physics runs at Rapier's fixed timestep (64 Hz, `dt = 1/64 s`, set via `TimestepMode::Fixed` + `RapierPhysicsPlugin::in_fixed_schedule()`). In the client/server split the authoritative sim runs in the **server** (`ml_planes_server`, `src/bin/server.rs`); the `net` client (`main.rs`) runs no physics and renders interpolated replicated state. The headless `observe_state` example and `tests/common/mod.rs` mirror this exact 64 Hz fixed-schedule setup. **`ml_planes::plane::PHYSICS_DT` is the single source** — Rapier's timestep, Bevy's `Time<Fixed>` (which drives the replicon server tick), *and* the self-contained training Euler integrator (`training/flight_env.rs`) all read it, so training can never again validate against a different plant than the one it flies in. Never reintroduce a bare `1.0 / 64.0` literal.
- The ground is a flat infinite collider acting as a death plane — no terrain, no landing.

---

## 8. Implementation Roadmap

> Full plan: `plans/roadmap.md`
> WASM feasibility: `plans/wasm_feasibility.md`

M0–M12 (environment phase + level hold + formation flight + orbit + RL training) are complete. M13 (aerial refueling) is next.
