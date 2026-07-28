Vibe coded flight sim using Rapier/Bevy.

AI Summary:

# ml_planes

6-DOF flight simulation sandbox for traditional and ML-based autopilot research, split
client/server: an authoritative headless physics server plus a thin rendering client.
Planes are physically simulated rigid bodies (Bevy Rapier 3D) with a coefficient-based
linear stability aerodynamic model. Flight controllers — PID autopilots, L1 flight-plan
guidance, and trained ML policies — are hot-swappable at runtime via a trait interface.
All ML runs in pure Rust with the `burn` crate; no Python, no IPC.

See `CLAUDE.md` for the full architecture reference (module layout, key types, physics
model, networking protocol, and test strategy). This file is a lighter-weight intro.

## Quick Start

```bash
# Interactive visual client (default) — opens the main menu.
# "Start New Server" launches a local ml_planes_server child process and connects to it;
# "Connect to Server" joins a remote one.
cargo run

# Headless authoritative server (own the sim, let others connect)
cargo run --features server --bin ml_planes_server -- \
  --scenario assets/scenarios/default.scenario.ron --port 5555

# All tests (always headless; core sim suite only needs --no-default-features)
cargo test --no-default-features

# Full supported test matrix (see justfile)
just test-all        # core + net/mcp/server + RL inference
just test-visual      # UI/camera tests (visual-gated, headless egui — no display)
just test-training    # RL training-loop tests (CPU/ndarray, the default training backend)
just test-training-gpu # RL training-loop tests on the opt-in wgpu backend (heavy build, needs a GPU)
```

`cargo run --no-default-features` builds headless with no rendering and no networking —
useful for CI/build checks, not for interactive play.

## Feature Flags

| Flag | Effect |
|---|---|
| `client` (**default**) | `visual` + `net` — the pure networked renderer. No local physics; planes arrive via replication and every mutation goes out as a command to the server. |
| `visual` | Full Bevy rendering pipeline, egui HUD, native file dialogs (`rfd`) |
| `net` | Shared client/server replication protocol (`bevy_replicon` + renet transport) |
| `server` | Headless authoritative sim (`ml_planes_server` binary) — Rapier + controllers + fuel at 64 Hz, no rendering |
| `mcp` | `ml_planes_mcp` binary — headless MCP control client exposing a running server to an LLM agent |
| `inference` | `burn` CPU (ndarray) backend — loads/runs trained RL policies headlessly (incl. `evaluate_policy`), no training stack |
| `training` | Builds on `inference`; adds the `burn` autodiff/train stack for the PPO/BC training loops. Defaults to the CPU (ndarray) backend — no GPU required |
| `wgpu` | Opt-in GPU training backend (builds on `training`); adds `burn/wgpu`. Pass `--backend wgpu` to `train_ppo`/`train_bc` to use it |
| `wasm` | `visual` + `inference` — browser build target (CPU inference in the renderer); networking not yet supported in-browser (see `plans/wasm_feasibility.md`) |
| *(none — `--no-default-features`)* | Headless, no rendering, no networking; used for tests and the training loops |

## Architecture

```
src/
  aerodynamics/   # coefficient model: (FlightState, ControlInputs, PlaneConfig) → (F, τ)
  plane/          # PlaneConfig asset, FlightState, ControlInputs, physics systems
  controllers/    # FlightController trait + PID, Manual, LevelHold, HeadingHold, Ascent,
                  #   Orbit, Wingman, L1 (flight-plan), + RL variants          [inference]
  environment/    # infinite ground collider, plane spawner, runtime spawn/remove lifecycle
  camera/         # FreeLook and Follow camera modes                          [visual only]
  ui/             # egui HUD, map panel, main menu / scenario select, lifecycle panel [visual only]
  net/            # replicated components + client→server commands, shared by client & server [net]
  mcp/            # MCP control client: replicon client + rmcp stdio server                [mcp]
  training/       # TrainingEnv trait, self-contained 6-DOF integrator, PPO/BC training loops
  scenario.rs     # multi-plane .scenario.ron model + controller factory
  bin/            # train_ppo, train_bc, evaluate_policy, ml_planes_server, ml_planes_mcp
```

### Physics Stack

```
FlightController  →  Aero Model  →  Rapier RigidBody
```

The authoritative 64 Hz Rapier simulation — every `FlightController`, the aero model, and
fuel burn — runs in the headless `ml_planes_server` binary. The visual client renders
interpolated replicated state and sends commands; it runs no physics of its own.

Each plane is a full Rapier `RigidBody`. The aero model computes forces and torques in
body frame from current `FlightState`, `ControlInputs`, and the plane's `PlaneConfig`
asset, with air density and thrust scaled by altitude (ISA atmosphere model):

| Force/Moment | Equation |
|---|---|
| Lift | `L = q·S·(CL0 + CLα·α + CLδe·δe)` |
| Drag | `D = q·S·(CD0 + CDi·CL²)` |
| Pitching moment | `M = q·S·c̄·(Cm0 + Cmα·α + Cmq·(q·c̄/2V) + Cmδe·δe)` |
| Roll / Yaw | Lateral-directional coefficients + stability derivatives |

## Plane Assets & Scenarios

All aerodynamic data lives in `.plane.ron` files under `assets/planes/` (e.g.
`generic_jet`, `business_jet`, `cargo_jet`, `tanker`, `electric_trainer`), each paired
with a `.tuning.ron` PID gain pool. The asset loader parses these at runtime — no
compile-time plane data. Each plane also declares a `powerplant` (jet fuel or electric
charge) that burns down over a flight and lightens (jets) or holds constant (electric)
mass.

Multi-plane setups are described by `.scenario.ron` files under `assets/scenarios/`
(e.g. `default`, `wingman_formation`, `orbit`, `fleet_demo`, `mixed_powerplant`,
`stress_100`) — per-plane spawn state, config, fuel fraction, and controller. The main
menu's Scenario Select screen and `examples/observe_state.rs --scenario` both drive off
this same model.

## Controllers

| Controller | Description |
|---|---|
| `ManualController` | Keyboard/stick input; direct control-surface commands |
| `LevelHoldController` | Cascade PID holding target altitude and airspeed |
| `HeadingHoldController` | Holds a commanded heading via an inner level-hold cascade |
| `AscentController` | Climbs to a target altitude then hands off to level hold |
| `OrbitController` | 3-level cascade PID flying a circular orbit around a world-frame point |
| `WingmanController` | Formation flight; holds a fixed offset in a leader's body frame |
| `L1Controller` | Follows a preset `FlightPlan` (waypoints + orbit legs) via L1 nonlinear lateral guidance |
| `Rl*Controller` (`inference`) | Trained `burn` policies for level hold, orbit (direct/residual), and recurrent LSTM orbit |

Add a new controller by implementing `FlightController`:

```rust
pub trait FlightController: Send + Sync {
    fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs;
}
```

## MCP Server

`ml_planes_mcp` (built with `--features mcp`) is a headless control client that joins a running
`ml_planes_server` and exposes the live simulation to an LLM agent over the **Model Context
Protocol** (MCP stdio). An agent can inspect plane state and spawn/remove/reconfigure planes; it
does **not** fly manually. The binary reuses the net protocol verbatim — no server changes.

**Launch** (from the project root, so relative `assets/` paths resolve):

```bash
# 1. Start the authoritative sim server
cargo run --features server --bin ml_planes_server -- \
  --scenario assets/scenarios/default.scenario.ron --port 5555 &

# 2. Start the MCP client (normally launched by the MCP host, not by hand)
cargo run --features mcp --bin ml_planes_mcp -- --connect 127.0.0.1:5555
```

Options: `--connect host:port` (default `127.0.0.1:5555`), `--connect-timeout SECS` (connect +
handshake window and initial reconnect backoff, default 5), `--quiet` (drop logging to errors;
`RUST_LOG` still wins). stdout is the MCP JSON-RPC channel — all logs go to stderr. The client
**auto-reconnects** if the server restarts, and exits cleanly when the MCP host closes stdin.

**Register with Claude Code / Desktop** (example MCP config entry):

```json
{
  "mcpServers": {
    "ml_planes": {
      "command": "cargo",
      "args": ["run", "--features", "mcp", "--bin", "ml_planes_mcp", "--",
               "--connect", "127.0.0.1:5555"]
    }
  }
}
```

Point `command` at a prebuilt `target/release/ml_planes_mcp` for a faster start.

**Tools**

| Tool | Kind | Input → effect |
|---|---|---|
| `get_sim_status` | read | connection + plane count + last-requested sim speed |
| `list_planes` | read | roster: id, index, controller, position, altitude, airspeed, fuel |
| `get_plane_state` | read | `{ plane_id }` → full per-plane state (attitude, α/β, inputs, telemetry) |
| `spawn_plane` | write | `{ config_path, controller_kind, position?, … }` → add a plane (confirms new id) |
| `remove_plane` | write | `{ plane_id }` → remove (confirms gone) |
| `switch_controller` | write | `{ plane_id, controller_kind }` |
| `set_tuning_profile` | write | `{ plane_id, profile }` |
| `set_sim_speed` | write | `{ speed: Paused\|X1\|X5\|X10 }` (server-global) |
| `set_model` | write | `{ plane_id, model_path_stem }` — inference builds only |

`controller_kind` uses serde variant names (`Manual`, `LevelHold`, `HeadingHold`, `Ascent`,
`Orbit`, and under inference `RlLevelHold`/`RlOrbit`/`RlOrbitResidual`/`RlLstmOrbit`); `Wingman`
and `FlightPlan` are rejected. Writes are eventually-consistent — re-read after a write to observe
the applied change.

> **Feature parity with the server is mandatory:** `inference` changes the wire protocol, so the
> MCP client's features must match the server's — `mcp` ↔ `server`, `mcp,inference` ↔
> `server,inference`. A mismatched pair still completes the renet handshake and then mis-aligns
> replication silently.

**Manual stdio smoke check** (no MCP host needed) — pipe a handshake and one read tool:

```bash
printf '%s\n' \
  '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2024-11-05","capabilities":{},"clientInfo":{"name":"smoke","version":"0"}}}' \
  '{"jsonrpc":"2.0","method":"notifications/initialized"}' \
  '{"jsonrpc":"2.0","id":2,"method":"tools/list"}' \
  '{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"list_planes","arguments":{}}}' \
  | cargo run --features mcp --bin ml_planes_mcp -- --connect 127.0.0.1:5555
```

## Maneuver Roadmap

1. **Level flight hold** — complete. RL policy trained (`RlLevelHoldController`).
2. **Ascent** — complete. Climbs to target altitude then hands off to level hold.
3. **Formation flight (wingman)** — complete. Fixed body-frame offset behind a leader.
4. **Circular orbit** — complete. PID cascade plus direct, residual, and recurrent-LSTM RL variants.
5. **Flight-plan following** — complete. `L1Controller` sequences waypoint/orbit legs via L1 guidance.
6. **Aerial refueling** — next. Approach a lead plane from the rear to a docking position.

## Scope / Non-Goals

- No takeoff or landing. Planes spawn mid-air; ground contact ends the episode.
- No terrain, no compressibility, no structural limits.
- No client-side prediction — the renderer only interpolates replicated server state.
- ML runtime is pure Rust (`burn`). No Python, no IPC, no C extensions.
