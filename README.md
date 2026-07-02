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
| `inference` | `burn` CPU (ndarray) backend only — loads/runs trained RL policies headlessly (incl. `evaluate_policy`), no GPU/training stack |
| `training` | Builds on `inference`; adds the `burn` GPU/autodiff/train stack for the training loops |
| `wasm` | `visual` + `inference` — browser build (CPU inference in the renderer) |
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

1. **Level hold** — pitch/throttle PID to maintain altitude and airspeed (complete)
2. **Formation flight** — follow a lead plane at a fixed relative offset (next)
3. **Aerial refueling** — approach lead from the rear to a docking position (planned)

## Scope / Non-Goals

- No takeoff or landing. Planes spawn mid-air; ground contact ends the episode.
- No terrain, no compressibility, no structural limits.
- ML runtime is pure Rust (`burn`). No Python, no IPC, no C extensions.
