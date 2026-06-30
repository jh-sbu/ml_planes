# Plan: MCP Server for Live Simulation Inspection & Plane Spawning

## Context

`ml_planes` now runs client/server (`plans/client_server.md`, Phases 0–6 complete). The
dedicated `ml_planes_server` binary runs the authoritative headless 64 Hz Rapier sim and
broadcasts replicated component state (`FlightState`, `ControlInputs`, `PlaneId`,
`PlaneIndex`, `ControllerKind`, `SelectedTuningProfile`, `PlaneTuningPath`,
`ControllerTelemetry`, `SelectedModel`) over renet/replicon. Thin clients read those
components and send `FromClient` command events (`SpawnPlaneNetCommand`,
`RemovePlaneNetCommand`, `SwitchControllerCommand`, `SetTuningProfileCommand`,
`SetModelCommand`, `SetSimSpeedCommand`, `ManualInputCommand`) via `commands.client_trigger(..)`
(`bevy_replicon::prelude::ClientTriggerExt`). Connection readiness is observable as
`Res<State<ClientState>> == ClientState::Connected`.

We want an **MCP (Model Context Protocol) server** that lets an LLM agent (Claude Code /
Claude Desktop) **inspect the live simulation state and spawn planes** — plus the
additional mutating operations selected below — by speaking to a running `ml_planes_server`.

### Decisions locked with the user

- **Integration:** a **separate binary** that joins the running sim as a *headless
  renet/replicon client*. It reuses the existing net protocol verbatim — no changes to
  `ml_planes_server`. Works against any reachable server (local child process or remote
  host).
- **Write scope (beyond inspect + spawn):** **remove planes**, **switch controller /
  tuning profile / RL model**, and **sim speed / pause**. **Manual flight input is OUT of
  scope** (ill-suited to MCP's request/response model).
- **Transport:** **MCP stdio** — the binary is launched as a subprocess by the MCP client
  (Claude Code / Desktop). One MCP client per process.
- **Language / SDK:** **Rust** with the official **`rmcp`** SDK, consistent with the
  project's pure-Rust convention.

### Non-goals (this plan)

- No changes to `ml_planes_server`, the net protocol, or `PROTOCOL_ID`.
- No manual stick/keyboard flying over MCP.
- No HTTP/SSE transport (stdio only; revisit later if multi/remote MCP clients are needed).
- No rendering, no Bevy `App` window, no GPU. Headless throughout.
- No new scenario authoring; spawning is per-plane via `SpawnPlaneNetCommand`.

---

## Architecture Overview

```
┌─ MCP client (Claude Code / Desktop) ─┐
│  speaks MCP over stdio                │
└───────────────┬──────────────────────┘
                │ JSON-RPC (stdin/stdout)
┌───────────────▼─────────────────────── ml_planes_mcp (bin, --features mcp) ──────────┐
│  main thread: tokio runtime + rmcp stdio server (ServerHandler with #[tool] methods) │
│      tool handlers READ  ──────────────► Arc<RwLock<SimSnapshot>>   (latest mirror)   │
│      tool handlers WRITE ──────────────► mpsc::Sender<ControlRequest>                 │
│                                                                                        │
│  sim thread: headless Bevy App (ScheduleRunnerPlugin loop, ~64 Hz)                    │
│      MinimalPlugins + AssetPlugin + StatesPlugin                                       │
│      RepliconPlugins + RepliconRenetPlugins + NetProtocolPlugin                       │
│      start_renet_client(ConnectTarget)            ◄── connects to ml_planes_server    │
│      collect_snapshot system  → writes Arc<RwLock<SimSnapshot>> each frame            │
│      drain_control_requests system → client_trigger(SpawnPlaneNetCommand, …)          │
└───────────────┬──────────────────────────────────────────────────────────────────────┘
                │ renet/UDP (existing protocol, PROTOCOL_ID)
       ┌────────▼─────────┐
       │ ml_planes_server │  authoritative 64 Hz sim, unchanged
       └──────────────────┘
```

The MCP binary is **two threads bridged by two shared primitives**:

1. **`Arc<RwLock<SimSnapshot>>`** — a plain serde-friendly mirror of the replicated world
   (connection status + per-plane state). A Bevy system rewrites it every frame; rmcp read
   tools clone/read it. Lock is held briefly (write once per tick, read per tool call).
2. **`mpsc` channel `ControlRequest`** — rmcp write tools push a request; a Bevy system
   drains the receiver each frame and fires the matching `client_trigger(..)`. Use
   `crossbeam-channel` (or `std::sync::mpsc`) so the Bevy (sync) side drains without async.
   Requests are fire-and-forget at the protocol level; tools that need confirmation poll the
   snapshot (see Phase 3).

The Bevy `App` does **not** need the main thread (headless, no winit), so it runs on a
spawned thread while the main thread owns stdio + the tokio runtime that `rmcp` requires.

---

## Crate / Feature / Dependency Changes

`Cargo.toml`:

```toml
[features]
# Existing: default/client/visual/wasm/inference/training/net/server …
# MCP control client: a headless replicon client + the rmcp stdio server. Builds on
# `net` (protocol + transport). Inference is optional — only the set-model tool needs it.
mcp = ["net", "dep:rmcp", "dep:tokio", "dep:crossbeam-channel"]

[dependencies]
# rmcp is at 2.0.0 (a recent major bump). Pin the exact 2.0.x and verify its feature
# names — stdio/macros gating moved across versions (see "rmcp 2.0 API" below).
rmcp = { version = "2", optional = true, features = ["server", "transport-io", "macros"] }
tokio = { version = "1", optional = true, features = ["rt-multi-thread", "macros", "io-std", "sync"] }
crossbeam-channel = { version = "0.5", optional = true }
```

> **Do NOT add a separate `schemars` dependency.** Derive `JsonSchema` for tool-arg structs
> via rmcp's re-export (`rmcp::schemars`). A separately-pinned `schemars` (0.8 vs 1.0) yields
> a `JsonSchema` impl the `Parameters<T>` macro won't accept — a confusing trait-bound error.

```toml
[[bin]]
name = "ml_planes_mcp"
path = "src/bin/mcp.rs"
required-features = ["mcp"]
```

### rmcp 2.0 API surface (verify against the *installed* crate, not web tutorials)

`rmcp` is now **2.0.0** and its public API has broken repeatedly (`0.1→0.2→0.3→1.x→2.0`).
Nearly all online examples target 0.1–0.3 and **will not compile** on 2.0 — read the pinned
crate's own `cargo doc` and the rust-sdk repo's `examples/` at the matching git tag. The
shape `service.rs` is written against (confirm names/attributes before coding):

```rust
use rmcp::{handler::server::wrapper::Parameters, tool, tool_router, tool_handler,
           ServerHandler, ServiceExt, transport::stdio, schemars};

#[derive(serde::Deserialize, schemars::JsonSchema)]  // schemars via rmcp's re-export
struct GetPlaneStateArgs { plane_id: u32 }

#[tool_router]                                   // on the impl block (router codegen)
impl PlanesService {
    #[tool(description = "Full state of one plane")]
    async fn get_plane_state(&self, Parameters(a): Parameters<GetPlaneStateArgs>)
        -> Result<CallToolResult, ErrorData> { /* read snapshot */ }
}

#[tool_handler]                                  // on the ServerHandler impl (wires router)
impl ServerHandler for PlanesService {
    fn get_info(&self) -> ServerInfo { /* name, version, instructions, capabilities */ }
}

// main thread: let service = svc.serve(stdio()).await?; service.waiting().await?;
```

Points that moved and that this plan depends on: the **`#[tool_router]` + `#[tool]` +
`#[tool_handler]` split** (was a single `#[tool(tool_box)]`); the **`Parameters<T>`
extractor** (was `#[tool(aggr)]`/`#[tool(param)]`); the **error type** (`McpError` ↔
`ErrorData`); and the **`serve(stdio()).await` + `.waiting()`** startup (whose cancellation
handle is what Phase 5 uses for clean shutdown). Confirm whether name/version come from a
`#[tool_handler(..)]` arg or from `get_info()`.

- `bevy/serialize` is already enabled by `net`; reuse it. `SimSnapshot` is a fresh plain
  struct (its own `Serialize`) so it does not depend on Bevy components being serde.
- `mcp` is **purely additive** — it does not touch `client`, `server`, or test builds.
- Tests still run `--no-default-features`; MCP-specific tests run under `--features mcp`
  (headless; the snapshot/bridge logic is socket-free and unit-testable).
- The **set-model** tool is additionally gated `#[cfg(feature = "inference")]` because
  `SetModelCommand` only exists under `inference`. `mcp` alone ⇒ all tools except set-model;
  `mcp,inference` ⇒ + set-model.

> **Quarantine rmcp in `service.rs`.** Keep every rmcp type (macros, `Parameters`,
> `CallToolResult`, `ErrorData`) inside `src/mcp/service.rs`. The snapshot
> (`Arc<RwLock<SimSnapshot>>`) and the `ControlRequest` channel are plain Rust, so a future
> rmcp bump's blast radius is one file and the Bevy/bridge tests keep passing.
- `bevy/serialize` is already enabled by `net`; reuse it. `SimSnapshot` is a fresh plain
  struct (its own `Serialize`) so it does not depend on Bevy components being serde.
- `mcp` is **purely additive** — it does not touch `client`, `server`, or test builds.
- Tests still run `--no-default-features`; MCP-specific tests run under `--features mcp`
  (headless; the snapshot/bridge logic is socket-free and unit-testable).
- The **set-model** tool is additionally gated `#[cfg(feature = "inference")]` because
  `SetModelCommand` only exists under `inference`. `mcp` alone ⇒ all tools except set-model;
  `mcp,inference` ⇒ + set-model.

New module: `src/mcp/` (gated `#[cfg(feature = "mcp")]`, declared from `lib.rs`):

```
src/mcp/
  mod.rs        # McpBridgePlugin, shared types re-exports, feature gate
  snapshot.rs   # SimSnapshot + PlaneSnapshot + collect_snapshot system
  bridge.rs     # ControlRequest enum, channel resources, drain_control_requests system
  service.rs    # rmcp ServerHandler impl + #[tool] methods (PlanesService)
  args.rs       # CLI arg parsing (--connect host:port, --timeout)
src/bin/mcp.rs  # wires the two threads: spawn Bevy app thread, run rmcp on tokio main
```

---

## MCP Tool Catalog

All inputs/outputs are JSON (rmcp derives schemas from `#[derive(JsonSchema, Deserialize)]`
arg structs). `plane_id` is the `PlaneId(u32)` value.

**Read (snapshot-backed):**

| Tool | Input | Output |
|---|---|---|
| `get_sim_status` | — | `{ connected, server_addr, protocol_id, sim_speed, plane_count, sim_time }` |
| `list_planes` | — | `[{ plane_id, plane_index, controller_kind, position:[x,y,z], altitude, airspeed, fuel_remaining }]` |
| `get_plane_state` | `{ plane_id }` | full `PlaneSnapshot`: position, velocity, attitude (quat), angular_velocity, alpha, beta, airspeed, altitude, consumable_remaining, control_inputs `{aileron,elevator,rudder,throttle}`, controller_kind, tuning_profile?, model?, telemetry |

**Write (channel-backed → `client_trigger`):**

| Tool | Input | Maps to |
|---|---|---|
| `spawn_plane` | `{ config_path, controller_kind, position?, velocity?, attitude?, angular_velocity?, fuel_fraction? }` | `SpawnPlaneNetCommand { config_path, kind, spec: SpawnSpec }` |
| `remove_plane` | `{ plane_id }` | `RemovePlaneNetCommand { plane }` |
| `switch_controller` | `{ plane_id, controller_kind }` | `SwitchControllerCommand { plane, kind }` |
| `set_tuning_profile` | `{ plane_id, profile }` | `SetTuningProfileCommand { plane, profile }` |
| `set_sim_speed` | `{ speed: "Paused"\|"X1"\|"X5"\|"X10" }` | `SetSimSpeedCommand { speed }` |
| `set_model` *(inference only)* | `{ plane_id, model_stem }` | `SetModelCommand { plane, model_stem }` |

`controller_kind` accepts the `ControllerKind` variant names (`"Manual"`, `"LevelHold"`,
`"Orbit"`, `"Wingman"`, `"Ascent"`, `"HeadingHold"`, `"FlightPlan"`, `"RlLevelHold"`,
`"RlOrbit"`, `"RlOrbitResidual"`, `"RlLstmOrbit"`). Validation maps unknown names to a
descriptive MCP tool error rather than a panic.

**Confirmation semantics:** writes are asynchronous round-trips through the server. Tools
return immediately with `{ status: "sent" }` by default; `spawn_plane` and `remove_plane`
optionally **poll the snapshot** for up to a short deadline (e.g. 1 s) to confirm the plane
count / target id changed, returning `{ status: "confirmed", plane_id }` or `{ status:
"timeout" }`. Document this so the agent knows results are eventually-consistent.

---

## Phase 0 — Dependencies, Feature, Binary Skeleton

Stand up a buildable `ml_planes_mcp` that connects and serves an empty MCP surface.

0. **Pin rmcp 2.0 and spike it in isolation first.** Add the `mcp` feature + deps, then write
   a throwaway ~30-line calculator tool server (the canonical 2.0 example) behind the feature
   and confirm `initialize` + `tools/list` work over stdio — **before** touching any Bevy or
   snapshot code. This separates "did I get the rmcp 2.0 API right" from "did I get the
   bridge right." Read the pinned crate's own `cargo doc` + the rust-sdk `examples/` at the
   matching tag; ignore external tutorials (they target 0.1–0.3). Derive `JsonSchema` via
   `rmcp::schemars`. Delete the spike once Phase 2 lands real tools.
1. Add the `mcp` feature, `rmcp`/`tokio`/`crossbeam-channel` deps, and the `[[bin]]` entry
   (above), with the exact verified 2.0.x version and feature names.
2. Create `src/mcp/mod.rs` gated `#[cfg(feature = "mcp")]`; declare it from `lib.rs`.
3. `src/mcp/args.rs` — parse `--connect host:port` (default `127.0.0.1:5555` via
   `DEFAULT_PORT`) and `--connect-timeout` (default 5 s), mirroring `server.rs`'s arg style
   and the client `parse_addr` helper.
4. `src/bin/mcp.rs` skeleton:
   - Build the headless Bevy `App` (MinimalPlugins + ScheduleRunnerPlugin loop +
     AssetPlugin + StatesPlugin + RepliconPlugins + RepliconRenetPlugins +
     `NetProtocolPlugin`), insert `ConnectTarget`, add `start_renet_client` at `Startup`.
     Spawn it on a dedicated thread (`std::thread::spawn` running `app.run()`).
   - On the main thread, start a tokio runtime and an `rmcp` stdio server exposing a
     **stub** `PlanesService` (no tools yet, or a single `ping`).
5. **Logging discipline:** stdio is the MCP channel — route all logs to **stderr** only
   (no `println!`/`info!` to stdout). Configure Bevy/log accordingly.

**TDD:** `args.rs` parse unit tests (host:port, defaults, bad input). Build check:
`cargo build --features mcp` and `cargo build --features "mcp inference"`.

**Milestone 0:** `cargo build --features mcp` succeeds; running `ml_planes_mcp` against a
live `ml_planes_server` connects (renet handshake logs to stderr) and an MCP `initialize`
+ `tools/list` handshake over stdio returns cleanly (empty/ping tool set). `cargo test
--no-default-features` and `--features mcp` both green; `cargo fmt`.

---

## Phase 1 — Snapshot Mirror (read path, no MCP yet)

Mirror the replicated world into a shared, serde-friendly snapshot.

1. `src/mcp/snapshot.rs` — define `SimSnapshot { connected: bool, server_addr: String,
   sim_speed: SimSpeed, sim_time: f32, planes: Vec<PlaneSnapshot> }` and `PlaneSnapshot`
   (all fields from the tool catalog). Plain `#[derive(Clone, Serialize, Default)]` structs
   — independent of Bevy component serde so they are trivially unit-testable.
2. `SnapshotHandle(Arc<RwLock<SimSnapshot>>)` Bevy `Resource` (clone of the Arc the rmcp
   side also holds).
3. `collect_snapshot` system (Update): query
   `(&PlaneId, &PlaneIndex, &FlightState, &ControlInputs, &ControllerKind,
   Option<&SelectedTuningProfile>, Option<&ControllerTelemetry>, Option<&SelectedModel>)`,
   read `State<ClientState>` for `connected`, `Res<SimSpeed>`-equivalent (or the replicated
   value if exposed) for `sim_speed`, and write a freshly-built `SimSnapshot` under the
   write lock once per frame.
   - Note: the server owns `SimSpeed`; if it is not replicated, report the **last value the
     MCP set** (track locally) and mark it `requested` vs `authoritative`. Confirm during
     implementation whether `SimSpeed` arrives on the client; adjust the field accordingly.
4. `McpBridgePlugin` adds `collect_snapshot` and holds the `SnapshotHandle`.

**TDD:** unit-test a `build_snapshot(planes, connected, …)` pure helper (feed synthetic
`FlightState`/`ControllerKind` → assert `PlaneSnapshot` fields, telemetry mapping, fuel,
controller-kind serialization). Headless app test (`--features mcp`): register the protocol
+ plugin without a transport, spawn a `PlaneId`+`FlightState` entity, run two updates,
assert the snapshot reflects it (mirrors the `tests/server_sim.rs` transport-free pattern).

**Milestone 1:** with a synthetic plane in a headless app, the shared `SimSnapshot` is
populated correctly; serialization round-trips to JSON. Tests green; `cargo fmt`.

---

## Phase 2 — Read Tools Over MCP

Expose the snapshot through `rmcp`.

1. `src/mcp/service.rs` — `PlanesService { snapshot: Arc<RwLock<SimSnapshot>>, tx:
   Sender<ControlRequest> }` implementing rmcp's `ServerHandler` with a `#[tool_router]`.
2. Implement read tools: `get_sim_status`, `list_planes`, `get_plane_state { plane_id }`.
   - Each acquires the read lock, clones the needed slice, releases, serializes to the MCP
     result. `get_plane_state` on an unknown id → tool error "no plane with id N".
   - Guard on `connected`: if not connected, return a structured error/notice so the agent
     can retry rather than seeing empty data as "no planes".
3. Wire `PlanesService` into `mcp.rs` (replace the stub), sharing the same `Arc` the Bevy
   thread writes.

**TDD:** unit tests construct a `PlanesService` over a hand-built `SimSnapshot` (no Bevy, no
sockets) and assert each tool's JSON output, including the unknown-id and disconnected
cases. (rmcp tool methods are plain async fns and callable directly in tests.)

**Milestone 2:** against a live server running `default.scenario.ron`, an MCP client sees
`list_planes` return the scenario's planes with correct positions/controllers, and
`get_plane_state` returns full per-plane state that tracks the live sim across repeated
calls. Tests green; `cargo fmt`.

---

## Phase 3 — Command Bridge + spawn/remove Tools

Add the write path and the two headline mutating tools.

1. `src/mcp/bridge.rs` — `enum ControlRequest { Spawn { config_path, kind, spec },
   Remove { plane }, SwitchController { plane, kind }, SetTuningProfile { plane, profile },
   SetSimSpeed { speed }, #[cfg(inference)] SetModel { plane, model_stem } }`. Channel
   endpoints: `ControlSender(Sender<ControlRequest>)` held by the service; `ControlReceiver`
   Bevy `Resource` wrapping the `Receiver`.
2. `drain_control_requests` system (Update): `try_recv` until empty, matching each variant
   to its `commands.client_trigger(..)` (the exact calls the visual client makes in
   `ui/lifecycle_panel.rs`, `ui/hud.rs`, `ui/time_control.rs`, `main.rs`). Keep this the
   single client→server send path.
3. Implement `spawn_plane` and `remove_plane` tools: build the `SpawnSpec` (Vec3/Quat from
   the optional JSON arrays; `None` ⇒ scenario/spawn defaults), push the `ControlRequest`.
   - **Confirmation:** after sending, poll the snapshot up to a deadline for the plane-count
     increase (spawn) / id disappearance (remove); return confirmed/timeout (see catalog).
     For spawn, report the new `PlaneId` (the highest/new id not previously present).

**TDD:** unit test the JSON-args → `ControlRequest` / `SpawnSpec` mapping (incl. optional
fields and bad Vec3 lengths → error). Headless app test (`--features mcp`, transport-free):
push a `ControlRequest::Spawn` on the channel, run updates, assert `drain_control_requests`
fired the right `client_trigger` (observe via a test observer capturing the event, mirroring
the existing net command tests).

**Milestone 3:** against a live server, `spawn_plane` adds a plane that subsequently appears
in `list_planes` (and in a connected visual client's view), and `remove_plane` removes it.
Confirmation polling returns the new `PlaneId`. Tests green; `cargo fmt`.

---

## Phase 4 — Controller / Tuning / Model / Sim-Speed Tools

Complete the selected write scope.

1. Implement `switch_controller`, `set_tuning_profile`, `set_sim_speed`, and (under
   `inference`) `set_model`, each pushing its `ControlRequest` and returning `{status:
   "sent"}`.
2. `controller_kind` parsing: a `&str` → `ControllerKind` map with a clear error for unknown
   names; reuse `ControllerKind::ALL`/`name()` so the tool description can enumerate valid
   values. Note RL kinds only *build* server-side under `inference` (the server falls back to
   the PID equivalent otherwise) — surface this in the tool doc string.
3. `set_sim_speed`: parse the `SimSpeed` variant name; document that pause/accel is global
   (server authoritative).
4. Snapshot feedback: after `switch_controller`/`set_tuning_profile`/`set_model`, the agent
   can re-`get_plane_state` to observe the applied change (server replicates the new
   `ControllerKind`/`SelectedTuningProfile`/`SelectedModel`). Note the eventual-consistency
   delay in the tool docs.

**TDD:** unit tests for the controller-kind/sim-speed string parsers (valid + invalid).
Headless channel test asserting each tool variant drains to the correct `client_trigger`.

**Milestone 4:** against a live server, switching a plane's controller, selecting a tuning
profile, and changing sim speed all take effect (observable via `get_plane_state` /
`get_sim_status` and in a visual client). With `--features "mcp inference"`, `set_model`
swaps an RL model. Tests green; `cargo fmt`.

---

## Phase 5 — Robustness, Lifecycle, Docs

Make it dependable and easy to register with an MCP client.

1. **Connection lifecycle:** handle "server not up yet" (retry/backoff on
   `start_renet_client` failure), disconnect/timeout (reflect `connected:false` in the
   snapshot; tools return a clear "not connected" notice), and clean shutdown when stdin
   closes (signal the Bevy thread to exit, e.g. an `AtomicBool`/shutdown channel).
2. **Graceful tool errors:** never panic in a tool handler — every failure path returns an
   MCP error result. Add a top-level catch so a poisoned lock or send failure degrades
   gracefully.
3. **stderr-only logging** verified; optional `--quiet`/`RUST_LOG` respect.
4. **Docs:**
   - Module headers in `src/mcp/*` following the codebase style.
   - A `plans/`-adjacent or `README`/`CLAUDE.md` note: how to launch the server, then
     register `ml_planes_mcp` with Claude Code/Desktop (example MCP config invoking
     `cargo run --features mcp --bin ml_planes_mcp -- --connect 127.0.0.1:5555`, or a built
     binary path), and the tool catalog.
   - Update `CLAUDE.md` §2 module map (`src/mcp/`), §5 features/bins (`mcp` feature +
     `ml_planes_mcp` bin), and the test list.
5. **Optional ergonomics (confirm before building):** a `--spawn-server` flag that launches
   a local `ml_planes_server` child (like the visual menu's `launch_local_server`) so a
   single `ml_planes_mcp` invocation both hosts and controls a sim. Defer unless wanted.

**TDD/Verification:** a small integration test (`--features mcp`) that boots a real
`ml_planes_server` in-process or as a child on an ephemeral port, runs the MCP binary's
connect + a couple of tool calls end-to-end, and asserts results — if feasible without
flakiness; otherwise document a manual verification script.

**Milestone 5:** server-down, mid-session disconnect, and stdin-close are all handled
without panics or hangs; logs never corrupt the MCP stream; an LLM agent can complete a
full "inspect → spawn → reconfigure → remove" loop against a running server using only the
documented MCP config. Full suite green (`--no-default-features`, `--features mcp`,
`--features "mcp inference"`); `cargo fmt`.

---

## Testing Strategy (summary)

- **Pure helpers first (TDD):** `build_snapshot`, JSON-args→`ControlRequest`/`SpawnSpec`,
  controller-kind & sim-speed parsers, telemetry mapping — no Bevy, no sockets, no rmcp
  transport.
- **rmcp tool methods:** called directly in unit tests over hand-built snapshots/channels.
- **Headless Bevy (transport-free):** register `NetProtocolPlugin` + `McpBridgePlugin` with
  MinimalPlugins (mirroring `tests/server_sim.rs`); assert snapshot population and that
  `drain_control_requests` emits the right `client_trigger` events (captured via a test
  observer).
- **End-to-end (Phase 5):** real server + MCP binary on an ephemeral port, if non-flaky.
- All existing suites continue under `cargo test --no-default-features`. `cargo fmt` ends
  every cycle.

## Risks / Open Questions

- **`rmcp` API churn (highest-churn dependency):** rmcp is at **2.0.0** with a history of
  breaking every minor (`0.1→0.2→0.3→1.x→2.0`); most online examples target 0.1–0.3 and
  won't compile. Mitigated by pinning exact 2.0.x, reading the *installed* crate's docs +
  rust-sdk `examples/` (not tutorials), the **spike-first** Phase 0 step, deriving
  `JsonSchema` via **`rmcp::schemars`** (avoids the 0.8/1.0 schemars version-mismatch
  footgun), and **quarantining all rmcp types in `service.rs`** so a future bump touches one
  file. Confirm the `#[tool_router]`/`#[tool]`/`#[tool_handler]` attribute forms, the
  `Parameters<T>` extractor, the error type (`McpError`/`ErrorData`), whether name/version is
  a `#[tool_handler]` arg vs `get_info()`, and the exact stdio/macros feature-flag names.
- **Async↔Bevy bridge:** two threads + `Arc<RwLock>` + sync channel is the simplest correct
  bridge; keep lock hold times minimal (write once/tick, clone-on-read). Avoid blocking the
  tokio runtime on the Bevy lock.
- **`SimSpeed` visibility on the client:** confirm whether it is replicated; if not, the
  snapshot reports the MCP's last-requested value (flagged as requested vs authoritative).
- **Confirmation latency:** writes are eventually-consistent round-trips; the polling
  deadline must tolerate replication delay without hanging the tool call.
- **Replication completeness for spawn id:** identifying the *new* `PlaneId` after spawn
  relies on snapshot diffing; document the small race if multiple spawns are in flight.
- **stdout hygiene:** any stray stdout write corrupts the MCP JSON-RPC stream — enforce
  stderr-only logging from the start.

## Future Extensions (out of scope here)

- HTTP/SSE transport for multiple/remote MCP clients.
- Manual flight-input streaming tool (if a non-request/response control model is added).
- Scenario load/save tools; episode reset; bulk spawn from a scenario file.
- Read tools for aggregate metrics (formation error, orbit radial error history).
- Embedding the MCP endpoint directly in `ml_planes_server` (the alternative integration).
```
