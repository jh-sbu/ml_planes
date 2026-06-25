Plan: Client/Server Refactor of ml_planes Visual Mode

 Context

 Today ml_planes runs as a single monolithic visual binary: the same process renders
 the scene and runs the authoritative 64 Hz Rapier physics, all FlightControllers,
 fuel burn, and aerodynamics. The main menu's Start Scenario button drops the user
 straight into local play (AppState::MainMenu → ScenarioSelect → InGame, spawning
 planes on OnEnter(InGame)).

 We want to split this into client/server: physics + controllers run on an
 authoritative dedicated server binary, and the visual app becomes a pure
 display/control client that renders interpolated state and sends commands. The main
 menu's single play button is replaced by Start New Server (launch a local dedicated
 server and connect) and Connect to Server (join a remote host).

 Decisions locked with the user:
 - Clients: native desktop only (WASM/browser networking deferred — see Out of Scope).
 - Hosting: dedicated server binary. The client is always a pure client; even
 local play goes through a server process.
 - Authority: shared control — any connected client may cycle camera, switch
 controllers, manually fly, and spawn/remove any plane by PlaneId. No per-client
 ownership.
 - Networking crate: bevy_replicon (Bevy-native component replication) with its
 renet UDP transport backend.

 Why this is tractable: the codebase is already cleanly layered. Physics
 (PlanePlugin, EnvironmentPlugin, LifecyclePlugin, Rapier) is headless-safe and
 already feature-gated away from rendering (#[cfg(feature = "visual")]). A proven
 headless-app pattern exists (tests/common/mod.rs, examples/observe_state.rs:
 MinimalPlugins + AssetPlugin + TransformPlugin + TimestepMode::Fixed +
 RapierPhysicsPlugin::in_fixed_schedule() + app.finish()). The main gaps are: (1) no
 networking deps, (2) core sim-state types aren't serde, (3) the controller-switch /
 tuning / model / flight-plan rebuild systems live in main.rs behind
 #[cfg(feature = "visual")] and must move server-side, and (4) the client must stop
 running physics and instead render replicated state.

 ---
 Architecture Overview

 ┌──────────────── ml_planes_server (bin, --features server) ──────────────┐
 │ MinimalPlugins + AssetPlugin + TransformPlugin                          │
 │ RapierPhysicsPlugin::in_fixed_schedule()  (64 Hz, authoritative)        │
 │ PlanePlugin + EnvironmentPlugin(headless) + LifecyclePlugin             │
 │ SimControlPlugin   (relocated controller-switch/tuning/model rebuilds)  │
 │ RepliconServerPlugins + renet server transport                          │
 │  - loads scenario (--scenario), spawns planes with `Replicated`         │
 │  - runs controllers + physics, broadcasts state                         │
 │  - applies FromClient commands (switch/tune/model/manual/spawn/remove)  │
 └─────────────────────────────────────────────────────────────────────────┘
                 ▲ commands (FromClient)     │ replicated state ▼
 ┌──────────────── ml_planes (bin, --features client = visual + net) ──────┐
 │ DefaultPlugins + EguiPlugin + CameraPlugin + UiPlugin                    │
 │ RepliconClientPlugins + renet client transport                          │
 │  - NO Rapier physics, NO PlanePlugin FixedUpdate sim systems            │
 │  - decorate replicated plane entities with mesh/material + interp       │
 │  - HUD/map/camera read replicated FlightState/ControlInputs (read-only) │
 │  - dropdowns/keys/panels SEND commands instead of mutating components    │
 └─────────────────────────────────────────────────────────────────────────┘

 Shared, network-neutral code (aerodynamics/, controllers/, plane/,
 environment/ core, scenario.rs) is unchanged in behavior and compiled into both.

 ---
 Feature Flags & Binaries (Cargo.toml)

 [features]
 default  = ["client"]
 client   = ["visual", "net"]                      # the renderer + networking
 visual   = ["bevy/default", "bevy_egui", "rfd"]
 server   = ["net", "bevy_asset_only"]             # headless authoritative sim
 net      = ["dep:bevy_replicon", "dep:bevy_replicon_renet", "bevy/serialize"]
 wasm     = ["visual", "inference"]                # unchanged; no net (deferred)
 inference = ["burn/std", "burn/ndarray"]
 training  = ["inference", "burn/wgpu", "burn/autodiff", "burn/train", "burn/tui"]

 [[bin]]                       # existing main.rs — now the client
 name = "ml_planes"
 required-features = ["client"]

 [[bin]]                       # new dedicated server
 name = "ml_planes_server"
 path = "src/bin/server.rs"
 required-features = ["server"]

 - bevy/serialize is enabled by net so Vec3/Quat/Transform get Serialize/
 Deserialize (needed by replicon). Verify the exact bevy_replicon /
 bevy_replicon_renet releases that target bevy 0.18 before pinning versions.
 - Tests still run --no-default-features (no rendering, no net). Net/server tests run
 under --features server (headless, replicon is headless-friendly).
 - The default = ["client"] means the plain cargo run is the networked client.
 Single-player as it exists today is removed; the equivalent is Start New
 Server, which launches a local server child process and connects to it.

 ---
 Phase 0 — Foundations: serde + deps + headless extraction

 Make the core types serializable and add dependencies, with no behavior change yet.

 1. Add deps (Cargo.toml): bevy_replicon, bevy_replicon_renet (optional, behind
 net). Confirm bevy-0.18-compatible versions.
 2. Add serde derives (TDD: write round-trip tests first in a new
 tests/net_serde.rs gated to server/net):
   - FlightState (src/plane/state.rs) — all 6-DOF fields.
   - ControlInputs (src/plane/inputs.rs).
   - PlaneId, PlaneIndex (src/plane/context.rs, src/plane/mod.rs).
   - ControllerKind (src/controllers/kind.rs) — already Clone/Copy/PartialEq.
   - SpawnSpec / spawn params if sent as commands (src/environment/spawner.rs).
   - ControllerSpec, PlaneSpec already Deserialize; add Serialize if commands
 carry them.
   - Vec3/Quat rely on bevy/serialize (enabled via net).
 3. Milestone 0: cargo test --no-default-features green (unchanged);
 cargo build --features server compiles; cargo test --features server net_serde
 round-trips every type above byte-for-byte.

 ---
 Phase 1 — Shared network protocol module (src/net/)

 A new module compiled into both client and server (gated #[cfg(feature = "net")],
 pub mod net; in lib.rs).

 1. src/net/mod.rs + src/net/protocol.rs: a NetProtocolPlugin registering, in the
 same order on client and server:
   - Replicated components: .replicate::<Transform>(), .replicate::<FlightState>(),
 .replicate::<ControlInputs>(), .replicate::<PlaneId>(),
 .replicate::<PlaneIndex>(), .replicate::<ControllerKind>().
   - Client→server command events (replicon add_client_event / trigger-based):
       - SwitchControllerCommand { plane: PlaneId, kind: ControllerKind }
     - SetTuningProfileCommand { plane: PlaneId, profile: String }
     - SetModelCommand { plane: PlaneId, model_stem: String }  (net + inference)
     - ManualInputCommand { plane: PlaneId, inputs: ControlInputs }
     - SpawnPlaneNetCommand { config_path: String, kind: ControllerKind, spec: SpawnSpec }
     - RemovePlaneNetCommand { plane: PlaneId }
     - SetSimSpeedCommand { speed: SimSpeed } (time accel/pause is now server-side)
 2. Define shared constants: default port, protocol id/version.
 3. Milestone 1: unit tests for each command type's serde round-trip;
 NetProtocolPlugin builds in a headless app without panicking.

 ---
 Phase 2 — Relocate sim-control systems out of main.rs (headless-capable)

 The controller-rebuild logic must run on the server. Today it lives in main.rs behind
 #[cfg(feature = "visual")]: apply_initial_tuning, apply_controller_switch,
 apply_flight_plan, apply_rl_controller_switch, apply_model_switch (the
 apply_*/rebuild half), plus the helper logic they call.

 1. Create src/controllers/sim_control.rs (or src/plane/sim_control.rs) exposing a
 headless SimControlPlugin that registers the rebuild systems on PostUpdate
 (no run_if(in_state(InGame)) — the server has no AppState). RL arms stay
 #[cfg(feature = "inference")].
 2. Keep the input-polling / hotkey halves (poll_controller_inputs,
 switch_controller, cycle_tune_profile, cycle_rl_model) in the client only —
 they become command senders in Phase 6.
 3. The rebuild systems already react to Changed<ControllerKind> /
 Changed<SelectedTuningProfile> / Changed<SelectedModel>, so the server applies a
 client command simply by mutating those components; the existing system rebuilds the
 ActiveController. Move SelectedTuningProfile/SelectedModel component definitions
 so they are available headlessly (they're currently effectively visual-only).
 4. apply_flight_plan needs the FlightPlanHandle asset path — runs on the server
 (which has AssetPlugin), unchanged.
 5. TDD: write tests/sim_control.rs (headless) that spawns a plane, flips
 ControllerKind, runs app.update(), and asserts ActiveController rebuilt to the
 new controller — before moving the code. Then move and make it pass.
 6. Milestone 2: the relocated systems run in a MinimalPlugins app; client visual
 build still compiles and behaves identically; tests/sim_control.rs passes.

 ---
 Phase 3 — Dedicated server binary (src/bin/server.rs, ml_planes_server)

 1. Headless app mirroring tests/common/mod.rs: MinimalPlugins + AssetPlugin +
 TransformPlugin + TimestepMode::Fixed{dt:1/64} +
 RapierPhysicsPlugin::in_fixed_schedule() + PlanePlugin + EnvironmentPlugin
 (headless half only) + LifecyclePlugin + SimControlPlugin + NetProtocolPlugin.
 2. Add RepliconServerPlugins + bevy_replicon_renet server transport bound to a port
 (CLI --bind/--port, default from Phase 1).
 3. CLI args (reuse scenario.rs): --scenario <path> (default
 assets/scenarios/default.scenario.ron). On startup load + resolve() + spawn via
 the existing spawn_resolved_scenario (src/environment/scenario_spawn.rs),
 which already calls spawn_plane (full Rapier body) and build_controller.
 4. spawn_plane change: under #[cfg(feature = "server")] (or whenever net is on
 and acting as server) insert the replicon Replicated marker so every spawned plane
 replicates. Keep PhysicsInterp insertion #[cfg(feature = "visual")] only.
 5. Server command handlers (systems/observers reading FromClient<…>):
   - SwitchControllerCommand → set ControllerKind on the target entity (rebuild via
 SimControlPlugin).
   - SetTuningProfileCommand/SetModelCommand → set SelectedTuningProfile/
 SelectedModel.
   - ManualInputCommand → downcast the target ActiveController to ManualController
 and set its inputs (mirrors the existing poll_input write path), so the next
 run_flight_controllers tick emits them.
   - SpawnPlaneNetCommand/RemovePlaneNetCommand → commands.trigger(SpawnPlaneCommand …) / RemovePlaneCommand(entity) reusing the existing LifecyclePlugin observers
 (resolve PlaneId → Entity for removal).
   - SetSimSpeedCommand → scale Time<Virtual> server-side (the authoritative clock),
 reusing SimSpeed semantics from src/ui/time_control.rs (move SimSpeed enum to
 a shared, non-visual location).
 6. TDD: tests/server_sim.rs (headless, --features server): boot the server app,
 step it, assert planes spawned with Replicated + PlaneId, and that injecting a
 FromClient switch command rebuilds the controller. Replicon provides in-memory test
 harness utilities for client/server in one process — use them to assert a command
 round-trips into a state change.
 7. Milestone 3: cargo run --features server --bin ml_planes_server -- --scenario assets/scenarios/default.scenario.ron boots, listens, spawns the default
 scene, and steps physics; integration test green.

 ---
 Phase 4 — Client networking + render replicated state

 Turn the visual app into a pure client.

 1. Add RepliconClientPlugins + renet client transport (behind net). Add a
 ConnectToServer { addr } action that configures + opens the transport.
 2. Stop client-side physics. Gate PlanePlugin's FixedUpdate sim chain
 (sync_flight_state, run_flight_controllers, consume_fuel, update_plane_mass,
 apply_aerodynamic_forces) and the RapierPhysicsPlugin so they are not added in
 the client build (add them only under server, or a shared sim/server predicate).
 The client keeps PlanePlugin's asset loaders + NextPlaneId-free parts it still
 needs for rendering. Net result: client entities are driven purely by replication.
 3. Decorate replicated planes: a client system keyed on Added<PlaneId> (or
 Added<Replicated>) inserts the visual bundle (Mesh3d, material, PhysicsInterp/
 interpolation component) onto the replicated entity. This replaces the visual half of
 spawn_plane for the client. Reuse the mesh/material currently built in
 spawn_plane.
 4. Interpolation (no prediction): add a NetInterpolation component buffering the
 last two replicated Transforms with arrival times; a client Update system renders
 the entity Transform at now - render_delay by lerping position and slerping
 rotation. Reuse the existing PhysicsInterp smoothing approach
 (environment/plugin.rs save_prev/curr_physics_pose), but source poses from
 replication instead of Rapier. One small render-delay buffer (~2 server ticks) is
 enough.
 5. HUD (ui/hud.rs), map (ui/map.rs), camera (camera/), gizmos read the replicated
 FlightState/ControlInputs/PlaneIndex exactly as today (read-only) — no query
 changes needed since the same components now arrive via replication.
 6. TDD: a headless --features server-style test that runs a replicon
 server+client pair in-process, spawns a plane on the server, steps, and asserts the
 client entity received FlightState/Transform. Interpolation math gets a pure unit
 test (two snapshots → expected lerp).
 7. Milestone 4: with a server running locally, launch the client, point it at
 127.0.0.1, and see the default scenario's planes flying smoothly (interpolated),
 HUD/map/camera all live.

 ---
 Phase 5 — Menu refactor: Start New Server / Connect to Server

 Rework src/ui/menu.rs and the AppState flow.

 1. AppState becomes: MainMenu, ScenarioSelect (host path), ConnectEntry
 (address entry), Connecting, InGame. (InGame now means "connected & rendering",
 not "local sim".)
 2. Main menu (draw_main_menu): replace Start Scenario with:
   - Start New Server → ScenarioSelect.
   - Connect to Server → ConnectEntry.
   - Keep Quit; keep Train placeholder (#[cfg(feature = "training")]).
 3. Start New Server flow: draw_scenario_select (reuse existing scanner
 discover_scenarios) picks a .scenario.ron; on confirm:
   - Launch the server as a child process: std::process::Command on
 std::env::current_exe() with the filename swapped to ml_planes_server, passing
 --scenario <path> + --port <p>. Store the Child in a resource so OnExit/quit
 can kill it.
   - Transition to Connecting, opening a client transport to 127.0.0.1:<p> with retry
 until the freshly-spawned server accepts; then → InGame.
 4. Connect to Server flow: draw_connect_entry — a text field for host:port
 (default 127.0.0.1:<port>) + Connect/Back. Connect → Connecting → InGame (or
 back to MainMenu with a Notifications error on failure/timeout).
 5. OnExit(InGame) (despawn_in_game_planes): now disconnect the client transport
 and, if we launched a local server child, terminate it. Replicated entities are
 removed by replicon on disconnect; reset camera to FreeLook as today.
 6. Remove spawn_selected_scenario's direct spawn_resolved_scenario call — spawning is
 now the server's job; the client only selects the scenario to hand to the server
 it launches.
 7. Milestone 5: end-to-end on one machine — launch client, Start New Server, pick
 default, fly; launch a second client, Connect to Server at 127.0.0.1, and see
 the same planes. Returning to the main menu disconnects and kills the local server.

 ---
 Phase 6 — Route client controls through commands (shared control)

 Convert every client-side mutation into a network command (server stays authoritative).

 1. Controller dropdown / C key (ui/hud.rs, switch_controller): instead of
 mutating local ControllerKind, send SwitchControllerCommand { plane, kind }.
 2. Tuning dropdown / T → SetTuningProfileCommand. RL model dropdown / cycle
 → SetModelCommand (net + inference).
 3. Manual flight: when the selected plane's kind is Manual, the client polls
 keyboard (ManualController::poll_input logic, manual.rs) locally each frame and
 sends ManualInputCommand { plane, inputs }; the server feeds the target
 ManualController (Phase 3 handler). Send at client frame rate; server uses latest.
 4. Lifecycle panel + N/Delete hotkeys (ui/lifecycle_panel.rs): fire
 SpawnPlaneNetCommand / RemovePlaneNetCommand instead of the local
 SpawnPlaneCommand/RemovePlaneCommand triggers.
 5. Time control (ui/time_control.rs): Pause/1x/5x/10x send SetSimSpeedCommand;
 the client no longer scales its own Time<Virtual> (it just renders interpolated
 state). The displayed speed reflects server state if replicated, else optimistic.
 6. Camera cycling, map pan/zoom, free-look stay purely local (they don't affect
 the sim) — no commands needed.
 7. Milestone 6: from a client, switching controllers, manually flying, spawning/
 removing planes, and changing time-acceleration all take effect on the server and are
 reflected on every connected client.

 ---
 Phase 7 — Cleanup, docs, verification

 1. Update CLAUDE.md: new client/server/net features, ml_planes_server bin, the
 src/net/ module, the client/server split of the "Visual App Flow" and "Runtime Plane
 Lifecycle" sections, and the new command/replication protocol.
 2. cargo fmt (mandatory per project rules).
 3. Full test sweep: cargo test --no-default-features (must stay green) and
 cargo test --features server (net/server tests).
 4. Milestone 7: all tests green, cargo fmt clean, manual end-to-end verified.

 ---
 Key Files to Create / Modify

 New
 - src/bin/server.rs — ml_planes_server dedicated server.
 - src/net/mod.rs, src/net/protocol.rs — replication + command protocol (NetProtocolPlugin).
 - src/controllers/sim_control.rs (or plane/sim_control.rs) — relocated rebuild systems (SimControlPlugin).
 - tests/net_serde.rs, tests/sim_control.rs, tests/server_sim.rs, client-render/interp tests.

 Modified
 - Cargo.toml — features (client/server/net), bevy/serialize, replicon deps, second [[bin]].
 - src/main.rs — drop in-process physics + add_plugins(RapierPhysicsPlugin/...) for the client; add RepliconClientPlugins + transport; move rebuild-system registration out.
 - src/lib.rs — pub mod net; (gated); export new plugins.
 - src/plane/state.rs, src/plane/inputs.rs, src/plane/context.rs, src/plane/mod.rs, src/controllers/kind.rs — serde derives.
 - src/environment/spawner.rs — insert Replicated server-side; keep visual bundle client-side (split via the "decorate replicated plane" client system).
 - src/ui/menu.rs — AppState expansion, Start New Server / Connect to Server, child-process launch, connect/disconnect lifecycle.
 - src/ui/hud.rs, src/ui/lifecycle_panel.rs, src/ui/time_control.rs — send commands instead of mutating local components.
 - src/ui/time_control.rs / shared — move SimSpeed to a non-visual location.
 - src/plane/plugin.rs, src/environment/plugin.rs — gate the FixedUpdate sim chain + physics-interp so they run server-side / render-side respectively.

 Reused as-is (server side)
 - src/environment/scenario_spawn.rs::spawn_resolved_scenario, src/environment/spawner.rs::spawn_plane/load_spawn_config, src/scenario.rs (resolve/build_controller),
 src/environment/lifecycle.rs observers, src/aerodynamics/*, all controllers/* control laws.

 ---
 Verification

 - Unit/integration tests: cargo test --no-default-features (core, unchanged) and
 cargo test --features server (serde round-trips, relocated sim-control rebuilds,
 server boot/spawn/replication, in-process replicon client↔server command + state
 round-trip, interpolation math).
 - Manual end-to-end (the real proof):
   a. cargo run --features server --bin ml_planes_server -- --scenario assets/scenarios/default.scenario.ron
   b. cargo run (client) → Connect to Server → 127.0.0.1 → confirm planes fly,
 interpolated; HUD/map/camera live.
   c. From the client, switch a controller, manually fly a Manual plane, spawn/remove a
 plane, and change time-acceleration; confirm each takes effect server-side.
   d. Launch a second client, Connect to Server, confirm it sees the same world and
 can also issue commands (shared control).
   e. Single-button path: a fresh client → Start New Server → pick default →
 auto-launches a local server and connects; returning to the main menu disconnects and
 kills the child server.
 - Regression: examples/observe_state.rs, train_ppo/evaluate_policy, and the
 headless test harness still build/run (they don't touch net).

 ---
 Out of Scope (deferred)

 - Client-side prediction / reconciliation — interpolation only, per request.
 - Browser/WASM clients — needs a WebSocket/WebTransport replicon backend; the wasm
 feature keeps building the renderer but without networking for now.
 - Per-client plane ownership / auth — shared god-mode control for all clients.
 - Snapshot compression / delta-encoding tuning, lag compensation, NAT traversal —
 start with replicon defaults over LAN/localhost.
 - Server discovery / lobby browser — manual host:port entry.
