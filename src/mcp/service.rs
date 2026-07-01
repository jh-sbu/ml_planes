//! The `rmcp` MCP server handler — **all `rmcp` types are quarantined to this file**.
//!
//! Keeping every `rmcp` macro / type behind this single module means a future SDK bump
//! (rmcp has broken every minor: 0.1→0.2→0.3→1.x→2.0) has a one-file blast radius; the
//! shared snapshot + command-bridge primitives added in later phases stay plain Rust.
//!
//! Read tools (`get_sim_status`, `list_planes`, `get_plane_state`) are backed by the shared
//! [`SnapshotHandle`] the Bevy thread rewrites each frame: each takes a `read()` lock, clones
//! the [`SimSnapshot`], releases, then shapes JSON via the pure `*_value` helpers below
//! (unit-testable without rmcp or sockets). Write tools (Phase 3: `spawn_plane`, `remove_plane`)
//! enqueue a [`ControlRequest`] on the [`ControlSender`] the Bevy thread drains; they guard on
//! `connected`, then poll the snapshot to confirm the eventually-consistent result.

use std::collections::HashSet;
use std::time::{Duration, Instant};

use bevy::math::{Quat, Vec3};
use rmcp::handler::server::wrapper::Parameters;
use rmcp::model::{CallToolResult, ContentBlock};
use rmcp::serde_json::{json, to_value, Value};
use rmcp::{schemars, tool, tool_handler, tool_router, ServerHandler};

use crate::controllers::ControllerKind;
use crate::mcp::bridge::{
    parse_sim_speed, parse_spawnable_controller_kind, ControlRequest, ControlSender,
};
use crate::mcp::snapshot::{PlaneSnapshot, SimSnapshot, SnapshotHandle};
use crate::plane::PlaneId;
use crate::training::SpawnSpec;

/// Deadline for spawn/remove confirmation polling — the round-trip through the server
/// (command → apply → replicate back) is eventually consistent, so tools poll the snapshot
/// this long before giving up with `{ status: "timeout" }`.
const CONFIRM_DEADLINE: Duration = Duration::from_secs(1);
/// Interval between confirmation-poll snapshot reads.
const CONFIRM_INTERVAL: Duration = Duration::from_millis(50);

/// MCP server handler for the live `ml_planes` simulation.
///
/// Holds a clone of the shared [`SnapshotHandle`] (read path; the Bevy thread rewrites the same
/// `Arc` each frame) and a [`ControlSender`] (write path; the Bevy thread drains it into
/// `client_trigger` commands).
#[derive(Clone)]
pub struct PlanesService {
    snapshot: SnapshotHandle,
    control: ControlSender,
}

impl PlanesService {
    /// Build a service backed by the shared snapshot mirror and the command channel.
    pub fn new(snapshot: SnapshotHandle, control: ControlSender) -> Self {
        Self { snapshot, control }
    }

    /// Clone the latest [`SimSnapshot`] out of the shared lock, holding it only briefly.
    /// A poisoned lock degrades to a default (disconnected, empty) snapshot rather than
    /// panicking inside a tool handler.
    fn read_snapshot(&self) -> SimSnapshot {
        match self.snapshot.0.read() {
            Ok(guard) => guard.clone(),
            Err(poisoned) => (**poisoned.get_ref()).clone(),
        }
    }

    /// Live plane ids in the latest snapshot.
    fn plane_ids(&self) -> HashSet<u32> {
        self.read_snapshot()
            .planes
            .iter()
            .map(|p| p.plane_id)
            .collect()
    }

    /// Push a [`ControlRequest`] onto the command channel; a closed channel (Bevy thread gone)
    /// degrades to a tool error rather than a panic.
    fn send(&self, req: ControlRequest) -> Result<(), CallToolResult> {
        self.control
            .0
            .send(req)
            .map_err(|e| tool_error(format!("command channel closed: {e}")))
    }

    /// Guard + parse + enqueue a spawn, returning the pre-spawn id set for confirmation polling.
    /// Synchronous (no polling) so the send path is unit-testable without the 1 s confirm wait.
    fn dispatch_spawn(&self, args: &SpawnPlaneArgs) -> Result<HashSet<u32>, CallToolResult> {
        let snap = self.read_snapshot();
        if !snap.connected {
            return Err(tool_error(
                "not connected to ml_planes_server; retry shortly",
            ));
        }
        let (config_path, kind, spec) = spawn_spec_from_args(args).map_err(tool_error)?;
        let before: HashSet<u32> = snap.planes.iter().map(|p| p.plane_id).collect();
        self.send(ControlRequest::Spawn {
            config_path,
            kind,
            spec,
        })?;
        Ok(before)
    }

    /// Guard: the client is connected. Non-plane-targeted writes (e.g. `set_sim_speed`) use this.
    fn require_connected(&self) -> Result<(), CallToolResult> {
        if self.read_snapshot().connected {
            Ok(())
        } else {
            Err(tool_error(
                "not connected to ml_planes_server; retry shortly",
            ))
        }
    }

    /// Guard: connected **and** a plane with `plane_id` is live. Shared by every plane-targeted
    /// write dispatcher so they all reject an unknown id (and a disconnected client) uniformly.
    fn require_live_plane(&self, plane_id: u32) -> Result<(), CallToolResult> {
        let snap = self.read_snapshot();
        if !snap.connected {
            return Err(tool_error(
                "not connected to ml_planes_server; retry shortly",
            ));
        }
        if !snap.planes.iter().any(|p| p.plane_id == plane_id) {
            return Err(tool_error(format!("no plane with id {plane_id}")));
        }
        Ok(())
    }

    /// Guard + existence-check + enqueue a remove. Synchronous, like [`Self::dispatch_spawn`].
    fn dispatch_remove(&self, plane_id: u32) -> Result<(), CallToolResult> {
        self.require_live_plane(plane_id)?;
        self.send(ControlRequest::Remove {
            plane: PlaneId(plane_id),
        })
    }

    /// Guard + parse + enqueue a controller switch for a live plane.
    fn dispatch_switch_controller(
        &self,
        plane_id: u32,
        controller_kind: &str,
    ) -> Result<(), CallToolResult> {
        self.require_live_plane(plane_id)?;
        let kind = parse_spawnable_controller_kind(controller_kind).map_err(tool_error)?;
        self.send(ControlRequest::SwitchController {
            plane: PlaneId(plane_id),
            kind,
        })
    }

    /// Guard + enqueue a tuning-profile selection for a live plane. The profile is a free-form
    /// string — the server overwrites any existing profile and rebuilds the controller.
    fn dispatch_set_tuning_profile(
        &self,
        plane_id: u32,
        profile: String,
    ) -> Result<(), CallToolResult> {
        self.require_live_plane(plane_id)?;
        self.send(ControlRequest::SetTuningProfile {
            plane: PlaneId(plane_id),
            profile,
        })
    }

    /// Guard + parse + enqueue a global sim-speed change (not plane-targeted).
    fn dispatch_set_sim_speed(&self, speed: &str) -> Result<(), CallToolResult> {
        self.require_connected()?;
        let speed = parse_sim_speed(speed).map_err(tool_error)?;
        self.send(ControlRequest::SetSimSpeed { speed })
    }

    /// Guard + enqueue an RL model selection for a live plane. `model_stem` is a full path stem
    /// rooted at `models/` (see the tool doc); the server silently drops a stem that does not
    /// match the plane's controller kind.
    #[cfg(feature = "inference")]
    fn dispatch_set_model(&self, plane_id: u32, model_stem: String) -> Result<(), CallToolResult> {
        self.require_live_plane(plane_id)?;
        self.send(ControlRequest::SetModel {
            plane: PlaneId(plane_id),
            model_stem,
        })
    }

    /// Poll the snapshot until a new plane id appears (vs `before`) or the deadline elapses.
    async fn await_new_plane(&self, before: HashSet<u32>) -> CallToolResult {
        let deadline = Instant::now() + CONFIRM_DEADLINE;
        loop {
            tokio::time::sleep(CONFIRM_INTERVAL).await;
            if let Some(new_id) = newest_added(&before, &self.plane_ids()) {
                return CallToolResult::structured(
                    json!({ "status": "confirmed", "plane_id": new_id }),
                );
            }
            if Instant::now() >= deadline {
                return CallToolResult::structured(json!({ "status": "timeout" }));
            }
        }
    }

    /// Poll the snapshot until `plane_id` disappears or the deadline elapses.
    async fn await_removed_plane(&self, plane_id: u32) -> CallToolResult {
        let deadline = Instant::now() + CONFIRM_DEADLINE;
        loop {
            tokio::time::sleep(CONFIRM_INTERVAL).await;
            if !self.plane_ids().contains(&plane_id) {
                return CallToolResult::structured(
                    json!({ "status": "confirmed", "plane_id": plane_id }),
                );
            }
            if Instant::now() >= deadline {
                return CallToolResult::structured(json!({ "status": "timeout" }));
            }
        }
    }
}

/// The highest plane id present in `after` but not `before` — the freshly-spawned plane (with a
/// documented race if several spawns are in flight at once). Pure confirmation seam.
fn newest_added(before: &HashSet<u32>, after: &HashSet<u32>) -> Option<u32> {
    after.difference(before).copied().max()
}

/// Typed arguments for [`PlanesService::get_plane_state`].
#[derive(serde::Deserialize, schemars::JsonSchema)]
struct GetPlaneStateArgs {
    /// The `PlaneId(u32)` of the plane to inspect (as reported by `list_planes`).
    plane_id: u32,
}

/// Typed arguments for [`PlanesService::spawn_plane`].
///
/// `controller_kind` is a serde variant name of a *spawnable* `ControllerKind` (`"Manual"`,
/// `"LevelHold"`, `"HeadingHold"`, `"Ascent"`, `"Orbit"`, and — on inference builds —
/// `"RlLevelHold"`/`"RlOrbit"`/`"RlOrbitResidual"`/`"RlLstmOrbit"`). Spatial fields are optional;
/// omitting one uses the server's spawn default.
#[derive(serde::Deserialize, schemars::JsonSchema)]
struct SpawnPlaneArgs {
    /// Asset-relative `.plane.ron` path, e.g. `"planes/generic_jet.plane.ron"`.
    config_path: String,
    /// Serde variant name of the controller kind to spawn with.
    controller_kind: String,
    /// World-frame position `[x, y, z]` (m). `None` ⇒ spawn default.
    position: Option<[f32; 3]>,
    /// World-frame velocity `[x, y, z]` (m/s). `None` ⇒ spawn default.
    velocity: Option<[f32; 3]>,
    /// Attitude quaternion `[x, y, z, w]`. `None` ⇒ spawn default.
    attitude: Option<[f32; 4]>,
    /// Body-frame angular velocity `[x, y, z]` (rad/s). `None` ⇒ spawn default.
    angular_velocity: Option<[f32; 3]>,
    /// Powerplant fill fraction in `[0, 1]`. `None` ⇒ full tanks.
    fuel_fraction: Option<f32>,
}

/// Typed arguments for [`PlanesService::remove_plane`].
#[derive(serde::Deserialize, schemars::JsonSchema)]
struct RemovePlaneArgs {
    /// The `PlaneId(u32)` of the plane to remove (as reported by `list_planes`).
    plane_id: u32,
}

/// Typed arguments for [`PlanesService::switch_controller`].
#[derive(serde::Deserialize, schemars::JsonSchema)]
struct SwitchControllerArgs {
    /// The `PlaneId(u32)` of the plane whose controller to switch.
    plane_id: u32,
    /// Serde variant name of the controller kind to switch to (same allow-list as `spawn_plane`).
    controller_kind: String,
}

/// Typed arguments for [`PlanesService::set_tuning_profile`].
#[derive(serde::Deserialize, schemars::JsonSchema)]
struct SetTuningProfileArgs {
    /// The `PlaneId(u32)` of the plane to retune.
    plane_id: u32,
    /// The tuning-profile name to select (free-form; must exist in the plane's `.tuning.ron`).
    profile: String,
}

/// Typed arguments for [`PlanesService::set_sim_speed`].
#[derive(serde::Deserialize, schemars::JsonSchema)]
struct SetSimSpeedArgs {
    /// Serde variant name of the playback speed: `"Paused"`, `"X1"`, `"X5"`, or `"X10"`.
    speed: String,
}

/// Typed arguments for [`PlanesService::set_model`].
#[cfg(feature = "inference")]
#[derive(serde::Deserialize, schemars::JsonSchema)]
struct SetModelArgs {
    /// The `PlaneId(u32)` of the plane (must be running an RL controller kind).
    plane_id: u32,
    /// Full path stem rooted at `models/`, e.g. `"models/orbit/ppo_orbit_1"` (no `.mpk`).
    model_path_stem: String,
}

/// Pure mapping from `spawn_plane`'s JSON args to a [`ControlRequest::Spawn`] payload. The
/// TDD seam for spawn: array→`Vec3`/`Quat` conversion and controller-kind validation, no rmcp
/// or sockets. Fixed-size arrays make the JSON schema enforce element counts, so the only
/// failure path here is an unknown / non-spawnable `controller_kind`.
fn spawn_spec_from_args(
    args: &SpawnPlaneArgs,
) -> Result<(String, ControllerKind, SpawnSpec), String> {
    let kind = parse_spawnable_controller_kind(&args.controller_kind)?;
    let spec = SpawnSpec {
        position: args.position.map(Vec3::from_array),
        velocity: args.velocity.map(Vec3::from_array),
        attitude: args.attitude.map(Quat::from_array),
        angular_velocity: args.angular_velocity.map(Vec3::from_array),
        fuel_fraction: args.fuel_fraction,
    };
    Ok((args.config_path.clone(), kind, spec))
}

/// A tool-level error the caller's MCP client renders (vs a protocol `Err(ErrorData)`, which
/// clients show opaquely). Right for "the tool ran but produced no useful result".
fn tool_error(message: impl Into<String>) -> CallToolResult {
    CallToolResult::error(vec![ContentBlock::text(message)])
}

/// The immediate `{ status: "sent" }` result for fire-and-forget writes (no confirmation poll).
/// The change is eventually consistent — the agent re-reads `get_plane_state` / `get_sim_status`
/// to observe it applied.
fn sent() -> CallToolResult {
    CallToolResult::structured(json!({ "status": "sent" }))
}

/// Connection-status object: `connected` + the net identity + plane count + the MCP's
/// last-*requested* sim speed. `speed_authority` is always `"requested"` — `SimSpeed` is a
/// server-side resource and is not replicated, so this is never the server's true speed.
fn sim_status_value(snap: &SimSnapshot) -> Value {
    json!({
        "connected": snap.connected,
        "server_addr": snap.server_addr,
        "protocol_id": snap.protocol_id,
        "plane_count": snap.planes.len(),
        "requested_sim_speed": to_value(snap.requested_sim_speed).unwrap_or(Value::Null),
        "speed_authority": "requested",
    })
}

/// Compact per-plane summary for `list_planes`. `position` is shaped explicitly as `[x, y, z]`
/// (independent of glam's serde representation); `fuel_remaining` mirrors `consumable_remaining`.
fn plane_summary_value(p: &PlaneSnapshot) -> Value {
    json!({
        "plane_id": p.plane_id,
        "plane_index": p.plane_index,
        "controller_kind": to_value(p.controller_kind).unwrap_or(Value::Null),
        "position": [p.position.x, p.position.y, p.position.z],
        "altitude": p.altitude,
        "airspeed": p.airspeed,
        "fuel_remaining": p.consumable_remaining,
    })
}

/// JSON array of per-plane summaries.
fn list_planes_value(snap: &SimSnapshot) -> Value {
    Value::Array(snap.planes.iter().map(plane_summary_value).collect())
}

/// Full state of one plane, or `None` if no plane carries that id. The full [`PlaneSnapshot`]
/// serializes directly (position/velocity/attitude, alpha/beta, control inputs, telemetry, …).
fn plane_state_value(snap: &SimSnapshot, plane_id: u32) -> Option<Value> {
    snap.planes
        .iter()
        .find(|p| p.plane_id == plane_id)
        .map(|p| to_value(p).unwrap_or(Value::Null))
}

#[tool_router]
impl PlanesService {
    /// Liveness probe — confirms the MCP stdio handshake works end to end.
    #[tool(description = "Liveness check; returns \"pong\".")]
    async fn ping(&self) -> String {
        "pong".to_string()
    }

    /// Connection + high-level status.
    #[tool(
        description = "Report MCP↔server connection status: { connected, server_addr, \
                       protocol_id, plane_count, requested_sim_speed, speed_authority }. \
                       `speed_authority` is always \"requested\" — SimSpeed is not replicated, \
                       so this echoes the MCP's last set_sim_speed value (null if unset), never \
                       the server's true playback speed."
    )]
    async fn get_sim_status(&self) -> CallToolResult {
        CallToolResult::structured(sim_status_value(&self.read_snapshot()))
    }

    /// Compact roster of live planes.
    #[tool(
        description = "List live planes as [{ plane_id, plane_index, controller_kind, \
                       position:[x,y,z], altitude, airspeed, fuel_remaining }]. Returns a \
                       not-connected error (distinct from an empty list) while the client is \
                       not yet connected to the server."
    )]
    async fn list_planes(&self) -> CallToolResult {
        let snap = self.read_snapshot();
        if !snap.connected {
            return tool_error("not connected to ml_planes_server; retry shortly");
        }
        CallToolResult::structured(list_planes_value(&snap))
    }

    /// Full per-plane state.
    #[tool(
        description = "Full state of one plane by id: position/velocity/attitude(quat)/\
                       angular_velocity, alpha, beta, airspeed, altitude, \
                       consumable_remaining, control_inputs, controller_kind, tuning_profile, \
                       telemetry (and model on inference builds). Errors if not connected or \
                       if no plane carries that id."
    )]
    async fn get_plane_state(
        &self,
        Parameters(args): Parameters<GetPlaneStateArgs>,
    ) -> CallToolResult {
        let snap = self.read_snapshot();
        if !snap.connected {
            return tool_error("not connected to ml_planes_server; retry shortly");
        }
        match plane_state_value(&snap, args.plane_id) {
            Some(value) => CallToolResult::structured(value),
            None => tool_error(format!("no plane with id {}", args.plane_id)),
        }
    }

    /// Spawn a new plane on the server.
    #[tool(
        description = "Spawn a plane on the server: { config_path, controller_kind, position?, \
                       velocity?, attitude?, angular_velocity?, fuel_fraction? }. \
                       `config_path` is an asset-relative `.plane.ron` (e.g. \
                       \"planes/generic_jet.plane.ron\"); `controller_kind` is a serde variant \
                       name — one of Manual, LevelHold, HeadingHold, Ascent, Orbit (and, on \
                       inference builds, RlLevelHold, RlOrbit, RlOrbitResidual, RlLstmOrbit). \
                       Wingman and FlightPlan are rejected. position/velocity/angular_velocity \
                       are [x,y,z]; attitude is a quaternion [x,y,z,w]; omitted fields use spawn \
                       defaults. Asynchronous: polls up to ~1s and returns { status: \
                       \"confirmed\", plane_id } (the new id) or { status: \"timeout\" }."
    )]
    async fn spawn_plane(&self, Parameters(args): Parameters<SpawnPlaneArgs>) -> CallToolResult {
        match self.dispatch_spawn(&args) {
            Ok(before) => self.await_new_plane(before).await,
            Err(err) => err,
        }
    }

    /// Remove a plane from the server by id.
    #[tool(
        description = "Remove a plane by id: { plane_id }. Errors if not connected or if no \
                       plane carries that id. Asynchronous: polls up to ~1s and returns \
                       { status: \"confirmed\", plane_id } once the plane is gone, or \
                       { status: \"timeout\" }."
    )]
    async fn remove_plane(&self, Parameters(args): Parameters<RemovePlaneArgs>) -> CallToolResult {
        match self.dispatch_remove(args.plane_id) {
            Ok(()) => self.await_removed_plane(args.plane_id).await,
            Err(err) => err,
        }
    }

    /// Switch a plane's active controller.
    #[tool(
        description = "Switch a plane's active controller: { plane_id, controller_kind }. \
                       `controller_kind` is a serde variant name — one of Manual, LevelHold, \
                       HeadingHold, Ascent, Orbit (and, on inference builds, RlLevelHold, \
                       RlOrbit, RlOrbitResidual, RlLstmOrbit); RL kinds only take effect against \
                       an inference-capable server (PID fallback otherwise). Wingman and \
                       FlightPlan are rejected (the generic builder would substitute a different \
                       controller). Errors if not connected or if no plane carries that id. \
                       Fire-and-forget: returns { status: \"sent\" }; re-read get_plane_state to \
                       observe the applied controller_kind (eventually consistent)."
    )]
    async fn switch_controller(
        &self,
        Parameters(args): Parameters<SwitchControllerArgs>,
    ) -> CallToolResult {
        match self.dispatch_switch_controller(args.plane_id, &args.controller_kind) {
            Ok(()) => sent(),
            Err(err) => err,
        }
    }

    /// Select a named PID tuning profile for a plane.
    #[tool(
        description = "Select a plane's PID tuning profile: { plane_id, profile }. `profile` is \
                       a free-form name that must exist in the plane's `.tuning.ron` gain pool; \
                       the server overwrites any current profile and rebuilds the controller. \
                       Errors if not connected or if no plane carries that id. Fire-and-forget: \
                       returns { status: \"sent\" }; re-read get_plane_state to observe the \
                       applied tuning_profile (eventually consistent)."
    )]
    async fn set_tuning_profile(
        &self,
        Parameters(args): Parameters<SetTuningProfileArgs>,
    ) -> CallToolResult {
        match self.dispatch_set_tuning_profile(args.plane_id, args.profile) {
            Ok(()) => sent(),
            Err(err) => err,
        }
    }

    /// Set the global simulation playback speed.
    #[tool(
        description = "Set the global simulation playback speed: { speed }. `speed` is a serde \
                       variant name — one of Paused, X1, X5, X10 (not the display labels like \
                       \"1x\"). Pause/accel is server-authoritative and affects the whole sim. \
                       Errors if not connected. Fire-and-forget: returns { status: \"sent\" }; \
                       get_sim_status echoes it back as requested_sim_speed (SimSpeed is not \
                       replicated, so this is the MCP's last request, not the server's true \
                       speed)."
    )]
    async fn set_sim_speed(&self, Parameters(args): Parameters<SetSimSpeedArgs>) -> CallToolResult {
        match self.dispatch_set_sim_speed(&args.speed) {
            Ok(()) => sent(),
            Err(err) => err,
        }
    }
}

/// The `set_model` tool lives in its own `inference`-gated router: rmcp 2.0's `#[tool_router]`
/// cannot `#[cfg]` a *single* tool (it emits an unconditional `with_route` for every `#[tool]`
/// method it sees, so a stripped method leaves a dangling reference). Gating the whole router
/// block instead and merging it in [`PlanesService::all_tools`] keeps set_model out of the tool
/// list on a non-inference build.
#[cfg(feature = "inference")]
#[tool_router(router = inference_tool_router)]
impl PlanesService {
    /// Select a trained RL model for a plane running an RL controller.
    #[tool(
        description = "Select a trained RL model for a plane: { plane_id, model_path_stem }. \
                       `model_path_stem` is a full path stem rooted at `models/`, e.g. \
                       \"models/orbit/ppo_orbit_1\" (no `.mpk` extension). The plane must be \
                       running the matching RL controller kind — the server silently drops a \
                       stem that does not match `models/<dir>/` for that kind. Errors if not \
                       connected or if no plane carries that id. Fire-and-forget: returns \
                       { status: \"sent\" }; re-read get_plane_state to observe the applied \
                       model (eventually consistent)."
    )]
    async fn set_model(&self, Parameters(args): Parameters<SetModelArgs>) -> CallToolResult {
        match self.dispatch_set_model(args.plane_id, args.model_path_stem) {
            Ok(()) => sent(),
            Err(err) => err,
        }
    }
}

impl PlanesService {
    /// The full tool router. The base `tool_router()` (all always-available tools) is merged
    /// with the `inference`-gated `inference_tool_router()` (just `set_model`) — see the note on
    /// the inference router block for why set_model must live in its own router.
    fn all_tools() -> rmcp::handler::server::router::tool::ToolRouter<Self> {
        #[allow(unused_mut)]
        let mut router = Self::tool_router();
        #[cfg(feature = "inference")]
        {
            router += Self::inference_tool_router();
        }
        router
    }
}

#[tool_handler(
    router = Self::all_tools(),
    name = "ml_planes_mcp",
    instructions = "Inspect and control a running ml_planes flight simulation. Read tools: \
                    `get_sim_status` (connection + plane count), `list_planes` (roster), and \
                    `get_plane_state { plane_id }` (full per-plane state). Write tools: \
                    `spawn_plane` (add a plane), `remove_plane { plane_id }`, \
                    `switch_controller { plane_id, controller_kind }`, \
                    `set_tuning_profile { plane_id, profile }`, `set_sim_speed { speed }`, and \
                    (on inference builds) `set_model { plane_id, model_path_stem }`. State \
                    tracks the live sim and updates across calls; writes are \
                    eventually-consistent round-trips through the server. spawn_plane/\
                    remove_plane poll briefly and report confirmed/timeout; the other writes \
                    return { status: \"sent\" } immediately — re-read get_plane_state / \
                    get_sim_status to observe the applied change."
)]
impl ServerHandler for PlanesService {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controllers::{ControllerKind, ControllerTelemetry, SelectedTuningProfile};
    use crate::mcp::snapshot::{build_snapshot, plane_snapshot};
    use crate::plane::{ControlInputs, FlightState, PlaneId, PlaneIndex};
    use crate::sim_speed::SimSpeed;
    use bevy::prelude::*;

    /// A synthetic plane with distinctive field values (mirrors `snapshot.rs::tests`).
    fn sample_plane() -> PlaneSnapshot {
        let state = FlightState {
            position: Vec3::new(1.0, 2.0, 3.0),
            velocity: Vec3::new(10.0, 0.0, -5.0),
            attitude: Quat::from_rotation_y(0.5),
            angular_velocity: Vec3::new(0.1, 0.2, 0.3),
            alpha: 0.04,
            beta: -0.01,
            airspeed: 123.0,
            altitude: 1500.0,
            consumable_remaining: 1800.0,
        };
        let inputs = ControlInputs {
            aileron: 0.1,
            elevator: -0.2,
            rudder: 0.05,
            throttle: 0.8,
        };
        let tuning = SelectedTuningProfile("aggressive".to_string());
        let telemetry = ControllerTelemetry::Orbit { radial_error: 12.5 };
        plane_snapshot(
            &PlaneId(7),
            &PlaneIndex(2),
            &state,
            &inputs,
            &ControllerKind::Orbit,
            Some(&tuning),
            Some(&telemetry),
            #[cfg(feature = "inference")]
            None,
        )
    }

    /// A connected snapshot carrying the sample plane.
    fn connected_snapshot() -> SimSnapshot {
        build_snapshot(
            vec![sample_plane()],
            true,
            "127.0.0.1:5555".to_string(),
            Some(SimSpeed::X5),
        )
    }

    /// Build a service over a fixed snapshot plus a fresh command channel, returning the
    /// service and the receiving end so tests can assert what a write tool enqueued.
    fn service_and_rx(snap: SimSnapshot) -> (PlanesService, crate::mcp::bridge::ControlReceiver) {
        let handle = SnapshotHandle::new();
        *handle.0.write().unwrap() = snap;
        let (tx, rx) = crate::mcp::bridge::control_channel();
        (PlanesService::new(handle, tx), rx)
    }

    fn service_with(snap: SimSnapshot) -> PlanesService {
        service_and_rx(snap).0
    }

    #[test]
    fn sim_status_value_reports_connection_and_requested_speed() {
        let value = sim_status_value(&connected_snapshot());
        assert_eq!(value["connected"], json!(true));
        assert_eq!(value["server_addr"], json!("127.0.0.1:5555"));
        assert_eq!(value["protocol_id"], json!(crate::net::PROTOCOL_ID));
        assert_eq!(value["plane_count"], json!(1));
        assert_eq!(
            value["requested_sim_speed"],
            to_value(SimSpeed::X5).unwrap()
        );
        assert_eq!(value["speed_authority"], json!("requested"));
        // The client cannot observe an authoritative sim clock — no `sim_time` field.
        assert!(value.get("sim_time").is_none());
    }

    #[test]
    fn sim_status_value_requested_speed_is_null_when_unset() {
        let snap = build_snapshot(vec![], false, "127.0.0.1:5555".to_string(), None);
        let value = sim_status_value(&snap);
        assert_eq!(value["connected"], json!(false));
        assert_eq!(value["plane_count"], json!(0));
        assert_eq!(value["requested_sim_speed"], Value::Null);
    }

    #[test]
    fn list_planes_value_shapes_each_summary() {
        let value = list_planes_value(&connected_snapshot());
        let arr = value.as_array().expect("array");
        assert_eq!(arr.len(), 1);
        let p = &arr[0];
        assert_eq!(p["plane_id"], json!(7));
        assert_eq!(p["plane_index"], json!(2));
        assert_eq!(p["controller_kind"], json!("Orbit"));
        assert_eq!(p["position"], json!([1.0, 2.0, 3.0]));
        assert_eq!(p["altitude"], json!(1500.0));
        assert_eq!(p["airspeed"], json!(123.0));
        assert_eq!(p["fuel_remaining"], json!(1800.0));
    }

    #[test]
    fn plane_state_value_returns_full_state_for_known_id() {
        let snap = connected_snapshot();
        let value = plane_state_value(&snap, 7).expect("known id resolves");
        assert_eq!(value["plane_id"], json!(7));
        assert_eq!(value["controller_kind"], json!("Orbit"));
        assert!(value.get("beta").is_some());
        assert!(value.get("alpha").is_some());
        assert!(value.get("velocity").is_some());
        assert!(value.get("attitude").is_some());
        assert!(value.get("angular_velocity").is_some());
        assert!(value.get("control_inputs").is_some());
        assert!(value.get("telemetry").is_some());
    }

    #[test]
    fn plane_state_value_is_none_for_unknown_id() {
        assert!(plane_state_value(&connected_snapshot(), 9999).is_none());
    }

    #[tokio::test]
    async fn get_sim_status_tool_returns_structured_status() {
        let service = service_with(connected_snapshot());
        let result = service.get_sim_status().await;
        assert_eq!(result.is_error, Some(false));
        assert_eq!(
            result.structured_content.expect("structured")["plane_count"],
            json!(1)
        );
    }

    #[tokio::test]
    async fn list_planes_tool_errors_while_disconnected() {
        let snap = build_snapshot(vec![], false, "127.0.0.1:5555".to_string(), None);
        let result = service_with(snap).list_planes().await;
        assert_eq!(result.is_error, Some(true));
    }

    #[tokio::test]
    async fn list_planes_tool_returns_roster_when_connected() {
        let result = service_with(connected_snapshot()).list_planes().await;
        assert_eq!(result.is_error, Some(false));
        let planes = result.structured_content.expect("structured");
        assert_eq!(planes.as_array().expect("array").len(), 1);
    }

    #[tokio::test]
    async fn get_plane_state_tool_resolves_known_id() {
        let service = service_with(connected_snapshot());
        let result = service
            .get_plane_state(Parameters(GetPlaneStateArgs { plane_id: 7 }))
            .await;
        assert_eq!(result.is_error, Some(false));
        assert_eq!(
            result.structured_content.expect("structured")["plane_id"],
            json!(7)
        );
    }

    #[tokio::test]
    async fn get_plane_state_tool_errors_on_unknown_id() {
        let service = service_with(connected_snapshot());
        let result = service
            .get_plane_state(Parameters(GetPlaneStateArgs { plane_id: 9999 }))
            .await;
        assert_eq!(result.is_error, Some(true));
    }

    // ---- Phase 3: write path ----

    fn spawn_args(kind: &str) -> SpawnPlaneArgs {
        SpawnPlaneArgs {
            config_path: "planes/generic_jet.plane.ron".to_string(),
            controller_kind: kind.to_string(),
            position: Some([100.0, 1500.0, -50.0]),
            velocity: Some([120.0, 0.0, 0.0]),
            attitude: None,
            angular_velocity: None,
            fuel_fraction: Some(0.5),
        }
    }

    #[test]
    fn spawn_spec_from_args_maps_arrays_and_kind() {
        let (config, kind, spec) = spawn_spec_from_args(&spawn_args("LevelHold")).unwrap();
        assert_eq!(config, "planes/generic_jet.plane.ron");
        assert_eq!(kind, ControllerKind::LevelHold);
        assert_eq!(spec.position, Some(Vec3::new(100.0, 1500.0, -50.0)));
        assert_eq!(spec.velocity, Some(Vec3::new(120.0, 0.0, 0.0)));
        assert_eq!(spec.attitude, None);
        assert_eq!(spec.angular_velocity, None);
        assert_eq!(spec.fuel_fraction, Some(0.5));
    }

    #[test]
    fn spawn_spec_from_args_maps_attitude_quat() {
        let mut args = spawn_args("Orbit");
        args.attitude = Some([0.0, 0.0, 0.0, 1.0]);
        let (_, _, spec) = spawn_spec_from_args(&args).unwrap();
        assert_eq!(spec.attitude, Some(Quat::from_array([0.0, 0.0, 0.0, 1.0])));
    }

    #[test]
    fn spawn_spec_from_args_rejects_bad_kind() {
        assert!(spawn_spec_from_args(&spawn_args("Wingman")).is_err());
        assert!(spawn_spec_from_args(&spawn_args("Nonsense")).is_err());
    }

    #[test]
    fn newest_added_reports_highest_new_id() {
        let before: HashSet<u32> = [1, 2].into_iter().collect();
        let after: HashSet<u32> = [1, 2, 5, 7].into_iter().collect();
        assert_eq!(newest_added(&before, &after), Some(7));
        assert_eq!(newest_added(&after, &after), None);
    }

    #[test]
    fn dispatch_spawn_enqueues_spawn_request() {
        let (service, rx) = service_and_rx(connected_snapshot());
        let before = service.dispatch_spawn(&spawn_args("LevelHold")).unwrap();
        assert_eq!(before, [7].into_iter().collect::<HashSet<u32>>());
        match rx.0.try_recv().expect("a spawn request should be queued") {
            ControlRequest::Spawn {
                config_path, kind, ..
            } => {
                assert_eq!(config_path, "planes/generic_jet.plane.ron");
                assert_eq!(kind, ControllerKind::LevelHold);
            }
            other => panic!("unexpected request: {other:?}"),
        }
    }

    #[test]
    fn dispatch_spawn_errors_while_disconnected() {
        let snap = build_snapshot(vec![], false, "127.0.0.1:5555".to_string(), None);
        let (service, rx) = service_and_rx(snap);
        let err = service
            .dispatch_spawn(&spawn_args("LevelHold"))
            .unwrap_err();
        assert_eq!(err.is_error, Some(true));
        assert!(
            rx.0.try_recv().is_err(),
            "nothing enqueued when disconnected"
        );
    }

    #[test]
    fn dispatch_remove_enqueues_remove_for_known_id() {
        let (service, rx) = service_and_rx(connected_snapshot());
        service.dispatch_remove(7).unwrap();
        match rx.0.try_recv().expect("a remove request should be queued") {
            ControlRequest::Remove { plane } => assert_eq!(plane, PlaneId(7)),
            other => panic!("unexpected request: {other:?}"),
        }
    }

    #[test]
    fn dispatch_remove_errors_on_unknown_id() {
        let (service, rx) = service_and_rx(connected_snapshot());
        assert_eq!(
            service.dispatch_remove(9999).unwrap_err().is_error,
            Some(true)
        );
        assert!(
            rx.0.try_recv().is_err(),
            "nothing enqueued for a missing id"
        );
    }

    #[tokio::test]
    async fn spawn_plane_tool_errors_while_disconnected() {
        let snap = build_snapshot(vec![], false, "127.0.0.1:5555".to_string(), None);
        let result = service_with(snap)
            .spawn_plane(Parameters(spawn_args("LevelHold")))
            .await;
        assert_eq!(result.is_error, Some(true));
    }

    #[tokio::test]
    async fn await_new_plane_confirms_when_a_new_id_appears() {
        // The snapshot already carries plane 7; polling against an empty `before` sees it.
        let service = service_with(connected_snapshot());
        let result = service.await_new_plane(HashSet::new()).await;
        assert_eq!(result.is_error, Some(false));
        assert_eq!(
            result.structured_content.expect("structured")["plane_id"],
            json!(7)
        );
    }

    #[tokio::test]
    async fn await_removed_plane_confirms_when_id_is_gone() {
        // Empty snapshot ⇒ id 7 is already absent, so removal confirms immediately.
        let snap = build_snapshot(vec![], true, "127.0.0.1:5555".to_string(), None);
        let result = service_with(snap).await_removed_plane(7).await;
        assert_eq!(result.is_error, Some(false));
        assert_eq!(
            result.structured_content.expect("structured")["status"],
            json!("confirmed")
        );
    }

    // ---- Phase 4: controller / tuning / sim-speed / model write tools ----

    #[test]
    fn dispatch_switch_controller_enqueues_for_known_id() {
        let (service, rx) = service_and_rx(connected_snapshot());
        service.dispatch_switch_controller(7, "Ascent").unwrap();
        match rx.0.try_recv().expect("a switch request should be queued") {
            ControlRequest::SwitchController { plane, kind } => {
                assert_eq!(plane, PlaneId(7));
                assert_eq!(kind, ControllerKind::Ascent);
            }
            other => panic!("unexpected request: {other:?}"),
        }
    }

    #[test]
    fn dispatch_switch_controller_rejects_bad_kind() {
        let (service, rx) = service_and_rx(connected_snapshot());
        assert!(service.dispatch_switch_controller(7, "Wingman").is_err());
        assert!(service.dispatch_switch_controller(7, "Nonsense").is_err());
        assert!(rx.0.try_recv().is_err(), "nothing enqueued for a bad kind");
    }

    #[test]
    fn dispatch_switch_controller_errors_on_unknown_id() {
        let (service, rx) = service_and_rx(connected_snapshot());
        assert_eq!(
            service
                .dispatch_switch_controller(9999, "Orbit")
                .unwrap_err()
                .is_error,
            Some(true)
        );
        assert!(
            rx.0.try_recv().is_err(),
            "nothing enqueued for a missing id"
        );
    }

    #[test]
    fn dispatch_switch_controller_errors_while_disconnected() {
        let snap = build_snapshot(vec![], false, "127.0.0.1:5555".to_string(), None);
        let (service, rx) = service_and_rx(snap);
        assert!(service.dispatch_switch_controller(7, "Orbit").is_err());
        assert!(rx.0.try_recv().is_err());
    }

    #[test]
    fn dispatch_set_tuning_profile_enqueues_for_known_id() {
        let (service, rx) = service_and_rx(connected_snapshot());
        service
            .dispatch_set_tuning_profile(7, "aggressive".to_string())
            .unwrap();
        match rx.0.try_recv().expect("a tuning request should be queued") {
            ControlRequest::SetTuningProfile { plane, profile } => {
                assert_eq!(plane, PlaneId(7));
                assert_eq!(profile, "aggressive");
            }
            other => panic!("unexpected request: {other:?}"),
        }
    }

    #[test]
    fn dispatch_set_tuning_profile_errors_on_unknown_id() {
        let (service, rx) = service_and_rx(connected_snapshot());
        assert!(service
            .dispatch_set_tuning_profile(9999, "x".to_string())
            .is_err());
        assert!(rx.0.try_recv().is_err());
    }

    #[test]
    fn dispatch_set_sim_speed_enqueues_parsed_speed() {
        let (service, rx) = service_and_rx(connected_snapshot());
        service.dispatch_set_sim_speed("X10").unwrap();
        match rx
            .0
            .try_recv()
            .expect("a sim-speed request should be queued")
        {
            ControlRequest::SetSimSpeed { speed } => assert_eq!(speed, SimSpeed::X10),
            other => panic!("unexpected request: {other:?}"),
        }
    }

    #[test]
    fn dispatch_set_sim_speed_rejects_bad_name() {
        let (service, rx) = service_and_rx(connected_snapshot());
        assert!(service.dispatch_set_sim_speed("1x").is_err());
        assert!(rx.0.try_recv().is_err());
    }

    #[test]
    fn dispatch_set_sim_speed_errors_while_disconnected() {
        let snap = build_snapshot(vec![], false, "127.0.0.1:5555".to_string(), None);
        let (service, rx) = service_and_rx(snap);
        assert!(service.dispatch_set_sim_speed("X5").is_err());
        assert!(rx.0.try_recv().is_err());
    }

    #[cfg(feature = "inference")]
    #[test]
    fn dispatch_set_model_enqueues_for_known_id() {
        let (service, rx) = service_and_rx(connected_snapshot());
        service
            .dispatch_set_model(7, "models/orbit/ppo_orbit_1".to_string())
            .unwrap();
        match rx.0.try_recv().expect("a model request should be queued") {
            ControlRequest::SetModel { plane, model_stem } => {
                assert_eq!(plane, PlaneId(7));
                assert_eq!(model_stem, "models/orbit/ppo_orbit_1");
            }
            other => panic!("unexpected request: {other:?}"),
        }
    }

    #[cfg(feature = "inference")]
    #[test]
    fn dispatch_set_model_errors_on_unknown_id() {
        let (service, rx) = service_and_rx(connected_snapshot());
        assert!(service
            .dispatch_set_model(9999, "models/orbit/ppo_orbit_1".to_string())
            .is_err());
        assert!(rx.0.try_recv().is_err());
    }

    #[tokio::test]
    async fn switch_controller_tool_returns_sent_when_connected() {
        // Hold the receiver: dropping it closes the channel and `send` would fail.
        let (service, _rx) = service_and_rx(connected_snapshot());
        let result = service
            .switch_controller(Parameters(SwitchControllerArgs {
                plane_id: 7,
                controller_kind: "Orbit".to_string(),
            }))
            .await;
        assert_eq!(result.is_error, Some(false));
        assert_eq!(
            result.structured_content.expect("structured")["status"],
            json!("sent")
        );
    }

    #[tokio::test]
    async fn set_sim_speed_tool_errors_while_disconnected() {
        let snap = build_snapshot(vec![], false, "127.0.0.1:5555".to_string(), None);
        let result = service_with(snap)
            .set_sim_speed(Parameters(SetSimSpeedArgs {
                speed: "X5".to_string(),
            }))
            .await;
        assert_eq!(result.is_error, Some(true));
    }

    #[tokio::test]
    async fn set_tuning_profile_tool_returns_sent_when_connected() {
        let (service, _rx) = service_and_rx(connected_snapshot());
        let result = service
            .set_tuning_profile(Parameters(SetTuningProfileArgs {
                plane_id: 7,
                profile: "aggressive".to_string(),
            }))
            .await;
        assert_eq!(result.is_error, Some(false));
    }
}
