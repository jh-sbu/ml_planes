//! The `rmcp` MCP server handler — **all `rmcp` types are quarantined to this file**.
//!
//! Keeping every `rmcp` macro / type behind this single module means a future SDK bump
//! (rmcp has broken every minor: 0.1→0.2→0.3→1.x→2.0) has a one-file blast radius; the
//! shared snapshot + command-bridge primitives added in later phases stay plain Rust.
//!
//! Phase 2 surface: the read tools `get_sim_status`, `list_planes`, and `get_plane_state`,
//! backed by the shared [`SnapshotHandle`] the Bevy thread rewrites each frame. Each tool
//! takes a `read()` lock, clones the [`SimSnapshot`], releases, then shapes JSON via the pure
//! `*_value` helpers below (unit-testable without rmcp or sockets). Write tools (Phases 3–4)
//! will add a `ControlSender` field alongside `snapshot`.

use rmcp::handler::server::wrapper::Parameters;
use rmcp::model::{CallToolResult, ContentBlock};
use rmcp::serde_json::{json, to_value, Value};
use rmcp::{schemars, tool, tool_handler, tool_router, ServerHandler};

use crate::mcp::snapshot::{PlaneSnapshot, SimSnapshot, SnapshotHandle};

/// MCP server handler for the live `ml_planes` simulation.
///
/// Holds a clone of the shared [`SnapshotHandle`]; the Bevy thread rewrites the same `Arc`
/// each frame. Later phases add the command channel for the write path.
#[derive(Clone)]
pub struct PlanesService {
    snapshot: SnapshotHandle,
}

impl PlanesService {
    /// Build a service backed by the shared snapshot mirror.
    pub fn new(snapshot: SnapshotHandle) -> Self {
        Self { snapshot }
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
}

/// Typed arguments for [`PlanesService::get_plane_state`].
#[derive(serde::Deserialize, schemars::JsonSchema)]
struct GetPlaneStateArgs {
    /// The `PlaneId(u32)` of the plane to inspect (as reported by `list_planes`).
    plane_id: u32,
}

/// A tool-level error the caller's MCP client renders (vs a protocol `Err(ErrorData)`, which
/// clients show opaquely). Right for "the tool ran but produced no useful result".
fn tool_error(message: impl Into<String>) -> CallToolResult {
    CallToolResult::error(vec![ContentBlock::text(message)])
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
}

#[tool_handler(
    name = "ml_planes_mcp",
    instructions = "Inspect a running ml_planes flight simulation. Read tools: \
                    `get_sim_status` (connection + plane count), `list_planes` (roster), and \
                    `get_plane_state { plane_id }` (full per-plane state). State tracks the \
                    live sim and updates across calls. Mutating tools (spawn/remove/switch \
                    controller/…) arrive in a later phase."
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

    fn service_with(snap: SimSnapshot) -> PlanesService {
        let handle = SnapshotHandle::new();
        *handle.0.write().unwrap() = snap;
        PlanesService::new(handle)
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
}
