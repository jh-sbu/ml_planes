//! The shared, serde-friendly mirror of the replicated world (read path).
//!
//! [`collect_snapshot`] (a Bevy `Update` system) rebuilds a [`SimSnapshot`] every frame
//! from the replicated components the MCP client receives, writing it under the
//! [`SnapshotHandle`]'s write lock. Phase 2's read tools clone/read the same `Arc` from
//! the rmcp side. The snapshot owns its own `Serialize`/`Deserialize` derives and is built
//! by the pure [`build_snapshot`] helper, so the assembly logic is unit-testable without a
//! Bevy `App`, a socket, or rmcp.
//!
//! `requested_sim_speed` is the MCP's *last-requested* playback speed, not the server's
//! authoritative one: `SimSpeed` is a server-side resource and is **not** in the replicated
//! component set (`src/net/protocol.rs`), so the client cannot observe the true speed. The
//! `set_sim_speed` tool (Phase 4) writes [`RequestedSimSpeed`]; until then it stays `None`.

use std::sync::{Arc, RwLock};

use bevy::prelude::*;
use bevy_replicon::prelude::ClientState;
use serde::{Deserialize, Serialize};

use crate::controllers::{ControllerKind, ControllerTelemetry, SelectedTuningProfile};
use crate::net::{ConnectTarget, PROTOCOL_ID};
use crate::plane::{ControlInputs, FlightState, PlaneId, PlaneIndex};
use crate::sim_speed::SimSpeed;

#[cfg(feature = "inference")]
use crate::controllers::SelectedModel;

/// Plain, serde-friendly mirror of one plane's replicated state.
///
/// Embeds the rich domain types (`Vec3`/`Quat`/[`ControllerKind`]/[`ControllerTelemetry`]/
/// [`ControlInputs`]) directly — all are `Serialize`/`Deserialize` under the `net` feature
/// (which `mcp` requires), so no parallel mirror structs are needed. The exact JSON output
/// shaping (e.g. position as `[x, y, z]`) is a Phase 2 tool concern.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlaneSnapshot {
    pub plane_id: u32,
    pub plane_index: u32,
    pub controller_kind: ControllerKind,
    pub position: Vec3,
    pub velocity: Vec3,
    pub attitude: Quat,
    pub angular_velocity: Vec3,
    pub alpha: f32,
    pub beta: f32,
    pub airspeed: f32,
    pub altitude: f32,
    pub consumable_remaining: f32,
    pub control_inputs: ControlInputs,
    pub tuning_profile: Option<String>,
    pub telemetry: ControllerTelemetry,
    /// Selected RL model path stem, if any. Only present in `inference` builds (the
    /// `SelectedModel` component is `inference`-gated, like its replication).
    #[cfg(feature = "inference")]
    pub model: Option<String>,
}

/// Whole-simulation snapshot: connection status + the per-plane mirror.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct SimSnapshot {
    /// Whether the replicon client is currently `ClientState::Connected`.
    pub connected: bool,
    /// Address of the `ml_planes_server` this client targets.
    pub server_addr: String,
    /// The net `PROTOCOL_ID` this client speaks.
    pub protocol_id: u64,
    /// The MCP's last-requested sim speed (`None` until `set_sim_speed` is called). This is
    /// **not** authoritative — see the module docs.
    pub requested_sim_speed: Option<SimSpeed>,
    pub planes: Vec<PlaneSnapshot>,
}

/// Shared handle to the live [`SimSnapshot`]. The Bevy thread holds one clone (rewriting it
/// each frame); Phase 2 hands a second clone to the rmcp service (read-only).
#[derive(Resource, Clone)]
pub struct SnapshotHandle(pub Arc<RwLock<SimSnapshot>>);

impl SnapshotHandle {
    /// Fresh handle wrapping a default (disconnected, empty) snapshot.
    pub fn new() -> Self {
        Self(Arc::new(RwLock::new(SimSnapshot::default())))
    }
}

impl Default for SnapshotHandle {
    fn default() -> Self {
        Self::new()
    }
}

/// The MCP's last-requested playback speed, mirrored into [`SimSnapshot::requested_sim_speed`].
/// Written by the `set_sim_speed` tool (Phase 4); read by [`collect_snapshot`]. Stays `None`
/// in Phase 1.
#[derive(Resource, Default)]
pub struct RequestedSimSpeed(pub Option<SimSpeed>);

/// Map one plane's replicated components into a [`PlaneSnapshot`]. Pure — the system and the
/// unit tests both call it.
#[allow(clippy::too_many_arguments)]
pub fn plane_snapshot(
    id: &PlaneId,
    index: &PlaneIndex,
    state: &FlightState,
    inputs: &ControlInputs,
    kind: &ControllerKind,
    tuning: Option<&SelectedTuningProfile>,
    telemetry: Option<&ControllerTelemetry>,
    #[cfg(feature = "inference")] model: Option<&SelectedModel>,
) -> PlaneSnapshot {
    PlaneSnapshot {
        plane_id: id.0,
        plane_index: index.0,
        controller_kind: *kind,
        position: state.position,
        velocity: state.velocity,
        attitude: state.attitude,
        angular_velocity: state.angular_velocity,
        alpha: state.alpha,
        beta: state.beta,
        airspeed: state.airspeed,
        altitude: state.altitude,
        consumable_remaining: state.consumable_remaining,
        control_inputs: inputs.clone(),
        tuning_profile: tuning.map(|t| t.0.clone()),
        telemetry: telemetry.cloned().unwrap_or_default(),
        #[cfg(feature = "inference")]
        model: model.map(|m| m.0.clone()),
    }
}

/// Assemble a [`SimSnapshot`] from already-built per-plane snapshots and the scalar fields.
/// Pure (no Bevy, no sockets) — the unit-test seam.
pub fn build_snapshot(
    planes: Vec<PlaneSnapshot>,
    connected: bool,
    server_addr: String,
    requested_sim_speed: Option<SimSpeed>,
) -> SimSnapshot {
    SimSnapshot {
        connected,
        server_addr,
        protocol_id: PROTOCOL_ID,
        requested_sim_speed,
        planes,
    }
}

/// `Update` system: rebuild the shared [`SimSnapshot`] from the replicated world once per
/// frame. Holds the write lock only for the final write.
///
/// `SelectedModel` (inference only) lives on the same entity but is looked up via a separate
/// read-only query keyed on `Entity`, keeping the main query's tuple arity tractable.
#[allow(clippy::type_complexity)]
pub fn collect_snapshot(
    handle: Res<SnapshotHandle>,
    connect_target: Res<ConnectTarget>,
    client_state: Option<Res<State<ClientState>>>,
    requested: Res<RequestedSimSpeed>,
    planes: Query<(
        Entity,
        &PlaneId,
        &PlaneIndex,
        &FlightState,
        &ControlInputs,
        &ControllerKind,
        Option<&SelectedTuningProfile>,
        Option<&ControllerTelemetry>,
    )>,
    #[cfg(feature = "inference")] models: Query<&SelectedModel>,
) {
    let connected = client_state
        .map(|s| *s.get() == ClientState::Connected)
        .unwrap_or(false);

    let snapshots: Vec<PlaneSnapshot> = planes
        .iter()
        .map(
            |(_entity, id, index, state, inputs, kind, tuning, telemetry)| {
                plane_snapshot(
                    id,
                    index,
                    state,
                    inputs,
                    kind,
                    tuning,
                    telemetry,
                    #[cfg(feature = "inference")]
                    models.get(_entity).ok(),
                )
            },
        )
        .collect();

    let snapshot = build_snapshot(
        snapshots,
        connected,
        connect_target.0.to_string(),
        requested.0,
    );

    if let Ok(mut guard) = handle.0.write() {
        *guard = snapshot;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A synthetic plane with distinctive field values for assertion.
    fn sample_plane() -> PlaneSnapshot {
        let id = PlaneId(7);
        let index = PlaneIndex(2);
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
        let kind = ControllerKind::Orbit;
        let tuning = SelectedTuningProfile("aggressive".to_string());
        let telemetry = ControllerTelemetry::Orbit { radial_error: 12.5 };

        plane_snapshot(
            &id,
            &index,
            &state,
            &inputs,
            &kind,
            Some(&tuning),
            Some(&telemetry),
            #[cfg(feature = "inference")]
            None,
        )
    }

    #[test]
    fn plane_snapshot_maps_all_fields() {
        let snap = sample_plane();
        assert_eq!(snap.plane_id, 7);
        assert_eq!(snap.plane_index, 2);
        assert_eq!(snap.controller_kind, ControllerKind::Orbit);
        assert_eq!(snap.position, Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(snap.velocity, Vec3::new(10.0, 0.0, -5.0));
        assert_eq!(snap.airspeed, 123.0);
        assert_eq!(snap.altitude, 1500.0);
        assert_eq!(snap.consumable_remaining, 1800.0);
        assert_eq!(snap.control_inputs.throttle, 0.8);
        assert_eq!(snap.tuning_profile.as_deref(), Some("aggressive"));
        assert_eq!(
            snap.telemetry,
            ControllerTelemetry::Orbit { radial_error: 12.5 },
            "controller telemetry should carry through unchanged"
        );
    }

    #[test]
    fn plane_snapshot_defaults_telemetry_when_absent() {
        let id = PlaneId(0);
        let index = PlaneIndex(0);
        let state = FlightState::default();
        let inputs = ControlInputs::default();
        let snap = plane_snapshot(
            &id,
            &index,
            &state,
            &inputs,
            &ControllerKind::LevelHold,
            None,
            None,
            #[cfg(feature = "inference")]
            None,
        );
        assert_eq!(snap.telemetry, ControllerTelemetry::None);
        assert_eq!(snap.tuning_profile, None);
    }

    #[test]
    fn build_snapshot_carries_scalar_fields() {
        let snap = build_snapshot(
            vec![sample_plane()],
            true,
            "127.0.0.1:5555".to_string(),
            Some(SimSpeed::X5),
        );
        assert!(snap.connected);
        assert_eq!(snap.server_addr, "127.0.0.1:5555");
        assert_eq!(snap.protocol_id, PROTOCOL_ID);
        assert_eq!(snap.requested_sim_speed, Some(SimSpeed::X5));
        assert_eq!(snap.planes.len(), 1);
        assert_eq!(snap.planes[0].plane_id, 7);
    }

    #[test]
    fn default_snapshot_is_disconnected_and_empty() {
        let snap = SimSnapshot::default();
        assert!(!snap.connected);
        assert!(snap.planes.is_empty());
        assert_eq!(snap.requested_sim_speed, None);
    }

    #[test]
    fn snapshot_round_trips_through_ron() {
        let snap = build_snapshot(
            vec![sample_plane()],
            true,
            "192.168.0.10:5555".to_string(),
            Some(SimSpeed::Paused),
        );
        let serialized = ron::to_string(&snap).expect("snapshot serializes");
        let back: SimSnapshot = ron::from_str(&serialized).expect("snapshot deserializes");
        let reserialized = ron::to_string(&back).expect("round-tripped snapshot serializes");
        assert_eq!(
            serialized, reserialized,
            "snapshot should round-trip through RON unchanged"
        );
    }
}
