//! Shared client/server network protocol: the replicated components and the
//! client→server command events, registered identically on both ends by
//! [`NetProtocolPlugin`].
//!
//! Replicon requires the protocol (replication rules + remote events) to be
//! registered in the **same order** on the client and the server, which is why a
//! single plugin owns all of it. The plugin only declares the protocol; the actual
//! transport (renet server/client) and the command-handling systems are wired up by
//! the server binary (Phase 3) and client (Phases 4–6).

use bevy::prelude::*;
use bevy_replicon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::controllers::{ControllerKind, SelectedTuningProfile};
use crate::plane::{ControlInputs, FlightState, PlaneId, PlaneIndex, PlaneTuningPath};
use crate::sim_speed::SimSpeed;
use crate::training::SpawnSpec;

#[cfg(feature = "inference")]
use crate::controllers::SelectedModel;

/// Default UDP port the dedicated server binds to and clients connect to.
pub const DEFAULT_PORT: u16 = 5555;

/// Protocol identity/version. Bumped whenever the replicated component set or the
/// command set changes incompatibly; the renet transport (Phase 3) uses it to reject
/// mismatched peers.
pub const PROTOCOL_ID: u64 = 1;

/// Switch the target plane's active controller (server rebuilds it).
#[derive(Event, Serialize, Deserialize, Clone, Debug)]
pub struct SwitchControllerCommand {
    pub plane: PlaneId,
    pub kind: ControllerKind,
}

/// Select a named PID tuning profile for the target plane.
#[derive(Event, Serialize, Deserialize, Clone, Debug)]
pub struct SetTuningProfileCommand {
    pub plane: PlaneId,
    pub profile: String,
}

/// Select a trained RL model (by file stem) for the target plane. Only meaningful
/// in builds that can run inference, so it is gated to the `inference` feature.
#[cfg(feature = "inference")]
#[derive(Event, Serialize, Deserialize, Clone, Debug)]
pub struct SetModelCommand {
    pub plane: PlaneId,
    pub model_stem: String,
}

/// Feed manual control-surface inputs to the target plane's `ManualController`.
/// Sent every client frame while manually flying; server uses the latest.
#[derive(Event, Serialize, Deserialize, Clone, Debug)]
pub struct ManualInputCommand {
    pub plane: PlaneId,
    pub inputs: ControlInputs,
}

/// Spawn a new plane on the server from a `.plane.ron` config, controller kind, and
/// spawn state.
#[derive(Event, Serialize, Deserialize, Clone, Debug)]
pub struct SpawnPlaneNetCommand {
    pub config_path: String,
    pub kind: ControllerKind,
    pub spec: SpawnSpec,
}

/// Remove the plane with the given id from the server.
#[derive(Event, Serialize, Deserialize, Clone, Debug)]
pub struct RemovePlaneNetCommand {
    pub plane: PlaneId,
}

/// Set the authoritative simulation playback speed (time accel / pause is
/// server-side).
#[derive(Event, Serialize, Deserialize, Clone, Debug)]
pub struct SetSimSpeedCommand {
    pub speed: SimSpeed,
}

/// Registers the shared replication rules and client→server command events. Must be
/// added on both client and server (after `RepliconPlugins`) so the protocol matches.
pub struct NetProtocolPlugin;

impl Plugin for NetProtocolPlugin {
    fn build(&self, app: &mut App) {
        // Replicated components — server → client. Order must match client/server.
        app.replicate::<Transform>()
            .replicate::<FlightState>()
            .replicate::<ControlInputs>()
            .replicate::<PlaneId>()
            .replicate::<PlaneIndex>()
            .replicate::<ControllerKind>()
            // Selection state (Phase 6) so the client can display + enumerate the
            // current tuning profile / RL model. `PlaneTuningPath` lets the client
            // rebuild a `PlaneTuningHandle` and reuse the existing enumeration.
            .replicate::<SelectedTuningProfile>()
            .replicate::<PlaneTuningPath>();

        #[cfg(feature = "inference")]
        app.replicate::<SelectedModel>();

        // Client → server commands. Reliable + ordered so a command is never dropped
        // or applied out of order (manual input is latest-wins, but ordered delivery
        // keeps the newest packet authoritative).
        app.add_client_event::<SwitchControllerCommand>(Channel::Ordered)
            .add_client_event::<SetTuningProfileCommand>(Channel::Ordered)
            .add_client_event::<ManualInputCommand>(Channel::Ordered)
            .add_client_event::<SpawnPlaneNetCommand>(Channel::Ordered)
            .add_client_event::<RemovePlaneNetCommand>(Channel::Ordered)
            .add_client_event::<SetSimSpeedCommand>(Channel::Ordered);

        #[cfg(feature = "inference")]
        app.add_client_event::<SetModelCommand>(Channel::Ordered);
    }
}
