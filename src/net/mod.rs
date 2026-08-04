//! Shared network protocol compiled into both the client and the dedicated server
//! (gated behind the `net` feature). See [`protocol`] for the replicated components,
//! client→server command events, and [`protocol::NetProtocolPlugin`].

pub mod protocol;

pub use protocol::*;

pub mod client;

pub use client::{
    check_local_server_staleness, connect_to_server, local_server_path, push_snapshot, sample,
    start_renet_client, ClientNetPlugin, ConnectTarget, NetInterpolation, NetRenderClock,
    ServerProcess, Snapshot,
};

#[cfg(feature = "server")]
pub mod server;

#[cfg(feature = "server")]
pub use server::{start_renet_server, ServerPort, ServerScenario, ServerSimPlugin};
