//! Shared network protocol compiled into both the client and the dedicated server
//! (gated behind the `net` feature). See [`protocol`] for the replicated components,
//! client→server command events, and [`protocol::NetProtocolPlugin`].

pub mod protocol;

pub use protocol::*;

pub mod client;

pub use client::{
    connect_to_server, interpolate, start_renet_client, ClientNetPlugin, ConnectTarget,
    NetInterpolation,
};

#[cfg(feature = "server")]
pub mod server;

#[cfg(feature = "server")]
pub use server::{start_renet_server, ServerPort, ServerScenario, ServerSimPlugin};
