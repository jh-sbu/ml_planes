//! Shared network protocol compiled into both the client and the dedicated server
//! (gated behind the `net` feature). See [`protocol`] for the replicated components,
//! client→server command events, and [`protocol::NetProtocolPlugin`].

pub mod protocol;

pub use protocol::*;

#[cfg(feature = "server")]
pub mod server;

#[cfg(feature = "server")]
pub use server::{start_renet_server, ServerPort, ServerScenario, ServerSimPlugin};
