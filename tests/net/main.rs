//! Consolidated net/mcp/server integration tests (formerly 9 separate test binaries; merged
//! per plans/test_compile_speed.md to cut link steps). Entirely compiled out without `net`.
//! Run one former file's tests with e.g.
//! `cargo test --no-default-features --features "mcp server" --test net net_serde::`.
#![cfg(feature = "net")]

#[path = "../common/mod.rs"]
mod common; // used by server_sim / mcp_bridge / mcp_lifecycle / mcp_snapshot

mod client_net;
mod local_server;
mod net_protocol;
mod net_serde;

#[cfg(feature = "server")]
mod server_sim;

#[cfg(feature = "mcp")]
mod mcp_bridge;
#[cfg(feature = "mcp")]
mod mcp_lifecycle;
#[cfg(feature = "mcp")]
mod mcp_snapshot;

#[cfg(all(feature = "mcp", feature = "server"))]
mod mcp_e2e;
