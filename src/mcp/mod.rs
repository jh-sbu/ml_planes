//! MCP control client (`ml_planes_mcp`).
//!
//! A standalone, headless renet/replicon **client** that joins a running
//! `ml_planes_server` over the existing net protocol and exposes the live simulation
//! to an LLM agent over **MCP stdio** (the `rmcp` SDK). It reuses `net::NetProtocolPlugin`
//! verbatim — no server-side changes.
//!
//! The binary (`src/bin/mcp.rs`) runs two cooperating runtimes bridged by shared state:
//! a headless Bevy `App` on a dedicated thread (the replicon client) and an `rmcp` stdio
//! server on the tokio main thread. See `plans/mcp_server.md` for the full design.
//!
//! Phase 0 (this revision) stands up the buildable skeleton: arg parsing, a stub MCP
//! surface (`PlanesService::ping`), and the dual-runtime wiring. The snapshot mirror
//! (Phase 1), read tools (Phase 2), and the command bridge + write tools (Phases 3–4)
//! land in later revisions.

pub mod args;
pub mod service;

pub use args::McpArgs;
pub use service::PlanesService;
