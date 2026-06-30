//! The `rmcp` MCP server handler — **all `rmcp` types are quarantined to this file**.
//!
//! Keeping every `rmcp` macro / type behind this single module means a future SDK bump
//! (rmcp has broken every minor: 0.1→0.2→0.3→1.x→2.0) has a one-file blast radius; the
//! shared snapshot + command-bridge primitives added in later phases stay plain Rust.
//!
//! Phase 0 surface: a single `ping` tool that proves the rmcp 2.0 `#[tool_router]` /
//! `#[tool]` / `#[tool_handler]` macro stack and the stdio `initialize` + `tools/list`
//! handshake. Read tools (Phase 2) and write tools (Phases 3–4) are added later, each
//! holding the `Arc<RwLock<SimSnapshot>>` / `ControlSender` this struct will gain.

use rmcp::{tool, tool_handler, tool_router, ServerHandler};

/// MCP server handler for the live `ml_planes` simulation.
///
/// Currently stateless; later phases add the snapshot mirror and the command channel.
#[derive(Clone, Default)]
pub struct PlanesService;

impl PlanesService {
    pub fn new() -> Self {
        Self
    }
}

#[tool_router]
impl PlanesService {
    /// Liveness probe — confirms the MCP stdio handshake works end to end.
    #[tool(description = "Liveness check; returns \"pong\".")]
    async fn ping(&self) -> String {
        "pong".to_string()
    }
}

#[tool_handler(
    name = "ml_planes_mcp",
    instructions = "Inspect and control a running ml_planes flight simulation. \
                    Phase 0: only `ping` is available."
)]
impl ServerHandler for PlanesService {}
