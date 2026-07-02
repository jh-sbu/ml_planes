//! Connection lifecycle for the headless replicon client — auto-reconnect + clean shutdown.
//!
//! Plain Rust + Bevy, no `rmcp` (kept quarantined in `service.rs`). Two concerns live here:
//!
//! * **Auto-reconnect** ([`ReconnectState`] + [`poll_reconnect`]). `start_renet_client`
//!   (`crate::net::client`) connects once at `Startup`; a handshake timeout or a dropped
//!   connection then leaves the client in [`ClientState::Disconnected`] with a dead transport
//!   and nothing to rebuild it. [`poll_reconnect`] watches the replicon [`ClientState`] and,
//!   while disconnected past a growing backoff deadline, rebuilds the transport via the shared
//!   [`connect_to_server`] builder — so the MCP client tolerates a server that starts late or
//!   restarts mid-session. The visual client has no equivalent (it fails a connect attempt back
//!   to its menu, `src/ui/menu.rs`), so this is MCP-specific.
//!
//! * **Clean shutdown** ([`ShutdownFlag`] + [`check_shutdown`]). When the rmcp stdio stream
//!   closes (the MCP client hangs up / stdin EOF), `main` sets the shared flag; [`check_shutdown`]
//!   turns it into an [`AppExit`] so the Bevy thread drops its transport and exits its `run_loop`
//!   deterministically instead of being torn down abruptly when the process exits.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use bevy::prelude::*;
use bevy_replicon::prelude::{ClientState, RepliconChannels};

use crate::net::{connect_to_server, ConnectTarget};

/// Upper bound on the reconnect backoff interval — repeated failures never wait longer than
/// this between attempts.
pub const RECONNECT_BACKOFF_CAP: Duration = Duration::from_secs(5);

/// Backoff bookkeeping for [`poll_reconnect`]. `initial_backoff` (the MCP's `--connect-timeout`)
/// is both the window each fresh transport gets to complete its handshake and the reset value
/// once connected; `backoff` grows (doubling, capped at [`RECONNECT_BACKOFF_CAP`]) while the
/// server stays unreachable; `next_attempt` is the wall-clock second the next rebuild is allowed.
#[derive(Resource, Debug, Clone)]
pub struct ReconnectState {
    initial_backoff: Duration,
    backoff: Duration,
    next_attempt: f64,
}

impl ReconnectState {
    /// Build with `initial_backoff` as both the first backoff and the reset value. `next_attempt`
    /// starts one `initial_backoff` out so the `Startup` connection gets that long to succeed
    /// before the first rebuild fires.
    pub fn new(initial_backoff: Duration) -> Self {
        Self {
            initial_backoff,
            backoff: initial_backoff,
            next_attempt: initial_backoff.as_secs_f64(),
        }
    }
}

impl Default for ReconnectState {
    fn default() -> Self {
        Self::new(Duration::from_secs(5))
    }
}

/// Next backoff after a failed attempt: double, clamped to `cap`.
pub fn next_backoff(current: Duration, cap: Duration) -> Duration {
    current.saturating_mul(2).min(cap)
}

/// Whether a reconnect should be attempted now: only when fully `Disconnected` (a `Connecting`
/// handshake is left to run) and the backoff deadline has passed.
pub fn should_attempt(state: ClientState, now: f64, next_attempt: f64) -> bool {
    matches!(state, ClientState::Disconnected) && now >= next_attempt
}

/// `Update` system: keep the replicon client connected.
///
/// `Connected` resets the backoff so the *next* drop retries promptly; `Connecting` is left
/// alone; `Disconnected` past the deadline rebuilds the `RenetClient` + transport (overwriting
/// the dead pair) and schedules the next attempt with a grown backoff. A build error (only a
/// local-socket failure — an unreachable server does not error here) is logged to stderr and
/// retried next tick.
pub fn poll_reconnect(
    mut commands: Commands,
    state: Res<State<ClientState>>,
    time: Res<Time>,
    target: Res<ConnectTarget>,
    channels: Res<RepliconChannels>,
    mut reconnect: ResMut<ReconnectState>,
) {
    match *state.get() {
        ClientState::Connected => {
            reconnect.backoff = reconnect.initial_backoff;
        }
        ClientState::Connecting => {}
        ClientState::Disconnected => {
            let now = time.elapsed_secs_f64();
            if should_attempt(ClientState::Disconnected, now, reconnect.next_attempt) {
                match connect_to_server(target.0, &channels) {
                    Ok((client, transport)) => {
                        commands.insert_resource(client);
                        commands.insert_resource(transport);
                        info!("mcp: (re)connecting to {}", target.0);
                    }
                    Err(err) => {
                        error!("mcp: failed to (re)build client transport: {err}");
                    }
                }
                reconnect.next_attempt = now + reconnect.backoff.as_secs_f64();
                reconnect.backoff = next_backoff(reconnect.backoff, RECONNECT_BACKOFF_CAP);
            }
        }
    }
}

/// Cross-thread shutdown signal. `main` (rmcp/tokio side) sets it once the stdio stream closes;
/// the Bevy thread's [`check_shutdown`] turns it into an [`AppExit`]. A cheap `Arc<AtomicBool>`
/// rather than a channel because it is a one-shot, read-every-frame flag.
#[derive(Resource, Clone)]
pub struct ShutdownFlag(pub Arc<AtomicBool>);

impl ShutdownFlag {
    /// Fresh, un-signalled flag.
    pub fn new() -> Self {
        Self(Arc::new(AtomicBool::new(false)))
    }

    /// Signal the Bevy thread to exit.
    pub fn request(&self) {
        self.0.store(true, Ordering::SeqCst);
    }

    /// Whether shutdown has been requested.
    pub fn is_requested(&self) -> bool {
        self.0.load(Ordering::SeqCst)
    }
}

impl Default for ShutdownFlag {
    fn default() -> Self {
        Self::new()
    }
}

/// `Update` system: emit [`AppExit`] once [`ShutdownFlag`] is set, ending the headless
/// `run_loop` cleanly.
pub fn check_shutdown(flag: Res<ShutdownFlag>, mut exit: MessageWriter<AppExit>) {
    if flag.is_requested() {
        exit.write(AppExit::Success);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn next_backoff_doubles_then_clamps() {
        let cap = Duration::from_secs(5);
        assert_eq!(
            next_backoff(Duration::from_secs(1), cap),
            Duration::from_secs(2)
        );
        assert_eq!(
            next_backoff(Duration::from_secs(2), cap),
            Duration::from_secs(4)
        );
        // Doubling 4 → 8 clamps to the 5 s cap.
        assert_eq!(next_backoff(Duration::from_secs(4), cap), cap);
        // Already at/over the cap stays at the cap.
        assert_eq!(next_backoff(cap, cap), cap);
    }

    #[test]
    fn should_attempt_only_when_disconnected_past_deadline() {
        // Disconnected + past the deadline → attempt.
        assert!(should_attempt(ClientState::Disconnected, 10.0, 5.0));
        // Disconnected but before the deadline → wait.
        assert!(!should_attempt(ClientState::Disconnected, 2.0, 5.0));
        // Connecting / Connected never trigger a rebuild, deadline notwithstanding.
        assert!(!should_attempt(ClientState::Connecting, 10.0, 5.0));
        assert!(!should_attempt(ClientState::Connected, 10.0, 5.0));
    }

    #[test]
    fn reconnect_state_seeds_first_attempt_one_backoff_out() {
        let state = ReconnectState::new(Duration::from_secs(3));
        assert_eq!(state.backoff, Duration::from_secs(3));
        assert_eq!(state.next_attempt, 3.0);
    }

    #[test]
    fn shutdown_flag_starts_clear_and_latches() {
        let flag = ShutdownFlag::new();
        assert!(!flag.is_requested());
        // A clone shares the same atomic (the cross-thread contract).
        let clone = flag.clone();
        flag.request();
        assert!(clone.is_requested());
    }
}
