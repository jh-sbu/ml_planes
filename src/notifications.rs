//! Transient notification state (render-neutral).
//!
//! A small message stack used to surface non-fatal warnings — e.g. when an
//! incompatible (stale-dimension) RL model is skipped and the controller falls
//! back to PID. Each notification fades out after [`NOTIFICATION_TTL_SECS`].
//!
//! The pure data type lives here (no rendering deps) so it is available to the
//! headless server and the controller-rebuild systems in
//! [`controllers::sim_control`](crate::controllers::sim_control), mirroring how
//! [`SimSpeed`](crate::sim_speed) was extracted. The egui banner that renders it
//! (`draw_notifications`) stays in [`ui::notifications`](crate::ui::notifications),
//! which re-exports [`Notifications`].

use bevy::prelude::*;

/// How long (seconds) a notification stays on screen before it fades out.
pub const NOTIFICATION_TTL_SECS: f32 = 5.0;

/// One transient message with its remaining time-to-live.
#[derive(Clone, Debug)]
struct Notification {
    message: String,
    remaining: f32,
}

/// Stack of active transient notifications. Drained by elapsed time each frame.
#[derive(Resource, Default)]
pub struct Notifications {
    items: Vec<Notification>,
}

impl Notifications {
    /// Queue a message to display for [`NOTIFICATION_TTL_SECS`].
    pub fn push(&mut self, message: impl Into<String>) {
        self.items.push(Notification {
            message: message.into(),
            remaining: NOTIFICATION_TTL_SECS,
        });
    }

    /// Advance all timers by `dt` seconds and drop expired notifications.
    pub fn tick(&mut self, dt: f32) {
        for n in &mut self.items {
            n.remaining -= dt;
        }
        self.items.retain(|n| n.remaining > 0.0);
    }

    /// Currently-visible messages, oldest first.
    pub fn active(&self) -> impl Iterator<Item = &str> {
        self.items.iter().map(|n| n.message.as_str())
    }

    /// Number of active notifications.
    pub fn len(&self) -> usize {
        self.items.len()
    }

    /// Whether there are no active notifications.
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_then_active() {
        let mut n = Notifications::default();
        assert!(n.is_empty());
        n.push("hello");
        assert_eq!(n.len(), 1);
        assert_eq!(n.active().collect::<Vec<_>>(), vec!["hello"]);
    }

    #[test]
    fn tick_expires_after_ttl() {
        let mut n = Notifications::default();
        n.push("bye");
        n.tick(NOTIFICATION_TTL_SECS - 0.1);
        assert_eq!(n.len(), 1, "should still be visible just before TTL");
        n.tick(0.2);
        assert!(n.is_empty(), "should expire after TTL");
    }

    #[test]
    fn independent_lifetimes_preserve_order() {
        let mut n = Notifications::default();
        n.push("first");
        n.tick(NOTIFICATION_TTL_SECS - 1.0);
        n.push("second");
        // Age past the first's TTL but not the second's.
        n.tick(1.5);
        assert_eq!(n.active().collect::<Vec<_>>(), vec!["second"]);
    }
}
