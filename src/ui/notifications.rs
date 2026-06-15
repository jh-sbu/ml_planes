//! Transient on-screen notifications (visual mode only).
//!
//! A small top-centre banner stack used to surface non-fatal warnings to the
//! user without spamming — e.g. when an incompatible (stale-dimension) RL model
//! is skipped and the controller falls back to PID. Each notification fades out
//! after [`NOTIFICATION_TTL_SECS`].
//!
//! As with [`map`](crate::ui::map) and [`time_control`](crate::ui::time_control),
//! the egui-free state ([`Notifications`]) is kept separate and unit-tested;
//! [`draw_notifications`] is a thin egui shell.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

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

/// Ages out notifications and renders the active ones as a top-centre banner.
pub fn draw_notifications(
    mut contexts: EguiContexts,
    mut notes: ResMut<Notifications>,
    time: Res<Time>,
) {
    notes.tick(time.delta_secs());
    if notes.is_empty() {
        return;
    }

    let Ok(ctx) = contexts.ctx_mut() else { return };

    egui::Area::new(egui::Id::new("notifications"))
        .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 10.0))
        .show(ctx, |ui| {
            for message in notes.active() {
                egui::Frame::NONE
                    .fill(egui::Color32::from_rgb(80, 50, 0))
                    .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(200, 150, 0)))
                    .inner_margin(egui::Margin::symmetric(10, 6))
                    .corner_radius(4.0)
                    .show(ui, |ui| {
                        ui.colored_label(
                            egui::Color32::from_rgb(255, 210, 120),
                            format!("⚠ {message}"),
                        );
                    });
                ui.add_space(4.0);
            }
        });
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
