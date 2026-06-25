//! On-screen notification banner (visual mode only).
//!
//! The render-neutral state ([`Notifications`]) lives in [`crate::notifications`]
//! so the headless server and the controller-rebuild systems can use it; this
//! module is just the egui shell that draws it.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

pub use crate::notifications::Notifications;

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
