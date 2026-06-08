//! Upper-right **sim-speed** control (visual mode only).
//!
//! Four buttons — Pause / 1x / 5x / 10x — scale the playback speed of the
//! simulation. Rapier physics runs inside Bevy's `FixedUpdate`
//! (`RapierPhysicsPlugin::in_fixed_schedule()`), which is driven by
//! `Time<Virtual>`. Scaling virtual time therefore changes how often physics
//! steps without altering the per-step `dt = 1/64 s`, preserving integration
//! accuracy. Pause uses `Time<Virtual>::pause()`.
//!
//! As with [`map`](crate::ui::map), the egui-free state ([`SimSpeed`]) is kept
//! separate and unit-tested; [`draw_time_control`] is a thin egui shell.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

use crate::ui::map::MapState;

/// Discrete simulation playback speeds selectable from the HUD.
#[derive(Resource, Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum SimSpeed {
    /// Simulation frozen (`Time<Virtual>` paused).
    Paused,
    /// Real-time playback.
    #[default]
    X1,
    /// 5× real-time.
    X5,
    /// 10× real-time.
    X10,
}

impl SimSpeed {
    /// Virtual-time relative speed to apply when not paused.
    pub fn relative_speed(self) -> f32 {
        match self {
            SimSpeed::Paused => 1.0,
            SimSpeed::X1 => 1.0,
            SimSpeed::X5 => 5.0,
            SimSpeed::X10 => 10.0,
        }
    }

    /// Whether the clock should be frozen.
    pub fn is_paused(self) -> bool {
        matches!(self, SimSpeed::Paused)
    }

    /// Human-readable button label.
    pub fn label(self) -> &'static str {
        match self {
            SimSpeed::Paused => "Pause",
            SimSpeed::X1 => "1x",
            SimSpeed::X5 => "5x",
            SimSpeed::X10 => "10x",
        }
    }
}

/// Draws the upper-right "Sim Speed" panel and applies the selection to
/// `Time<Virtual>` whenever it changes.
pub fn draw_time_control(
    map: Res<MapState>,
    mut contexts: EguiContexts,
    mut sim_speed: ResMut<SimSpeed>,
    mut virtual_time: ResMut<Time<Virtual>>,
) {
    // The full-screen map replaces the 3D view (and the HUD) while open.
    if map.open {
        return;
    }

    let Ok(ctx) = contexts.ctx_mut() else { return };

    egui::Window::new("Sim Speed")
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-10.0, 10.0))
        .collapsible(false)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                for option in [SimSpeed::Paused, SimSpeed::X1, SimSpeed::X5, SimSpeed::X10] {
                    if ui
                        .selectable_label(*sim_speed == option, option.label())
                        .clicked()
                    {
                        *sim_speed = option;
                    }
                }
            });
        });

    // Only push to the clock on change so an externally paused/scaled clock is
    // not clobbered every frame.
    if sim_speed.is_changed() {
        let vt = &mut *virtual_time;
        if sim_speed.is_paused() {
            vt.pause();
        } else {
            vt.unpause();
            vt.set_relative_speed(sim_speed.relative_speed());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_real_time() {
        assert_eq!(SimSpeed::default(), SimSpeed::X1);
    }

    #[test]
    fn relative_speeds() {
        assert_eq!(SimSpeed::X1.relative_speed(), 1.0);
        assert_eq!(SimSpeed::X5.relative_speed(), 5.0);
        assert_eq!(SimSpeed::X10.relative_speed(), 10.0);
    }

    #[test]
    fn only_paused_is_paused() {
        assert!(SimSpeed::Paused.is_paused());
        assert!(!SimSpeed::X1.is_paused());
        assert!(!SimSpeed::X5.is_paused());
        assert!(!SimSpeed::X10.is_paused());
    }

    #[test]
    fn labels() {
        assert_eq!(SimSpeed::Paused.label(), "Pause");
        assert_eq!(SimSpeed::X1.label(), "1x");
        assert_eq!(SimSpeed::X5.label(), "5x");
        assert_eq!(SimSpeed::X10.label(), "10x");
    }
}
