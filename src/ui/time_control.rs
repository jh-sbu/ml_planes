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

#[cfg(feature = "net")]
use crate::net::SetSimSpeedCommand;
#[cfg(feature = "net")]
use bevy_replicon::prelude::ClientTriggerExt;

// `SimSpeed` now lives in the render-neutral `crate::sim_speed` module so the
// network protocol and headless server can reference it; re-exported here so the
// HUD call sites (`super::time_control::SimSpeed`) are unchanged.
pub use crate::sim_speed::SimSpeed;

/// Draws the upper-right "Sim Speed" panel. In the local-sim build it applies the
/// selection to `Time<Virtual>`; the networked client sends a
/// [`SetSimSpeedCommand`] (the server clock is authoritative) and just tracks the
/// chosen speed locally so the buttons highlight correctly.
pub fn draw_time_control(
    map: Res<MapState>,
    mut contexts: EguiContexts,
    mut sim_speed: ResMut<SimSpeed>,
    #[cfg(not(feature = "net"))] mut virtual_time: ResMut<Time<Virtual>>,
    #[cfg(feature = "net")] mut commands: Commands,
) {
    // The full-screen map replaces the 3D view (and the HUD) while open.
    if map.open {
        return;
    }

    let Ok(ctx) = contexts.ctx_mut() else { return };

    egui::Window::new("Sim Speed")
        // Sit just below the in-game "Menu" window (also RIGHT_TOP at y=10) so the
        // two top-right panels don't overlap.
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-10.0, 70.0))
        .collapsible(false)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                for option in [SimSpeed::Paused, SimSpeed::X1, SimSpeed::X5, SimSpeed::X10] {
                    if ui
                        .selectable_label(*sim_speed == option, option.label())
                        .clicked()
                    {
                        *sim_speed = option;
                        #[cfg(feature = "net")]
                        commands.client_trigger(SetSimSpeedCommand { speed: option });
                    }
                }
            });
        });

    // Local sim only: push the speed onto the clock on change. The networked
    // client renders interpolated state and never scales its own clock — the
    // server applies `SetSimSpeedCommand` to its authoritative `Time<Virtual>`.
    #[cfg(not(feature = "net"))]
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
