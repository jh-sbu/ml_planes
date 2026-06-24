use bevy::prelude::*;
use bevy_egui::EguiPrimaryContextPass;

use super::file_load::{poll_pending_loads, PendingLoads};
use super::hud::draw_flight_hud;
use super::lifecycle_panel::{draw_plane_panel, plane_lifecycle_hotkeys, PlanePanelState};
use super::map::{draw_map, MapState};
use super::menu::{AppState, MenuPlugin};
use super::notifications::{draw_notifications, Notifications};
use super::time_control::{draw_time_control, SimSpeed};

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        // The menu plugin owns the AppState machine + scenario spawn/teardown.
        app.add_plugins(MenuPlugin)
            .init_resource::<PendingLoads>()
            .init_resource::<MapState>()
            .init_resource::<SimSpeed>()
            .init_resource::<PlanePanelState>()
            .init_resource::<Notifications>()
            // Gameplay HUD/panels only draw while a scenario is flying, so the
            // menu screens (which use the same egui context) stay uncluttered.
            .add_systems(
                EguiPrimaryContextPass,
                (
                    draw_flight_hud,
                    draw_map,
                    draw_time_control,
                    draw_plane_panel,
                )
                    .run_if(in_state(AppState::InGame)),
            )
            // Notifications are produced on the menu too (e.g. the Train
            // placeholder), so render them in every state — not just InGame.
            .add_systems(EguiPrimaryContextPass, draw_notifications)
            .add_systems(
                Update,
                plane_lifecycle_hotkeys.run_if(in_state(AppState::InGame)),
            )
            .add_systems(PostUpdate, poll_pending_loads);
    }
}
