use bevy::prelude::*;
use bevy_egui::EguiPrimaryContextPass;

use super::file_load::{poll_pending_loads, PendingLoads};
use super::hud::draw_flight_hud;
use super::lifecycle_panel::{draw_plane_panel, plane_lifecycle_hotkeys, PlanePanelState};
use super::map::{draw_map, MapState};
use super::notifications::{draw_notifications, Notifications};
use super::time_control::{draw_time_control, SimSpeed};

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PendingLoads>()
            .init_resource::<MapState>()
            .init_resource::<SimSpeed>()
            .init_resource::<PlanePanelState>()
            .init_resource::<Notifications>()
            .add_systems(
                EguiPrimaryContextPass,
                (
                    draw_flight_hud,
                    draw_map,
                    draw_time_control,
                    draw_plane_panel,
                    draw_notifications,
                ),
            )
            .add_systems(Update, plane_lifecycle_hotkeys)
            .add_systems(PostUpdate, poll_pending_loads);
    }
}
