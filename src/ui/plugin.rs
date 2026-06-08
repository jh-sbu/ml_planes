use bevy::prelude::*;
use bevy_egui::EguiPrimaryContextPass;

use super::file_load::{poll_pending_loads, PendingLoads};
use super::hud::draw_flight_hud;
use super::map::{draw_map, MapState};
use super::time_control::{draw_time_control, SimSpeed};

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PendingLoads>()
            .init_resource::<MapState>()
            .init_resource::<SimSpeed>()
            .add_systems(
                EguiPrimaryContextPass,
                (draw_flight_hud, draw_map, draw_time_control),
            )
            .add_systems(PostUpdate, poll_pending_loads);
    }
}
