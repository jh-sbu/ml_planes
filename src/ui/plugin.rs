use bevy::prelude::*;
use bevy_egui::EguiPrimaryContextPass;

use super::file_load::{poll_pending_loads, PendingLoads};
use super::hud::draw_flight_hud;

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PendingLoads>()
            .add_systems(EguiPrimaryContextPass, draw_flight_hud)
            .add_systems(PostUpdate, poll_pending_loads);
    }
}
