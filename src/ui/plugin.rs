use bevy::prelude::*;
use bevy_egui::EguiPrimaryContextPass;

use super::hud::draw_flight_hud;

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(EguiPrimaryContextPass, draw_flight_hud);
    }
}
