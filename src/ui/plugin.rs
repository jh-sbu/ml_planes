use bevy::prelude::*;

use super::hud::draw_flight_hud;

pub struct UiPlugin;

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, draw_flight_hud);
    }
}
