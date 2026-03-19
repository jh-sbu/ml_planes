use bevy::prelude::*;
use super::traits::FlightController;

#[derive(Component)]
pub struct ActiveController(pub Box<dyn FlightController>);
