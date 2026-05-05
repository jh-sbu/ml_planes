use super::traits::FlightController;
use bevy::prelude::*;

#[derive(Component)]
pub struct ActiveController(pub Box<dyn FlightController>);
