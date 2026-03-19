use bevy::ecs::component::Component;

/// Normalized control surface commands produced by a `FlightController`.
#[derive(Component, Default, Debug, Clone)]
pub struct ControlInputs {
    pub aileron: f32,   // [-1, 1]
    pub elevator: f32,  // [-1, 1]
    pub rudder: f32,    // [-1, 1]
    pub throttle: f32,  // [0, 1]
}

impl ControlInputs {
    /// Clamp all channels to their legal ranges.
    pub fn clamp(&mut self) {
        self.aileron  = self.aileron.clamp(-1.0, 1.0);
        self.elevator = self.elevator.clamp(-1.0, 1.0);
        self.rudder   = self.rudder.clamp(-1.0, 1.0);
        self.throttle = self.throttle.clamp(0.0, 1.0);
    }
}
