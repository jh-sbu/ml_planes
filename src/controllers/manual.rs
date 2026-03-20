use crate::plane::ControlInputs;
use super::traits::FlightController;
use crate::plane::FlightState;

pub struct ManualController {
    pub inputs: ControlInputs,
}

impl ManualController {
    pub fn new() -> Self {
        Self { inputs: ControlInputs::default() }
    }

    #[cfg(feature = "visual")]
    pub fn read_keyboard(&mut self, keys: &bevy::input::ButtonInput<bevy::prelude::KeyCode>, dt: f32) {
        use bevy::prelude::KeyCode;
        let rate = 0.5 * dt;

        if keys.pressed(KeyCode::KeyW) { self.inputs.elevator += rate; }
        if keys.pressed(KeyCode::KeyS) { self.inputs.elevator -= rate; }
        if keys.pressed(KeyCode::KeyA) { self.inputs.aileron -= rate; }
        if keys.pressed(KeyCode::KeyD) { self.inputs.aileron += rate; }
        if keys.pressed(KeyCode::KeyQ) { self.inputs.rudder -= rate; }
        if keys.pressed(KeyCode::KeyE) { self.inputs.rudder += rate; }
        if keys.pressed(KeyCode::ShiftLeft) { self.inputs.throttle += rate; }
        if keys.pressed(KeyCode::ControlLeft) { self.inputs.throttle -= rate; }

        self.inputs.clamp();
    }
}

impl Default for ManualController {
    fn default() -> Self {
        Self::new()
    }
}

impl FlightController for ManualController {
    fn update(&mut self, _state: &FlightState, _dt: f32) -> ControlInputs {
        self.inputs.clone()
    }

    fn name(&self) -> &'static str { "Manual" }

    #[cfg(feature = "visual")]
    fn poll_input(
        &mut self,
        keys: &bevy::input::ButtonInput<bevy::prelude::KeyCode>,
        dt: f32,
    ) {
        self.read_keyboard(keys, dt);
    }
}
