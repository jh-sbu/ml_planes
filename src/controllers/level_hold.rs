//! Level-hold flight controller — cascade PID.
//!
//! Loops:
//!   altitude  → [outer PID] → α_target → [inner PID] → elevator
//!   airspeed  → [PID] → throttle
//!   roll angle → [PID] → aileron
//!   sideslip β → [PID] → rudder

use bevy::math::{Quat, Vec3};

use crate::controllers::{FlightController, PidController};
use crate::plane::{ControlInputs, FlightState};

// ---------------------------------------------------------------------------
// Attitude helpers
// ---------------------------------------------------------------------------

/// Roll (bank) angle from a body→world quaternion.
///
/// Positive = right wing low (right bank).
/// Returns 0 for level, unbanked flight.
fn roll_angle(attitude: Quat) -> f32 {
    // attitude is body→world.
    // right_world: direction the right wing (+Y body) points in world space.
    // up_world:    direction the cockpit-up (+Z body) points in world space.
    let right_world = attitude * Vec3::Y;
    let up_world    = attitude * Vec3::Z;
    // Project both onto the world vertical axis (Y) and compute bank angle.
    right_world.y.atan2(up_world.y)
}

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------

/// Cascade PID controller for level altitude and airspeed hold.
///
/// Default gains are tuned for `generic_jet.plane.ron` at ~80 m/s.
pub struct LevelHoldController {
    /// Target altitude [m].
    pub target_altitude: f32,
    /// Target airspeed [m/s].
    pub target_airspeed: f32,
    /// Outer altitude loop: altitude error [m] → α_target [rad], clamped ±0.3 rad.
    altitude_pid: PidController,
    /// Inner alpha loop: α error [rad] → elevator [-1, 1].
    alpha_pid: PidController,
    /// Airspeed loop: airspeed error [m/s] → throttle [0, 1].
    airspeed_pid: PidController,
    /// Wings-level loop: roll angle [rad] → aileron [-1, 1].
    roll_pid: PidController,
    /// Heading/sideslip loop: β [rad] → rudder [-1, 1].
    beta_pid: PidController,
}

impl LevelHoldController {
    /// Create a controller with default gains for the generic jet.
    pub fn new(target_altitude: f32, target_airspeed: f32) -> Self {
        Self {
            target_altitude,
            target_airspeed,
            // Outer altitude loop: error [m] → α_target [rad].
            // Small kp keeps α_target within ±0.3 rad; small ki handles trim offset.
            altitude_pid: PidController::new(0.003, 0.0002, 0.0, 1.0, -0.3, 0.3),
            // Inner alpha loop: error [rad] → elevator [-1, 1].
            // No integral (outer loop provides steady-state correction).
            // kd=0.5 gives ~50° phase margin on the pitch double-integrator plant.
            alpha_pid:    PidController::new(1.0,   0.0,    0.5,  0.0, -1.0, 1.0),
            // Airspeed loop: integral holds throttle at trim without steady-state error.
            airspeed_pid: PidController::new(0.01,  0.001,  0.0,  1.0,  0.0, 1.0),
            // Wings-level: Cl_p roll damping already provides ~53° PM; conservative kp/kd.
            roll_pid:     PidController::new(0.5,   0.0,    0.3,  0.0, -1.0, 1.0),
            // Sideslip: weathercock (Cn_β) provides natural yaw stiffness.
            beta_pid:     PidController::new(0.5,   0.0,    0.1,  0.0, -1.0, 1.0),
        }
    }
}

impl FlightController for LevelHoldController {
    fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs {
        // Altitude cascade: altitude error → α_target → elevator.
        //
        // Sign note: positive elevator → nose-UP → alpha INCREASES (standard convention:
        // nose rising means velocity appears further below nose in body frame, increasing α).
        // Inner loop error is (alpha_target − state.alpha): positive error → positive elevator
        // → nose up → alpha increases toward target.
        let alpha_target = self.altitude_pid.update(self.target_altitude - state.altitude, dt);
        let elevator     = self.alpha_pid.update(alpha_target - state.alpha, dt);

        // Airspeed: error → throttle (integral holds trim value).
        let throttle = self.airspeed_pid.update(self.target_airspeed - state.airspeed, dt);

        // Wings level: drive roll to zero.
        let roll    = roll_angle(state.attitude);
        let aileron = self.roll_pid.update(-roll, dt);

        // Sideslip / heading hold: drive β to zero.
        let rudder = self.beta_pid.update(-state.beta, dt);

        ControlInputs { aileron, elevator, rudder, throttle }
    }

    fn name(&self) -> &'static str { "LevelHold" }

    #[cfg(feature = "visual")]
    fn poll_input(
        &mut self,
        keys: &bevy::input::ButtonInput<bevy::prelude::KeyCode>,
        dt: f32,
    ) {
        use bevy::prelude::KeyCode;
        // Arrow Up/Down: adjust target altitude ±50 m per second held.
        if keys.pressed(KeyCode::ArrowUp)   { self.target_altitude += 50.0 * dt; }
        if keys.pressed(KeyCode::ArrowDown) { self.target_altitude -= 50.0 * dt; }
        // Equal(+) / Minus(-): adjust target airspeed ±5 m/s per second held.
        if keys.pressed(KeyCode::Equal) { self.target_airspeed += 5.0 * dt; }
        if keys.pressed(KeyCode::Minus) { self.target_airspeed -= 5.0 * dt; }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    /// Level-flight attitude: body +Z (cockpit up) aligns with world +Y (up).
    fn level_attitude() -> Quat {
        Quat::from_rotation_x(-FRAC_PI_2)
    }

    /// Minimal level-flight FlightState at given altitude and airspeed.
    fn level_state(altitude: f32, airspeed: f32) -> FlightState {
        FlightState {
            position:         Vec3::new(0.0, altitude, 0.0),
            velocity:         Vec3::new(airspeed, 0.0, 0.0),
            attitude:         level_attitude(),
            angular_velocity: Vec3::ZERO,
            alpha:            0.0,
            beta:             0.0,
            airspeed,
            altitude,
        }
    }

    #[test]
    fn roll_angle_is_zero_at_level_attitude() {
        let roll = roll_angle(level_attitude());
        assert!(roll.abs() < 1e-5, "roll={roll}");
    }

    #[test]
    fn roll_angle_nonzero_for_banked_attitude() {
        // Rotate past level by an extra quarter-turn: should produce a large roll angle.
        let banked = Quat::from_rotation_x(-FRAC_PI_2 + 0.4);
        let roll   = roll_angle(banked);
        assert!(roll.abs() > 0.3, "roll={roll} expected |roll|>0.3 for banked attitude");
    }

    // Positive elevator → nose-up → alpha increases (more lift, plane climbs).
    // To gain altitude, the controller pitches nose UP (positive elevator) so
    // alpha increases, lift exceeds weight, and altitude rises.
    #[test]
    fn altitude_below_target_gives_positive_elevator() {
        let mut ctrl = LevelHoldController::new(1000.0, 80.0);
        let state    = level_state(900.0, 80.0); // 100 m below target
        let inputs   = ctrl.update(&state, 1.0 / 60.0);
        assert!(inputs.elevator > 0.0, "elevator={}", inputs.elevator);
    }

    #[test]
    fn altitude_above_target_gives_negative_elevator() {
        let mut ctrl = LevelHoldController::new(1000.0, 80.0);
        let state    = level_state(1100.0, 80.0); // 100 m above target
        let inputs   = ctrl.update(&state, 1.0 / 60.0);
        assert!(inputs.elevator < 0.0, "elevator={}", inputs.elevator);
    }

    #[test]
    fn airspeed_below_target_gives_positive_throttle() {
        let mut ctrl = LevelHoldController::new(1000.0, 80.0);
        let state    = level_state(1000.0, 60.0); // 20 m/s slow
        let inputs   = ctrl.update(&state, 1.0 / 60.0);
        assert!(inputs.throttle > 0.0, "throttle={}", inputs.throttle);
    }

    #[test]
    fn zero_error_on_first_step_gives_near_zero_control_outputs() {
        // No integral has built up yet, no derivative, no error → outputs ≈ 0.
        let mut ctrl = LevelHoldController::new(1000.0, 80.0);
        let state    = level_state(1000.0, 80.0);
        let inputs   = ctrl.update(&state, 1.0 / 60.0);
        assert!(inputs.elevator.abs() < 0.05, "elevator={}", inputs.elevator);
        assert!(inputs.aileron.abs()  < 0.05, "aileron={}", inputs.aileron);
        assert!(inputs.rudder.abs()   < 0.05, "rudder={}", inputs.rudder);
        // Throttle: error=0, no integral yet → 0
        assert!(inputs.throttle.abs() < 0.05, "throttle={}", inputs.throttle);
    }
}
