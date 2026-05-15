//! Heading-hold flight controller — outer heading PID over inner level-hold.
//!
//! Cascade:
//!   heading error [rad] → [heading PID] → bank command [rad]
//!   bank command → inner LevelHoldController.target_roll
//!   inner LevelHoldController handles altitude, airspeed, roll, sideslip

use std::f32::consts::FRAC_PI_3;

use bevy::math::Vec3;

use crate::controllers::level_hold::LevelHoldController;
use crate::controllers::pid::PidController;
use crate::controllers::traits::FlightController;
use crate::controllers::tuning::HeadingHoldTuning;
use crate::plane::{ControlInputs, FlightState};

/// Cascade PID controller that holds a configurable heading.
///
/// The outer heading loop converts heading error to a bank command, then
/// delegates to an inner [`LevelHoldController`] for attitude stabilization.
#[derive(Clone)]
pub struct HeadingHoldController {
    /// Target heading [rad], measured as atan2(v_z, v_x) in the XZ plane.
    /// Captured from current velocity at construction; adjustable via HUD.
    pub target_heading: f32,
    /// Outer heading loop: heading error [rad] → bank command [rad], clamped ±60°.
    pub heading_pid: PidController,
    /// Inner level-hold controller — handles altitude, airspeed, roll, sideslip.
    pub inner: LevelHoldController,
}

impl HeadingHoldController {
    /// Construct with explicit target heading and default gains.
    /// Seeds inner controller from flight state for a bumpless handoff.
    pub fn new(state: &FlightState, target_heading: f32) -> Self {
        Self {
            target_heading,
            heading_pid: PidController::new(0.7, 0.0, 0.1, 0.0, -FRAC_PI_3, FRAC_PI_3),
            inner: LevelHoldController::from_state(state, &ControlInputs::default()),
        }
    }

    /// Construct by capturing the current heading from state.
    /// Use `prev_inputs` to seed inner controller integrals (bumpless handoff).
    pub fn from_state(state: &FlightState, prev_inputs: &ControlInputs) -> Self {
        Self {
            target_heading: heading_from_state(state),
            heading_pid: PidController::new(0.7, 0.0, 0.1, 0.0, -FRAC_PI_3, FRAC_PI_3),
            inner: LevelHoldController::from_state(state, prev_inputs),
        }
    }

    /// Construct with gains from `tuning`, capturing current heading and seeding
    /// inner integrals for a bumpless handoff.
    pub fn with_tuning(
        state: &FlightState,
        tuning: &HeadingHoldTuning,
        prev_inputs: &ControlInputs,
    ) -> Self {
        let mut ctrl = Self::from_state(state, prev_inputs);
        ctrl.heading_pid.kp = tuning.heading_kp;
        ctrl.heading_pid.kd = tuning.heading_kd;
        ctrl.inner = LevelHoldController::with_tuning(state, &tuning.inner, prev_inputs);
        ctrl
    }
}

/// Extract heading angle [rad] from the XZ velocity component.
/// Falls back to body-frame forward direction when speed is too low.
fn heading_from_state(state: &FlightState) -> f32 {
    let speed_xz = (state.velocity.x.powi(2) + state.velocity.z.powi(2)).sqrt();
    if speed_xz > 1.0 {
        state.velocity.z.atan2(state.velocity.x)
    } else {
        let fwd = state.attitude * Vec3::X;
        fwd.z.atan2(fwd.x)
    }
}

impl FlightController for HeadingHoldController {
    fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs {
        // Current heading unit vector from velocity (or attitude at low speed).
        let speed_xz = (state.velocity.x.powi(2) + state.velocity.z.powi(2)).sqrt();
        let (head_x, head_z) = if speed_xz > 1.0 {
            (state.velocity.x / speed_xz, state.velocity.z / speed_xz)
        } else {
            let fwd = state.attitude * Vec3::X;
            let len = (fwd.x.powi(2) + fwd.z.powi(2)).sqrt().max(1e-6);
            (fwd.x / len, fwd.z / len)
        };

        // Target direction unit vector from target_heading = atan2(z, x).
        let (tgt_z, tgt_x) = self.target_heading.sin_cos();

        // Signed heading error: positive = current heading is LEFT of target.
        // Positive error → bank right (positive roll) → turn right toward target.
        let cross = tgt_x * head_z - tgt_z * head_x;
        let dot = tgt_x * head_x + tgt_z * head_z;
        let heading_error = cross.atan2(dot);

        // Subtract correction (matches orbit.rs convention): positive heading_error means head
        // is CCW (left) of target, so we need to bank right → decrease target_roll.
        let bank_cmd = -self.heading_pid.update(heading_error, dt);
        self.inner.target_roll = bank_cmd;
        self.inner.update(state, dt)
    }

    fn name(&self) -> &'static str {
        "HeadingHold"
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }

    #[cfg(feature = "visual")]
    fn poll_input(&mut self, keys: &bevy::input::ButtonInput<bevy::prelude::KeyCode>, dt: f32) {
        use bevy::prelude::KeyCode;
        // Left/Right arrows: adjust target heading ±5°/s.
        let rate = 5.0_f32.to_radians();
        if keys.pressed(KeyCode::ArrowLeft) {
            self.target_heading -= rate * dt;
        }
        if keys.pressed(KeyCode::ArrowRight) {
            self.target_heading += rate * dt;
        }
        // Normalise to (-π, π].
        self.target_heading = (self.target_heading + std::f32::consts::PI)
            .rem_euclid(std::f32::consts::TAU)
            - std::f32::consts::PI;
        // Altitude/airspeed via inner controller's HUD DragValues only.
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controllers::FlightController;
    use std::f32::consts::FRAC_PI_2;

    fn level_attitude() -> bevy::math::Quat {
        bevy::math::Quat::from_rotation_x(-FRAC_PI_2)
    }

    fn level_state() -> FlightState {
        FlightState {
            position: Vec3::new(0.0, 1000.0, 0.0),
            velocity: Vec3::new(80.0, 0.0, 0.0),
            attitude: level_attitude(),
            angular_velocity: Vec3::ZERO,
            alpha: 0.0,
            beta: 0.0,
            airspeed: 80.0,
            altitude: 1000.0,
        }
    }

    #[test]
    fn from_state_captures_heading() {
        let state = level_state();
        let ctrl = HeadingHoldController::from_state(&state, &ControlInputs::default());
        // Velocity along +X → heading = atan2(0, 80) = 0.
        assert!(
            ctrl.target_heading.abs() < 1e-5,
            "heading={}",
            ctrl.target_heading
        );
    }

    #[test]
    fn zero_error_produces_zero_bank_command() {
        let state = level_state();
        let mut ctrl = HeadingHoldController::from_state(&state, &ControlInputs::default());
        ctrl.update(&state, 1.0 / 60.0);
        assert!(
            ctrl.inner.target_roll.abs() < 0.05,
            "target_roll={} expected ≈0 at sustained zero heading error",
            ctrl.inner.target_roll
        );
    }

    #[test]
    fn with_tuning_applies_gains() {
        use crate::controllers::tuning::{HeadingHoldTuning, LevelHoldTuning};
        let state = level_state();
        let tuning = HeadingHoldTuning {
            heading_kp: 1.5,
            heading_kd: 0.3,
            inner: LevelHoldTuning::default(),
        };
        let ctrl = HeadingHoldController::with_tuning(&state, &tuning, &ControlInputs::default());
        assert!((ctrl.heading_pid.kp - 1.5).abs() < 1e-6, "kp");
        assert!((ctrl.heading_pid.kd - 0.3).abs() < 1e-6, "kd");
    }
}
