//! Ascent controller — climbs to a target altitude, then holds.
//!
//! Wraps `LevelHoldController` and sets its `target_altitude` each tick.
//! Once the plane enters the threshold band around the target, `complete`
//! latches to `true` and the inner controller continues to hold altitude.

use crate::controllers::level_hold::LevelHoldController;
use crate::controllers::traits::FlightController;
use crate::plane::{ControlInputs, FlightState};

/// Controller that climbs to `target_altitude` and holds it thereafter.
///
/// Stabilization is fully delegated to an inner `LevelHoldController`.
/// The only addition is the `complete` latch: set once the plane is within
/// `threshold` metres of the target, and never cleared.
///
/// Construct explicitly with [`AscentController::new`]; see [`crate::controllers::ControllerKind::Ascent`]
/// for the corresponding kind variant.
pub struct AscentController {
    /// Altitude to climb to [m].
    pub target_altitude: f32,
    /// Band around `target_altitude` considered "at altitude" [m]. Default: 10.0.
    pub threshold: f32,
    /// Latches to `true` once the plane is within `threshold` of `target_altitude`.
    pub complete: bool,
    /// Inner cascade PID.
    /// `target_altitude` is overwritten each tick; `target_airspeed` is captured
    /// from `state` at construction for a bumpless handoff.
    pub inner: LevelHoldController,
}

impl AscentController {
    /// Bumpless construction.
    ///
    /// Captures current airspeed from `state` so the inner controller's
    /// airspeed PID sees no step command.  Sets inner `target_altitude` to
    /// `target_altitude` immediately.
    pub fn new(state: &FlightState, target_altitude: f32) -> Self {
        let mut inner = LevelHoldController::from_state(state, &ControlInputs::default());
        inner.target_altitude = target_altitude;
        Self {
            target_altitude,
            threshold: 10.0,
            complete: false,
            inner,
        }
    }
}

impl FlightController for AscentController {
    fn update(
        &mut self,
        state: &FlightState,
        ctx: &crate::plane::ControllerContext,
        dt: f32,
    ) -> ControlInputs {
        if (state.altitude - self.target_altitude).abs() < self.threshold {
            self.complete = true;
        }
        // Overwrite every tick (guard against external mutation of inner).
        self.inner.target_altitude = self.target_altitude;
        self.inner.update(state, ctx, dt)
    }

    fn name(&self) -> &'static str {
        "Ascent"
    }
    fn telemetry(
        &self,
        _state: &FlightState,
    ) -> crate::controllers::telemetry::ControllerTelemetry {
        crate::controllers::telemetry::ControllerTelemetry::Ascent {
            complete: self.complete,
        }
    }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::{Quat, Vec3};
    use std::f32::consts::FRAC_PI_2;

    fn level_state(altitude: f32, airspeed: f32) -> FlightState {
        FlightState {
            position: Vec3::new(0.0, altitude, 0.0),
            velocity: Vec3::new(airspeed, 0.0, 0.0),
            attitude: Quat::from_rotation_x(-FRAC_PI_2),
            angular_velocity: Vec3::ZERO,
            alpha: 0.0,
            beta: 0.0,
            airspeed,
            altitude,

            consumable_remaining: f32::INFINITY,
        }
    }

    #[test]
    fn positive_elevator_when_below_target() {
        let state = level_state(500.0, 100.0);
        let mut ctrl = AscentController::new(&state, 1000.0);
        let inputs = ctrl.update(
            &state,
            &crate::plane::ControllerContext::empty_for(crate::plane::PlaneId::TEST),
            1.0 / 60.0,
        );
        assert!(inputs.elevator > 0.0, "elevator={}", inputs.elevator);
    }

    #[test]
    fn complete_flag_set_within_threshold() {
        let state = level_state(995.0, 100.0); // 5 m below target, inside 10 m threshold
        let mut ctrl = AscentController::new(&state, 1000.0);
        ctrl.update(
            &state,
            &crate::plane::ControllerContext::empty_for(crate::plane::PlaneId::TEST),
            1.0 / 60.0,
        );
        assert!(
            ctrl.complete,
            "expected complete=true at altitude 995 with target 1000"
        );
    }

    #[test]
    fn not_complete_when_far_below() {
        let state = level_state(800.0, 100.0);
        let mut ctrl = AscentController::new(&state, 1000.0);
        ctrl.update(
            &state,
            &crate::plane::ControllerContext::empty_for(crate::plane::PlaneId::TEST),
            1.0 / 60.0,
        );
        assert!(
            !ctrl.complete,
            "expected complete=false at altitude 800 with target 1000"
        );
    }

    #[test]
    fn outputs_finite_on_first_tick() {
        let state = level_state(500.0, 100.0);
        let mut ctrl = AscentController::new(&state, 1500.0);
        let inputs = ctrl.update(
            &state,
            &crate::plane::ControllerContext::empty_for(crate::plane::PlaneId::TEST),
            1.0 / 60.0,
        );
        assert!(inputs.elevator.is_finite());
        assert!(inputs.aileron.is_finite());
        assert!(inputs.rudder.is_finite());
        assert!(inputs.throttle.is_finite());
    }

    #[test]
    fn complete_latches_and_stays_true() {
        let at_target = level_state(1000.0, 100.0);
        let mut ctrl = AscentController::new(&at_target, 1000.0);
        ctrl.update(
            &at_target,
            &crate::plane::ControllerContext::empty_for(crate::plane::PlaneId::TEST),
            1.0 / 60.0,
        );
        assert!(ctrl.complete);
        // Simulate a temporary dip back below threshold (e.g. phugoid).
        let below = level_state(985.0, 100.0);
        ctrl.update(
            &below,
            &crate::plane::ControllerContext::empty_for(crate::plane::PlaneId::TEST),
            1.0 / 60.0,
        );
        assert!(
            ctrl.complete,
            "complete should remain latched after leaving threshold"
        );
    }
}
