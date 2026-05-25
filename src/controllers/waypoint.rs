//! Waypoint-navigating flight controller — two-phase cascade PID.
//!
//! Phase 1 (Approach): bearing-error outer loop steers toward a 3-D target position.
//! Phase 2 (Orbit):    transitions to a circular orbit around the waypoint once
//!                     the horizontal distance falls below `arrival_radius`.
//!
//! Both phases delegate altitude/airspeed/roll stabilization to an inner
//! [`LevelHoldController`].

use std::f32::consts::FRAC_PI_3;

use bevy::math::Vec3;

use crate::controllers::level_hold::LevelHoldController;
use crate::controllers::orbit::OrbitDirection;
use crate::controllers::pid::PidController;
use crate::controllers::traits::FlightController;
use crate::controllers::tuning::WaypointTuning;
use crate::plane::{ControlInputs, FlightState};

const G: f32 = 9.81;

/// Two-phase flight state for the waypoint controller.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WaypointPhase {
    /// Steering toward the waypoint.
    Approach,
    /// Orbiting around the waypoint.
    Orbit,
}

/// Two-phase PID waypoint controller.
///
/// Flies toward `(target_x, target_z)` at `target_altitude` / `target_airspeed`,
/// then transitions to a circular orbit when within `arrival_radius`.
#[derive(Clone)]
pub struct WaypointController {
    /// World-frame X of the target waypoint [m].
    pub target_x: f32,
    /// World-frame Z of the target waypoint [m].
    pub target_z: f32,
    /// Target altitude [m].
    pub target_altitude: f32,
    /// Target airspeed [m/s].
    pub target_airspeed: f32,
    /// Horizontal distance threshold [m] at which phase transitions to Orbit.
    pub arrival_radius: f32,
    /// Orbit radius once in Orbit phase [m].
    pub orbit_radius: f32,
    /// Orbit direction (default: CounterClockwise).
    pub orbit_direction: OrbitDirection,
    /// Current phase.
    pub phase: WaypointPhase,
    /// Approach: bearing error [rad] → bank command [rad], clamped ±60°.
    pub bearing_pid: PidController,
    /// Orbit: radial error [m] → heading offset [rad], clamped ±0.5 rad.
    pub radial_pid: PidController,
    /// Orbit: heading error [rad] → Δbank [rad], clamped ±60°.
    pub heading_pid: PidController,
    /// Inner stabilization: altitude, airspeed, roll, sideslip.
    pub inner: LevelHoldController,
}

impl WaypointController {
    /// Construct from a target position, capturing current state for a bumpless handoff.
    pub fn from_state(
        state: &FlightState,
        target_x: f32,
        target_z: f32,
        target_altitude: f32,
        target_airspeed: f32,
        prev_inputs: &ControlInputs,
    ) -> Self {
        let mut inner = LevelHoldController::from_state(state, prev_inputs);
        inner.target_altitude = target_altitude;
        inner.target_airspeed = target_airspeed;
        Self {
            target_x,
            target_z,
            target_altitude,
            target_airspeed,
            arrival_radius: 300.0,
            orbit_radius: 500.0,
            orbit_direction: OrbitDirection::CounterClockwise,
            phase: WaypointPhase::Approach,
            bearing_pid: PidController::new(0.7, 0.0, 0.1, 0.0, -FRAC_PI_3, FRAC_PI_3),
            radial_pid: PidController::new(0.002, 0.0, 0.01, 0.0, -0.5, 0.5),
            heading_pid: PidController::new(0.7, 0.0, 0.1, 0.0, -FRAC_PI_3, FRAC_PI_3),
            inner,
        }
    }

    /// Construct with gains from `tuning`, placing a target 1 km ahead of the plane's
    /// current heading as the initial waypoint.
    pub fn with_tuning(
        state: &FlightState,
        tuning: &WaypointTuning,
        target_x: f32,
        target_z: f32,
        target_altitude: f32,
        target_airspeed: f32,
        prev_inputs: &ControlInputs,
    ) -> Self {
        let mut ctrl = Self::from_state(
            state,
            target_x,
            target_z,
            target_altitude,
            target_airspeed,
            prev_inputs,
        );
        ctrl.bearing_pid.kp = tuning.bearing_kp;
        ctrl.bearing_pid.kd = tuning.bearing_kd;
        ctrl.radial_pid.kp = tuning.radial_kp;
        ctrl.radial_pid.kd = tuning.radial_kd;
        ctrl.heading_pid.kp = tuning.heading_kp;
        ctrl.heading_pid.kd = tuning.heading_kd;
        ctrl.inner = LevelHoldController::with_tuning(state, &tuning.inner, prev_inputs);
        ctrl.inner.target_altitude = target_altitude;
        ctrl.inner.target_airspeed = target_airspeed;
        ctrl
    }

    /// Reset all guidance PIDs and return to Approach phase.
    ///
    /// Call after changing `target_x` / `target_z` so the integrals do not
    /// carry stale corrections from the previous target.
    pub fn reset_guidance(&mut self) {
        self.bearing_pid.reset();
        self.radial_pid.reset();
        self.heading_pid.reset();
        self.phase = WaypointPhase::Approach;
    }
}

impl FlightController for WaypointController {
    fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs {
        let dx = self.target_x - state.position.x;
        let dz = self.target_z - state.position.z;
        let horiz_dist = (dx * dx + dz * dz).sqrt();

        // Transition to Orbit when close enough.
        if self.phase == WaypointPhase::Approach && horiz_dist < self.arrival_radius {
            self.phase = WaypointPhase::Orbit;
            // Seed the orbit bank feedforward so the roll PID sees no step command.
            let radius = self.orbit_radius.max(1.0);
            let bank_ff =
                -self.orbit_direction.sign() * (state.airspeed.powi(2) / (G * radius)).atan();
            self.inner.target_roll = bank_ff.clamp(-FRAC_PI_3, FRAC_PI_3);
        }

        match self.phase {
            WaypointPhase::Approach => self.run_approach(state, dt),
            WaypointPhase::Orbit => self.run_orbit(state, dt),
        }
    }

    fn name(&self) -> &'static str {
        "Waypoint"
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

impl WaypointController {
    fn run_approach(&mut self, state: &FlightState, dt: f32) -> ControlInputs {
        let dx = self.target_x - state.position.x;
        let dz = self.target_z - state.position.z;

        // Bearing toward the waypoint in the XZ horizontal plane.
        let target_bearing = dz.atan2(dx);

        // Current heading unit vector from velocity (fall back to body forward at low speed).
        let speed_xz = (state.velocity.x.powi(2) + state.velocity.z.powi(2)).sqrt();
        let (head_x, head_z) = if speed_xz > 1.0 {
            (state.velocity.x / speed_xz, state.velocity.z / speed_xz)
        } else {
            let fwd = state.attitude * Vec3::X;
            let len = (fwd.x.powi(2) + fwd.z.powi(2)).sqrt().max(1e-6);
            (fwd.x / len, fwd.z / len)
        };

        // Target heading unit vector: target_bearing = atan2(z, x).
        let (tgt_z, tgt_x) = target_bearing.sin_cos();

        // Signed heading error: positive = current heading is CCW (left) of target.
        let cross = tgt_x * head_z - tgt_z * head_x;
        let dot = tgt_x * head_x + tgt_z * head_z;
        let heading_error = cross.atan2(dot);

        // Negative correction: positive error (head left of target) → bank right (negative roll).
        let bank_cmd = -self.bearing_pid.update(heading_error, dt);
        self.inner.target_roll = bank_cmd;
        self.inner.target_altitude = self.target_altitude;
        self.inner.target_airspeed = self.target_airspeed;
        self.inner.update(state, dt)
    }

    fn run_orbit(&mut self, state: &FlightState, dt: f32) -> ControlInputs {
        // Identical to OrbitController::update() with center = (target_x, target_z)
        // and target_radius = orbit_radius.
        let rx = state.position.x - self.target_x;
        let rz = state.position.z - self.target_z;
        let current_radius = (rx * rx + rz * rz).sqrt();

        if current_radius < 1.0 {
            self.inner.target_altitude = self.target_altitude;
            self.inner.target_airspeed = self.target_airspeed;
            return self.inner.update(state, dt);
        }

        let (tang_x, tang_z) = match self.orbit_direction {
            OrbitDirection::CounterClockwise => (-rz / current_radius, rx / current_radius),
            OrbitDirection::Clockwise => (rz / current_radius, -rx / current_radius),
        };

        let radial_error = current_radius - self.orbit_radius;
        let correction = self.radial_pid.update(radial_error, dt);

        let rotation_angle = -self.orbit_direction.sign() * correction;
        let (sin_a, cos_a) = rotation_angle.sin_cos();
        let desired_x = cos_a * tang_x - sin_a * tang_z;
        let desired_z = sin_a * tang_x + cos_a * tang_z;

        let speed_xz = (state.velocity.x.powi(2) + state.velocity.z.powi(2)).sqrt();
        let (head_x, head_z) = if speed_xz > 1.0 {
            (state.velocity.x / speed_xz, state.velocity.z / speed_xz)
        } else {
            (desired_x, desired_z)
        };

        let cross = desired_x * head_z - desired_z * head_x;
        let dot = desired_x * head_x + desired_z * head_z;
        let heading_error = cross.atan2(dot);

        let radius = self.orbit_radius.max(1.0);
        let bank_ff = -self.orbit_direction.sign() * (state.airspeed.powi(2) / (G * radius)).atan();
        let heading_correction = self.heading_pid.update(heading_error, dt);
        let target_roll = (bank_ff - heading_correction).clamp(-FRAC_PI_3, FRAC_PI_3);

        self.inner.target_roll = target_roll;
        self.inner.target_altitude = self.target_altitude;
        self.inner.target_airspeed = self.target_airspeed;
        self.inner.update(state, dt)
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

    fn level_attitude() -> Quat {
        Quat::from_rotation_x(-FRAC_PI_2)
    }

    fn make_state(position: Vec3, velocity: Vec3) -> FlightState {
        let airspeed = velocity.length();
        FlightState {
            position,
            velocity,
            attitude: level_attitude(),
            angular_velocity: Vec3::ZERO,
            alpha: 0.0,
            beta: 0.0,
            airspeed,
            altitude: position.y,
        }
    }

    #[test]
    fn from_state_starts_in_approach() {
        let state = make_state(Vec3::new(0.0, 1000.0, 0.0), Vec3::new(80.0, 0.0, 0.0));
        let ctrl = WaypointController::from_state(
            &state,
            5000.0,
            0.0,
            1000.0,
            80.0,
            &ControlInputs::default(),
        );
        assert_eq!(ctrl.phase, WaypointPhase::Approach);
    }

    #[test]
    fn zero_bearing_error_produces_zero_bank() {
        // Plane heading +X, waypoint in +X direction → bearing error = 0.
        let state = make_state(Vec3::new(0.0, 1000.0, 0.0), Vec3::new(80.0, 0.0, 0.0));
        let mut ctrl = WaypointController::from_state(
            &state,
            5000.0, // far ahead in +X
            0.0,
            1000.0,
            80.0,
            &ControlInputs::default(),
        );
        ctrl.update(&state, 1.0 / 64.0);
        assert!(
            ctrl.inner.target_roll.abs() < 0.05,
            "zero bearing error should produce near-zero bank, got {}",
            ctrl.inner.target_roll
        );
    }

    #[test]
    fn reset_guidance_returns_to_approach() {
        let state = make_state(Vec3::new(150.0, 1000.0, 0.0), Vec3::new(80.0, 0.0, 0.0));
        let mut ctrl = WaypointController::from_state(
            &state,
            0.0,
            0.0,
            1000.0,
            80.0,
            &ControlInputs::default(),
        );
        // Force into orbit by transitioning.
        ctrl.update(&state, 1.0 / 64.0);
        // May have transitioned to Orbit since distance ≈ 150 m < 300 m arrival_radius.
        ctrl.reset_guidance();
        assert_eq!(ctrl.phase, WaypointPhase::Approach);
    }
}
