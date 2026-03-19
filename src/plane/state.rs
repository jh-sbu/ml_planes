//! FlightState — 6-DOF state for a rigid plane entity.
//!
//! Body-frame convention:
//!   +X = forward (nose)
//!   +Y = right wing
//!   +Z = up through cockpit

use bevy::math::{Quat, Vec3};
use bevy::ecs::component::Component;

/// Full 6-DOF kinematic state of a plane, updated each physics tick.
#[derive(Component, Default, Debug, Clone)]
pub struct FlightState {
    pub position: Vec3,          // world frame [m]
    pub velocity: Vec3,          // world frame [m/s]
    pub attitude: Quat,          // world→body rotation
    pub angular_velocity: Vec3,  // body frame [rad/s]  p, q, r
    pub alpha: f32,              // angle of attack [rad]
    pub beta: f32,               // sideslip angle [rad]
    pub airspeed: f32,           // |velocity| [m/s]
    pub altitude: f32,           // position.y [m]
}

impl FlightState {
    /// Recompute derived air-data quantities from position/velocity/attitude.
    ///
    /// Signs:
    ///   alpha positive  → nose above velocity vector (climb)
    ///   beta  positive  → wind from right (+Y body)
    pub fn update_air_data(&mut self) {
        self.altitude = self.position.y;
        self.airspeed = self.velocity.length();

        if self.airspeed < 1e-3 {
            self.alpha = 0.0;
            self.beta = 0.0;
            return;
        }

        // Rotate velocity from world frame into body frame.
        let v_body = self.attitude.conjugate().mul_vec3(self.velocity);

        self.alpha = v_body.z.atan2(v_body.x);
        self.beta = (v_body.y / self.airspeed).asin();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    fn default_state() -> FlightState {
        FlightState {
            attitude: Quat::IDENTITY,
            ..Default::default()
        }
    }

    #[test]
    fn pure_forward_velocity() {
        let mut s = default_state();
        s.velocity = Vec3::X * 100.0;
        s.update_air_data();
        assert!((s.airspeed - 100.0).abs() < 1e-4, "airspeed={}", s.airspeed);
        assert!(s.alpha.abs() < 1e-4, "alpha={}", s.alpha);
        assert!(s.beta.abs() < 1e-4, "beta={}", s.beta);
    }

    #[test]
    fn ten_deg_nose_up() {
        let angle = 10.0_f32 * PI / 180.0; // 0.17453 rad
        // velocity in XZ plane tilted 10° above X axis
        let mut s = default_state();
        s.velocity = Vec3::new(angle.cos(), 0.0, angle.sin()) * 100.0;
        s.update_air_data();
        assert!(
            (s.alpha - angle).abs() < 1e-4,
            "alpha={} expected={}",
            s.alpha,
            angle
        );
        assert!(s.beta.abs() < 1e-4, "beta={}", s.beta);
    }

    #[test]
    fn near_zero_airspeed() {
        let mut s = default_state();
        s.velocity = Vec3::ZERO;
        s.update_air_data(); // must not panic or produce NaN
        assert!(!s.alpha.is_nan(), "alpha is NaN");
        assert!(!s.beta.is_nan(), "beta is NaN");
        assert_eq!(s.alpha, 0.0);
        assert_eq!(s.beta, 0.0);
    }
}
