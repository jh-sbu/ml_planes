//! FlightState — 6-DOF state for a rigid plane entity.
//!
//! Body-frame convention:
//!   +X = forward (nose)
//!   +Y = right wing
//!   +Z = up through cockpit

use bevy::ecs::component::Component;
use bevy::math::{Quat, Vec3};

/// Normalization scale for the remaining-consumable RL observation term. Matches the
/// generic jet's fuel capacity so a full tank reads as 1.0; both the training
/// environments and the inference controllers divide by this exact constant, so the
/// observation is identical on both sides regardless of the airframe's actual capacity.
pub const FUEL_OBS_SCALE: f32 = 2000.0;

/// Full 6-DOF kinematic state of a plane, updated each physics tick.
#[derive(Component, Debug, Clone)]
#[cfg_attr(feature = "net", derive(serde::Serialize, serde::Deserialize))]
pub struct FlightState {
    pub position: Vec3,         // world frame [m]
    pub velocity: Vec3,         // world frame [m/s]
    pub attitude: Quat,         // body→world rotation  (matches Bevy Transform.rotation)
    pub angular_velocity: Vec3, // body frame [rad/s]  p, q, r
    pub alpha: f32,             // angle of attack [rad]
    pub beta: f32,              // sideslip angle [rad]
    pub airspeed: f32,          // |velocity| [m/s]
    pub altitude: f32,          // position.y [m]
    /// Remaining consumable: fuel mass [kg] for jet powerplants, charge [kWh] for
    /// electric. Interpreted via the plane's [`Powerplant`](crate::plane::Powerplant).
    ///
    /// Defaults to `f32::INFINITY` = "unmodelled / unlimited tank": such a state burns
    /// nothing, never flames out, and adds no fuel mass, so code paths that never opt
    /// into the fuel model (most tests, ad-hoc states) behave exactly as before.
    /// Spawn/reset assigns a finite capacity to enable the fuel model.
    pub consumable_remaining: f32,
}

impl Default for FlightState {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            attitude: Quat::IDENTITY,
            angular_velocity: Vec3::ZERO,
            alpha: 0.0,
            beta: 0.0,
            airspeed: 0.0,
            altitude: 0.0,
            consumable_remaining: f32::INFINITY,
        }
    }
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

        self.alpha = (-v_body.z).atan2(v_body.x);
        self.beta = (v_body.y / self.airspeed).asin();
    }

    /// Normalized remaining-consumable observation term in [0, 1] for RL policies.
    /// An unmodelled (infinite) tank reads as full (1.0). Shared by the training
    /// environments and the inference controllers via [`FUEL_OBS_SCALE`].
    pub fn fuel_fraction_obs(&self) -> f32 {
        (self.consumable_remaining / FUEL_OBS_SCALE).clamp(0.0, 1.0)
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
                                           // velocity in XZ plane tilted 10° below X axis — nose 10° above velocity vector (standard positive AoA)
        let mut s = default_state();
        s.velocity = Vec3::new(angle.cos(), 0.0, -angle.sin()) * 100.0;
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
