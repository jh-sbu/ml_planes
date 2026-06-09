//! Shared lateral-guidance primitives used by the orbit and L1 controllers.
//!
//! Both functions are pure (aside from advancing the supplied PID integrators)
//! and output a **target bank angle** [rad] in the body-axis convention used by
//! [`LevelHoldController`](crate::controllers::level_hold::LevelHoldController):
//! positive roll is a left bank. All horizontal geometry is world-frame XZ.

use std::f32::consts::{FRAC_PI_3, PI};

use bevy::math::Vec2;

use crate::controllers::orbit::OrbitDirection;
use crate::controllers::pid::PidController;
use crate::plane::FlightState;

const G: f32 = 9.81;

/// 2-D cross product `u × v` (scalar), with `Vec2` mapping `x→X`, `y→Z`.
#[inline]
fn cross2(u: Vec2, v: Vec2) -> f32 {
    u.x * v.y - u.y * v.x
}

/// Horizontal ground-track unit vector from velocity, falling back to body
/// forward (then `+X`) below ~1 m/s. Mirrors the convention in the orbit and
/// level-hold controllers.
pub(crate) fn ground_heading(state: &FlightState) -> Vec2 {
    let speed_xz = (state.velocity.x.powi(2) + state.velocity.z.powi(2)).sqrt();
    if speed_xz > 1.0 {
        Vec2::new(state.velocity.x / speed_xz, state.velocity.z / speed_xz)
    } else {
        let fwd = state.attitude * bevy::math::Vec3::X;
        let len = (fwd.x.powi(2) + fwd.z.powi(2)).sqrt();
        if len > 1e-6 {
            Vec2::new(fwd.x / len, fwd.z / len)
        } else {
            Vec2::new(1.0, 0.0)
        }
    }
}

/// Orbit/loiter target bank [rad] for one tick.
///
/// Reuses the proven cascade: radial-error PID → heading offset, heading-error
/// PID → bank correction, plus the steady centripetal feedforward
/// `-direction.sign() · atan(V² / (g·R))`. Returns `None` when the aircraft is
/// essentially at the orbit center (geometry undefined); the caller should then
/// hold its previous bank command.
pub fn orbit_bank_command(
    state: &FlightState,
    center_x: f32,
    center_z: f32,
    radius: f32,
    direction: OrbitDirection,
    radial_pid: &mut PidController,
    heading_pid: &mut PidController,
    dt: f32,
) -> Option<f32> {
    let rx = state.position.x - center_x;
    let rz = state.position.z - center_z;
    let current_radius = (rx * rx + rz * rz).sqrt();
    if current_radius < 1.0 {
        return None;
    }

    // Tangent (unit vector perpendicular to radial in XZ; visual chirality).
    let (tang_x, tang_z) = match direction {
        OrbitDirection::CounterClockwise => (rz / current_radius, -rx / current_radius),
        OrbitDirection::Clockwise => (-rz / current_radius, rx / current_radius),
    };

    // Radial error rotates the tangent inward/outward to capture the circle.
    let radial_error = current_radius - radius;
    let correction = radial_pid.update(radial_error, dt);
    let rotation_angle = -direction.sign() * correction;
    let (sin_a, cos_a) = rotation_angle.sin_cos();
    let desired_x = cos_a * tang_x - sin_a * tang_z;
    let desired_z = sin_a * tang_x + cos_a * tang_z;

    let head = ground_heading(state);
    let (head_x, head_z) = if (state.velocity.x.powi(2) + state.velocity.z.powi(2)).sqrt() > 1.0 {
        (head.x, head.y)
    } else {
        (desired_x, desired_z)
    };

    let cross = desired_x * head_z - desired_z * head_x;
    let dot = desired_x * head_x + desired_z * head_z;
    let heading_error = cross.atan2(dot);

    let bank_ff = -direction.sign() * (state.airspeed.powi(2) / (G * radius)).atan();
    let heading_correction = heading_pid.update(heading_error, dt);
    Some((bank_ff - heading_correction).clamp(-FRAC_PI_3, FRAC_PI_3))
}

/// Output of the L1 straight-segment guidance law ([`l1_straight_bank`]).
///
/// `eta` and `xtrack` are byproducts of the law, surfaced here so callers (e.g.
/// the HUD) can display tracking error without re-deriving the geometry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct L1Guidance {
    /// Commanded bank angle [rad] (positive = left bank), clamped ±60°.
    pub bank: f32,
    /// L1 angle `η = η1 + η2` [rad]: combined path-following + heading error.
    pub eta: f32,
    /// Signed cross-track error [m] (positive = aircraft on the `+Z` side).
    pub xtrack: f32,
}

/// L1 nonlinear lateral-guidance command for following the straight segment
/// `a → b` (world XZ endpoints).
///
/// `L1 = (1/π) · damping · period · V` is the lookahead distance. The aircraft
/// is steered onto the A→B line and along it via the demanded lateral
/// acceleration `2·V²/L1·sin(η)`, mapped to bank by `atan(a/g)` and clamped to
/// ±60°. With `+Z` downward in the top-down view, a positive cross-track error
/// (aircraft on the `+Z` / screen-down side of the path) commands a left bank
/// (positive roll) that curves it back — verified by the convergence rollout in
/// this module's tests. Returns the bank command alongside the `η` and
/// cross-track diagnostics (see [`L1Guidance`]).
pub fn l1_straight_bank(
    state: &FlightState,
    a: Vec2,
    b: Vec2,
    l1_period: f32,
    l1_damping: f32,
) -> L1Guidance {
    let vg = Vec2::new(state.velocity.x, state.velocity.z);
    let speed = vg.length().max(1.0);
    let l1 = ((1.0 / PI) * l1_damping * l1_period * speed).max(1.0);

    let ab = b - a;
    let path = if ab.length() > 1e-3 {
        ab.normalize()
    } else {
        vg / speed
    };

    let p = Vec2::new(state.position.x, state.position.z) - a;
    let xtrack = cross2(p, path);
    let xtrack_vel = cross2(vg, path);
    let ltrack_vel = vg.dot(path);

    let eta2 = xtrack_vel.atan2(ltrack_vel);
    let sin_eta1 = (xtrack / l1).clamp(-0.7071, 0.7071);
    let eta1 = sin_eta1.asin();
    let eta = eta1 + eta2;

    let lat_accel = 2.0 * speed * speed / l1 * eta.sin();
    let bank = (lat_accel / G).atan().clamp(-FRAC_PI_3, FRAC_PI_3);
    L1Guidance { bank, eta, xtrack }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controllers::level_hold::LevelHoldController;
    use crate::plane::{ControlInputs, PlaneConfig};
    use crate::training::integrate_state;
    use bevy::math::{Quat, Vec3};
    use std::f32::consts::FRAC_PI_2;

    /// Generic-jet plane config (matches `assets/planes/generic_jet.plane.ron`),
    /// duplicated here per the per-test-module convention used across the crate.
    fn jet_cfg() -> PlaneConfig {
        PlaneConfig {
            wing_area: 20.0,
            mean_chord: 2.0,
            wing_span: 10.0,
            mass: 5000.0,
            inertia: Vec3::new(10000.0, 40000.0, 45000.0),
            cl0: 0.1,
            cl_alpha: 4.5,
            cl_delta_e: 0.4,
            cl_max: 1.4,
            cd0: 0.02,
            cd_induced: 0.05,
            cm0: -0.02,
            cm_alpha: 0.6,
            cm_q: -14.0,
            cm_delta_e: -1.2,
            cl_beta: -0.08,
            cl_p: -0.45,
            cl_r: 0.12,
            cl_delta_a: 0.18,
            cn_beta: 0.10,
            cn_r: -0.12,
            cn_delta_r: -0.10,
            thrust_max: 60000.0,
            aileron_limit: 0.4363,
            elevator_limit: 0.3491,
            rudder_limit: 0.2618,
        }
    }

    fn level_attitude_for_heading(vx: f32, vz: f32) -> Quat {
        // Yaw so body +X points along the velocity heading, then pitch level.
        let yaw = (-vz).atan2(vx);
        Quat::from_rotation_y(yaw) * Quat::from_rotation_x(-FRAC_PI_2)
    }

    fn make_state(position: Vec3, velocity: Vec3) -> FlightState {
        let mut s = FlightState {
            position,
            velocity,
            attitude: level_attitude_for_heading(velocity.x, velocity.z),
            angular_velocity: Vec3::ZERO,
            ..Default::default()
        };
        s.update_air_data();
        s
    }

    #[test]
    fn on_path_aligned_heading_produces_zero_bank() {
        // Path along +X, aircraft on the path heading +X.
        let state = make_state(Vec3::new(0.0, 1000.0, 0.0), Vec3::new(100.0, 0.0, 0.0));
        let g = l1_straight_bank(&state, Vec2::ZERO, Vec2::new(5000.0, 0.0), 20.0, 0.75);
        assert!(
            g.bank.abs() < 1e-3,
            "expected ~0 bank on-path/aligned, got {}",
            g.bank
        );
        // On-path and aligned: both the cross-track error and the L1 angle vanish.
        assert!(
            g.xtrack.abs() < 1e-3,
            "expected ~0 cross-track, got {}",
            g.xtrack
        );
        assert!(g.eta.abs() < 1e-3, "expected ~0 eta, got {}", g.eta);
    }

    #[test]
    fn far_off_path_saturates_bank() {
        // 2 km to the +Z side of a +X path, heading +X: should hard-bank.
        let state = make_state(Vec3::new(0.0, 1000.0, 2000.0), Vec3::new(100.0, 0.0, 0.0));
        let g = l1_straight_bank(&state, Vec2::ZERO, Vec2::new(5000.0, 0.0), 20.0, 0.75);
        assert!(
            g.bank.abs() > 0.5,
            "expected near-saturated bank, got {}",
            g.bank
        );
        // Diagnostics reflect the large offset: ~2 km cross-track, large L1 angle.
        assert!(
            (g.xtrack.abs() - 2000.0).abs() < 1.0,
            "expected ~2000 m cross-track, got {}",
            g.xtrack
        );
        assert!(
            g.eta.abs() > 0.5,
            "expected large eta off-path, got {}",
            g.eta
        );
    }

    /// Closed-loop convergence pins the cross-track sign: starting 300 m off a
    /// +X path, the L1 bank + inner level-hold must drive cross-track toward 0.
    #[test]
    fn l1_converges_onto_path() {
        let cfg = jet_cfg();
        let a = Vec2::ZERO;
        let b = Vec2::new(10000.0, 0.0);

        // Start 300 m off-path on the +Z side, flying +X.
        let mut state = make_state(Vec3::new(0.0, 1000.0, 300.0), Vec3::new(100.0, 0.0, 0.0));
        let mut inner = LevelHoldController::from_state(&state, &ControlInputs::default());
        inner.target_altitude = 1000.0;
        inner.target_airspeed = 100.0;

        let dt = 1.0 / 64.0;
        let initial_xtrack = state.position.z.abs();
        let mut min_xtrack = initial_xtrack;

        for _ in 0..3000 {
            inner.target_roll = l1_straight_bank(&state, a, b, 20.0, 0.75).bank;
            let inputs = {
                use crate::controllers::FlightController;
                inner.update(
                    &state,
                    &crate::plane::ControllerContext::empty_for(crate::plane::PlaneId::TEST),
                    dt,
                )
            };
            integrate_state(&mut state, &inputs, &cfg, dt);
            min_xtrack = min_xtrack.min(state.position.z.abs());
            assert!(state.altitude.is_finite() && state.altitude > 500.0);
        }

        assert!(
            min_xtrack < 50.0,
            "L1 should capture the path (min cross-track {min_xtrack:.1} m from {initial_xtrack:.0} m)"
        );
    }
}
