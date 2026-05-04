use std::f32::consts::FRAC_PI_3;

use crate::controllers::level_hold::LevelHoldController;
use crate::controllers::pid::PidController;
use crate::controllers::traits::FlightController;
use crate::controllers::tuning::OrbitTuning;
use crate::plane::{ControlInputs, FlightState};

const G: f32 = 9.81;

/// Orbit direction as seen from above (+Y up, XZ horizontal).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum OrbitDirection {
    /// Right-hand turn (bank right). Center is to the right of the plane's heading.
    Clockwise,
    /// Left-hand turn (bank left). Center is to the left of the plane's heading.
    CounterClockwise,
}

impl OrbitDirection {
    /// +1.0 for CW (right bank), -1.0 for CCW (left bank).
    pub fn sign(self) -> f32 {
        match self {
            OrbitDirection::Clockwise => 1.0,
            OrbitDirection::CounterClockwise => -1.0,
        }
    }
}

pub(crate) const ORBIT_OBS_DIM: usize = 12;

#[derive(Clone, Copy, Debug)]
pub(crate) struct OrbitObservationTerms {
    pub radial_error: f32,
    pub heading_error: f32,
    pub bank_ff: f32,
}

pub(crate) fn orbit_observation_terms(
    state: &FlightState,
    center_x: f32,
    center_z: f32,
    target_radius: f32,
    direction: OrbitDirection,
) -> OrbitObservationTerms {
    let rx = state.position.x - center_x;
    let rz = state.position.z - center_z;
    let current_radius = (rx * rx + rz * rz).sqrt();
    let safe_radius = current_radius.max(1.0);

    let (tang_x, tang_z) = match direction {
        OrbitDirection::CounterClockwise => (-rz / safe_radius, rx / safe_radius),
        OrbitDirection::Clockwise => (rz / safe_radius, -rx / safe_radius),
    };

    let speed_xz = (state.velocity.x.powi(2) + state.velocity.z.powi(2)).sqrt();
    let (head_x, head_z) = if speed_xz > 1.0 {
        (state.velocity.x / speed_xz, state.velocity.z / speed_xz)
    } else {
        (tang_x, tang_z)
    };

    let cross = tang_x * head_z - tang_z * head_x;
    let dot = tang_x * head_x + tang_z * head_z;
    let heading_error = cross.atan2(dot);
    let radius = target_radius.max(1.0);
    let bank_ff = direction.sign() * (state.airspeed.powi(2) / (G * radius)).atan();

    OrbitObservationTerms {
        radial_error: current_radius - target_radius,
        heading_error,
        bank_ff,
    }
}

pub(crate) fn build_orbit_observation(
    state: &FlightState,
    center_x: f32,
    center_z: f32,
    target_radius: f32,
    target_altitude: f32,
    target_airspeed: f32,
    direction: OrbitDirection,
) -> Vec<f32> {
    let terms = orbit_observation_terms(state, center_x, center_z, target_radius, direction);
    let alt_err = state.altitude - target_altitude;
    let speed_err = state.airspeed - target_airspeed;
    let roll = roll_angle(state.attitude);
    let pitch = pitch_angle(state.attitude);
    let p = state.angular_velocity.x;
    let q = state.angular_velocity.y;
    let r = state.angular_velocity.z;

    vec![
        terms.radial_error / 500.0,
        terms.heading_error / 0.5,
        terms.bank_ff / FRAC_PI_3,
        alt_err / 200.0,
        speed_err / 50.0,
        state.alpha / 0.5,
        pitch / 0.5,
        q / 1.0,
        roll / 0.5,
        p / 1.0,
        state.beta / 0.5,
        r / 1.0,
    ]
}

fn roll_angle(attitude: bevy::math::Quat) -> f32 {
    let right_world = attitude * bevy::math::Vec3::Y;
    let up_world = attitude * bevy::math::Vec3::Z;
    right_world.y.atan2(up_world.y)
}

fn pitch_angle(attitude: bevy::math::Quat) -> f32 {
    let fwd_world = attitude * bevy::math::Vec3::X;
    fwd_world
        .y
        .atan2((fwd_world.x * fwd_world.x + fwd_world.z * fwd_world.z).sqrt())
}

/// PID-based circular orbit controller around a fixed world-frame point.
///
/// Guidance cascade:
///   1. Radial error [m] → `radial_pid` → heading offset [rad]
///   2. Heading error [rad] → `heading_pid` → Δbank [rad]
///   3. Curvature feedforward: `atan(V² / (g·R)) · direction_sign`
///   4. Total bank = feedforward + Δbank, clamped ±60°, set on inner controller
pub struct OrbitController {
    /// World-frame X coordinate of the orbit center [m].
    pub center_x: f32,
    /// World-frame Z coordinate of the orbit center [m].
    pub center_z: f32,
    /// Desired orbit radius [m].
    pub target_radius: f32,
    /// Target altitude [m].
    pub target_altitude: f32,
    /// Target airspeed [m/s].
    pub target_airspeed: f32,
    /// Orbit direction (CW = right-hand, CCW = left-hand).
    pub direction: OrbitDirection,
    /// Radial error [m] → heading rotation offset [rad]. kp=0.002, output ±0.5 rad.
    pub radial_pid: PidController,
    /// Heading error [rad] → Δbank correction [rad]. kp=0.7, output ±FRAC_PI_3.
    pub heading_pid: PidController,
    /// Inner stabilization controller. target_altitude, target_airspeed, and target_roll
    /// are overwritten each tick by the guidance law.
    pub inner: LevelHoldController,
}

impl OrbitController {
    const DEFAULT_RADIUS: f32 = 1000.0;

    /// Construct with bumpless engagement, applying per-plane tuning gains.
    ///
    /// Calls `from_state` for geometry and bank feedforward pre-seeding, then
    /// overrides PID gains and rebuilds the inner controller with tuned gains.
    pub fn with_tuning(
        state: &FlightState,
        tuning: &OrbitTuning,
        prev_inputs: &ControlInputs,
    ) -> Self {
        let mut ctrl = Self::from_state(state, prev_inputs);
        ctrl.radial_pid.kp = tuning.radial_kp;
        ctrl.radial_pid.kd = tuning.radial_kd;
        ctrl.heading_pid.kp = tuning.heading_kp;
        ctrl.heading_pid.kd = tuning.heading_kd;
        // Rebuild inner with tuned level-hold gains; preserve bank feedforward already seeded.
        let target_roll = ctrl.inner.target_roll;
        ctrl.inner = LevelHoldController::with_tuning(state, &tuning.inner, prev_inputs);
        ctrl.inner.target_altitude = ctrl.target_altitude;
        ctrl.inner.target_airspeed = ctrl.target_airspeed;
        ctrl.inner.target_roll = target_roll;
        ctrl
    }

    /// Construct with bumpless engagement from the current flight state.
    ///
    /// The orbit center is auto-placed perpendicular to the current velocity so the
    /// plane is already tangent to its starting circle — heading_error ≈ 0 at tick 1.
    pub fn from_state(state: &FlightState, prev_inputs: &ControlInputs) -> Self {
        let direction = OrbitDirection::CounterClockwise;
        let radius = Self::DEFAULT_RADIUS;

        let speed_xz = (state.velocity.x.powi(2) + state.velocity.z.powi(2)).sqrt();
        let (hx, hz) = if speed_xz > 1.0 {
            (state.velocity.x / speed_xz, state.velocity.z / speed_xz)
        } else {
            (1.0_f32, 0.0_f32)
        };

        // Center is perpendicular to heading: left for CCW, right for CW.
        let (perp_x, perp_z) = match direction {
            OrbitDirection::CounterClockwise => (-hz, hx),
            OrbitDirection::Clockwise => (hz, -hx),
        };
        let center_x = state.position.x + perp_x * radius;
        let center_z = state.position.z + perp_z * radius;

        let mut inner = LevelHoldController::from_state(state, prev_inputs);
        inner.target_altitude = state.altitude;
        inner.target_airspeed = state.airspeed;
        // Pre-seed bank so the roll PID sees no step command on the first tick.
        let bank_ff = direction.sign() * (state.airspeed.powi(2) / (G * radius)).atan();
        inner.target_roll = bank_ff.clamp(-FRAC_PI_3, FRAC_PI_3);

        Self {
            center_x,
            center_z,
            target_radius: radius,
            target_altitude: state.altitude,
            target_airspeed: state.airspeed,
            direction,
            radial_pid: PidController::new(0.002, 0.0, 0.01, 0.0, -0.5, 0.5),
            heading_pid: PidController::new(0.7, 0.0, 0.1, 0.0, -FRAC_PI_3, FRAC_PI_3),
            inner,
        }
    }
}

impl FlightController for OrbitController {
    fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs {
        let rx = state.position.x - self.center_x;
        let rz = state.position.z - self.center_z;
        let current_radius = (rx * rx + rz * rz).sqrt();

        // Guard: at the orbit center geometry is undefined; hold altitude and pass through.
        if current_radius < 1.0 {
            self.inner.target_altitude = self.target_altitude;
            self.inner.target_airspeed = self.target_airspeed;
            return self.inner.update(state, dt);
        }

        // Tangent direction (unit vector perpendicular to radial, in XZ plane).
        // CCW (viewed from +Y): (-rz, rx) / r
        // CW:                   ( rz, -rx) / r
        let (tang_x, tang_z) = match self.direction {
            OrbitDirection::CounterClockwise => (-rz / current_radius, rx / current_radius),
            OrbitDirection::Clockwise => (rz / current_radius, -rx / current_radius),
        };

        // Radial error > 0: too far out; < 0: too close.
        let radial_error = current_radius - self.target_radius;
        let correction = self.radial_pid.update(radial_error, dt);

        // Rotate tangent to steer toward the circle.
        // Positive radial error needs inward steering: rotate tangent inward by
        // (correction) for CCW (dir_sign=-1) or (-correction) for CW (dir_sign=+1).
        let rotation_angle = -self.direction.sign() * correction;
        let (sin_a, cos_a) = rotation_angle.sin_cos();
        let desired_x = cos_a * tang_x - sin_a * tang_z;
        let desired_z = sin_a * tang_x + cos_a * tang_z;

        // Current heading from velocity (fall back to desired if near-zero airspeed).
        let speed_xz = (state.velocity.x.powi(2) + state.velocity.z.powi(2)).sqrt();
        let (head_x, head_z) = if speed_xz > 1.0 {
            (state.velocity.x / speed_xz, state.velocity.z / speed_xz)
        } else {
            (desired_x, desired_z)
        };

        // Signed heading error: positive = head is CCW (left) of desired.
        let cross = desired_x * head_z - desired_z * head_x;
        let dot = desired_x * head_x + desired_z * head_z;
        let heading_error = cross.atan2(dot);

        // Curvature feedforward holds steady orbit bank; PID corrects perturbations.
        let bank_ff =
            self.direction.sign() * (state.airspeed.powi(2) / (G * self.target_radius)).atan();
        let heading_correction = self.heading_pid.update(heading_error, dt);
        let target_roll = (bank_ff + heading_correction).clamp(-FRAC_PI_3, FRAC_PI_3);

        self.inner.target_roll = target_roll;
        self.inner.target_altitude = self.target_altitude;
        self.inner.target_airspeed = self.target_airspeed;
        self.inner.update(state, dt)
    }

    fn name(&self) -> &'static str {
        "Orbit"
    }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::{Quat, Vec3};
    use std::f32::consts::FRAC_PI_2;

    /// Body +X (nose) → world +X; body +Z (up) → world +Y; body +Y (right wing) → world -Z.
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

    const V: f32 = 100.0;
    const R: f32 = 1000.0;
    const ALT: f32 = 1000.0;

    /// Build an OrbitController with an explicit center/radius/direction, bypassing auto-center.
    fn make_orbit(
        center_x: f32,
        center_z: f32,
        radius: f32,
        direction: OrbitDirection,
    ) -> OrbitController {
        let seed_state = make_state(Vec3::new(0.0, ALT, 0.0), Vec3::new(V, 0.0, 0.0));
        let mut ctrl = OrbitController::from_state(&seed_state, &ControlInputs::default());
        ctrl.center_x = center_x;
        ctrl.center_z = center_z;
        ctrl.target_radius = radius;
        ctrl.direction = direction;
        ctrl.target_altitude = ALT;
        ctrl.target_airspeed = V;
        ctrl.radial_pid.reset();
        ctrl.heading_pid.reset();
        ctrl
    }

    #[test]
    fn tangent_ccw_at_pos_x_radial() {
        // Plane at radial (R, 0) from center → CCW tangent should be (0, +1).
        let (rx, rz, r) = (R, 0.0_f32, R);
        let (tang_x, tang_z) = (-rz / r, rx / r);
        assert!((tang_x - 0.0).abs() < 1e-5, "tang_x={tang_x}");
        assert!((tang_z - 1.0).abs() < 1e-5, "tang_z={tang_z}");
    }

    #[test]
    fn tangent_cw_at_pos_x_radial() {
        // Plane at radial (R, 0) from center → CW tangent should be (0, -1).
        let (rx, rz, r) = (R, 0.0_f32, R);
        let (tang_x, tang_z) = (rz / r, -rx / r);
        assert!((tang_x - 0.0).abs() < 1e-5, "tang_x={tang_x}");
        assert!((tang_z - (-1.0)).abs() < 1e-5, "tang_z={tang_z}");
    }

    #[test]
    fn zero_error_produces_bank_ff() {
        // CCW orbit: plane at (0, -R) in XZ from center → heading +X is the CCW tangent.
        // radial_error = 0, heading_error = 0 → inner.target_roll ≈ bank_ff.
        let mut ctrl = make_orbit(0.0, 0.0, R, OrbitDirection::CounterClockwise);
        let state = make_state(Vec3::new(0.0, ALT, -R), Vec3::new(V, 0.0, 0.0));
        ctrl.update(&state, 1.0 / 60.0);
        let expected = OrbitDirection::CounterClockwise.sign() * (V * V / (G * R)).atan();
        assert!(
            (ctrl.inner.target_roll - expected).abs() < 0.05,
            "inner.target_roll={} expected≈{expected}",
            ctrl.inner.target_roll
        );
    }

    #[test]
    fn positive_radial_error_steers_inward() {
        // CCW orbit: plane 100 m outside the circle. Guidance should command more left bank
        // (inner.target_roll more negative than bank_ff alone).
        let extra = 100.0;
        let mut ctrl = make_orbit(0.0, 0.0, R, OrbitDirection::CounterClockwise);
        // Plane at (0, -(R+extra)): radial = (0, -(R+extra)), CCW tangent = (+1, 0) = heading +X.
        let state = make_state(Vec3::new(0.0, ALT, -(R + extra)), Vec3::new(V, 0.0, 0.0));
        ctrl.update(&state, 1.0 / 60.0);
        let bank_ff = OrbitDirection::CounterClockwise.sign() * (V * V / (G * R)).atan();
        assert!(
            ctrl.inner.target_roll < bank_ff,
            "inner.target_roll={} should be < bank_ff={bank_ff} (more left bank)",
            ctrl.inner.target_roll
        );
    }

    #[test]
    fn direction_sign_reverses_roll() {
        // Both planes on their respective circles with zero heading error.
        // CCW: plane at (0, -R) → roll negative; CW: plane at (0, +R) → roll positive.
        let mut ctrl_ccw = make_orbit(0.0, 0.0, R, OrbitDirection::CounterClockwise);
        let mut ctrl_cw = make_orbit(0.0, 0.0, R, OrbitDirection::Clockwise);

        // CCW: radial=(0,-R), CCW tangent=(+1,0); CW: radial=(0,+R), CW tangent=(+1,0).
        let state_ccw = make_state(Vec3::new(0.0, ALT, -R), Vec3::new(V, 0.0, 0.0));
        let state_cw = make_state(Vec3::new(0.0, ALT, R), Vec3::new(V, 0.0, 0.0));

        ctrl_ccw.update(&state_ccw, 1.0 / 60.0);
        ctrl_cw.update(&state_cw, 1.0 / 60.0);

        assert!(
            ctrl_ccw.inner.target_roll < 0.0,
            "CCW should bank left, got {}",
            ctrl_ccw.inner.target_roll
        );
        assert!(
            ctrl_cw.inner.target_roll > 0.0,
            "CW should bank right, got {}",
            ctrl_cw.inner.target_roll
        );
        assert!(
            (ctrl_ccw.inner.target_roll + ctrl_cw.inner.target_roll).abs() < 0.05,
            "CW and CCW rolls should be symmetric: {} vs {}",
            ctrl_cw.inner.target_roll,
            ctrl_ccw.inner.target_roll
        );
    }

    #[test]
    fn direction_sign_values_match_bank_convention() {
        assert_eq!(OrbitDirection::Clockwise.sign(), 1.0);
        assert_eq!(OrbitDirection::CounterClockwise.sign(), -1.0);
    }

    #[test]
    fn orbit_observation_geometry_zero_error() {
        let state = make_state(Vec3::new(0.0, ALT, -R), Vec3::new(V, 0.0, 0.0));
        let obs = build_orbit_observation(
            &state,
            0.0,
            0.0,
            R,
            ALT,
            V,
            OrbitDirection::CounterClockwise,
        );
        assert_eq!(obs.len(), ORBIT_OBS_DIM);
        assert!(
            obs.iter().all(|v| v.is_finite()),
            "obs contains NaN/inf: {obs:?}"
        );
        assert!(obs[0].abs() < 1e-5, "radial obs={}", obs[0]);
        assert!(obs[1].abs() < 1e-5, "heading obs={}", obs[1]);
        assert!(
            obs[2] < 0.0,
            "CCW bank feedforward should be negative, got {}",
            obs[2]
        );
    }

    #[test]
    fn orbit_observation_direction_reverses_bank_ff() {
        let state_ccw = make_state(Vec3::new(0.0, ALT, -R), Vec3::new(V, 0.0, 0.0));
        let state_cw = make_state(Vec3::new(0.0, ALT, R), Vec3::new(V, 0.0, 0.0));
        let obs_ccw = build_orbit_observation(
            &state_ccw,
            0.0,
            0.0,
            R,
            ALT,
            V,
            OrbitDirection::CounterClockwise,
        );
        let obs_cw =
            build_orbit_observation(&state_cw, 0.0, 0.0, R, ALT, V, OrbitDirection::Clockwise);
        assert!(
            (obs_ccw[2] + obs_cw[2]).abs() < 1e-5,
            "bank feedforward should be symmetric"
        );
    }
}
