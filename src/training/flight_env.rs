use bevy::math::{Quat, Vec3};

use crate::aerodynamics::compute_aero_forces;
use crate::plane::{ControlInputs, FlightState, PlaneConfig};

pub(crate) fn roll_angle(attitude: Quat) -> f32 {
    let right_world = attitude * Vec3::Y;
    let up_world = attitude * Vec3::Z;
    right_world.y.atan2(up_world.y)
}

pub(crate) fn pitch_angle(attitude: Quat) -> f32 {
    let fwd_world = attitude * Vec3::X;
    fwd_world
        .y
        .atan2((fwd_world.x * fwd_world.x + fwd_world.z * fwd_world.z).sqrt())
}

#[derive(Clone)]
pub(crate) struct Lcg(u64);

impl Lcg {
    pub(crate) fn new(seed: u64) -> Self {
        Self(seed)
    }

    /// Return a value in [min, max).
    pub(crate) fn next_f32(&mut self, min: f32, max: f32) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let frac = ((self.0 >> 32) as u32 as f32) / (u32::MAX as f32);
        min + frac * (max - min)
    }
}

pub(crate) fn integrate_state(
    state: &mut FlightState,
    inputs: &ControlInputs,
    cfg: &PlaneConfig,
    dt: f32,
) {
    let aero = compute_aero_forces(state, inputs, cfg);

    // Linear dynamics (world frame).
    let gravity_world = Vec3::new(0.0, -9.81 * cfg.mass, 0.0);
    let force_world = state.attitude * aero.force_body + gravity_world;
    state.velocity += (force_world / cfg.mass) * dt;
    state.position += state.velocity * dt;

    // Angular dynamics (body frame, diagonal inertia).
    let i = cfg.inertia;
    let tau = aero.torque_body;
    state.angular_velocity += Vec3::new(tau.x / i.x, tau.y / i.y, tau.z / i.z) * dt;

    // Attitude integration (body-to-world quaternion):
    // dq/dt = 0.5 * q * [omega, 0] for body-frame angular velocity.
    let p = state.angular_velocity.x;
    let q = state.angular_velocity.y;
    let r = state.angular_velocity.z;
    let (ax, ay, az, aw) = (
        state.attitude.x,
        state.attitude.y,
        state.attitude.z,
        state.attitude.w,
    );
    let dax = 0.5 * (aw * p + ay * r - az * q);
    let day = 0.5 * (aw * q + az * p - ax * r);
    let daz = 0.5 * (aw * r + ax * q - ay * p);
    let daw = 0.5 * (-ax * p - ay * q - az * r);
    state.attitude =
        Quat::from_xyzw(ax + dax * dt, ay + day * dt, az + daz * dt, aw + daw * dt).normalize();

    state.update_air_data();
}

pub(crate) fn direct_action_to_inputs(action: &[f32]) -> ControlInputs {
    // action = [elevator, throttle_norm, aileron, rudder], each in [-1, 1]
    let elevator = action.first().copied().unwrap_or(0.0);
    let throttle_raw = action.get(1).copied().unwrap_or(0.0);
    let aileron = action.get(2).copied().unwrap_or(0.0);
    let rudder = action.get(3).copied().unwrap_or(0.0);
    let mut inputs = ControlInputs {
        elevator,
        throttle: (throttle_raw + 1.0) / 2.0,
        aileron,
        rudder,
    };
    inputs.clamp();
    inputs
}

/// Inverse of [`direct_action_to_inputs`]: map physical `ControlInputs` back to a
/// normalized direct-action vector `[elevator, throttle_norm, aileron, rudder]` in [-1, 1].
///
/// Required for supervised / behavior-cloning targets built from PID outputs: training a
/// network on raw `ControlInputs.throttle` (in [0, 1]) would be double-mapped by
/// `direct_action_to_inputs` at inference (e.g. 0.2 → 0.6). The throttle channel must be
/// pre-converted via `throttle * 2 - 1`.
///
/// Consumed by the behavior-cloning collector (`training::bc`) to turn PID
/// `ControlInputs` into supervised action targets.
pub(crate) fn inputs_to_direct_action(inputs: &ControlInputs) -> [f32; 4] {
    [
        inputs.elevator,
        inputs.throttle * 2.0 - 1.0,
        inputs.aileron,
        inputs.rudder,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn direct_action_round_trips_through_inverse() {
        // throttle in [0, 1], other channels in [-1, 1].
        let cases = [
            ControlInputs {
                elevator: 0.0,
                throttle: 0.0,
                aileron: 0.0,
                rudder: 0.0,
            },
            ControlInputs {
                elevator: 0.3,
                throttle: 0.2,
                aileron: -0.7,
                rudder: 0.5,
            },
            ControlInputs {
                elevator: -1.0,
                throttle: 1.0,
                aileron: 1.0,
                rudder: -1.0,
            },
            ControlInputs {
                elevator: -0.4,
                throttle: 0.65,
                aileron: 0.9,
                rudder: -0.2,
            },
        ];

        for expected in cases {
            let action = inputs_to_direct_action(&expected);
            let got = direct_action_to_inputs(&action);
            assert!((got.elevator - expected.elevator).abs() < 1e-6);
            assert!((got.throttle - expected.throttle).abs() < 1e-6);
            assert!((got.aileron - expected.aileron).abs() < 1e-6);
            assert!((got.rudder - expected.rudder).abs() < 1e-6);
        }
    }
}
