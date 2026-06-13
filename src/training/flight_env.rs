use bevy::math::{Quat, Vec3};

use crate::aerodynamics::{compute_aero_forces, engine_thrust};
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
    // Thrust for this step, evaluated on the incoming state (same altitude as the aero
    // forces above), so the fuel burned matches the thrust actually produced.
    let thrust = engine_thrust(state, inputs, cfg);

    // Linear dynamics (world frame). Effective mass = empty + remaining fuel for jets
    // (the airframe gets lighter as it burns); constant for electric.
    let mass = cfg
        .powerplant
        .effective_mass(cfg.mass, state.consumable_remaining);
    let gravity_world = Vec3::new(0.0, -9.81 * mass, 0.0);
    let force_world = state.attitude * aero.force_body + gravity_world;
    state.velocity += (force_world / mass) * dt;
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

    // Consume fuel/charge for the thrust produced this step (clamped at empty). A
    // non-finite (unmodelled) tank stays non-finite, so unfuelled states never deplete.
    state.consumable_remaining =
        (state.consumable_remaining - cfg.powerplant.burn_rate(thrust) * dt).max(0.0);

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
    use crate::aerodynamics::engine_thrust;
    use crate::plane::{FuelType, Powerplant};

    /// Generic jet with an explicit jet powerplant: empty mass 3500 kg, 2000 kg fuel.
    fn jet_cfg() -> PlaneConfig {
        let mut cfg = crate::environment::generic_jet_spawn_config();
        cfg.mass = 3500.0;
        cfg.thrust_max = 60000.0;
        cfg.powerplant = Powerplant::JetFuel {
            capacity_kg: 2000.0,
            tsfc: 2.0e-5,
            fuel_type: FuelType::JetA,
        };
        cfg
    }

    fn level_state(consumable: f32) -> FlightState {
        let mut s = FlightState {
            velocity: Vec3::new(100.0, 0.0, 0.0),
            attitude: Quat::IDENTITY,
            consumable_remaining: consumable,
            ..Default::default()
        };
        s.update_air_data(); // altitude 0, airspeed 100
        s
    }

    #[test]
    fn jet_burns_fuel_and_loses_mass() {
        let cfg = jet_cfg();
        let inputs = ControlInputs {
            throttle: 1.0,
            ..Default::default()
        };
        let mut state = level_state(2000.0);
        let dt = 1.0 / 60.0;

        // Thrust used for this step (computed on the pre-integration state).
        let thrust = engine_thrust(&state, &inputs, &cfg);
        let m0 = cfg
            .powerplant
            .effective_mass(cfg.mass, state.consumable_remaining);

        integrate_state(&mut state, &inputs, &cfg, dt);

        let expected_burn = cfg.powerplant.burn_rate(thrust) * dt;
        let actual_burn = 2000.0 - state.consumable_remaining;
        assert!(expected_burn > 0.0);
        assert!(
            (actual_burn - expected_burn).abs() < 1e-3,
            "actual burn {actual_burn} != expected {expected_burn}"
        );
        let m1 = cfg
            .powerplant
            .effective_mass(cfg.mass, state.consumable_remaining);
        assert!(
            m1 < m0,
            "effective mass should drop as fuel burns: {m1} !< {m0}"
        );
    }

    #[test]
    fn electric_drains_charge_at_constant_mass() {
        let mut cfg = jet_cfg();
        cfg.powerplant = Powerplant::Electric {
            capacity: 100.0,
            consumption: 1.0e-5,
        };
        let inputs = ControlInputs {
            throttle: 1.0,
            ..Default::default()
        };
        let mut state = level_state(100.0);

        let m0 = cfg
            .powerplant
            .effective_mass(cfg.mass, state.consumable_remaining);
        integrate_state(&mut state, &inputs, &cfg, 1.0 / 60.0);

        assert!(state.consumable_remaining < 100.0, "charge should deplete");
        let m1 = cfg
            .powerplant
            .effective_mass(cfg.mass, state.consumable_remaining);
        assert_eq!(m0, cfg.mass, "electric mass is the airframe mass");
        assert_eq!(
            m1, cfg.mass,
            "electric mass stays constant as charge drains"
        );
    }

    #[test]
    fn fuel_depletes_then_flames_out() {
        let mut cfg = jet_cfg();
        // Tiny tank: ~0.05 kg drains in a fraction of a second at full throttle.
        cfg.powerplant = Powerplant::JetFuel {
            capacity_kg: 0.05,
            tsfc: 2.0e-5,
            fuel_type: FuelType::JetA,
        };
        let inputs = ControlInputs {
            throttle: 1.0,
            ..Default::default()
        };
        let mut state = level_state(0.05);

        for _ in 0..600 {
            integrate_state(&mut state, &inputs, &cfg, 1.0 / 60.0);
        }
        assert_eq!(state.consumable_remaining, 0.0, "tank should be empty");
        assert_eq!(
            engine_thrust(&state, &inputs, &cfg),
            0.0,
            "engine should be flamed out when empty"
        );
    }

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
