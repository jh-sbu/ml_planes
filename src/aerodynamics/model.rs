use bevy::math::Vec3;
use crate::plane::{PlaneConfig, FlightState, ControlInputs};

const AIR_DENSITY: f32 = 1.225; // kg/m³ at sea level

/// Aerodynamic and thrust forces/torques in the body frame.
#[derive(Debug, Clone, Default)]
pub struct AeroForces {
    pub force_body: Vec3,   // [N]   body frame (+X fwd, +Y right, +Z up)
    pub torque_body: Vec3,  // [N·m] body frame
}

/// Compute aerodynamic forces and torques given the current flight state,
/// control inputs, and plane configuration.
///
/// Returns [`AeroForces::default`] (all zeros) when airspeed is below 1 mm/s
/// to avoid NaN from division by near-zero velocity.
pub fn compute_aero_forces(
    state: &FlightState,
    inputs: &ControlInputs,
    cfg: &PlaneConfig,
) -> AeroForces {
    let v = state.airspeed;
    if v < 1e-3 {
        return AeroForces::default();
    }

    // Dynamic pressure
    let q_bar = 0.5 * AIR_DENSITY * v * v;

    // Control surface deflections [rad]
    let delta_a = inputs.aileron  * cfg.aileron_limit;
    let delta_e = inputs.elevator * cfg.elevator_limit;
    let delta_r = inputs.rudder   * cfg.rudder_limit;

    // Angular rates from body frame
    let p = state.angular_velocity.x; // roll rate
    let q = state.angular_velocity.y; // pitch rate
    let r = state.angular_velocity.z; // yaw rate

    let alpha = state.alpha;
    let beta  = state.beta;
    let b = cfg.wing_span;
    let c = cfg.mean_chord;

    // --- Lift coefficient (clamped at stall) ---
    let cl_raw = cfg.cl0 + cfg.cl_alpha * alpha + cfg.cl_delta_e * delta_e;
    let cl = cl_raw.clamp(-cfg.cl_max, cfg.cl_max);

    // --- Drag coefficient (always positive) ---
    let cd = cfg.cd0 + cfg.cd_induced * cl * cl;

    // --- Forces ---
    let lift   = q_bar * cfg.wing_area * cl;
    let drag   = q_bar * cfg.wing_area * cd;
    let thrust = inputs.throttle * cfg.thrust_max;

    let force_body = Vec3::new(
        thrust - drag,
        0.0,
        lift,
    );

    // --- Moment coefficients ---
    let cm = cfg.cm0
        + cfg.cm_alpha   * alpha
        + cfg.cm_q       * (q * c / (2.0 * v))
        + cfg.cm_delta_e * delta_e;

    // Local name `cl_coef` avoids shadowing the longitudinal `cl` above.
    let cl_coef = cfg.cl_beta    * beta
        + cfg.cl_p       * (p * b / (2.0 * v))
        + cfg.cl_r       * (r * b / (2.0 * v))
        + cfg.cl_delta_a * delta_a;

    let cn = cfg.cn_beta    * beta
        + cfg.cn_r       * (r * b / (2.0 * v))
        + cfg.cn_delta_r * delta_r;

    // --- Moments ---
    let roll_moment  = q_bar * cfg.wing_area * b * cl_coef;
    let pitch_moment = q_bar * cfg.wing_area * c * cm;
    let yaw_moment   = q_bar * cfg.wing_area * b * cn;

    let torque_body = Vec3::new(roll_moment, pitch_moment, yaw_moment);

    AeroForces { force_body, torque_body }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Returns a `PlaneConfig` matching `assets/planes/generic_jet.plane.ron`.
    fn jet_config() -> PlaneConfig {
        PlaneConfig {
            wing_area:      20.0,
            mean_chord:      2.0,
            wing_span:      10.0,
            mass:         5000.0,
            inertia: Vec3::new(10000.0, 40000.0, 45000.0),
            cl0:          0.1,
            cl_alpha:     4.5,
            cl_delta_e:   0.4,
            cl_max:       1.4,
            cd0:          0.02,
            cd_induced:   0.05,
            cm0:         -0.02,
            cm_alpha:    -0.6,
            cm_q:        -8.0,
            cm_delta_e:  -1.2,
            cl_beta:     -0.08,
            cl_p:        -0.45,
            cl_r:         0.12,
            cl_delta_a:   0.18,
            cn_beta:      0.10,
            cn_r:        -0.12,
            cn_delta_r:  -0.10,
            thrust_max:  60000.0,
            aileron_limit:  0.4363,
            elevator_limit: 0.3491,
            rudder_limit:   0.2618,
        }
    }

    fn zero_state() -> FlightState {
        FlightState::default()
    }

    fn zero_inputs() -> ControlInputs {
        ControlInputs::default()
    }

    #[test]
    fn zero_airspeed_returns_zero() {
        let state = zero_state(); // airspeed == 0.0
        let forces = compute_aero_forces(&state, &zero_inputs(), &jet_config());
        assert_eq!(forces.force_body,  Vec3::ZERO);
        assert_eq!(forces.torque_body, Vec3::ZERO);
    }

    #[test]
    fn level_flight_lift_vs_weight() {
        // q_bar = 0.5 * 1.225 * 10000 = 6125
        // CL = 0.1 + 4.5 * 0.0667 ≈ 0.40015
        // lift = 6125 * 20 * 0.40015 ≈ 49018 N  (within 200 N of 49050)
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 100.0;
        state.alpha    = 0.0667;

        let forces = compute_aero_forces(&state, &zero_inputs(), &cfg);
        let lift = forces.force_body.z;
        assert!(
            (lift - 49050.0).abs() < 200.0,
            "lift={lift:.1} expected ≈49050 N"
        );
    }

    #[test]
    fn drag_is_positive() {
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 100.0;
        state.alpha    = 0.05;

        let forces = compute_aero_forces(&state, &zero_inputs(), &cfg);
        // drag = thrust - force_body.x  (thrust=0 here)
        let drag = -forces.force_body.x;
        assert!(drag > 0.0, "drag={drag} should be positive");
    }

    #[test]
    fn aileron_positive_produces_positive_roll_moment() {
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 100.0;
        let mut inputs = zero_inputs();
        inputs.aileron = 1.0;

        let forces = compute_aero_forces(&state, &inputs, &cfg);
        assert!(
            forces.torque_body.x > 0.0,
            "roll moment={} should be positive for aileron=1.0",
            forces.torque_body.x
        );
    }

    #[test]
    fn elevator_up_increases_lift_and_nose_up_pitch() {
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 100.0;
        state.alpha    = 0.05;

        let f0 = compute_aero_forces(&state, &zero_inputs(), &cfg);

        let mut inputs_up = zero_inputs();
        inputs_up.elevator = 1.0;
        let f1 = compute_aero_forces(&state, &inputs_up, &cfg);

        assert!(
            f1.force_body.z > f0.force_body.z,
            "elevator up should increase lift: {:.1} vs {:.1}",
            f1.force_body.z, f0.force_body.z
        );
        // cm_delta_e = -1.2 → more negative Cm → more negative pitch torque (nose-up)
        assert!(
            f1.torque_body.y < f0.torque_body.y,
            "elevator up should produce more negative pitch moment: {:.1} vs {:.1}",
            f1.torque_body.y, f0.torque_body.y
        );
    }

    #[test]
    fn cl_clamped_at_cl_max() {
        // α = 1.0 rad → CL_raw = 0.1 + 4.5 = 4.6 → clamped to 1.4
        // lift = 0.5 * 1.225 * 2500 * 20 * 1.4 = 1531.25 * 20 * 1.4 = 42875 N
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 50.0;
        state.alpha    = 1.0;

        let forces = compute_aero_forces(&state, &zero_inputs(), &cfg);
        let expected_lift = 0.5 * AIR_DENSITY * 50.0f32 * 50.0 * cfg.wing_area * cfg.cl_max;
        assert_eq!(
            forces.force_body.z, expected_lift,
            "lift should equal clamped-CL value exactly"
        );
    }

    #[test]
    fn cm_q_damping_sign() {
        // positive pitch rate q → cm_q*(q*c/2V) negative → Cm more negative → pitch torque < 0
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 100.0;
        state.angular_velocity = Vec3::new(0.0, 1.0, 0.0); // q = 1 rad/s

        let forces = compute_aero_forces(&state, &zero_inputs(), &cfg);
        assert!(
            forces.torque_body.y < 0.0,
            "pitch damping torque={:.1} should be negative for positive pitch rate",
            forces.torque_body.y
        );
    }
}
