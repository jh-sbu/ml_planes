//! Coefficient-based aerodynamic force/torque model, shared verbatim by the live
//! Rapier sim and the self-contained training integrator.
//!
//! # Model simplifications
//!
//! - **Side force is a body-axis buildup**: `CY = cy_beta·β + cy_p·(pb/2V) +
//!   cy_r·(rb/2V) + cy_delta_r·δr` is applied straight along body +Y rather than as a
//!   wind-axis `Y` rotated through the full 3-axis wind→body transform. Lift likewise
//!   stays normal to the wind in the plane of symmetry (no `beta` dependence), and
//!   there is no `cy_delta_a`. Drag *is* resolved along the full 3-D velocity vector.
//! - **No sideslip drag**: `CD` depends on `CL` only, so a crab costs the resolved
//!   `−D·sin β` component but carries no explicit `CD_β²` penalty. Real airframes pay
//!   noticeably more than that at large `beta`.
//! - **Thrust is body-fixed along +X** (see `compute_aero_forces`); no thrust
//!   incidence or offset moments.
//! - **No gyroscopic coupling**: the training integrator omits the ω×(Iω) Euler
//!   term, and Rapier's `gyroscopic_forces_enabled` defaults to off, so the two
//!   integrators agree analytically (both reduce to `ω̇ = I⁻¹τ`).
//! - **Gravity** is 9.81 m/s² in the dynamics (Rapier default and
//!   `integrate_state`); the ISA density model uses the standard 9.80665
//!   internally — the ~0.04% mismatch only shifts the density profile.
//!
//! # Sign conventions (body frame: +X fwd, +Y right wing, +Z up)
//!
//! This frame mirrors NED about the roll and pitch axes: **positive roll torque =
//! +Y wing UP** and **positive pitch torque = nose DOWN**. Stability derivatives
//! copied from standard (NED) references must flip sign accordingly: `cm_alpha`
//! and `cl_beta` are positive here for static stability, `cl_r` negative. Rate
//! dampings (`cl_p`, `cm_q`, `cn_r`) oppose their own rate and keep the usual
//! negative sign; the yaw sense (positive = nose toward +Y) matches NED, so
//! `cn_beta` stays positive. Guarded by `tests/core/plane_assets.rs`.
//!
//! **The Y axis itself is *not* mirrored**, so a *force* along +Y keeps its NED sign.
//! A side-force derivative therefore flips only if the quantity it multiplies has a
//! mirrored sense — which is true of roll rate `p` and of nothing else here:
//!
//! | Derivative | Sign | vs NED | Why |
//! |---|---|---|---|
//! | `cy_beta` | negative | same | β>0 (wind from +Y) pushes the airframe −Y |
//! | `cy_r` | positive | same | r>0 swings the aft fin −Y → local β<0 → force +Y |
//! | `cy_delta_r` | positive | same | rudder pushes the tail +Y, yawing the nose −Y |
//! | `cy_p` | **positive** | **FLIPPED** | p>0 is +Y wing *up* here, so the fin (above the CG) swings −Y → force +Y |
//!
//! `cy_p` is the trap: it is neither a damping (which would keep its sign) nor one of
//! the roll/pitch-mirrored moment derivatives the paragraph above covers.
//!
//! Because the fin sits aft of the CG, each fin-driven side force produces the
//! **opposite-signed** yaw moment. That pairing — `sign(cy_beta) = -sign(cn_beta)`,
//! `sign(cy_r) = -sign(cn_r)`, `sign(cy_delta_r) = -sign(cn_delta_r)` — is the most
//! reliable check on a new airframe's numbers, and is asserted for every shipped asset.

use crate::aerodynamics::atmosphere::{air_density, density_ratio};
use crate::plane::{ControlInputs, FlightState, PlaneConfig};
use bevy::math::Vec3;

/// Aerodynamic and thrust forces/torques in the body frame.
#[derive(Debug, Clone, Default)]
pub struct AeroForces {
    pub force_body: Vec3,  // [N]   body frame (+X fwd, +Y right, +Z up)
    pub torque_body: Vec3, // [N·m] body frame
}

/// Engine thrust [N] for the current throttle, scaled by air density at altitude
/// (air-breathing thrust falls off with altitude). Returns 0 when the plane is out of
/// consumable (flameout / dead battery). Shared by [`compute_aero_forces`] and the
/// per-tick fuel-burn accounting so the live sim and training integrator agree.
pub fn engine_thrust(state: &FlightState, inputs: &ControlInputs, cfg: &PlaneConfig) -> f32 {
    if state.consumable_remaining <= 0.0 {
        // Out of fuel / charge → flameout.
        return 0.0;
    }
    inputs.throttle * cfg.thrust_max * density_ratio(state.altitude)
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

    // Dynamic pressure (air density varies with altitude per the ISA model)
    let rho = air_density(state.altitude);
    let q_bar = 0.5 * rho * v * v;

    // Control surface deflections [rad]
    let delta_a = inputs.aileron * cfg.aileron_limit;
    let delta_e = inputs.elevator * cfg.elevator_limit;
    let delta_r = inputs.rudder * cfg.rudder_limit;

    // Angular rates from body frame
    let p = state.angular_velocity.x; // roll rate
    let q = state.angular_velocity.y; // pitch rate
    let r = state.angular_velocity.z; // yaw rate

    let alpha = state.alpha;
    let beta = state.beta;
    let b = cfg.wing_span;
    let c = cfg.mean_chord;

    // --- Lift coefficient (clamped at stall) ---
    let cl_raw = cfg.cl0 + cfg.cl_alpha * alpha + cfg.cl_delta_e * delta_e;
    let cl = cl_raw.clamp(-cfg.cl_max, cfg.cl_max);

    // --- Drag coefficient (always positive) ---
    let cd = cfg.cd0 + cfg.cd_induced * cl * cl;

    // --- Forces ---
    let lift = q_bar * cfg.wing_area * cl;
    let drag = q_bar * cfg.wing_area * cd;
    // Air-breathing thrust falls off with air density at altitude; zero when out of
    // consumable (flameout). Shared with the per-tick fuel-burn accounting.
    let thrust = engine_thrust(state, inputs, cfg);

    // --- Side force coefficient ---
    // CY is a body-axis coefficient: it is applied directly along +Y and is NOT rotated
    // with the wind frame. `cy_p` carries the flipped sign (see the module header).
    let cy = cfg.cy_beta * beta
        + cfg.cy_p * (p * b / (2.0 * v))
        + cfg.cy_r * (r * b / (2.0 * v))
        + cfg.cy_delta_r * delta_r;

    // Resolve the wind-axis forces into body axes. The unit velocity vector in body
    // axes is (cos α·cos β, sin β, −sin α·cos β), matching how `FlightState` defines
    // alpha = atan2(-v_body.z, v_body.x) and beta = asin(v_body.y / V).
    //
    // Drag opposes that full 3-D vector. The alpha rotation is what keeps the model
    // stable at large alpha (steep dive): without it, negative CL at large negative
    // alpha produces a downward force that grows as V² and blows up to NaN. The beta
    // terms vanish identically at beta = 0 (cos β = 1, sin β = 0).
    //
    // Lift stays normal to the wind in the aircraft's plane of symmetry, so it has no
    // beta dependence. Thrust is body-fixed along +X (the engine turns with the
    // airframe) and must NOT be rotated with the wind frame.
    let (sin_a, cos_a) = alpha.sin_cos();
    let (sin_b, cos_b) = beta.sin_cos();

    let drag_body = Vec3::new(-drag * cos_a * cos_b, -drag * sin_b, drag * sin_a * cos_b);
    let lift_body = Vec3::new(lift * sin_a, 0.0, lift * cos_a);
    let side_body = Vec3::new(0.0, q_bar * cfg.wing_area * cy, 0.0);

    let force_body = Vec3::new(thrust, 0.0, 0.0) + drag_body + lift_body + side_body;

    // --- Moment coefficients ---
    let cm =
        cfg.cm0 + cfg.cm_alpha * alpha + cfg.cm_q * (q * c / (2.0 * v)) + cfg.cm_delta_e * delta_e;

    // Local name `cl_coef` avoids shadowing the longitudinal `cl` above.
    let cl_coef = cfg.cl_beta * beta
        + cfg.cl_p * (p * b / (2.0 * v))
        + cfg.cl_r * (r * b / (2.0 * v))
        + cfg.cl_delta_a * delta_a;

    let cn = cfg.cn_beta * beta + cfg.cn_r * (r * b / (2.0 * v)) + cfg.cn_delta_r * delta_r;

    // --- Moments ---
    let roll_moment = q_bar * cfg.wing_area * b * cl_coef;
    let pitch_moment = q_bar * cfg.wing_area * c * cm;
    let yaw_moment = q_bar * cfg.wing_area * b * cn;

    let torque_body = Vec3::new(roll_moment, pitch_moment, yaw_moment);

    AeroForces {
        force_body,
        torque_body,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The shared frozen test airframe (`fixtures/generic_jet.plane.ron`).
    ///
    /// NOTE: this used to be a hand-typed literal that had drifted — it still carried
    /// `cm_q: -8.0` long after the shipped generic jet moved to -14.0. The fixture is a
    /// snapshot rather than a live mirror, but at least it is a single one.
    fn jet_config() -> PlaneConfig {
        crate::plane::config::fixture_jet_config()
    }

    fn zero_state() -> FlightState {
        FlightState::default()
    }

    fn zero_inputs() -> ControlInputs {
        ControlInputs::default()
    }

    #[test]
    fn engine_thrust_flames_out_when_empty() {
        let cfg = jet_config();
        let mut inputs = zero_inputs();
        inputs.throttle = 1.0;
        let mut state = zero_state();
        state.altitude = 0.0;

        // Fuel remaining → normal (positive) thrust.
        state.consumable_remaining = 100.0;
        let thrust_full = engine_thrust(&state, &inputs, &cfg);
        assert!(
            thrust_full > 0.0,
            "expected positive thrust, got {thrust_full}"
        );

        // Empty tank → flameout (zero thrust).
        state.consumable_remaining = 0.0;
        assert_eq!(engine_thrust(&state, &inputs, &cfg), 0.0);
    }

    #[test]
    fn zero_airspeed_returns_zero() {
        let state = zero_state(); // airspeed == 0.0
        let forces = compute_aero_forces(&state, &zero_inputs(), &jet_config());
        assert_eq!(forces.force_body, Vec3::ZERO);
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
        state.alpha = 0.0667;

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
        state.alpha = 0.05;

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
        state.alpha = 0.05;

        let f0 = compute_aero_forces(&state, &zero_inputs(), &cfg);

        let mut inputs_up = zero_inputs();
        inputs_up.elevator = 1.0;
        let f1 = compute_aero_forces(&state, &inputs_up, &cfg);

        assert!(
            f1.force_body.z > f0.force_body.z,
            "elevator up should increase lift: {:.1} vs {:.1}",
            f1.force_body.z,
            f0.force_body.z
        );
        // cm_delta_e = -1.2 → more negative Cm → more negative pitch torque (nose-up)
        assert!(
            f1.torque_body.y < f0.torque_body.y,
            "elevator up should produce more negative pitch moment: {:.1} vs {:.1}",
            f1.torque_body.y,
            f0.torque_body.y
        );
    }

    #[test]
    fn cl_clamped_at_cl_max() {
        // α = 1.0 rad → CL_raw = 0.1 + 4.5 = 4.6 → clamped to 1.4
        // Wind-frame lift = q_bar * S * CL_max = 1531.25 * 20 * 1.4 = 42875 N
        // Body-frame Fz = -Fx_wind*sin(α) + lift*cos(α)  (wind-to-body rotation)
        // We verify the body-frame Fz is consistent with the clamped CL (not the
        // unclamped 4.6), i.e. force_body.z is strictly less than it would be with
        // CL_raw = 4.6 applied without clamping.
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 50.0;
        state.alpha = 1.0;

        let forces = compute_aero_forces(&state, &zero_inputs(), &cfg);
        let q_bar = 0.5 * air_density(state.altitude) * 50.0f32 * 50.0;
        let lift_clamped = q_bar * cfg.wing_area * cfg.cl_max; // CL = 1.4
        let lift_raw = q_bar * cfg.wing_area * (cfg.cl0 + cfg.cl_alpha * 1.0); // CL = 4.6
                                                                               // After the wind-to-body rotation the z-component scales with cos(alpha),
                                                                               // so clamping should result in a smaller Fz than if CL_raw were used.
        let drag = q_bar * cfg.wing_area * (cfg.cd0 + cfg.cd_induced * cfg.cl_max * cfg.cl_max);
        let expected_fz = lift_clamped * 1.0f32.cos() + drag * 1.0f32.sin();
        let unclamped_fz = lift_raw * 1.0f32.cos()
            + q_bar
                * cfg.wing_area
                * (cfg.cd0 + cfg.cd_induced * (cfg.cl0 + cfg.cl_alpha * 1.0).powi(2))
                * 1.0f32.sin();
        assert!(
            forces.force_body.z < unclamped_fz,
            "clamped Fz={:.1} should be less than unclamped {:.1}",
            forces.force_body.z,
            unclamped_fz
        );
        assert!(
            (forces.force_body.z - expected_fz).abs() < 1.0,
            "Fz={:.1} should match expected {:.1} (wind-to-body rotation of clamped lift)",
            forces.force_body.z,
            expected_fz
        );
    }

    #[test]
    fn sideslip_rolls_windward_wing_up() {
        // Stable dihedral: wind from the +Y side (beta > 0) must raise the windward
        // (+Y) wing. In this codebase positive roll torque = +Y wing up (the mirror
        // of NED, where positive roll = right wing down), so cl_beta must be POSITIVE.
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 100.0;
        state.beta = 0.1;

        let forces = compute_aero_forces(&state, &zero_inputs(), &cfg);
        assert!(
            forces.torque_body.x > 0.0,
            "dihedral roll torque={:.1} should be positive (windward wing up) for beta=+0.1",
            forces.torque_body.x
        );
    }

    #[test]
    fn sideslip_produces_restoring_side_force() {
        // beta > 0 means the relative wind comes from +Y, so the fin and fuselage are
        // pushed toward -Y. The Y axis is NOT mirrored vs NED (only roll and pitch are),
        // so cy_beta keeps its standard NEGATIVE sign and the force must be negative.
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 100.0;
        state.beta = 0.1;

        let forces = compute_aero_forces(&state, &zero_inputs(), &cfg);
        assert!(
            forces.force_body.y < 0.0,
            "side force={:.1} should be negative (restoring) for beta=+0.1",
            forces.force_body.y
        );
    }

    #[test]
    fn rudder_produces_side_force_and_opposite_yaw() {
        // The fin is aft of the CG, so its side force and the yaw moment it generates
        // always have opposite signs: positive rudder pushes the tail toward +Y, which
        // swings the nose toward -Y. cy_delta_r > 0 pairs with cn_delta_r < 0.
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 100.0;
        let mut inputs = zero_inputs();
        inputs.rudder = 1.0;

        let forces = compute_aero_forces(&state, &inputs, &cfg);
        assert!(
            forces.force_body.y > 0.0,
            "rudder side force={:.1} should be positive for rudder=+1",
            forces.force_body.y
        );
        assert!(
            forces.torque_body.z < 0.0,
            "rudder yaw moment={:.1} should be negative (opposite the side force)",
            forces.torque_body.z
        );
    }

    #[test]
    fn yaw_rate_produces_side_force() {
        // Yaw rate r > 0 swings the nose toward +Y, so the aft fin swings toward -Y and
        // sees a negative local sideslip → positive side force. Yaw sense matches NED,
        // so cy_r keeps its standard POSITIVE sign.
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 100.0;
        state.angular_velocity = Vec3::new(0.0, 0.0, 0.5); // r = 0.5 rad/s

        let forces = compute_aero_forces(&state, &zero_inputs(), &cfg);
        assert!(
            forces.force_body.y > 0.0,
            "yaw-rate side force={:.1} should be positive for r=+0.5",
            forces.force_body.y
        );
    }

    #[test]
    fn roll_rate_produces_side_force() {
        // The sign trap. Positive roll rate here raises the +Y wing (the mirror of NED),
        // so the fin — which sits ABOVE the CG — swings toward -Y, sees a negative local
        // sideslip, and generates a POSITIVE side force. cy_p is therefore the one side-
        // force derivative whose sign is flipped relative to NED references.
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 100.0;
        state.angular_velocity = Vec3::new(0.5, 0.0, 0.0); // p = 0.5 rad/s

        let forces = compute_aero_forces(&state, &zero_inputs(), &cfg);
        assert!(
            forces.force_body.y > 0.0,
            "roll-rate side force={:.1} should be positive for p=+0.5 (sign flipped vs NED)",
            forces.force_body.y
        );
    }

    #[test]
    fn drag_resolves_along_full_velocity() {
        // Drag opposes the FULL 3-D velocity vector, not just its projection into the
        // body x-z plane. With every cy_* zeroed the only remaining lateral force is the
        // -D*sin(beta) component of drag, which must still push against the sideslip.
        let mut cfg = jet_config();
        cfg.cy_beta = 0.0;
        cfg.cy_p = 0.0;
        cfg.cy_r = 0.0;
        cfg.cy_delta_r = 0.0;
        let mut state = zero_state();
        state.airspeed = 100.0;
        state.beta = 0.2;

        let forces = compute_aero_forces(&state, &zero_inputs(), &cfg);
        assert!(
            forces.force_body.y < 0.0,
            "drag-only side force={:.1} should be negative for beta=+0.2",
            forces.force_body.y
        );
    }

    #[test]
    fn zero_sideslip_matches_alpha_only_rotation() {
        // At beta = 0 the 3-D resolution must reduce to the alpha-only rotation.
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 100.0;
        state.alpha = 0.1;
        state.beta = 0.0;
        let inputs = zero_inputs();

        let forces = compute_aero_forces(&state, &inputs, &cfg);

        let q_bar = 0.5 * air_density(state.altitude) * state.airspeed * state.airspeed;
        let cl = (cfg.cl0 + cfg.cl_alpha * state.alpha).clamp(-cfg.cl_max, cfg.cl_max);
        let lift = q_bar * cfg.wing_area * cl;
        let drag = q_bar * cfg.wing_area * (cfg.cd0 + cfg.cd_induced * cl * cl);
        let (sin_a, cos_a) = state.alpha.sin_cos();

        assert!(
            (forces.force_body.x - (-drag * cos_a + lift * sin_a)).abs() < 1.0,
            "Fx={:.3} should match the alpha-only rotation",
            forces.force_body.x
        );
        assert!(
            (forces.force_body.z - (drag * sin_a + lift * cos_a)).abs() < 1.0,
            "Fz={:.3} should match the alpha-only rotation",
            forces.force_body.z
        );
        assert_eq!(
            forces.force_body.y, 0.0,
            "no side force at zero sideslip, zero rates, zero rudder"
        );
    }

    #[test]
    fn yaw_rate_rolls_toward_inside_wing() {
        // Yaw rate r > 0 swings the nose toward +Y; the outer (-Y) wing advances
        // faster, gains lift, and rises — a NEGATIVE roll torque in this frame
        // (positive roll torque = +Y wing up), so cl_r must be NEGATIVE.
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 100.0;
        state.angular_velocity = Vec3::new(0.0, 0.0, 0.5); // r = 0.5 rad/s

        let forces = compute_aero_forces(&state, &zero_inputs(), &cfg);
        assert!(
            forces.torque_body.x < 0.0,
            "yaw-rate roll torque={:.1} should be negative (outer wing up) for r=+0.5",
            forces.torque_body.x
        );
    }

    #[test]
    fn thrust_acts_along_body_x() {
        // Thrust is body-fixed along +X; only lift/drag live in the wind frame.
        // At nonzero alpha the force delta between full and zero throttle must be
        // exactly (thrust_max, 0, 0) — no spurious -T·sin(alpha) body-down component.
        let cfg = jet_config();
        let mut state = zero_state();
        state.airspeed = 100.0;
        state.alpha = 0.1;

        let mut full = zero_inputs();
        full.throttle = 1.0;
        let f_full = compute_aero_forces(&state, &full, &cfg);
        let f_idle = compute_aero_forces(&state, &zero_inputs(), &cfg);
        let delta = f_full.force_body - f_idle.force_body;

        assert!(
            (delta.x - cfg.thrust_max).abs() < 1.0,
            "thrust delta x={:.1} expected {:.1}",
            delta.x,
            cfg.thrust_max
        );
        assert!(
            delta.z.abs() < 1.0,
            "thrust delta z={:.1} should be 0 (thrust must not tilt with alpha)",
            delta.z
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

    #[test]
    fn lift_decreases_with_altitude() {
        // Same airspeed/alpha at sea level vs 10 km: thinner air → less lift,
        // tracking the ISA density ratio (q̄ ∝ ρ).
        let cfg = jet_config();
        let mut sea = zero_state();
        sea.airspeed = 100.0;
        sea.alpha = 0.0667;
        let mut high = sea.clone();
        high.altitude = 10_000.0;

        let lift_sea = compute_aero_forces(&sea, &zero_inputs(), &cfg).force_body.z;
        let lift_high = compute_aero_forces(&high, &zero_inputs(), &cfg)
            .force_body
            .z;

        assert!(
            lift_high < lift_sea,
            "lift_high={lift_high:.1} should be < lift_sea={lift_sea:.1}"
        );
        let expected = lift_sea * density_ratio(10_000.0);
        assert!(
            (lift_high - expected).abs() < 1.0,
            "lift_high={lift_high:.1} expected ≈{expected:.1} (sea×density_ratio)"
        );
    }

    #[test]
    fn drag_decreases_with_altitude() {
        // Zero alpha, no throttle → body +X force is pure -drag; thinner air at
        // altitude makes drag smaller in magnitude.
        let cfg = jet_config();
        let mut sea = zero_state();
        sea.airspeed = 100.0;
        let mut high = sea.clone();
        high.altitude = 10_000.0;

        let drag_sea = -compute_aero_forces(&sea, &zero_inputs(), &cfg).force_body.x;
        let drag_high = -compute_aero_forces(&high, &zero_inputs(), &cfg)
            .force_body
            .x;

        assert!(drag_sea > 0.0 && drag_high > 0.0);
        assert!(
            drag_high < drag_sea,
            "drag_high={drag_high:.1} should be < drag_sea={drag_sea:.1}"
        );
        let expected = drag_sea * density_ratio(10_000.0);
        assert!((drag_high - expected).abs() < 1.0);
    }

    #[test]
    fn thrust_decreases_with_altitude() {
        // Full throttle, zero alpha: body +X = thrust - drag. Thrust scales with
        // the density ratio, so the net forward force drops at altitude.
        let cfg = jet_config();
        let mut inputs = zero_inputs();
        inputs.throttle = 1.0;
        let mut sea = zero_state();
        sea.airspeed = 100.0;
        let mut high = sea.clone();
        high.altitude = 10_000.0;

        // Isolate thrust: net_fx + drag = thrust, both at sea level and altitude.
        let drag_only_sea = -compute_aero_forces(&sea, &zero_inputs(), &cfg).force_body.x;
        let drag_only_high = -compute_aero_forces(&high, &zero_inputs(), &cfg)
            .force_body
            .x;
        let thrust_sea = compute_aero_forces(&sea, &inputs, &cfg).force_body.x + drag_only_sea;
        let thrust_high = compute_aero_forces(&high, &inputs, &cfg).force_body.x + drag_only_high;

        assert!(
            thrust_high < thrust_sea,
            "thrust_high={thrust_high:.1} should be < thrust_sea={thrust_sea:.1}"
        );
        let expected = cfg.thrust_max * density_ratio(10_000.0);
        assert!(
            (thrust_high - expected).abs() < 1.0,
            "thrust_high={thrust_high:.1} expected ≈{expected:.1}"
        );
    }
}
