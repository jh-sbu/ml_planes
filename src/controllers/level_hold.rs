//! Level-hold flight controller — cascade PID.
//!
//! Loops:
//!   altitude  → [outer PID] → pitch_target → [inner PID] → elevator
//!   airspeed  → [PID] → throttle (+ feedforward from pitch_target)
//!   roll angle → [PID] → aileron
//!   sideslip β → [PID] → rudder

use bevy::math::{Quat, Vec3};

use crate::controllers::tuning::LevelHoldTuning;
use crate::controllers::{FlightController, PidController};
use crate::plane::{ControlInputs, FlightState};

// ---------------------------------------------------------------------------
// Attitude helpers
// ---------------------------------------------------------------------------

/// Roll (bank) angle from a body→world quaternion.
///
/// Positive = right wing low (right bank).
/// Returns 0 for level, unbanked flight.
fn roll_angle(attitude: Quat) -> f32 {
    // attitude is body→world.
    // right_world: direction the right wing (+Y body) points in world space.
    // up_world:    direction the cockpit-up (+Z body) points in world space.
    let right_world = attitude * Vec3::Y;
    let up_world    = attitude * Vec3::Z;
    // Project both onto the world vertical axis (Y) and compute bank angle.
    right_world.y.atan2(up_world.y)
}

/// Pitch angle from a body→world quaternion.
///
/// Positive = nose above the world horizontal plane.
/// Returns 0 for level, unclimbing flight.
fn pitch_angle(attitude: Quat) -> f32 {
    // Body +X is the nose direction. y-component in world space = sin(pitch).
    let forward_world = attitude * Vec3::X;
    forward_world.y.asin()
}

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------

/// Cascade PID controller for level altitude and airspeed hold.
///
/// Default gains are tuned for `generic_jet.plane.ron` at 100 m/s.
pub struct LevelHoldController {
    /// Target altitude [m].
    pub target_altitude: f32,
    /// Target airspeed [m/s].
    pub target_airspeed: f32,
    /// Outer altitude loop: altitude error [m] → pitch_target [rad], clamped ±0.3 rad (≈ ±17°).
    pub altitude_pid: PidController,
    /// Inner pitch loop: pitch error [rad] → elevator [-1, 1].
    pub pitch_pid: PidController,
    /// Airspeed loop: airspeed error [m/s] → throttle [0, 1].
    pub airspeed_pid: PidController,
    /// Wings-level loop: roll angle [rad] → aileron [-1, 1].
    pub roll_pid: PidController,
    /// Heading/sideslip loop: β [rad] → rudder [-1, 1].
    pub beta_pid: PidController,
    /// Throttle feedforward gain: scales α_target [rad] into a throttle increment.
    ///
    /// At α_target = 0.3 rad (max climb command), induced drag is ~6× cruise drag;
    /// the extra drag force (~12 800 N) divided by thrust_max (60 000 N) ≈ 0.21,
    /// giving a natural gain of 0.21/0.3 ≈ 0.7.
    pub throttle_ff_gain: f32,
    /// Commanded bank angle [rad]. 0.0 = wings level (default).
    /// Set by WingmanController to steer laterally toward the formation slot.
    pub target_roll: f32,
}

impl LevelHoldController {
    /// Create a controller whose targets and integrators are seeded from the
    /// given flight state and previous controller output.
    ///
    /// `prev_inputs` is the last `ControlInputs` produced by the outgoing
    /// controller. The airspeed and altitude integrals are pre-loaded so that
    /// the first `update` call produces outputs close to trim with no transient
    /// sag. Pass `&ControlInputs::default()` when there is no previous output
    /// (fresh spawn).
    pub fn from_state(state: &FlightState, prev_inputs: &ControlInputs) -> Self {
        let mut ctrl = Self::new(state.altitude, state.airspeed);
        ctrl.seed_integrals(state, prev_inputs);
        ctrl
    }

    /// Create a controller from the given flight state, overriding gains from `tuning`.
    ///
    /// Gains are applied before integrators are seeded so that the seed
    /// calculation uses the correct `ki` values.
    pub fn with_tuning(state: &FlightState, tuning: &LevelHoldTuning, prev_inputs: &ControlInputs) -> Self {
        let mut ctrl = Self::new(state.altitude, state.airspeed);
        ctrl.altitude_pid.kp    = tuning.alt_kp;
        ctrl.altitude_pid.ki    = tuning.alt_ki;
        ctrl.altitude_pid.kd    = tuning.alt_kd;
        ctrl.pitch_pid.kp       = tuning.pitch_kp;
        ctrl.pitch_pid.kd       = tuning.pitch_kd;
        ctrl.airspeed_pid.kp    = tuning.spd_kp;
        ctrl.airspeed_pid.ki    = tuning.spd_ki;
        ctrl.throttle_ff_gain   = tuning.throttle_ff_gain;
        ctrl.seed_integrals(state, prev_inputs);
        ctrl
    }

    /// Seed the airspeed and altitude integrators so that the first update
    /// produces outputs near trim rather than starting from zero.
    ///
    /// - Airspeed integral: `I = throttle / ki` → first `throttle_fb ≈ prev_inputs.throttle`
    /// - Altitude integral: `I = pitch / ki` → first `pitch_target ≈ current_pitch`,
    ///   which keeps the inner loop at near-zero elevator error.
    fn seed_integrals(&mut self, state: &FlightState, prev_inputs: &ControlInputs) {
        if self.airspeed_pid.ki > 0.0 {
            self.airspeed_pid.seed_integral(prev_inputs.throttle / self.airspeed_pid.ki);
        }
        if self.altitude_pid.ki > 0.0 {
            self.altitude_pid.seed_integral(pitch_angle(state.attitude) / self.altitude_pid.ki);
        }
    }

    /// Create a controller with default gains for the generic jet at 100 m/s.
    pub fn new(target_altitude: f32, target_airspeed: f32) -> Self {
        Self {
            target_altitude,
            target_airspeed,
            // Outer altitude loop: altitude error [m] → α_target [rad], output clamped ±0.3 rad.
            //
            // kp=0.01: 10 m error → 0.1 rad proportional term; proportional is now a meaningful
            // fraction of the clamp so the integral does not carry all the corrective load.
            //
            // ki=0.12, integral_clamp=0.93: steady-state integral I_ss =
            //   (α_trim + δe_trim/kp_α) / ki = (0.0653 + 0.046) / 0.12 ≈ 0.925,
            // just under the 0.93 clamp. The integral cannot wind up beyond the physically
            // required trim offset, breaking the limit cycle driven by integral overshoot.
            //
            // kd=0.04: at a 2 m/s phugoid climb rate the derivative contributes −0.08 rad
            // to α_target, comparable to the proportional term at small errors.
            altitude_pid: PidController::new(0.01,  0.12,  0.04, 0.93, -0.3, 0.3),
            // Inner pitch loop: pitch error [rad] → elevator [-1, 1].
            // No integral (outer loop provides steady-state correction).
            // kd=0.5 gives ~50° phase margin on the pitch double-integrator plant.
            pitch_pid:    PidController::new(1.0,   0.0,    0.5,  0.0, -1.0, 1.0),
            // Airspeed loop: integral holds throttle at trim without steady-state error.
            // ki=0.06 with integral_clamp=1.0 → max throttle from integral = 0.06,
            // which covers the ~0.058 trim throttle at 100 m/s cruise.
            airspeed_pid: PidController::new(0.01,  0.06,   0.0,  1.0,  0.0, 1.0),
            // Wings-level: Cl_p roll damping already provides ~53° PM; conservative kp/kd.
            roll_pid:     PidController::new(0.5,   0.0,    0.3,  0.0, -1.0, 1.0),
            // Sideslip: weathercock (Cn_β) provides natural yaw stiffness.
            beta_pid:     PidController::new(0.5,   0.0,    0.1,  0.0, -1.0, 1.0),
            // Feedforward gain: proactively adds throttle proportional to the pitch
            // command so the airspeed loop does not have to react to speed loss first.
            throttle_ff_gain: 0.7,
            target_roll: 0.0,
        }
    }
}

impl FlightController for LevelHoldController {
    fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs {
        // Altitude cascade: altitude error → pitch_target → elevator.
        //
        // Pitch angle is a world-frame quantity: it oscillates during phugoid motion,
        // giving the inner loop a measurable error to correct (alpha stays nearly
        // constant during a phugoid and cannot provide this signal).
        // Sign note: positive elevator → nose-UP → pitch INCREASES.
        // Inner loop error is (pitch_target − current_pitch): positive error → positive
        // elevator → nose up → pitch increases toward target.
        let pitch_target  = self.altitude_pid.update(self.target_altitude - state.altitude, dt);
        let current_pitch = pitch_angle(state.attitude);
        // Derivative on measurement: use pitch rate q to avoid derivative kick from
        // frame-to-frame changes in pitch_target (outer loop output).
        // Body-frame convention: positive q (angular_velocity.y) rotates the nose toward -Z
        // (nose-DOWN), so pitch *increases* when q is *negative*.
        // d(pitch)/dt ≈ -angular_velocity.y → measured_rate for DoM is -q.
        let elevator      = self.pitch_pid.update_dom(
            pitch_target - current_pitch,
            -state.angular_velocity.y,
            dt,
        );

        // Airspeed: feedback from speed error + feedforward from pitch command.
        //
        // When pitch_target > current_pitch (climb demanded), induced drag rises sharply
        // (∝ CL²) and thrust tilts away from the flight path.  The feedforward adds
        // throttle proportional to the *inner-loop pitch error* (pitch_target − current_pitch,
        // floored at zero) — it fires only while the aircraft is still pitching toward
        // the commanded attitude, i.e. during the transient.  At steady level flight
        // current_pitch ≈ pitch_target, so pitch_error → 0 and the feedforward is zero;
        // the airspeed integral alone holds trim throttle.
        let pitch_error = (pitch_target - current_pitch).max(0.0);
        let throttle_fb = self.airspeed_pid.update(self.target_airspeed - state.airspeed, dt);
        let throttle    = (throttle_fb + pitch_error * self.throttle_ff_gain).clamp(0.0, 1.0);

        // Wings level (or bank-to-command): drive roll toward target_roll (default 0 = wings level).
        let roll    = roll_angle(state.attitude);
        // Derivative on measurement: use roll rate p directly.
        // p = angular_velocity.x; positive p = right roll = roll angle increasing.
        let aileron = self.roll_pid.update_dom(
            self.target_roll - roll,
            state.angular_velocity.x,
            dt,
        );

        // Sideslip / heading hold: drive β to zero.
        let rudder = self.beta_pid.update(-state.beta, dt);

        ControlInputs { aileron, elevator, rudder, throttle }
    }

    fn name(&self) -> &'static str { "LevelHold" }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any { self }

    #[cfg(feature = "visual")]
    fn poll_input(
        &mut self,
        keys: &bevy::input::ButtonInput<bevy::prelude::KeyCode>,
        dt: f32,
    ) {
        use bevy::prelude::KeyCode;
        // Arrow Up/Down: adjust target altitude ±50 m per second held.
        if keys.pressed(KeyCode::ArrowUp)   { self.target_altitude += 50.0 * dt; }
        if keys.pressed(KeyCode::ArrowDown) { self.target_altitude -= 50.0 * dt; }
        // Equal(+) / Minus(-): adjust target airspeed ±5 m/s per second held.
        if keys.pressed(KeyCode::Equal) { self.target_airspeed += 5.0 * dt; }
        if keys.pressed(KeyCode::Minus) { self.target_airspeed -= 5.0 * dt; }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    /// Level-flight attitude: body +Z (cockpit up) aligns with world +Y (up).
    fn level_attitude() -> Quat {
        Quat::from_rotation_x(-FRAC_PI_2)
    }

    /// Minimal level-flight FlightState at given altitude and airspeed.
    fn level_state(altitude: f32, airspeed: f32) -> FlightState {
        FlightState {
            position:         Vec3::new(0.0, altitude, 0.0),
            velocity:         Vec3::new(airspeed, 0.0, 0.0),
            attitude:         level_attitude(),
            angular_velocity: Vec3::ZERO,
            alpha:            0.0,
            beta:             0.0,
            airspeed,
            altitude,
        }
    }

    #[test]
    fn roll_angle_is_zero_at_level_attitude() {
        let roll = roll_angle(level_attitude());
        assert!(roll.abs() < 1e-5, "roll={roll}");
    }

    #[test]
    fn roll_angle_nonzero_for_banked_attitude() {
        // Rotate past level by an extra quarter-turn: should produce a large roll angle.
        let banked = Quat::from_rotation_x(-FRAC_PI_2 + 0.4);
        let roll   = roll_angle(banked);
        assert!(roll.abs() > 0.3, "roll={roll} expected |roll|>0.3 for banked attitude");
    }

    #[test]
    fn pitch_angle_is_zero_at_level_attitude() {
        let pitch = pitch_angle(level_attitude());
        assert!(pitch.abs() < 1e-5, "pitch={pitch}");
    }

    #[test]
    fn pitch_angle_positive_for_nose_up_attitude() {
        // In body frame, positive Ry takes +X toward -Z (nose down in body convention).
        // Negative Ry tips the nose up toward body +Z = world Y.
        let nose_up = level_attitude() * Quat::from_rotation_y(-0.2);
        let pitch   = pitch_angle(nose_up);
        assert!(pitch > 0.1, "pitch={pitch} expected positive for nose-up attitude");
    }

    // Positive elevator → nose-up → alpha increases (more lift, plane climbs).
    // To gain altitude, the controller pitches nose UP (positive elevator) so
    // alpha increases, lift exceeds weight, and altitude rises.
    #[test]
    fn altitude_below_target_gives_positive_elevator() {
        let mut ctrl = LevelHoldController::new(1000.0, 80.0);
        let state    = level_state(900.0, 80.0); // 100 m below target
        let inputs   = ctrl.update(&state, 1.0 / 60.0);
        assert!(inputs.elevator > 0.0, "elevator={}", inputs.elevator);
    }

    #[test]
    fn altitude_above_target_gives_negative_elevator() {
        let mut ctrl = LevelHoldController::new(1000.0, 80.0);
        let state    = level_state(1100.0, 80.0); // 100 m above target
        let inputs   = ctrl.update(&state, 1.0 / 60.0);
        assert!(inputs.elevator < 0.0, "elevator={}", inputs.elevator);
    }

    #[test]
    fn airspeed_below_target_gives_positive_throttle() {
        let mut ctrl = LevelHoldController::new(1000.0, 80.0);
        let state    = level_state(1000.0, 60.0); // 20 m/s slow
        let inputs   = ctrl.update(&state, 1.0 / 60.0);
        assert!(inputs.throttle > 0.0, "throttle={}", inputs.throttle);
    }

    #[test]
    fn unseeded_zero_error_gives_near_zero_control_outputs() {
        // With no previous inputs (fresh spawn, prev_inputs = default = zeros),
        // integrals are seeded to zero. At zero error all outputs should be near zero.
        use crate::plane::ControlInputs;
        let mut ctrl = LevelHoldController::from_state(
            &level_state(1000.0, 80.0),
            &ControlInputs::default(),
        );
        let state  = level_state(1000.0, 80.0);
        let inputs = ctrl.update(&state, 1.0 / 60.0);
        assert!(inputs.elevator.abs() < 0.05, "elevator={}", inputs.elevator);
        assert!(inputs.aileron.abs()  < 0.05, "aileron={}", inputs.aileron);
        assert!(inputs.rudder.abs()   < 0.05, "rudder={}", inputs.rudder);
        assert!(inputs.throttle.abs() < 0.05, "throttle={}", inputs.throttle);
    }

    #[test]
    fn bumpless_engagement_preserves_trim_throttle() {
        // Simulate engaging from a previous controller that was outputting trim throttle.
        // With seeding: first-step throttle should match previous throttle closely.
        use crate::plane::ControlInputs;
        let trim_throttle = 0.058_f32;
        let prev_inputs = ControlInputs { throttle: trim_throttle, ..Default::default() };
        // state.alpha = 0 (synthetic level state), so altitude integral seeds to 0 too.
        let mut ctrl = LevelHoldController::from_state(
            &level_state(1000.0, 80.0),
            &prev_inputs,
        );
        let state  = level_state(1000.0, 80.0);
        let inputs = ctrl.update(&state, 1.0 / 60.0);
        // ki_spd = 0.06; seeded integral = 0.058/0.06 = 0.967 (clamped to 1.0 by integral_clamp).
        // throttle_fb = ki * integral ≈ 0.06 * 0.967 = 0.058, within 5 % of trim.
        let rel_err = (inputs.throttle - trim_throttle).abs() / trim_throttle;
        assert!(rel_err < 0.05, "throttle={} expected≈{}", inputs.throttle, trim_throttle);
    }

    #[test]
    fn bumpless_engagement_preserves_pitch_via_altitude_integral() {
        // When state pitch is near zero (level_state has level_attitude → pitch=0),
        // altitude integral seeds to ~0 so pitch_target ≈ 0 and elevator error ≈ 0 on
        // the first step. (Non-zero pitch at engagement would seed a non-zero integral
        // — the mechanism is the same as the old alpha seeding since pitch ≈ alpha in
        // level/trim flight.)
        use crate::plane::ControlInputs;
        let state = level_state(1000.0, 100.0);
        let prev_inputs = ControlInputs { throttle: 0.058, ..Default::default() };
        let mut ctrl = LevelHoldController::from_state(&state, &prev_inputs);
        let inputs = ctrl.update(&state, 1.0 / 60.0);
        // Elevator should be near zero: pitch_target ≈ current_pitch ≈ 0, error ≈ 0.
        assert!(inputs.elevator.abs() < 0.1, "elevator={} expected≈0", inputs.elevator);
    }
}
