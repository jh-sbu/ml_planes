//! L1 flight-plan controller.
//!
//! Follows a preset [`FlightPlan`] (a sequence of waypoint and orbit legs),
//! sequencing automatically between legs. Straight legs use the L1 nonlinear
//! lateral-guidance law; orbit legs reuse the shared orbit cascade. Both phases
//! emit only a bank command (plus per-leg altitude/airspeed setpoints) and
//! delegate stabilization to an inner [`LevelHoldController`], matching the
//! cascade pattern used across the codebase.
//!
//! Leg advancement:
//! - **Waypoint** — fly-by: advance once horizontal distance < `capture_radius`.
//! - **Orbit** — advance after `turns` full revolutions; `turns: None` loiters
//!   forever (terminal hold).
//! - After the final leg completes, the controller enters [`L1Phase::Finished`]
//!   and holds wings-level. (A `turns: None` final orbit never completes, so the
//!   aircraft loiters there instead.)

use std::f32::consts::{FRAC_PI_3, PI};

use bevy::math::Vec2;

use crate::controllers::flight_plan::{FlightPlan, FlightPlanLeg};
use crate::controllers::guidance::{l1_straight_bank, orbit_bank_command};
use crate::controllers::level_hold::LevelHoldController;
use crate::controllers::orbit::OrbitDirection;
use crate::controllers::pid::PidController;
use crate::controllers::traits::FlightController;
use crate::plane::{ControlInputs, FlightState};

const G: f32 = 9.81;
const TWO_PI: f32 = 2.0 * PI;

/// High-level state of the flight-plan follower.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum L1Phase {
    /// Actively flying `plan.legs[leg_index]`.
    Following,
    /// All finite legs complete; holding wings-level at the last setpoints.
    Finished,
}

/// Wrap an angle to `(-π, π]`.
fn wrap_to_pi(a: f32) -> f32 {
    let mut x = (a + PI) % TWO_PI;
    if x <= 0.0 {
        x += TWO_PI;
    }
    x - PI
}

/// Follows a [`FlightPlan`] via L1 lateral guidance.
pub struct L1Controller {
    /// The plan being flown.
    pub plan: FlightPlan,
    /// Index of the active leg in `plan.legs`.
    pub leg_index: usize,
    /// Current high-level phase.
    pub phase: L1Phase,
    /// World-frame XZ start point (segment `A`) of the active straight leg.
    pub leg_start: Vec2,
    /// Signed angle [rad] swept around the active orbit leg's center.
    pub orbit_accum: f32,
    /// Previous radial angle [rad] for unwrapping the swept angle.
    pub prev_radial: Option<f32>,
    /// Orbit radial-error PID (error [m] → heading offset [rad]).
    pub radial_pid: PidController,
    /// Orbit heading-error PID (error [rad] → Δbank [rad]).
    pub heading_pid: PidController,
    /// Inner stabilization (altitude, airspeed, roll, sideslip).
    pub inner: LevelHoldController,
}

impl L1Controller {
    /// Construct from a plan, capturing current state for a bumpless handoff.
    ///
    /// The active leg starts at index 0; the first straight leg's segment start
    /// is the aircraft's current XZ position. An empty plan starts `Finished`.
    pub fn from_plan(state: &FlightState, plan: FlightPlan, prev_inputs: &ControlInputs) -> Self {
        let inner = LevelHoldController::from_state(state, prev_inputs);
        let leg_start = Vec2::new(state.position.x, state.position.z);

        let phase = if plan.legs.is_empty() {
            L1Phase::Finished
        } else {
            L1Phase::Following
        };

        let mut ctrl = Self {
            plan,
            leg_index: 0,
            phase,
            leg_start,
            orbit_accum: 0.0,
            prev_radial: None,
            radial_pid: PidController::new(0.002, 0.0, 0.01, 0.0, -0.5, 0.5),
            heading_pid: PidController::new(0.7, 0.0, 0.1, 0.0, -FRAC_PI_3, FRAC_PI_3),
            inner,
        };

        // Seed inner targets from the active leg (or hold current state).
        match ctrl.active_leg().cloned() {
            Some(leg) => {
                ctrl.inner.target_altitude = leg.altitude();
                ctrl.inner.target_airspeed = leg.airspeed();
                if let FlightPlanLeg::Orbit {
                    radius, direction, ..
                } = leg
                {
                    ctrl.inner.target_roll = steady_bank(state.airspeed, radius, direction);
                }
            }
            None => {
                ctrl.inner.target_altitude = state.altitude;
                ctrl.inner.target_airspeed = state.airspeed;
            }
        }
        ctrl
    }

    /// The active leg, or `None` when finished / plan empty.
    pub fn active_leg(&self) -> Option<&FlightPlanLeg> {
        if self.phase == L1Phase::Finished {
            None
        } else {
            self.plan.legs.get(self.leg_index)
        }
    }

    /// Advance to the next leg, updating the segment start and resetting orbit
    /// bookkeeping. Sets `Finished` when the plan is exhausted.
    fn advance_leg(&mut self, state: &FlightState) {
        // Terminal point of the finished leg becomes the next segment start.
        self.leg_start = match self.plan.legs.get(self.leg_index) {
            Some(FlightPlanLeg::Waypoint { x, z, .. }) => Vec2::new(*x, *z),
            // Leaving an orbit: depart from the current position.
            _ => Vec2::new(state.position.x, state.position.z),
        };

        self.leg_index += 1;
        self.orbit_accum = 0.0;
        self.prev_radial = None;
        self.radial_pid.reset();
        self.heading_pid.reset();

        if self.leg_index >= self.plan.legs.len() {
            self.phase = L1Phase::Finished;
            return;
        }
        // Seed bank feedforward for a bumpless entry into an orbit leg.
        if let Some(FlightPlanLeg::Orbit {
            radius, direction, ..
        }) = self.plan.legs.get(self.leg_index)
        {
            self.inner.target_roll = steady_bank(state.airspeed, *radius, *direction);
        }
    }

    /// Accumulate swept angle for the active orbit leg and report whether the
    /// requested number of turns has been completed.
    fn orbit_turn_complete(
        &mut self,
        state: &FlightState,
        center_x: f32,
        center_z: f32,
        turns: Option<f32>,
    ) -> bool {
        let Some(turns) = turns else {
            return false; // hold forever
        };
        let rx = state.position.x - center_x;
        let rz = state.position.z - center_z;
        if (rx * rx + rz * rz) < 1.0 {
            return false;
        }
        let radial = rz.atan2(rx);
        if let Some(prev) = self.prev_radial {
            self.orbit_accum += wrap_to_pi(radial - prev);
        }
        self.prev_radial = Some(radial);
        self.orbit_accum.abs() >= turns * TWO_PI
    }
}

/// Steady orbit bank [rad]: `-direction.sign()·atan(V²/(g·R))`, clamped ±60°.
fn steady_bank(airspeed: f32, radius: f32, direction: OrbitDirection) -> f32 {
    let r = radius.max(1.0);
    (-direction.sign() * (airspeed.powi(2) / (G * r)).atan()).clamp(-FRAC_PI_3, FRAC_PI_3)
}

impl FlightController for L1Controller {
    fn update(
        &mut self,
        state: &FlightState,
        ctx: &crate::plane::ControllerContext,
        dt: f32,
    ) -> ControlInputs {
        // Finished: hold wings-level at the last setpoints.
        if self.phase == L1Phase::Finished {
            self.inner.target_roll = 0.0;
            return self.inner.update(state, ctx, dt);
        }

        // Snapshot the active leg's parameters (copy out to release the borrow).
        let leg = self.plan.legs[self.leg_index].clone();
        self.inner.target_altitude = leg.altitude();
        self.inner.target_airspeed = leg.airspeed();

        let advance = match leg {
            FlightPlanLeg::Waypoint {
                x,
                z,
                capture_radius,
                ..
            } => {
                let bank = l1_straight_bank(
                    state,
                    self.leg_start,
                    Vec2::new(x, z),
                    self.plan.l1_period,
                    self.plan.l1_damping,
                );
                self.inner.target_roll = bank;

                let dx = x - state.position.x;
                let dz = z - state.position.z;
                (dx * dx + dz * dz).sqrt() < capture_radius
            }
            FlightPlanLeg::Orbit {
                center_x,
                center_z,
                radius,
                direction,
                turns,
                ..
            } => {
                if let Some(bank) = orbit_bank_command(
                    state,
                    center_x,
                    center_z,
                    radius,
                    direction,
                    &mut self.radial_pid,
                    &mut self.heading_pid,
                    dt,
                ) {
                    self.inner.target_roll = bank;
                }
                self.orbit_turn_complete(state, center_x, center_z, turns)
            }
        };

        let inputs = self.inner.update(state, ctx, dt);

        if advance {
            self.advance_leg(state);
        }

        inputs
    }

    fn name(&self) -> &'static str {
        "Flight Plan (L1)"
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controllers::flight_plan::FlightPlanLeg;
    use bevy::math::{Quat, Vec3};
    use std::f32::consts::FRAC_PI_2;

    fn make_state(position: Vec3, velocity: Vec3) -> FlightState {
        let mut s = FlightState {
            position,
            velocity,
            attitude: Quat::from_rotation_x(-FRAC_PI_2),
            angular_velocity: Vec3::ZERO,
            ..Default::default()
        };
        s.update_air_data();
        s
    }

    fn ctx() -> crate::plane::ControllerContext {
        crate::plane::ControllerContext::empty_for(crate::plane::PlaneId::TEST)
    }

    fn wpt(x: f32, z: f32, capture_radius: f32) -> FlightPlanLeg {
        FlightPlanLeg::Waypoint {
            x,
            z,
            altitude: 1000.0,
            airspeed: 100.0,
            capture_radius,
        }
    }

    fn orbit(center_x: f32, center_z: f32, radius: f32, turns: Option<f32>) -> FlightPlanLeg {
        FlightPlanLeg::Orbit {
            center_x,
            center_z,
            radius,
            altitude: 1000.0,
            airspeed: 100.0,
            direction: OrbitDirection::CounterClockwise,
            turns,
        }
    }

    fn plan(legs: Vec<FlightPlanLeg>) -> FlightPlan {
        FlightPlan {
            legs,
            l1_period: 20.0,
            l1_damping: 0.75,
        }
    }

    /// Feed a state representing the aircraft at radial angle `theta` on a circle
    /// of `radius` about `center`, moving tangentially (increasing theta).
    fn orbit_state(center: Vec2, radius: f32, theta: f32, speed: f32, alt: f32) -> FlightState {
        let (s, c) = theta.sin_cos();
        let pos = Vec3::new(center.x + radius * c, alt, center.y + radius * s);
        // d/dtheta (cos, sin) = (-sin, cos): tangent for increasing theta.
        let vel = Vec3::new(-s * speed, 0.0, c * speed);
        make_state(pos, vel)
    }

    #[test]
    fn empty_plan_starts_finished() {
        let state = make_state(Vec3::new(0.0, 1000.0, 0.0), Vec3::new(100.0, 0.0, 0.0));
        let ctrl = L1Controller::from_plan(&state, plan(vec![]), &ControlInputs::default());
        assert_eq!(ctrl.phase, L1Phase::Finished);
        assert!(ctrl.active_leg().is_none());
    }

    #[test]
    fn waypoint_advances_within_capture_radius() {
        // Waypoint 100 m ahead, capture radius 200 m → captured immediately.
        let state = make_state(Vec3::new(0.0, 1000.0, 0.0), Vec3::new(100.0, 0.0, 0.0));
        let mut ctrl = L1Controller::from_plan(
            &state,
            plan(vec![wpt(100.0, 0.0, 200.0), orbit(0.0, 0.0, 1000.0, None)]),
            &ControlInputs::default(),
        );
        assert_eq!(ctrl.leg_index, 0);
        ctrl.update(&state, &ctx(), 1.0 / 64.0);
        assert_eq!(ctrl.leg_index, 1, "should advance after capturing waypoint");
    }

    #[test]
    fn waypoint_does_not_advance_when_far() {
        let state = make_state(Vec3::new(0.0, 1000.0, 0.0), Vec3::new(100.0, 0.0, 0.0));
        let mut ctrl = L1Controller::from_plan(
            &state,
            plan(vec![wpt(5000.0, 0.0, 200.0), orbit(0.0, 0.0, 1000.0, None)]),
            &ControlInputs::default(),
        );
        ctrl.update(&state, &ctx(), 1.0 / 64.0);
        assert_eq!(ctrl.leg_index, 0, "far waypoint should not advance");
    }

    #[test]
    fn single_waypoint_plan_finishes_after_capture() {
        let state = make_state(Vec3::new(0.0, 1000.0, 0.0), Vec3::new(100.0, 0.0, 0.0));
        let mut ctrl = L1Controller::from_plan(
            &state,
            plan(vec![wpt(100.0, 0.0, 200.0)]),
            &ControlInputs::default(),
        );
        ctrl.update(&state, &ctx(), 1.0 / 64.0);
        assert_eq!(ctrl.phase, L1Phase::Finished);
        // Subsequent ticks command wings-level.
        let s2 = make_state(Vec3::new(10.0, 1000.0, 0.0), Vec3::new(100.0, 0.0, 5.0));
        ctrl.update(&s2, &ctx(), 1.0 / 64.0);
        assert_eq!(ctrl.inner.target_roll, 0.0);
    }

    #[test]
    fn orbit_advances_after_one_turn() {
        let center = Vec2::new(0.0, 0.0);
        let radius = 1000.0;
        let start = orbit_state(center, radius, 0.0, 100.0, 1000.0);
        let mut ctrl = L1Controller::from_plan(
            &start,
            plan(vec![
                orbit(0.0, 0.0, radius, Some(1.0)),
                wpt(2000.0, 0.0, 200.0),
            ]),
            &ControlInputs::default(),
        );

        let steps = 400;
        // March most of one revolution; must NOT have advanced yet at 0.9 turns.
        for k in 0..(steps * 9 / 10) {
            let theta = TWO_PI * (k as f32) / (steps as f32);
            let s = orbit_state(center, radius, theta, 100.0, 1000.0);
            ctrl.update(&s, &ctx(), 1.0 / 64.0);
        }
        assert_eq!(
            ctrl.leg_index, 0,
            "should still be orbiting before one full turn"
        );

        // Complete just past one full revolution.
        for k in (steps * 9 / 10)..=(steps + steps / 20) {
            let theta = TWO_PI * (k as f32) / (steps as f32);
            let s = orbit_state(center, radius, theta, 100.0, 1000.0);
            ctrl.update(&s, &ctx(), 1.0 / 64.0);
        }
        assert_eq!(ctrl.leg_index, 1, "should advance after one full turn");
    }

    #[test]
    fn orbit_turns_none_holds_forever() {
        let center = Vec2::new(0.0, 0.0);
        let radius = 1000.0;
        let start = orbit_state(center, radius, 0.0, 100.0, 1000.0);
        let mut ctrl = L1Controller::from_plan(
            &start,
            plan(vec![orbit(0.0, 0.0, radius, None)]),
            &ControlInputs::default(),
        );

        // Three full revolutions: never advances, never finishes.
        let steps = 400;
        for k in 0..=(steps * 3) {
            let theta = TWO_PI * (k as f32) / (steps as f32);
            let s = orbit_state(center, radius, theta, 100.0, 1000.0);
            ctrl.update(&s, &ctx(), 1.0 / 64.0);
        }
        assert_eq!(ctrl.leg_index, 0);
        assert_eq!(ctrl.phase, L1Phase::Following);
    }
}
