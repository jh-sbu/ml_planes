use std::f32::consts::FRAC_PI_3;

use bevy::prelude::{Component, Vec3};

use crate::controllers::guidance::ground_heading;
use crate::controllers::level_hold::LevelHoldController;
use crate::controllers::pid::PidController;
use crate::controllers::traits::FlightController;
use crate::plane::{ControlInputs, ControllerContext, FlightState, PlaneId};

// ---------------------------------------------------------------------------
// Components

/// Desired formation slot in the leader's body frame.
/// Axes: +X = forward (nose), +Y = right wing, +Z = up cockpit.
/// Default: 20 m behind, 15 m to the right, same altitude.
#[derive(Component, Clone, Debug)]
pub struct FormationOffset {
    pub offset_body: Vec3,
}

impl Default for FormationOffset {
    fn default() -> Self {
        Self {
            offset_body: Vec3::new(-20.0, 15.0, 0.0),
        }
    }
}

/// Last-tick diagnostics published by `WingmanController::update()` for display
/// in the HUD. Not used by the control law — purely observational telemetry.
///
/// The three component fields are the projections each cascade channel actually
/// drives on (leader-right, leader-forward, world-Y); they are a channel-relevant
/// view, not a strict orthogonal decomposition of `pos_error_mag`.
#[derive(Clone, Debug, Default)]
pub struct WingmanDiagnostics {
    /// Was the leader present in `ControllerContext` this tick? When false, the
    /// geometric fields below are stale (last good value).
    pub leader_found: bool,
    /// World-frame slot error (`target_pos - own.position`) [m].
    pub pos_error: Vec3,
    /// `|pos_error|` [m].
    pub pos_error_mag: f32,
    /// Cross-track component along the leader's right wing (`lateral_error`) [m].
    pub cross_track: f32,
    /// Fore-aft component along the leader's heading (`range_error`) [m].
    pub range_error: f32,
    /// Vertical component (`pos_error.y`) [m].
    pub altitude_error: f32,
}

// ---------------------------------------------------------------------------
// WingmanController

/// A flight controller that holds a fixed formation offset relative to a lead plane.
///
/// Uses a cascade guidance law:
///   - Altitude target  ← formation geometry (world Y of desired slot)
///   - Airspeed target  ← leader airspeed + fore-aft range correction
///   - Bank angle cmd    ← two-stage lateral cascade (see below)
/// All stabilization is delegated to an inner `LevelHoldController`.
///
/// **Lateral cascade (heading-damped).** Rather than mapping lateral position
/// error straight to bank — which has no heading damping and spirals away once
/// perturbed — the lateral channel mirrors the orbit/L1 controllers:
///   1. lateral cross-track error [m] → `lateral_pid` → heading offset [rad]
///      (a crab angle that points the demanded ground track back toward the slot)
///   2. heading error (demanded heading vs. own ground track) → `heading_pid`
///      → commanded bank [rad]
/// Driving bank from *heading* error (with `heading_pid`'s `kd` damping the
/// heading rate) lets the command relax as the heading aligns, before the slot
/// is overshot — making the on-slot equilibrium stable over multi-minute holds.
///
/// The leader is identified by `leader_id`; its current state is read from
/// `ControllerContext` each tick — no ECS side-channel required.
pub struct WingmanController {
    /// Stable id of the leader plane. Set at construction; may be updated via
    /// `downcast_mut` when the formation is reconfigured (one-time edit, not per-tick).
    pub leader_id: PlaneId,
    /// Desired offset in leader body frame [m].
    pub offset: FormationOffset,
    /// Inner stabilization controller. Its target_altitude, target_airspeed,
    /// and target_roll are overwritten each tick by the guidance law.
    pub inner: LevelHoldController,
    /// Fore-aft range error [m] → Δairspeed [m/s].
    /// kp=0.2, ki=0.02, kd=0.5, clamp=15, limits=±10 m/s.
    pub range_pid: PidController,
    /// Lateral cross-track error [m] → demanded heading offset [rad] (cascade
    /// stage 1). Matches the orbit controller's radial_pid: kp=0.002, kd=0.01,
    /// limits=±0.5 rad — gentle enough to stay within the slow heading/roll
    /// inner-loop bandwidth (a larger kp drives a sustained lateral oscillation).
    pub lateral_pid: PidController,
    /// Heading error [rad] → commanded bank angle [rad] (cascade stage 2; the
    /// `kd` term supplies the heading-rate damping). Mirrors the orbit
    /// controller's `heading_pid`: kp=0.7, kd=0.1, limits=±π/3.
    pub heading_pid: PidController,
    /// Last-tick diagnostics for HUD display (written by `update`, read-only
    /// for the control law).
    pub diagnostics: WingmanDiagnostics,
}

impl WingmanController {
    /// Construct a wingman controller.
    ///
    /// `leader_id`      — stable `PlaneId` of the designated leader.
    /// `leader_initial` — approximate initial leader state (used to seed the
    ///                    inner controller's targets for a bumpless start).
    /// `own_initial`    — wingman's own initial `FlightState`.
    /// `offset`         — formation slot in leader body frame.
    pub fn new(
        leader_id: PlaneId,
        leader_initial: &FlightState,
        own_initial: &FlightState,
        offset: FormationOffset,
    ) -> Self {
        let mut inner = LevelHoldController::from_state(own_initial, &ControlInputs::default());
        // Pre-seed altitude/airspeed targets from formation geometry so the
        // inner PIDs don't see a step command on the very first tick.
        let desired_pos = leader_initial.position + leader_initial.attitude * offset.offset_body;
        inner.target_altitude = desired_pos.y;
        inner.target_airspeed = leader_initial.airspeed;

        Self {
            leader_id,
            offset,
            inner,
            range_pid: PidController::new(0.2, 0.02, 0.5, 15.0, -10.0, 10.0),
            lateral_pid: PidController::new(0.002, 0.0, 0.01, 0.5, -0.5, 0.5),
            heading_pid: PidController::new(0.7, 0.0, 0.1, 0.0, -FRAC_PI_3, FRAC_PI_3),
            diagnostics: WingmanDiagnostics::default(),
        }
    }
}

impl FlightController for WingmanController {
    fn update(&mut self, own: &FlightState, ctx: &ControllerContext, dt: f32) -> ControlInputs {
        let Some(leader_snap) = ctx.find(self.leader_id) else {
            // Leader not in context — hold current targets in inner controller.
            // Mark diagnostics stale; geometric fields keep their last value.
            self.diagnostics.leader_found = false;
            return self.inner.update(own, ctx, dt);
        };
        let leader = &leader_snap.state;

        // 1. Desired formation position in world frame.
        let target_pos = leader.position + leader.attitude * self.offset.offset_body;
        let pos_error = target_pos - own.position;

        // 2. Altitude: set inner target directly from formation geometry.
        self.inner.target_altitude = target_pos.y;

        // 3. Range (fore-aft along leader heading) → airspeed correction.
        let leader_fwd = leader.attitude * Vec3::X;
        let range_error = pos_error.dot(leader_fwd);
        let delta_spd = self.range_pid.update(range_error, dt);
        self.inner.target_airspeed = (leader.airspeed + delta_spd).max(20.0);

        // 4. Lateral cascade (heading-damped) → bank command.
        //    Stage 1: cross-track error → demanded heading offset (crab angle).
        let leader_right = leader.attitude * Vec3::Y;
        let lateral_error = pos_error.dot(leader_right);
        let heading_offset = self.lateral_pid.update(lateral_error, dt);

        //    Rotate the leader's ground-track heading by -offset so a positive
        //    cross-track error (wingman on the leader's right) crabs the demanded
        //    heading back toward the slot. (XZ Vec2 maps x→X, y→Z.)
        let lead_head = ground_heading(leader);
        let (so, co) = (-heading_offset).sin_cos();
        let desired_x = co * lead_head.x - so * lead_head.y;
        let desired_z = so * lead_head.x + co * lead_head.y;

        //    Stage 2: heading error (demanded vs. own ground track) → bank.
        //    Same signed-angle math and `-heading_pid` output sign as
        //    guidance::orbit_bank_command. heading_pid's kd damps the heading
        //    rate — the term the old direct position→bank loop lacked. This is
        //    negative (slot-restoring) feedback: a cross-track error crabs the
        //    demanded heading toward the slot and banks to fly that heading.
        let head = ground_heading(own);
        let cross = desired_x * head.y - desired_z * head.x;
        let dot = desired_x * head.x + desired_z * head.y;
        let heading_error = cross.atan2(dot);
        self.inner.target_roll = -self.heading_pid.update(heading_error, dt);

        // 5. Publish diagnostics for the HUD (observational only).
        self.diagnostics = WingmanDiagnostics {
            leader_found: true,
            pos_error,
            pos_error_mag: pos_error.length(),
            cross_track: lateral_error,
            range_error,
            altitude_error: pos_error.y,
        };

        // 6. Delegate stabilization to inner LevelHoldController.
        self.inner.update(own, ctx, dt)
    }

    fn name(&self) -> &'static str {
        "Wingman"
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

// ---------------------------------------------------------------------------
// Tests

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::Quat;
    use std::sync::Arc;

    use crate::plane::{PlaneId, PlaneSnapshot};

    fn level_state(pos: Vec3, airspeed: f32) -> FlightState {
        FlightState {
            position: pos,
            velocity: Vec3::new(airspeed, 0.0, 0.0),
            attitude: Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2),
            airspeed,
            altitude: pos.y,
            ..Default::default()
        }
    }

    fn make_ctx(
        own_id: PlaneId,
        own: &FlightState,
        leader_id: Option<PlaneId>,
        leader: Option<&FlightState>,
    ) -> ControllerContext {
        let mut snaps = vec![PlaneSnapshot {
            id: own_id,
            state: own.clone(),
        }];
        if let (Some(lid), Some(ls)) = (leader_id, leader) {
            snaps.push(PlaneSnapshot {
                id: lid,
                state: ls.clone(),
            });
        }
        ControllerContext {
            own_id,
            planes: Arc::from(snaps),
        }
    }

    const LEADER_ID: PlaneId = PlaneId(1);
    const OWN_ID: PlaneId = PlaneId(2);

    #[test]
    fn fallback_with_no_leader_in_ctx_returns_finite_outputs() {
        let own = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
        let leader_initial = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
        let mut ctrl =
            WingmanController::new(LEADER_ID, &leader_initial, &own, FormationOffset::default());

        // ctx contains only own plane — no leader snapshot.
        let ctx = make_ctx(OWN_ID, &own, None, None);
        let inputs = ctrl.update(&own, &ctx, 1.0 / 60.0);
        assert!(inputs.elevator.is_finite());
        assert!(inputs.throttle.is_finite());
        assert!(inputs.aileron.is_finite());
        assert!(inputs.rudder.is_finite());
    }

    #[test]
    fn zero_position_error_produces_minimal_corrections() {
        // Leader at (0, 1000, 0); offset (-20, 15, 0) body frame.
        // With rotation_x(-π/2): body +X maps to world +X, body +Y maps to world +Z,
        // body +Z maps to world -Y. So desired slot = (0,1000,0) + (-20, 0, 15) = (-20, 1000, 15).
        let leader = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
        let offset = FormationOffset {
            offset_body: Vec3::new(-20.0, 15.0, 0.0),
        };
        let desired = leader.position + leader.attitude * offset.offset_body;
        let own = level_state(desired, 100.0);

        let mut ctrl = WingmanController::new(LEADER_ID, &leader, &own, offset);
        let ctx = make_ctx(OWN_ID, &own, Some(LEADER_ID), Some(&leader));
        let dt = 1.0 / 60.0;
        let inputs = ctrl.update(&own, &ctx, dt);

        // PIDs start at zero, so first-tick corrections should be small.
        assert!(inputs.aileron.abs() < 0.5, "aileron={}", inputs.aileron);
    }

    #[test]
    fn lateral_error_commands_restoring_bank_direction() {
        // Pin the closed-loop sign: a cross-track offset must command a bank that
        // turns the wingman *back* toward the slot (negative feedback). The old
        // pure position→bank loop used the opposite (divergent) sign, which is
        // why it spiraled away from any offset; this test guards the fix.
        let leader = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
        let offset = FormationOffset {
            offset_body: Vec3::new(-20.0, 15.0, 0.0),
        };
        let desired = leader.position + leader.attitude * offset.offset_body;

        // Shift the wingman 5 m along +leader_right (the leader's right side of
        // the slot). leader_right maps to world -Z here, so this is the -Z side;
        // to recover it must turn back toward +Z, which is a positive target_roll
        // (positive roll turns the ground track toward +Z).
        let leader_right = leader.attitude * Vec3::Y;
        let own_pos = desired + leader_right * 5.0;
        let own = level_state(own_pos, 100.0);

        let mut ctrl = WingmanController::new(LEADER_ID, &leader, &own, offset);
        let ctx = make_ctx(OWN_ID, &own, Some(LEADER_ID), Some(&leader));
        ctrl.update(&own, &ctx, 1.0 / 60.0);

        assert!(
            ctrl.inner.target_roll > 0.0,
            "expected restoring (+Z, slot-ward) bank for a right-of-slot offset, target_roll={}",
            ctrl.inner.target_roll
        );
    }

    #[test]
    fn heading_misalignment_commands_corrective_bank() {
        // Wingman sits exactly on-slot (zero position error) but its ground track
        // is yawed away from the leader's heading. A pure lateral-position→bank
        // loop would command zero bank here (no position error); the heading-
        // damped cascade must command a non-zero corrective bank to re-align.
        let leader = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
        let offset = FormationOffset {
            offset_body: Vec3::new(-20.0, 15.0, 0.0),
        };
        let desired = leader.position + leader.attitude * offset.offset_body;

        // On-slot position, but velocity heading rotated ~0.3 rad toward +Z.
        let a = 0.3_f32;
        let (s, c) = a.sin_cos();
        let mut own = level_state(desired, 100.0);
        own.velocity = Vec3::new(100.0 * c, 0.0, 100.0 * s);

        let mut ctrl = WingmanController::new(LEADER_ID, &leader, &own, offset);
        let ctx = make_ctx(OWN_ID, &own, Some(LEADER_ID), Some(&leader));
        ctrl.update(&own, &ctx, 1.0 / 60.0);

        assert!(
            ctrl.inner.target_roll.abs() > 0.01,
            "expected corrective bank for heading misalignment, target_roll={}",
            ctrl.inner.target_roll
        );
    }

    #[test]
    fn update_populates_diagnostics_when_leader_present() {
        // Wingman shifted a known 5 m off the slot along +leader_right (the
        // cross-track axis). Diagnostics must record leader_found and the error.
        let leader = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
        let offset = FormationOffset {
            offset_body: Vec3::new(-20.0, 15.0, 0.0),
        };
        let desired = leader.position + leader.attitude * offset.offset_body;

        let leader_right = leader.attitude * Vec3::Y;
        let own_pos = desired + leader_right * 5.0;
        let own = level_state(own_pos, 100.0);

        let mut ctrl = WingmanController::new(LEADER_ID, &leader, &own, offset);
        let ctx = make_ctx(OWN_ID, &own, Some(LEADER_ID), Some(&leader));
        ctrl.update(&own, &ctx, 1.0 / 60.0);

        let d = &ctrl.diagnostics;
        assert!(d.leader_found, "leader should be found in ctx");
        assert!(
            (d.pos_error_mag - 5.0).abs() < 1e-3,
            "pos_error_mag={}",
            d.pos_error_mag
        );
        // The 5 m offset is purely cross-track. pos_error = target - own, so an
        // own position +5 m along leader_right yields cross_track = -5 m.
        assert!(
            (d.cross_track + 5.0).abs() < 1e-3,
            "cross_track={}",
            d.cross_track
        );
        assert!(d.range_error.abs() < 1e-3, "range_error={}", d.range_error);
        assert!(
            d.altitude_error.abs() < 1e-3,
            "altitude_error={}",
            d.altitude_error
        );
    }

    #[test]
    fn update_marks_leader_not_found_in_fallback() {
        let own = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
        let leader_initial = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
        let mut ctrl =
            WingmanController::new(LEADER_ID, &leader_initial, &own, FormationOffset::default());

        // ctx contains only own plane — no leader snapshot.
        let ctx = make_ctx(OWN_ID, &own, None, None);
        ctrl.update(&own, &ctx, 1.0 / 60.0);

        assert!(
            !ctrl.diagnostics.leader_found,
            "leader should be marked not found"
        );
    }

    #[test]
    fn range_error_commands_higher_airspeed() {
        // Wingman is 10 m too far behind leader — should speed up.
        let leader = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
        let offset = FormationOffset {
            offset_body: Vec3::new(-20.0, 15.0, 0.0),
        };
        let desired = leader.position + leader.attitude * offset.offset_body;

        // Move wingman 10 m further behind (negative leader forward).
        let leader_fwd = leader.attitude * Vec3::X;
        let own_pos = desired - leader_fwd * 10.0;
        let own = level_state(own_pos, 100.0);

        let mut ctrl = WingmanController::new(LEADER_ID, &leader, &own, offset);
        let ctx = make_ctx(OWN_ID, &own, Some(LEADER_ID), Some(&leader));
        ctrl.update(&own, &ctx, 1.0 / 60.0);

        assert!(
            ctrl.inner.target_airspeed > leader.airspeed,
            "target_airspeed={}",
            ctrl.inner.target_airspeed,
        );
    }
}
