use bevy::prelude::{Component, Entity, Vec3};

use crate::controllers::component::ActiveController;
use crate::controllers::level_hold::LevelHoldController;
use crate::controllers::pid::PidController;
use crate::controllers::traits::FlightController;
use crate::plane::{ControlInputs, FlightState};

// ---------------------------------------------------------------------------
// Components

/// Marks a wingman entity. Holds the ECS Entity of its designated leader.
/// Used by `feed_leader_state` to look up the leader's FlightState each tick.
#[derive(Component)]
pub struct LeaderRef(pub Entity);

/// Scratch-pad component on wingman entities.
/// Written by `feed_leader_state` before `run_flight_controllers`; not read
/// directly by the controller (it uses `cached_leader` inside the struct).
#[derive(Component, Default, Clone)]
pub struct LeaderState(pub Option<FlightState>);

/// Desired formation slot in the leader's body frame.
/// Axes: +X = forward (nose), +Y = right wing, +Z = up cockpit.
/// Default: 20 m behind, 15 m to the right, same altitude.
#[derive(Component, Clone, Debug)]
pub struct FormationOffset {
    pub offset_body: Vec3,
}

impl Default for FormationOffset {
    fn default() -> Self {
        Self { offset_body: Vec3::new(-20.0, 15.0, 0.0) }
    }
}

// ---------------------------------------------------------------------------
// WingmanController

/// A flight controller that holds a fixed formation offset relative to a lead plane.
///
/// Uses a cascade guidance law:
///   - Altitude target  ← formation geometry (world Y of desired slot)
///   - Airspeed target  ← leader airspeed + fore-aft range correction
///   - Bank angle cmd   ← lateral cross-track correction
/// All stabilization is delegated to an inner `LevelHoldController`.
pub struct WingmanController {
    /// Most-recently-injected leader FlightState. Written by the ECS system
    /// `feed_leader_state` before each controller tick.
    pub cached_leader: Option<FlightState>,
    /// Desired offset in leader body frame [m].
    pub offset: FormationOffset,
    /// Inner stabilization controller. Its target_altitude, target_airspeed,
    /// and target_roll are overwritten each tick by the guidance law.
    pub inner: LevelHoldController,
    /// Fore-aft range error [m] → Δairspeed [m/s].
    /// kp=0.2, ki=0.02, kd=0.5, clamp=15, limits=±10 m/s.
    pub range_pid: PidController,
    /// Lateral cross-track error [m] → commanded bank angle [rad].
    /// kp=0.008, ki=0.001, kd=0.04, clamp=0.5, limits=±0.52 rad (≈±30°).
    pub lateral_pid: PidController,
}

impl WingmanController {
    /// Construct a wingman controller.
    ///
    /// `leader_initial` — approximate initial leader state (used to seed the
    /// inner controller's targets for a bumpless start).
    /// `own_initial`    — wingman's own initial FlightState.
    /// `offset`         — formation slot in leader body frame.
    pub fn new(
        leader_initial: &FlightState,
        own_initial: &FlightState,
        offset: FormationOffset,
    ) -> Self {
        let mut inner = LevelHoldController::from_state(own_initial, &ControlInputs::default());
        // Pre-seed altitude/airspeed targets from formation geometry so the
        // inner PIDs don't see a step command on the very first tick.
        let desired_pos = leader_initial.position
            + leader_initial.attitude * offset.offset_body;
        inner.target_altitude = desired_pos.y;
        inner.target_airspeed = leader_initial.airspeed;

        Self {
            cached_leader: Some(leader_initial.clone()),
            offset,
            inner,
            range_pid:   PidController::new(0.2,   0.02, 0.5,  15.0, -10.0, 10.0),
            lateral_pid: PidController::new(0.008, 0.001, 0.04,  0.5,  -0.52, 0.52),
        }
    }
}

impl FlightController for WingmanController {
    fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs {
        let Some(ref leader) = self.cached_leader.clone() else {
            // No leader data yet — hold current targets in inner controller.
            return self.inner.update(state, dt);
        };

        // 1. Desired formation position in world frame.
        let target_pos = leader.position + leader.attitude * self.offset.offset_body;
        let pos_error  = target_pos - state.position;

        // 2. Altitude: set inner target directly from formation geometry.
        self.inner.target_altitude = target_pos.y;

        // 3. Range (fore-aft along leader heading) → airspeed correction.
        let leader_fwd  = leader.attitude * Vec3::X;
        let range_error = pos_error.dot(leader_fwd);
        let delta_spd   = self.range_pid.update(range_error, dt);
        self.inner.target_airspeed = (leader.airspeed + delta_spd).max(20.0);

        // 4. Lateral (cross-track along leader right-wing axis) → bank command.
        let leader_right  = leader.attitude * Vec3::Y;
        let lateral_error = pos_error.dot(leader_right);
        self.inner.target_roll = self.lateral_pid.update(lateral_error, dt);

        // 5. Delegate stabilization to inner LevelHoldController.
        self.inner.update(state, dt)
    }

    fn name(&self) -> &'static str { "Wingman" }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any { self }
}

// ---------------------------------------------------------------------------
// ECS system

/// Pre-controller system: reads the leader plane's `FlightState` and injects
/// it into each wingman's `WingmanController` via `cached_leader`.
///
/// Must run AFTER `sync_flight_state` (so leader state is current) and
/// BEFORE `run_flight_controllers` (so the wingman sees fresh data).
pub fn feed_leader_state(
    leader_query: bevy::prelude::Query<&FlightState>,
    mut wingman_query: bevy::prelude::Query<(&LeaderRef, &mut ActiveController)>,
) {
    for (leader_ref, mut controller) in wingman_query.iter_mut() {
        let leader_state = leader_query.get(leader_ref.0).ok().cloned();

        if let Some(wc) = controller.0.as_any_mut().downcast_mut::<WingmanController>() {
            wc.cached_leader = leader_state;
        }
    }
}

// ---------------------------------------------------------------------------
// Tests

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::Quat;

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

    #[test]
    fn fallback_with_no_leader_returns_finite_outputs() {
        let own = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
        let mut ctrl = WingmanController::new(&own, &own, FormationOffset::default());
        ctrl.cached_leader = None;

        let inputs = ctrl.update(&own, 1.0 / 60.0);
        assert!(inputs.elevator.is_finite());
        assert!(inputs.throttle.is_finite());
        assert!(inputs.aileron.is_finite());
        assert!(inputs.rudder.is_finite());
    }

    #[test]
    fn zero_position_error_produces_minimal_corrections() {
        // Leader at (0, 1000, 0); offset (-20, 15, 0) body frame.
        // At level flight with identity-ish attitude, desired slot ≈ (-20, 1000, 15).
        // Spawn wingman exactly at that slot — error should be zero.
        let leader = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
        let offset = FormationOffset { offset_body: Vec3::new(-20.0, 15.0, 0.0) };

        // With rotation_x(-π/2): body +X maps to world +X, body +Y maps to world +Z,
        // body +Z maps to world -Y. So desired slot = (0,1000,0) + (-20, 0, 15) = (-20, 1000, 15).
        // (The attitude rotates body Y (+right) to world Z.)
        let desired = leader.position + leader.attitude * offset.offset_body;
        let own = level_state(desired, 100.0);

        let mut ctrl = WingmanController::new(&leader, &own, offset);
        let dt = 1.0 / 60.0;
        let inputs = ctrl.update(&own, dt);

        // PIDs start at zero, so first-tick corrections should be small.
        assert!(inputs.aileron.abs() < 0.5, "aileron={}", inputs.aileron);
    }

    #[test]
    fn lateral_error_commands_correct_bank_direction() {
        // Wingman is 5 m too far left (negative lateral error relative to leader right).
        let leader = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
        let offset = FormationOffset { offset_body: Vec3::new(-20.0, 15.0, 0.0) };
        let desired = leader.position + leader.attitude * offset.offset_body;

        // Shift wingman 5 m to the left (world -Z with this attitude convention).
        let leader_right = leader.attitude * Vec3::Y; // world right axis
        let own_pos = desired - leader_right * 5.0;
        let own = level_state(own_pos, 100.0);

        let mut ctrl = WingmanController::new(&leader, &own, offset);
        ctrl.update(&own, 1.0 / 60.0);

        // Positive lateral error (wingman left of slot) should command positive bank (right).
        assert!(ctrl.inner.target_roll > 0.0, "target_roll={}", ctrl.inner.target_roll);
    }

    #[test]
    fn range_error_commands_higher_airspeed() {
        // Wingman is 10 m too far behind leader — should speed up.
        let leader = level_state(Vec3::new(0.0, 1000.0, 0.0), 100.0);
        let offset = FormationOffset { offset_body: Vec3::new(-20.0, 15.0, 0.0) };
        let desired = leader.position + leader.attitude * offset.offset_body;

        // Move wingman 10 m further behind (negative leader forward).
        let leader_fwd = leader.attitude * Vec3::X;
        let own_pos = desired - leader_fwd * 10.0;
        let own = level_state(own_pos, 100.0);

        let mut ctrl = WingmanController::new(&leader, &own, offset);
        ctrl.update(&own, 1.0 / 60.0);

        assert!(
            ctrl.inner.target_airspeed > leader.airspeed,
            "target_airspeed={}", ctrl.inner.target_airspeed,
        );
    }
}
