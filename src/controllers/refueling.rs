//! Staged aerial-refueling approach controller (PID).
//!
//! Flies a receiver from wherever it starts up to a docking position astern of a
//! tanker, through three stations, and breaks away when it loses the one it is
//! currently flying.
//!
//! The inner guidance is [`WingmanController`](crate::controllers::WingmanController)'s
//! cascade verbatim — altitude from the station's world Y, fore-aft range through an
//! airspeed correction, and a heading-damped lateral cascade (cross-track → demanded
//! heading offset → heading error → bank). What differs is *where* it aims:
//!
//! # Stations
//!
//! All three are offsets in the **tanker's** body frame (+X forward, +Y right, +Z up),
//! the same convention as [`FormationOffset`](crate::controllers::FormationOffset):
//! `Astern` (safe trail, also the recovery station), `Precontact` (closed up and
//! stabilized), `Contact` (the docking position). See [`RefuelConfig`] for the numbers.
//!
//! # Rate-limited station transit
//!
//! The commanded station is **not** stepped from one phase to the next — it is moved
//! toward the new phase's station at [`RefuelConfig::approach_rate`] m/s (and at
//! `breakaway_rate` on an abort). This is what makes closure a controlled quantity: a
//! stepped station would hand the inner loops a 110 m position error and let the range
//! PID pick whatever closure rate its output limit allows.
//!
//! # Two errors, two decisions
//!
//! Conflating these is the easy mistake here, so they are named apart:
//!
//! * `tracking_error` — distance to the **commanded** (mid-transit) station: how well the
//!   receiver is flying what it was actually asked to fly. Drives the **breakaway**.
//! * `phase_error` — distance to the phase's **final** station, still large while the
//!   transit runs. Drives the **advance**, so a phase cannot be advanced early.
//!
//! Judging the breakaway on `phase_error` would fire it spuriously: the Precontact→Contact
//! transit is ~25 m long, the same order as any sane abort radius.

use std::f32::consts::FRAC_PI_3;

use bevy::prelude::Vec3;

use crate::controllers::guidance::ground_heading;
use crate::controllers::level_hold::LevelHoldController;
use crate::controllers::pid::PidController;
use crate::controllers::traits::FlightController;
use crate::plane::{ControlInputs, ControllerContext, FlightState, PlaneId};

// ---------------------------------------------------------------------------
// Phase

/// Which station the receiver is currently being flown to.
///
/// Advances `Astern → Precontact → Contact` when the advance gate holds; a breakaway
/// from either of the last two drops straight back to `Astern` (never one step back —
/// the point of the recovery station is to be unambiguously clear of the tanker).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "net", derive(serde::Serialize, serde::Deserialize))]
pub enum RefuelPhase {
    /// Safe trail well behind and below. Capture station and recovery station.
    #[default]
    Astern,
    /// Closed up, aligned, stabilized — the last station before the boom envelope.
    Precontact,
    /// The docking position. Terminal: nothing advances past it.
    Contact,
}

impl RefuelPhase {
    /// Display label for the HUD.
    pub fn label(self) -> &'static str {
        match self {
            RefuelPhase::Astern => "Astern",
            RefuelPhase::Precontact => "Precontact",
            RefuelPhase::Contact => "Contact",
        }
    }
}

// ---------------------------------------------------------------------------
// Config

/// The resolved, per-airframe approach configuration: the three stations, the gates
/// between them, the station-transit rates, and the outer cascade gains.
///
/// This is [`RefuelingTuning`](crate::controllers::tuning::RefuelingTuning) minus its
/// inner level-hold block — the part a `RefuelController` actually needs at runtime.
/// `Copy`, so the tuning-rebuild path in `controllers::sim_control` can snapshot and
/// replace it without any allocation.
///
/// **Every default below is a starter value**, sized off the tanker's ~130 m/s cruise
/// and the wingman's proven behaviour, not flight-test data. Validate with
/// `examples/observe_state.rs` against `assets/scenarios/refueling.scenario.ron` before
/// trusting any of them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RefuelConfig {
    /// Safe-trail station, tanker body frame [m].
    pub astern: Vec3,
    /// Closed-up station, tanker body frame [m].
    pub precontact: Vec3,
    /// Docking station, tanker body frame [m].
    pub contact: Vec3,

    /// `Astern → Precontact` gate: `phase_error` must be under this [m].
    pub capture_radius: f32,
    /// `Precontact → Contact` gate: `phase_error` must be under this [m].
    pub contact_radius: f32,
    /// Both gates additionally require `|closure_rate|` under this [m/s].
    pub closure_tolerance: f32,
    /// The gate must hold continuously for this long before the phase advances [s].
    pub dwell_secs: f32,
    /// `tracking_error` above this outside `Astern` triggers a breakaway [m].
    pub abort_radius: f32,

    /// Rate the commanded station moves toward a new phase's station [m/s].
    pub approach_rate: f32,
    /// Rate it retreats to `Astern` after a breakaway [m/s]. Faster than `approach_rate`.
    pub breakaway_rate: f32,

    /// Fore-aft range error [m] → Δairspeed [m/s].
    pub range_kp: f32,
    pub range_ki: f32,
    pub range_kd: f32,
    /// Lateral cross-track error [m] → demanded heading offset [rad].
    ///
    /// Inherited from the wingman, and `lateral_kp` in particular should stay there:
    /// much above 0.002 re-introduces a sustained lateral oscillation, because the
    /// inner heading/roll loop is comparatively slow. Tighter station keeping at
    /// `Contact` is bought with a closer, rate-limited station — not with more gain.
    pub lateral_kp: f32,
    pub lateral_kd: f32,
    /// Heading error [rad] → commanded bank [rad]; `heading_kd` damps the heading rate.
    pub heading_kp: f32,
    pub heading_kd: f32,
}

impl Default for RefuelConfig {
    fn default() -> Self {
        Self {
            // 150 m astern / 30 m below, on the centerline.
            astern: Vec3::new(-150.0, 0.0, -30.0),
            precontact: Vec3::new(-40.0, 0.0, -10.0),
            contact: Vec3::new(-15.0, 0.0, -5.0),

            capture_radius: 12.0,
            contact_radius: 5.0,
            closure_tolerance: 2.0,
            dwell_secs: 3.0,
            abort_radius: 30.0,

            approach_rate: 4.0,
            breakaway_rate: 12.0,

            // Gains from `WingmanController::from_inner` — a proven formation cascade.
            range_kp: 0.2,
            range_ki: 0.02,
            range_kd: 0.5,
            lateral_kp: 0.002,
            lateral_kd: 0.01,
            heading_kp: 0.7,
            heading_kd: 0.1,
        }
    }
}

impl RefuelConfig {
    /// The station one phase flies to, tanker body frame.
    pub fn station(&self, phase: RefuelPhase) -> Vec3 {
        match phase {
            RefuelPhase::Astern => self.astern,
            RefuelPhase::Precontact => self.precontact,
            RefuelPhase::Contact => self.contact,
        }
    }

    /// The `phase_error` gate guarding the advance *out of* `phase`; `None` for the
    /// terminal `Contact`.
    fn advance_radius(&self, phase: RefuelPhase) -> Option<f32> {
        match phase {
            RefuelPhase::Astern => Some(self.capture_radius),
            RefuelPhase::Precontact => Some(self.contact_radius),
            RefuelPhase::Contact => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Diagnostics

/// Last-tick diagnostics published by [`RefuelController::update`] for the HUD.
/// Not read by the control law — purely observational.
#[derive(Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "net", derive(serde::Serialize, serde::Deserialize))]
pub struct RefuelDiagnostics {
    /// Was the tanker present in `ControllerContext` this tick? When false, every
    /// geometric field below is stale (last good value) and the phase is frozen.
    pub tanker_found: bool,
    /// Station currently being flown to.
    pub phase: RefuelPhase,
    /// World-frame error against the **commanded** (mid-transit) station [m].
    pub pos_error: Vec3,
    /// `|pos_error|` — the `tracking_error` the breakaway reads [m].
    pub pos_error_mag: f32,
    /// Component along the tanker's right wing [m].
    pub cross_track: f32,
    /// Component along the tanker's heading [m].
    pub range_error: f32,
    /// Vertical component [m].
    pub altitude_error: f32,
    /// Closure on the tanker along its forward axis [m/s]; positive = closing.
    pub closure_rate: f32,
    /// Distance to the phase's **final** station — what the advance gate reads [m].
    pub phase_error: f32,
    /// Monotone count of breakaways since construction.
    ///
    /// A counter rather than a flag because it is what makes "did this approach
    /// oscillate at any point in three minutes?" a single assertion instead of a
    /// per-tick sampling loop. An approach that reaches `Contact` and sits there is
    /// indistinguishable, at the final tick, from one that fell out and re-climbed
    /// six times.
    pub breakaways: u32,
}

// ---------------------------------------------------------------------------
// Controller

/// Move `from` toward `to` by at most `max_step`.
fn move_toward(from: Vec3, to: Vec3, max_step: f32) -> Vec3 {
    let delta = to - from;
    let len = delta.length();
    if len <= max_step || len < 1e-6 {
        to
    } else {
        from + delta * (max_step / len)
    }
}

/// Staged refueling approach: see the module docs for the stations, the rate-limited
/// station transit, and the tracking-vs-phase error split.
pub struct RefuelController {
    /// Stable id of the tanker. Read from `ControllerContext` each tick.
    pub tanker_id: PlaneId,
    /// Which station is being flown to.
    pub phase: RefuelPhase,
    /// The station actually commanded this tick, tanker body frame [m]. Rate-limited
    /// toward `config.station(phase)` so the inner loops never see a step.
    pub commanded_offset: Vec3,
    /// True while retreating to `Astern` after a breakaway (uses `breakaway_rate`).
    pub retreating: bool,
    /// Seconds the advance gate has held continuously.
    pub dwell: f32,
    /// Monotone breakaway count; mirrored into [`RefuelDiagnostics::breakaways`].
    pub breakaways: u32,
    /// Stations, gates, rates, and outer gains.
    pub config: RefuelConfig,
    /// Inner stabilization. Its `target_altitude`/`target_airspeed`/`target_roll` are
    /// overwritten every tick the tanker is found.
    pub inner: LevelHoldController,
    /// Fore-aft range error [m] → Δairspeed [m/s].
    pub range_pid: PidController,
    /// Cross-track error [m] → demanded heading offset [rad] (cascade stage 1).
    pub lateral_pid: PidController,
    /// Heading error [rad] → commanded bank [rad] (cascade stage 2).
    pub heading_pid: PidController,
    /// Last-tick diagnostics for the HUD.
    pub diagnostics: RefuelDiagnostics,
}

impl RefuelController {
    /// Construct a receiver starting at the `Astern` station.
    ///
    /// `tanker_initial` seeds the inner controller's targets from the astern station's
    /// geometry so the inner PIDs don't see a step on the very first tick — the same
    /// pre-seed `WingmanController::new` does.
    pub fn new(
        tanker_id: PlaneId,
        tanker_initial: &FlightState,
        own_initial: &FlightState,
        config: RefuelConfig,
    ) -> Self {
        let mut inner = LevelHoldController::from_state(own_initial, &ControlInputs::default());
        let desired_pos = tanker_initial.position + tanker_initial.attitude * config.astern;
        inner.target_altitude = desired_pos.y;
        inner.target_airspeed = tanker_initial.airspeed;

        Self::from_inner(tanker_id, RefuelPhase::Astern, config.astern, config, inner)
    }

    /// Construct a receiver from a full [`RefuelingTuning`] profile, applying the
    /// profile's `inner` block to the inner level-hold loops.
    ///
    /// Mirrors `LevelHoldController::with_tuning` / `OrbitController::with_tuning`, and is
    /// what the scenario builder uses. Prefer it over [`new`](Self::new): `new` leaves the
    /// inner controller on `LevelHoldController::from_state` defaults, which are
    /// noticeably under-damped in pitch for this task — a receiver flown on them sits in a
    /// large vertical phugoid instead of settling inside the capture gate.
    pub fn with_tuning(
        tanker_id: PlaneId,
        tanker_initial: &FlightState,
        own_initial: &FlightState,
        tuning: &crate::controllers::tuning::RefuelingTuning,
        prev_inputs: &ControlInputs,
    ) -> Self {
        let mut c = Self::new(tanker_id, tanker_initial, own_initial, tuning.config());
        let targets = (c.inner.target_altitude, c.inner.target_airspeed);
        c.inner = LevelHoldController::with_tuning(own_initial, &tuning.inner, prev_inputs);
        // `with_tuning` re-seeds the targets from `own_initial`; restore the station
        // pre-seed `new` computed, so the first tick is still bumpless.
        c.inner.target_altitude = targets.0;
        c.inner.target_airspeed = targets.1;
        c
    }

    /// Re-install the refueling law over an already-constructed inner controller.
    ///
    /// Used by the tuning-rebuild systems in `controllers::sim_control`:
    /// `ControllerKind::Refueling.build()` cannot produce a real `RefuelController`
    /// (the generic factory has no tanker reference), so they rebuild the inner
    /// `LevelHoldController` from the plane's `.tuning.ron` profile and call this to
    /// re-wrap it.
    ///
    /// Note what crosses the rebuild and what does not. The **approach state**
    /// (`tanker_id`, `phase`, `commanded_offset`) is passed in by the caller, because a
    /// profile switch must not restart a receiver that is already in `Contact`. The
    /// **config** — stations, gates and outer gains — comes from the newly applied
    /// profile, because those are exactly the values the `refueling` tuning family
    /// exists to change; carrying the old ones over would make a profile switch a no-op.
    /// `dwell` and `retreating` are reset here and re-assigned by the caller if needed.
    pub fn from_inner(
        tanker_id: PlaneId,
        phase: RefuelPhase,
        commanded_offset: Vec3,
        config: RefuelConfig,
        inner: LevelHoldController,
    ) -> Self {
        Self {
            tanker_id,
            phase,
            commanded_offset,
            retreating: false,
            dwell: 0.0,
            breakaways: 0,
            config,
            inner,
            // Structural limits (integral clamp, output range) mirror the wingman's and
            // are deliberately not per-airframe; only the gains come from tuning.
            range_pid: PidController::new(
                config.range_kp,
                config.range_ki,
                config.range_kd,
                15.0,
                -10.0,
                10.0,
            ),
            lateral_pid: PidController::new(
                config.lateral_kp,
                0.0,
                config.lateral_kd,
                0.5,
                -0.5,
                0.5,
            ),
            heading_pid: PidController::new(
                config.heading_kp,
                0.0,
                config.heading_kd,
                0.0,
                -FRAC_PI_3,
                FRAC_PI_3,
            ),
            diagnostics: RefuelDiagnostics::default(),
        }
    }

    /// Abandon the approach: retreat to `Astern` at `breakaway_rate` and drop the outer
    /// integrator state, so a wound-up range integral doesn't drive the retreat.
    pub fn break_away(&mut self) {
        self.breakaways = self.breakaways.saturating_add(1);
        self.phase = RefuelPhase::Astern;
        self.retreating = true;
        self.dwell = 0.0;
        self.range_pid.reset();
        self.lateral_pid.reset();
        self.heading_pid.reset();
    }

    /// Advance the phase machine one tick. Breakaway is checked first: a lost station
    /// outranks any advance gate.
    fn step_phase(&mut self, tracking_error: f32, phase_error: f32, closure_rate: f32, dt: f32) {
        if self.phase != RefuelPhase::Astern && tracking_error > self.config.abort_radius {
            self.break_away();
            return;
        }

        let Some(radius) = self.config.advance_radius(self.phase) else {
            // Contact is terminal.
            self.dwell = 0.0;
            return;
        };

        if phase_error < radius && closure_rate.abs() < self.config.closure_tolerance {
            self.dwell += dt;
            if self.dwell >= self.config.dwell_secs {
                self.phase = match self.phase {
                    RefuelPhase::Astern => RefuelPhase::Precontact,
                    RefuelPhase::Precontact | RefuelPhase::Contact => RefuelPhase::Contact,
                };
                self.dwell = 0.0;
            }
        } else {
            self.dwell = 0.0;
        }
    }
}

impl FlightController for RefuelController {
    fn update(&mut self, own: &FlightState, ctx: &ControllerContext, dt: f32) -> ControlInputs {
        let Some(tanker_snap) = ctx.find(self.tanker_id) else {
            // Tanker not in context — hold the phase and the inner targets. Geometric
            // diagnostics keep their last value; mark them stale.
            self.diagnostics.tanker_found = false;
            return self.inner.update(own, ctx, dt);
        };
        let tanker = &tanker_snap.state;

        // 1. Station transit: creep the commanded station toward the phase's station.
        let phase_station = self.config.station(self.phase);
        let rate = if self.retreating {
            self.config.breakaway_rate
        } else {
            self.config.approach_rate
        };
        self.commanded_offset = move_toward(self.commanded_offset, phase_station, rate * dt);
        if self.commanded_offset == phase_station {
            self.retreating = false;
        }

        // 2. Desired position in world frame, and the error the cascade flies on.
        let target_pos = tanker.position + tanker.attitude * self.commanded_offset;
        let pos_error = target_pos - own.position;

        // 3. Altitude: set the inner target directly from station geometry.
        self.inner.target_altitude = target_pos.y;

        // 4. Range (fore-aft along the tanker's heading) → airspeed correction.
        let tanker_fwd = tanker.attitude * Vec3::X;
        let range_error = pos_error.dot(tanker_fwd);
        let delta_spd = self.range_pid.update(range_error, dt);
        self.inner.target_airspeed = (tanker.airspeed + delta_spd).max(20.0);

        // 5. Lateral cascade (heading-damped) → bank command. Identical to the
        //    wingman's, including the output sign; see `WingmanController::update`.
        let tanker_right = tanker.attitude * Vec3::Y;
        let lateral_error = pos_error.dot(tanker_right);
        let heading_offset = self.lateral_pid.update(lateral_error, dt);

        let lead_head = ground_heading(tanker);
        let (so, co) = (-heading_offset).sin_cos();
        let desired_x = co * lead_head.x - so * lead_head.y;
        let desired_z = so * lead_head.x + co * lead_head.y;

        let head = ground_heading(own);
        let cross = desired_x * head.y - desired_z * head.x;
        let dot = desired_x * head.x + desired_z * head.y;
        let heading_error = cross.atan2(dot);
        self.inner.target_roll = -self.heading_pid.update(heading_error, dt);

        // 6. Phase machine. `closure_rate` comes from relative velocity rather than
        //    from differencing `range_error`, so it carries no numerical noise.
        let closure_rate = (own.velocity - tanker.velocity).dot(tanker_fwd);
        let phase_pos = tanker.position + tanker.attitude * phase_station;
        let phase_error = (phase_pos - own.position).length();
        let tracking_error = pos_error.length();
        self.step_phase(tracking_error, phase_error, closure_rate, dt);

        // 7. Publish diagnostics (observational only).
        self.diagnostics = RefuelDiagnostics {
            tanker_found: true,
            phase: self.phase,
            pos_error,
            pos_error_mag: tracking_error,
            cross_track: lateral_error,
            range_error,
            altitude_error: pos_error.y,
            closure_rate,
            phase_error,
            breakaways: self.breakaways,
        };

        // 8. Delegate stabilization to the inner LevelHoldController.
        self.inner.update(own, ctx, dt)
    }

    fn name(&self) -> &'static str {
        "Refueling"
    }

    fn telemetry(
        &self,
        _state: &FlightState,
    ) -> crate::controllers::telemetry::ControllerTelemetry {
        crate::controllers::telemetry::ControllerTelemetry::Refueling(self.diagnostics.clone())
    }

    fn targets(&self) -> crate::controllers::targets::ControllerTargets {
        crate::controllers::targets::ControllerTargets::Refueling {
            tanker: self.tanker_id,
        }
    }

    fn apply_targets(
        &mut self,
        targets: &crate::controllers::targets::ControllerTargets,
        _state: &FlightState,
    ) {
        if let crate::controllers::targets::ControllerTargets::Refueling { tanker } = targets {
            self.tanker_id = *tanker;
        }
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

    const TANKER_ID: PlaneId = PlaneId(1);
    const OWN_ID: PlaneId = PlaneId(2);
    const DT: f32 = crate::plane::PHYSICS_DT;

    /// Level attitude: body (x, y, z) → world (x, z, -y), so body +X is world +X
    /// (forward), body +Z is world +Y (up), and body +Y is world -Z (right).
    fn level_attitude() -> Quat {
        Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)
    }

    fn level_state(pos: Vec3, airspeed: f32) -> FlightState {
        FlightState {
            position: pos,
            velocity: Vec3::new(airspeed, 0.0, 0.0),
            attitude: level_attitude(),
            airspeed,
            altitude: pos.y,
            ..Default::default()
        }
    }

    fn make_ctx(own: &FlightState, tanker: Option<&FlightState>) -> ControllerContext {
        let mut snaps = vec![PlaneSnapshot {
            id: OWN_ID,
            state: own.clone(),
        }];
        if let Some(t) = tanker {
            snaps.push(PlaneSnapshot {
                id: TANKER_ID,
                state: t.clone(),
            });
        }
        ControllerContext {
            own_id: OWN_ID,
            planes: Arc::from(snaps),
        }
    }

    /// A tanker at the origin flying +X, and a receiver parked exactly on `station`.
    fn on_station(station: Vec3) -> (FlightState, FlightState) {
        let tanker = level_state(Vec3::new(0.0, 2000.0, 0.0), 120.0);
        let own = level_state(tanker.position + level_attitude() * station, 120.0);
        (tanker, own)
    }

    fn controller_at(
        phase: RefuelPhase,
        config: RefuelConfig,
    ) -> (RefuelController, FlightState, FlightState) {
        let (tanker, own) = on_station(config.station(phase));
        let mut c = RefuelController::new(TANKER_ID, &tanker, &own, config);
        c.phase = phase;
        c.commanded_offset = config.station(phase);
        (c, tanker, own)
    }

    // -- geometry invariants -------------------------------------------------

    /// The station ladder must stay on the tanker's centerline and close monotonically.
    ///
    /// The `y == 0` half is load-bearing, not cosmetic: `lateral_pid` is a
    /// derivative-on-*error* PID, so a station whose lateral component changed between
    /// phases would inject a one-tick derivative kick straight into the crab command.
    /// Keeping every station on the centerline makes that impossible by construction,
    /// which is why the phase machine can get away with not resetting the lateral PID on
    /// an advance.
    #[test]
    fn stations_are_on_the_centerline_and_close_monotonically() {
        let c = RefuelConfig::default();
        for (name, s) in [
            ("astern", c.astern),
            ("precontact", c.precontact),
            ("contact", c.contact),
        ] {
            assert_eq!(s.y, 0.0, "{name} station must be on the tanker centerline");
        }
        assert!(
            c.astern.x < c.precontact.x && c.precontact.x < c.contact.x && c.contact.x < 0.0,
            "stations must close in from astern and stay behind the tanker"
        );
        assert!(
            c.astern.z < c.precontact.z && c.precontact.z < c.contact.z && c.contact.z < 0.0,
            "stations must rise toward the tanker and stay below it"
        );
    }

    // -- station transit -----------------------------------------------------

    #[test]
    fn starts_at_astern_with_the_commanded_station_there() {
        let cfg = RefuelConfig::default();
        let (tanker, own) = on_station(cfg.astern);
        let c = RefuelController::new(TANKER_ID, &tanker, &own, cfg);
        assert_eq!(c.phase, RefuelPhase::Astern);
        assert_eq!(c.commanded_offset, cfg.astern);
    }

    /// The commanded station creeps toward the phase station at `approach_rate`; it must
    /// never jump. This is what bounds closure — without it the phase machine hands the
    /// inner loops a 110 m step and the range PID picks whatever closure its limits allow.
    #[test]
    fn commanded_station_is_rate_limited_toward_the_phase_station() {
        let cfg = RefuelConfig::default();
        let (mut c, tanker, own) = controller_at(RefuelPhase::Astern, cfg);
        c.phase = RefuelPhase::Precontact; // force a transit without advancing
        let ctx = make_ctx(&own, Some(&tanker));

        let before = c.commanded_offset;
        c.update(&own, &ctx, DT);
        let step = (c.commanded_offset - before).length();
        assert!(
            step <= cfg.approach_rate * DT + 1e-4,
            "station moved {step} m in one tick, limit is {}",
            cfg.approach_rate * DT
        );
        assert!(step > 0.0, "station should have moved at all");
    }

    // -- advance gate --------------------------------------------------------

    /// The advance gate must read `phase_error` (distance to the phase's *final* station),
    /// **not** `tracking_error` (distance to the mid-transit commanded station).
    ///
    /// This is the trap the two-error split exists to avoid, and it needs a receiver that
    /// tracks the moving station *perfectly* to expose it: a sloppy tracker fails both
    /// gates and the bug hides. Here the receiver is teleported onto the commanded station
    /// every tick, so `tracking_error` is identically zero while `phase_error` stays large
    /// for the whole ~28 s transit. Gate on the wrong one and it advances after the 3 s
    /// dwell, roughly 25 s early and ~100 m short of the station it claims to have reached.
    #[test]
    fn advance_gate_reads_the_phase_station_not_the_commanded_one() {
        let cfg = RefuelConfig::default();
        let (mut c, tanker, _) = controller_at(RefuelPhase::Astern, cfg);
        c.phase = RefuelPhase::Precontact; // begin a transit without advancing into it

        // Well under the transit time (113 m at 4 m/s ≈ 28 s), well over the 3 s dwell.
        let ticks = (10.0 / DT) as usize;
        for _ in 0..ticks {
            // Perfect tracking: sit exactly on the commanded station this tick.
            let own = level_state(
                tanker.position + level_attitude() * c.commanded_offset,
                120.0,
            );
            let ctx = make_ctx(&own, Some(&tanker));
            c.update(&own, &ctx, DT);
        }

        assert!(
            (c.commanded_offset - cfg.precontact).length() > cfg.contact_radius,
            "test precondition: the station transit should still be running"
        );
        assert_eq!(
            c.phase,
            RefuelPhase::Precontact,
            "advanced to Contact while still {:.0} m from the Precontact station — the gate \
             is reading the commanded station instead of the phase station",
            (c.commanded_offset - cfg.precontact).length()
        );
    }

    #[test]
    fn advances_only_after_the_gate_holds_for_the_full_dwell() {
        let cfg = RefuelConfig::default();
        let (mut c, tanker, own) = controller_at(RefuelPhase::Astern, cfg);
        let ctx = make_ctx(&own, Some(&tanker));

        let ticks = (cfg.dwell_secs / DT) as usize;
        for _ in 0..ticks - 2 {
            c.update(&own, &ctx, DT);
        }
        assert_eq!(
            c.phase,
            RefuelPhase::Astern,
            "advanced before the dwell elapsed"
        );

        for _ in 0..4 {
            c.update(&own, &ctx, DT);
        }
        assert_eq!(c.phase, RefuelPhase::Precontact);
    }

    /// One tick outside tolerance restarts the dwell — the gate is *continuous*, not
    /// cumulative.
    #[test]
    fn dwell_resets_when_the_gate_lapses() {
        let cfg = RefuelConfig::default();
        let (mut c, tanker, own) = controller_at(RefuelPhase::Astern, cfg);
        let ctx_good = make_ctx(&own, Some(&tanker));

        for _ in 0..(cfg.dwell_secs / DT) as usize / 2 {
            c.update(&own, &ctx_good, DT);
        }
        assert!(c.dwell > 0.0);

        // One tick well outside the capture radius.
        let far = level_state(own.position + Vec3::new(0.0, 0.0, 200.0), 120.0);
        let ctx_bad = make_ctx(&far, Some(&tanker));
        c.update(&far, &ctx_bad, DT);
        assert_eq!(c.dwell, 0.0, "a lapse must restart the dwell");
        assert_eq!(c.phase, RefuelPhase::Astern);
    }

    /// On station but closing fast: the gate must refuse. Dropping the closure half of
    /// the gate would let a receiver advance while overtaking the tanker.
    #[test]
    fn advance_requires_low_closure_rate() {
        let cfg = RefuelConfig::default();
        let (mut c, tanker, own_still) = controller_at(RefuelPhase::Astern, cfg);
        // On station, but 8 m/s faster than the tanker.
        let mut own = own_still.clone();
        own.velocity = Vec3::new(128.0, 0.0, 0.0);
        let ctx = make_ctx(&own, Some(&tanker));

        for _ in 0..(cfg.dwell_secs / DT) as usize + 20 {
            c.update(&own, &ctx, DT);
        }
        assert_eq!(
            c.phase,
            RefuelPhase::Astern,
            "closure of 8 m/s is well over the {} m/s tolerance",
            cfg.closure_tolerance
        );
    }

    // -- breakaway -----------------------------------------------------------

    #[test]
    fn tracking_error_past_the_abort_radius_breaks_away_from_contact() {
        let cfg = RefuelConfig::default();
        let (mut c, tanker, _) = controller_at(RefuelPhase::Contact, cfg);
        let displaced = level_state(
            tanker.position + level_attitude() * cfg.contact + Vec3::new(0.0, 0.0, 60.0),
            120.0,
        );
        let ctx = make_ctx(&displaced, Some(&tanker));

        c.update(&displaced, &ctx, DT);
        assert_eq!(
            c.phase,
            RefuelPhase::Astern,
            "must retreat to the recovery station"
        );
        assert_eq!(c.dwell, 0.0);
        assert_eq!(c.breakaways, 1);
        assert!(c.retreating, "the retreat uses the faster breakaway rate");
    }

    /// Astern is the recovery station: there is nowhere further back to go, so a large
    /// error there must not trip the breakaway (which would spin the counter forever).
    #[test]
    fn astern_never_breaks_away() {
        let cfg = RefuelConfig::default();
        let (mut c, tanker, _) = controller_at(RefuelPhase::Astern, cfg);
        let far = level_state(tanker.position + Vec3::new(-2000.0, -400.0, 800.0), 120.0);
        let ctx = make_ctx(&far, Some(&tanker));

        for _ in 0..200 {
            c.update(&far, &ctx, DT);
        }
        assert_eq!(c.phase, RefuelPhase::Astern);
        assert_eq!(c.breakaways, 0);
    }

    /// A breakaway drops the outer integrator state, so a range integral wound up during
    /// the approach doesn't drive the retreat.
    #[test]
    fn breakaway_clears_the_outer_integrators() {
        let cfg = RefuelConfig::default();
        let (mut c, tanker, _) = controller_at(RefuelPhase::Contact, cfg);

        // Hold a steady range error to wind the range integral up.
        let behind = level_state(
            tanker.position + level_attitude() * cfg.contact + Vec3::new(-20.0, 0.0, 0.0),
            120.0,
        );
        let ctx = make_ctx(&behind, Some(&tanker));
        for _ in 0..600 {
            c.update(&behind, &ctx, DT);
        }
        let wound = c.inner.target_airspeed;
        assert!(
            wound > tanker.airspeed + 0.1,
            "expected a wound-up speed correction, got {wound}"
        );

        c.break_away();
        // First tick after the breakaway: the correction must be proportional-only.
        let out = c.update(&behind, &ctx, DT);
        assert!(out.throttle.is_finite());
        let expected_p = cfg.range_kp * 20.0;
        assert!(
            (c.inner.target_airspeed - (tanker.airspeed + expected_p)).abs() < 1.0,
            "post-breakaway correction {} should be ~proportional ({}), i.e. no residual integral",
            c.inner.target_airspeed - tanker.airspeed,
            expected_p
        );
    }

    // -- cascade signs -------------------------------------------------------

    /// A lateral offset must command a *restoring* bank. The wingman's equivalent test
    /// exists because the original formation controller shipped with the divergent sign
    /// and spiraled away; this cascade is the same code, so it needs the same guard.
    #[test]
    fn lateral_error_commands_restoring_bank_direction() {
        let cfg = RefuelConfig::default();
        let (mut c, tanker, on) = controller_at(RefuelPhase::Contact, cfg);

        // Body +Y (tanker's right) maps to world -Z, so push the receiver to world +Z:
        // that puts it to the tanker's LEFT, and the station is off to its right.
        let left = level_state(on.position + Vec3::new(0.0, 0.0, 40.0), 120.0);
        let ctx = make_ctx(&left, Some(&tanker));
        c.update(&left, &ctx, DT);
        let bank_left_side = c.inner.target_roll;

        // Mirror it.
        let (mut c2, _, _) = controller_at(RefuelPhase::Contact, cfg);
        let right = level_state(on.position + Vec3::new(0.0, 0.0, -40.0), 120.0);
        let ctx2 = make_ctx(&right, Some(&tanker));
        c2.update(&right, &ctx2, DT);
        let bank_right_side = c2.inner.target_roll;

        assert!(
            bank_left_side.signum() != bank_right_side.signum(),
            "mirrored offsets must command opposite banks, got {bank_left_side} and {bank_right_side}"
        );
        assert!(
            bank_left_side.abs() > 1e-6,
            "a 40 m lateral offset must command some bank"
        );
    }

    #[test]
    fn range_error_commands_higher_airspeed() {
        let cfg = RefuelConfig::default();
        let (mut c, tanker, on) = controller_at(RefuelPhase::Contact, cfg);
        // 30 m behind the station ⇒ must speed up to catch it.
        let behind = level_state(on.position + Vec3::new(-30.0, 0.0, 0.0), 120.0);
        let ctx = make_ctx(&behind, Some(&tanker));
        c.update(&behind, &ctx, DT);
        assert!(
            c.inner.target_airspeed > tanker.airspeed,
            "expected a speed-up, got {} vs tanker {}",
            c.inner.target_airspeed,
            tanker.airspeed
        );
    }

    // -- tanker loss ---------------------------------------------------------

    #[test]
    fn fallback_with_no_tanker_in_ctx_returns_finite_outputs() {
        let cfg = RefuelConfig::default();
        let (mut c, _, own) = controller_at(RefuelPhase::Precontact, cfg);
        let ctx = make_ctx(&own, None);

        let out = c.update(&own, &ctx, DT);
        assert!(out.elevator.is_finite());
        assert!(out.throttle.is_finite());
        assert!(out.aileron.is_finite());
        assert!(out.rudder.is_finite());
        assert!(!c.diagnostics.tanker_found);
        assert_eq!(
            c.phase,
            RefuelPhase::Precontact,
            "a momentary context gap must hold the phase, not restart the approach"
        );
    }

    // -- rebuild path --------------------------------------------------------

    #[test]
    fn from_inner_keeps_the_tuned_inner_and_the_approach_state() {
        let cfg = RefuelConfig::default();
        let (_, own) = on_station(cfg.contact);
        let mut inner = LevelHoldController::from_state(&own, &ControlInputs::default());
        inner.altitude_pid.kp = 9.87;
        inner.target_altitude = 1234.0;

        let c =
            RefuelController::from_inner(TANKER_ID, RefuelPhase::Contact, cfg.contact, cfg, inner);
        assert_eq!(c.tanker_id, TANKER_ID);
        assert_eq!(c.phase, RefuelPhase::Contact);
        assert_eq!(c.commanded_offset, cfg.contact);
        assert_eq!(c.inner.altitude_pid.kp, 9.87);
        assert_eq!(c.inner.target_altitude, 1234.0);
    }

    /// `from_inner` must take its gains from the config it is handed — this is what makes
    /// a `refueling` profile switch actually change anything.
    #[test]
    fn from_inner_takes_its_outer_gains_from_the_config() {
        let mut cfg = RefuelConfig::default();
        cfg.range_kp = 0.75;
        cfg.heading_kp = 1.25;
        let (_, own) = on_station(cfg.contact);
        let inner = LevelHoldController::from_state(&own, &ControlInputs::default());

        let c =
            RefuelController::from_inner(TANKER_ID, RefuelPhase::Astern, cfg.astern, cfg, inner);
        assert_eq!(c.range_pid.kp, 0.75);
        assert_eq!(c.heading_pid.kp, 1.25);
    }

    // -- targets -------------------------------------------------------------

    #[test]
    fn targets_round_trip_the_tanker() {
        use crate::controllers::targets::ControllerTargets;
        let cfg = RefuelConfig::default();
        let (mut c, _, own) = controller_at(RefuelPhase::Astern, cfg);
        assert_eq!(
            c.targets(),
            ControllerTargets::Refueling { tanker: TANKER_ID }
        );

        c.apply_targets(&ControllerTargets::Refueling { tanker: PlaneId(9) }, &own);
        assert_eq!(c.tanker_id, PlaneId(9));

        // A stale command for a different controller shape must be ignored, not panic.
        c.apply_targets(
            &ControllerTargets::LevelHold {
                altitude: 1.0,
                airspeed: 2.0,
            },
            &own,
        );
        assert_eq!(c.tanker_id, PlaneId(9));
    }
}
