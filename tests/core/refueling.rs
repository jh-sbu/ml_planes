//! Staged refueling approach, end-to-end through the 6-DOF Rapier chain.
//!
//! Requires the sim chain (`PlanePlugin` FixedUpdate systems), which is compiled out on a
//! `net`-without-`server` build; the `sim_enabled` cfg gates this module from
//! `tests/core/main.rs`. See `tests/core/wingman.rs` for the same note.
//!
//! **Both planes here are generic jets.** `tests/common/mod.rs::generic_jet_config()` is
//! the only permitted airframe (CLAUDE.md forbids a hand-written `PlaneConfig` literal),
//! so the "tanker" is a generic jet flying level hold. The real tanker airframe is
//! exercised by `assets/scenarios/refueling.scenario.ron` through `observe_state`, which
//! loads assets for real. Don't "fix" this by inlining a tanker config.

use std::f32::consts::FRAC_PI_2;

use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use ml_planes::controllers::{
    ActiveController, LevelHoldController, LevelHoldTuning, RefuelController, RefuelPhase,
    RefuelingTuning,
};
use ml_planes::plane::{ControlInputs, FlightState, PlaneConfig, PlaneConfigHandle, PlaneId};

const TANKER_ID: PlaneId = PlaneId(1);
const RECEIVER_ID: PlaneId = PlaneId(2);
const ALT: f32 = 2000.0;
const SPD: f32 = 130.0;

/// Spawn a plane and return its Entity. Mirrors `tests/core/wingman.rs`.
fn spawn_plane_entity(
    app: &mut App,
    id: PlaneId,
    pos: Vec3,
    velocity: Vec3,
    controller: impl ml_planes::controllers::FlightController + 'static,
) -> Entity {
    let cfg = crate::common::generic_jet_config();
    let handle: Handle<PlaneConfig> = app
        .world_mut()
        .resource_mut::<Assets<PlaneConfig>>()
        .add(cfg.clone());
    let attitude = Quat::from_rotation_x(-FRAC_PI_2);

    app.world_mut()
        .spawn((
            RigidBody::Dynamic,
            Collider::cuboid(1.0, 0.5, 3.0),
            ColliderMassProperties::Mass(0.0),
            Velocity {
                linvel: velocity,
                angvel: Vec3::ZERO,
            },
            ExternalForce::default(),
            AdditionalMassProperties::MassProperties(MassProperties {
                local_center_of_mass: Vec3::ZERO,
                mass: cfg.mass,
                principal_inertia: cfg.inertia,
                principal_inertia_local_frame: Quat::IDENTITY,
            }),
            id,
            FlightState::default(),
            ControlInputs::default(),
            ActiveController(Box::new(controller)),
            PlaneConfigHandle(handle),
            Transform::from_translation(pos).with_rotation(attitude),
        ))
        .id()
}

/// The refueling profile the tests fly: default stations and gates, but the *validated*
/// generic-jet level-hold block as the inner loop.
///
/// `LevelHoldTuning::default()` is deliberately not used: it is under-damped in pitch
/// (`pitch_kp` 1.0 / `pitch_kd` 0.5) for this task and leaves the receiver in a ~48 m
/// peak-to-peak vertical phugoid that never settles inside the capture gate. That is a
/// real property of the shipped defaults, reproduced by `observe_state`, not a test
/// artifact — see the header comment on `assets/scenarios/refueling.scenario.ron`.
fn refuel_tuning() -> RefuelingTuning {
    RefuelingTuning {
        inner: LevelHoldTuning {
            alt_kp: 0.015,
            alt_ki: 0.22,
            alt_kd: 0.20,
            pitch_kp: 5.0,
            pitch_kd: 4.0,
            spd_kp: 0.03,
            spd_ki: 0.15,
            throttle_ff_gain: 0.3,
        },
        ..RefuelingTuning::default()
    }
}

/// Level tanker at the origin plus a receiver spawned exactly on the Astern station.
fn spawn_pair(app: &mut App) {
    let attitude = Quat::from_rotation_x(-FRAC_PI_2);
    let tanker_pos = Vec3::new(0.0, ALT, 0.0);
    let vel = Vec3::new(SPD, 0.0, 0.0);

    let tanker_initial = FlightState {
        position: tanker_pos,
        velocity: vel,
        attitude,
        airspeed: SPD,
        altitude: ALT,
        ..Default::default()
    };
    // The tanker gets the validated gains too, and that matters more than it looks: the
    // Astern station is 150 m aft, so every degree of tanker pitch swings the station's
    // world altitude by ~2.6 m. On `LevelHoldController::new`'s default gains this
    // airframe porpoises +-5.7 deg at 2000 m / 130 m/s, which waves the station +-15 m
    // and drives the receiver's pitch loop into a limit cycle. A real tanker flies
    // steadily; so must this one.
    let mut tanker_ctrl = LevelHoldController::with_tuning(
        &tanker_initial,
        &refuel_tuning().inner,
        &ControlInputs::default(),
    );
    tanker_ctrl.target_altitude = ALT;
    tanker_ctrl.target_airspeed = SPD;
    spawn_plane_entity(app, TANKER_ID, tanker_pos, vel, tanker_ctrl);

    let tuning = refuel_tuning();
    let cfg = tuning.config();
    let receiver_pos = tanker_pos + attitude * cfg.astern;
    let own_initial = FlightState {
        position: receiver_pos,
        velocity: vel,
        attitude,
        airspeed: SPD,
        altitude: receiver_pos.y,
        ..Default::default()
    };
    spawn_plane_entity(
        app,
        RECEIVER_ID,
        receiver_pos,
        vel,
        RefuelController::with_tuning(
            TANKER_ID,
            &tanker_initial,
            &own_initial,
            &tuning,
            &ControlInputs::default(),
        ),
    );
}

fn states(app: &mut App) -> (FlightState, Transform, FlightState) {
    let mut q = app
        .world_mut()
        .query::<(&PlaneId, &FlightState, &Transform)>();
    let (mut tanker, mut tanker_tf, mut receiver) = (None, None, None);
    for (id, fs, tf) in q.iter(app.world()) {
        match id.0 {
            1 => {
                tanker = Some(fs.clone());
                tanker_tf = Some(*tf);
            }
            2 => receiver = Some(fs.clone()),
            _ => {}
        }
    }
    (tanker.unwrap(), tanker_tf.unwrap(), receiver.unwrap())
}

/// Read the receiver's live phase / breakaway count off its controller.
fn approach(app: &mut App) -> (RefuelPhase, u32) {
    let mut q = app.world_mut().query::<(&PlaneId, &mut ActiveController)>();
    for (id, mut ctrl) in q.iter_mut(app.world_mut()) {
        if id.0 == 2 {
            let r = ctrl
                .0
                .as_any_mut()
                .downcast_mut::<RefuelController>()
                .expect("receiver must still be a RefuelController");
            return (r.phase, r.breakaways);
        }
    }
    panic!("no receiver");
}

/// The load-bearing test: the receiver must climb the whole ladder and hold Contact.
///
/// `breakaways == 0` is the stability assertion, and it is the one that catches what a
/// final-position check cannot. A too-hot lateral gain, a divergent feedback sign, or a
/// mis-set tolerance shows up as the receiver falling out of Contact and re-climbing —
/// which, sampled only at the last tick, looks identical to a clean approach.
#[test]
fn receiver_climbs_the_ladder_to_contact_and_holds() {
    let mut app = crate::common::build_headless_app();
    spawn_pair(&mut app);

    let mut reached_contact_at = None;
    let mut breakaways = 0;
    // 180 simulated seconds (11520 updates x 1/64 s), matching the wingman hold test.
    for tick in 0..11520 {
        app.update();
        let (phase, b) = approach(&mut app);
        breakaways = b;
        if phase == RefuelPhase::Contact && reached_contact_at.is_none() {
            reached_contact_at = Some(tick);
        }
    }

    let (phase, _) = approach(&mut app);
    assert_eq!(phase, RefuelPhase::Contact, "must end docked");

    let at = reached_contact_at.expect("never reached Contact");
    assert!(
        at < 7680,
        "Contact reached at t={:.0}s; the rate-limited transit plus dwells should get \
         there inside 120 s",
        at as f32 / 64.0
    );
    assert_eq!(
        breakaways, 0,
        "the approach fell out of station and re-climbed {breakaways} time(s) — a settled \
         hold, not just a final position, is the requirement"
    );

    let (tanker, tanker_tf, receiver) = states(&mut app);
    let cfg = refuel_tuning().config();
    // Station from the tanker's ACTUAL attitude, not the nominal level one.
    let station = tanker.position + tanker_tf.rotation * cfg.contact;
    let err = (receiver.position - station).length();
    // Sub-metre in practice (~0.5 m). 1.5 m leaves room for the residual phugoid without
    // being so loose that a real regression slips through: this is a docking position, and
    // the wingman test's 15 m slot tolerance would be meaningless here.
    assert!(
        err < 1.5,
        "receiver {err:.2} m off the contact station (station={station:?}, pos={:?})",
        receiver.position
    );

    for (name, s) in [("tanker", &tanker), ("receiver", &receiver)] {
        assert!(
            s.altitude.is_finite() && s.airspeed.is_finite(),
            "{name} state went non-finite"
        );
        assert!(s.altitude > 0.0, "{name} hit the ground");
        assert!(s.airspeed > 10.0, "{name} lost airspeed ({})", s.airspeed);
    }
}

/// A receiver shoved off the docking station must break away to Astern and then re-climb.
/// Proves the ladder is re-entrant, not a one-shot.
#[test]
fn receiver_breaks_away_when_displaced_and_recovers() {
    let mut app = crate::common::build_headless_app();
    spawn_pair(&mut app);

    // Fly to Contact.
    let mut reached = false;
    for _ in 0..7680 {
        app.update();
        if approach(&mut app).0 == RefuelPhase::Contact {
            reached = true;
            break;
        }
    }
    assert!(reached, "precondition: must reach Contact first");

    // Shove it well past the abort radius, laterally (so it is a station loss, not a
    // stall) by teleporting the rigid body.
    let cfg = refuel_tuning().config();
    {
        let mut q = app
            .world_mut()
            .query::<(&PlaneId, &mut Transform, &mut FlightState)>();
        for (id, mut tf, mut fs) in q.iter_mut(app.world_mut()) {
            if id.0 == 2 {
                let shove = Vec3::new(0.0, 0.0, cfg.abort_radius * 2.0);
                tf.translation += shove;
                fs.position += shove;
            }
        }
    }
    app.update();

    let (phase, breakaways) = approach(&mut app);
    assert_eq!(
        phase,
        RefuelPhase::Astern,
        "must retreat to the recovery station"
    );
    assert_eq!(breakaways, 1);

    // And re-climb.
    let mut back = false;
    for _ in 0..7680 {
        app.update();
        if approach(&mut app).0 == RefuelPhase::Contact {
            back = true;
            break;
        }
    }
    assert!(back, "the approach must be re-entrant after a breakaway");
}
