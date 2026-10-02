//! Batched RL inference (`controllers::policy_batch`) — requires `inference`.
//!   cargo test --no-default-features --features inference --test rl rl_batching::
//!
//! The load-bearing claim here is **batch invariance**: a plane's actions, and so
//! its trajectory, must not depend on how many other planes happen to share its
//! model this tick. burn does not promise that — it holds because the ndarray
//! matmul sums each output row in the same order whatever the row count — so it is
//! pinned at three levels: the forward pass, the controller split, and the live sim
//! with planes joining and leaving the batch mid-flight.
use std::f32::consts::FRAC_PI_2;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use bevy::math::{Quat, Vec3};
use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use burn::backend::NdArray;
use burn::module::Module;
use burn::record::{FullPrecisionSettings, NamedMpkBytesRecorder, Recorder};
use ml_planes::controllers::orbit::{OrbitDirection, ORBIT_OBS_DIM};
use ml_planes::controllers::policy_batch::{load_mlp_bytes, load_mlp_file};
use ml_planes::controllers::{
    ActiveController, FlightController, PolicyBatch, RlHeadingHoldConfig, RlHeadingHoldController,
    RlLevelHoldController, RlLstmOrbitConfig, RlLstmOrbitController, RlOrbitConfig,
    RlOrbitController, RlOrbitResidualConfig, RlOrbitResidualController,
};
use ml_planes::plane::{
    ControlInputs, ControllerContext, FlightState, PlaneConfig, PlaneConfigHandle, PlaneId,
    PlaneSnapshot, PHYSICS_DT,
};
use ml_planes::training::heading_hold_env::HEADING_HOLD_OBS_DIM;
use ml_planes::training::ppo::lstm_model::LstmActorCritic;
use ml_planes::training::ppo::model::ActorCritic;

use crate::common::{build_headless_app, generic_jet_config};

type InfB = NdArray;

const ORBIT_MPK: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/fixtures/models/orbit/ppo_orbit_1.mpk"
));
const LEVEL_HOLD_MPK: &[u8] = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/fixtures/models/level_hold/ppo_level_hold.mpk"
));

fn device() -> <InfB as burn::tensor::backend::Backend>::Device {
    Default::default()
}

/// Checkpoint bytes for a seeded, untrained feed-forward model. There is no frozen
/// heading-hold fixture, and batch invariance is a property of the forward pass,
/// not of what the weights learned.
fn seeded_mlp_bytes(obs_dim: usize, seed: u64) -> Vec<u8> {
    let model = ActorCritic::<InfB>::new_seeded(&device(), obs_dim, seed);
    NamedMpkBytesRecorder::<FullPrecisionSettings>::default()
        .record(model.into_record(), ())
        .expect("record seeded model")
}

fn seeded_lstm_bytes(seed: u64) -> Vec<u8> {
    let model = LstmActorCritic::<InfB>::new_seeded(&device(), ORBIT_OBS_DIM, seed);
    NamedMpkBytesRecorder::<FullPrecisionSettings>::default()
        .record(model.into_record(), ())
        .expect("record seeded lstm")
}

/// A distinct, plausible flight state per (member, tick), so every row of a batch
/// differs and an LSTM's hidden state actually evolves.
fn varied_state(member: usize, tick: usize) -> FlightState {
    let m = member as f32;
    let t = tick as f32;
    let mut state = FlightState {
        position: Vec3::new(40.0 * m + 3.0 * t, 900.0 + 25.0 * m, -1000.0 + 7.0 * m),
        velocity: Vec3::new(100.0 + 2.0 * m, 0.3 * t - 0.5 * m, 1.5 * m - 0.2 * t),
        attitude: Quat::from_rotation_x(-FRAC_PI_2)
            * Quat::from_rotation_z(0.05 * m - 0.01 * t)
            * Quat::from_rotation_x(0.03 * (m - 2.0)),
        angular_velocity: Vec3::new(0.01 * m, -0.005 * t, 0.002 * m),
        ..FlightState::default()
    };
    state.update_air_data();
    state
}

fn ctx_for(id: u32, state: &FlightState) -> ControllerContext {
    ControllerContext {
        own_id: PlaneId(id),
        planes: Arc::from(vec![PlaneSnapshot {
            id: PlaneId(id),
            state: state.clone(),
        }]),
    }
}

fn orbit_config() -> RlOrbitConfig {
    RlOrbitConfig {
        center_x: 0.0,
        center_z: 0.0,
        target_radius: 1000.0,
        target_altitude: 1000.0,
        target_airspeed: 100.0,
        direction: OrbitDirection::CounterClockwise,
    }
}

/// One factory per RL kind. Each call loads afresh, so these also exercise the
/// loader's model sharing: every controller a factory builds must share one model.
fn rl_factories() -> Vec<(&'static str, Box<dyn Fn() -> Box<dyn FlightController>>)> {
    let heading = seeded_mlp_bytes(HEADING_HOLD_OBS_DIM, 11);
    let lstm = seeded_lstm_bytes(12);
    vec![
        (
            "RlLevelHold",
            Box::new(|| {
                Box::new(RlLevelHoldController::load_bytes(LEVEL_HOLD_MPK, 1000.0, 100.0).unwrap())
            }),
        ),
        (
            "RlHeadingHold",
            Box::new(move || {
                let config = RlHeadingHoldConfig {
                    target_heading: 0.6,
                    target_altitude: 1000.0,
                    target_airspeed: 100.0,
                };
                Box::new(RlHeadingHoldController::load_bytes(&heading, config).unwrap())
            }),
        ),
        (
            "RlOrbit",
            Box::new(|| {
                Box::new(RlOrbitController::load_bytes(ORBIT_MPK, orbit_config()).unwrap())
            }),
        ),
        (
            "RlOrbitResidual",
            Box::new(|| {
                let o = orbit_config();
                let config = RlOrbitResidualConfig {
                    center_x: o.center_x,
                    center_z: o.center_z,
                    target_radius: o.target_radius,
                    target_altitude: o.target_altitude,
                    target_airspeed: o.target_airspeed,
                    direction: o.direction,
                    residual_scale: 0.3,
                };
                Box::new(
                    RlOrbitResidualController::load_bytes(
                        ORBIT_MPK,
                        config,
                        &varied_state(0, 0),
                        None,
                    )
                    .unwrap(),
                )
            }),
        ),
        (
            "RlLstmOrbit",
            Box::new(move || {
                let o = orbit_config();
                let config = RlLstmOrbitConfig {
                    center_x: o.center_x,
                    center_z: o.center_z,
                    target_radius: o.target_radius,
                    target_altitude: o.target_altitude,
                    target_airspeed: o.target_airspeed,
                    direction: o.direction,
                };
                Box::new(RlLstmOrbitController::load_bytes(&lstm, config).unwrap())
            }),
        ),
    ]
}

fn bits(i: &ControlInputs) -> [u32; 4] {
    [
        i.aileron.to_bits(),
        i.elevator.to_bits(),
        i.rudder.to_bits(),
        i.throttle.to_bits(),
    ]
}

/// Step `controllers` for one tick through a single shared `PolicyBatch`, exactly
/// as `run_flight_controllers` does (including its final clamp).
fn batched_tick(controllers: &mut [Box<dyn FlightController>], tick: usize) -> Vec<ControlInputs> {
    let mut batch = PolicyBatch::<usize>::default();
    for (i, c) in controllers.iter_mut().enumerate() {
        let state = varied_state(i, tick);
        let policy = c.batched().expect("RL controller exposes a batched policy");
        batch.push(
            i,
            policy,
            &state,
            &ctx_for(i as u32 + 1, &state),
            PHYSICS_DT,
        );
    }
    let mut out = vec![ControlInputs::default(); controllers.len()];
    batch.run(|i, action, hidden| {
        let mut inputs = controllers[i].batched().unwrap().finish(action, hidden);
        inputs.clamp();
        out[i] = inputs;
    });
    assert!(batch.is_empty(), "run() must leave the batch empty");
    out
}

// ---------------------------------------------------------------------------
// Model sharing

#[test]
fn identical_checkpoints_share_one_model() {
    let a = load_mlp_bytes(ORBIT_MPK).unwrap();
    let b = load_mlp_bytes(ORBIT_MPK).unwrap();
    assert!(Arc::ptr_eq(&a, &b), "same bytes must yield the same model");

    let other = load_mlp_bytes(LEVEL_HOLD_MPK).unwrap();
    assert!(
        !Arc::ptr_eq(&a, &other),
        "different checkpoints must not share"
    );

    // Keyed by content, not by how the bytes arrived: a file holding the same
    // checkpoint is the same model.
    let path = std::env::temp_dir().join(format!("ml_planes_batch_share_{}", std::process::id()));
    std::fs::write(path.with_extension("mpk"), ORBIT_MPK).unwrap();
    let from_file = load_mlp_file(path.to_str().unwrap()).unwrap();
    assert!(
        Arc::ptr_eq(&a, &from_file),
        "file and bytes of one checkpoint must share"
    );
    let _ = std::fs::remove_file(path.with_extension("mpk"));
}

/// A checkpoint retrained in place under the same name is a different model, never
/// a stale cache hit.
#[test]
fn rewritten_checkpoint_is_a_new_model() {
    let path = std::env::temp_dir().join(format!("ml_planes_batch_rewrite_{}", std::process::id()));
    let file = path.with_extension("mpk");
    std::fs::write(&file, seeded_mlp_bytes(ORBIT_OBS_DIM, 1)).unwrap();
    let first = load_mlp_file(path.to_str().unwrap()).unwrap();
    std::fs::write(&file, seeded_mlp_bytes(ORBIT_OBS_DIM, 2)).unwrap();
    let second = load_mlp_file(path.to_str().unwrap()).unwrap();
    assert!(!Arc::ptr_eq(&first, &second));
    let _ = std::fs::remove_file(file);
}

#[test]
fn every_rl_controller_shares_its_model_with_its_siblings() {
    for (name, make) in rl_factories() {
        let mut a = make();
        let mut b = make();
        let pa = a
            .batched()
            .unwrap_or_else(|| panic!("{name}: no batched policy"))
            .policy()
            .clone();
        let pb = b.batched().unwrap().policy().clone();
        assert!(
            pa.same_model(&pb),
            "{name}: two loads of one checkpoint must share a model"
        );
    }
}

// ---------------------------------------------------------------------------
// Batch invariance

/// The split path the live sim takes must reproduce each controller's own
/// `update()` bit for bit — over several ticks, so the LSTM's carried state and the
/// residual controller's PID memory are covered too.
#[test]
fn batched_ticks_match_update_bitwise_for_every_rl_kind() {
    const N: usize = 9;
    for (name, make) in rl_factories() {
        let mut solo: Vec<_> = (0..N).map(|_| make()).collect();
        let mut batched: Vec<_> = (0..N).map(|_| make()).collect();
        for tick in 0..6 {
            let got = batched_tick(&mut batched, tick);
            for (i, c) in solo.iter_mut().enumerate() {
                let state = varied_state(i, tick);
                let mut want = c.update(&state, &ctx_for(i as u32 + 1, &state), PHYSICS_DT);
                want.clamp();
                assert_eq!(
                    bits(&want),
                    bits(&got[i]),
                    "{name}: member {i} diverged from update() at tick {tick}"
                );
            }
        }
    }
}

/// The determinism guarantee itself: member 0's actions are identical whether it is
/// batched alone or alongside up to 128 batchmates.
#[test]
fn a_planes_actions_do_not_depend_on_its_batch_size() {
    for (name, make) in rl_factories() {
        let mut reference: Option<Vec<[u32; 4]>> = None;
        for size in [1usize, 2, 7, 34, 129] {
            let mut controllers: Vec<_> = (0..size).map(|_| make()).collect();
            let trace: Vec<[u32; 4]> = (0..4)
                .map(|tick| bits(&batched_tick(&mut controllers, tick)[0]))
                .collect();
            match &reference {
                None => reference = Some(trace),
                Some(r) => assert_eq!(r, &trace, "{name}: batch of {size} changed member 0"),
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Live sim

/// Counts `update()` calls, delegating everything to the wrapped controller.
struct Probe {
    inner: Box<dyn FlightController>,
    updates: Arc<AtomicUsize>,
}

impl FlightController for Probe {
    fn update(&mut self, own: &FlightState, ctx: &ControllerContext, dt: f32) -> ControlInputs {
        self.updates.fetch_add(1, Ordering::Relaxed);
        self.inner.update(own, ctx, dt)
    }
    fn batched(&mut self) -> Option<&mut dyn ml_planes::controllers::BatchedPolicy> {
        self.inner.batched()
    }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

/// Spawn a fully-built plane directly (no asset load), as `observe_state` does.
fn spawn_direct(
    app: &mut App,
    cfg: &PlaneConfig,
    id: u32,
    state: FlightState,
    controller: Box<dyn FlightController>,
) -> Entity {
    let handle = app
        .world_mut()
        .resource_mut::<Assets<PlaneConfig>>()
        .add(cfg.clone());
    app.world_mut()
        .spawn((
            RigidBody::Dynamic,
            Collider::cuboid(3.0, 0.5, 1.0),
            ColliderMassProperties::Mass(0.0),
            Velocity {
                linvel: state.velocity,
                angvel: Vec3::ZERO,
            },
            ExternalForce::default(),
            AdditionalMassProperties::MassProperties(MassProperties {
                local_center_of_mass: Vec3::ZERO,
                mass: cfg.mass,
                principal_inertia: cfg.inertia,
                principal_inertia_local_frame: Quat::IDENTITY,
            }),
            Transform::from_translation(state.position).with_rotation(state.attitude),
            state,
            ControlInputs::default(),
            ActiveController(controller),
            PlaneConfigHandle(handle),
            PlaneId(id),
        ))
        .id()
}

/// Planes far enough apart that Rapier never puts two in contact.
fn spread_state(slot: usize) -> FlightState {
    let mut state = FlightState {
        position: Vec3::new(5000.0 * slot as f32, 1000.0, -1000.0),
        velocity: Vec3::new(100.0, 0.0, 0.0),
        attitude: Quat::from_rotation_x(-FRAC_PI_2),
        ..FlightState::default()
    };
    state.update_air_data();
    state
}

fn orbit_ctrl() -> Box<dyn FlightController> {
    Box::new(RlOrbitController::load_bytes(ORBIT_MPK, orbit_config()).unwrap())
}

fn state_bits(s: &FlightState) -> Vec<u32> {
    [s.position, s.velocity, s.angular_velocity]
        .iter()
        .flat_map(|v| v.to_array())
        .chain(s.attitude.to_array())
        .map(f32::to_bits)
        .collect()
}

/// End to end: plane 1's trajectory flying alone equals its trajectory while
/// batchmates sharing its model are present, join mid-flight, and leave
/// mid-flight — the dynamic-population case the per-tick rebuild exists for.
#[test]
fn rl_plane_trajectory_is_independent_of_its_batchmates() {
    let cfg = generic_jet_config();
    const TICKS: usize = 160;

    let mut alone = build_headless_app();
    let a = spawn_direct(&mut alone, &cfg, 1, spread_state(0), orbit_ctrl());
    for _ in 0..TICKS {
        alone.update();
    }

    let mut crowded = build_headless_app();
    let b = spawn_direct(&mut crowded, &cfg, 1, spread_state(0), orbit_ctrl());
    let mut mates: Vec<Entity> = (1..6)
        .map(|slot| {
            spawn_direct(
                &mut crowded,
                &cfg,
                slot as u32 + 1,
                spread_state(slot),
                orbit_ctrl(),
            )
        })
        .collect();
    // A batchmate on a different model forms its own group.
    spawn_direct(
        &mut crowded,
        &cfg,
        20,
        spread_state(20),
        Box::new(RlLevelHoldController::load_bytes(LEVEL_HOLD_MPK, 1000.0, 100.0).unwrap()),
    );
    for tick in 0..TICKS {
        if tick == 40 {
            mates.push(spawn_direct(
                &mut crowded,
                &cfg,
                30,
                spread_state(30),
                orbit_ctrl(),
            ));
        }
        if tick == 80 {
            for e in mates.drain(..3) {
                crowded.world_mut().despawn(e);
            }
        }
        crowded.update();
    }

    let sa = alone.world().get::<FlightState>(a).unwrap().clone();
    let sb = crowded.world().get::<FlightState>(b).unwrap().clone();
    assert!(
        sa.position.distance(spread_state(0).position) > 100.0,
        "the plane must have flown"
    );
    assert_eq!(
        state_bits(&sa),
        state_bits(&sb),
        "batchmates changed plane 1's trajectory"
    );
}

/// The live sim must actually take the batched path for RL controllers — the
/// whole point — rather than falling back to per-plane `update()`.
#[test]
fn sim_steps_rl_controllers_through_the_batch_not_update() {
    let cfg = generic_jet_config();
    let mut app = build_headless_app();
    let updates = Arc::new(AtomicUsize::new(0));
    let planes: Vec<Entity> = (0..3)
        .map(|slot| {
            let probe = Probe {
                inner: orbit_ctrl(),
                updates: Arc::clone(&updates),
            };
            spawn_direct(
                &mut app,
                &cfg,
                slot as u32 + 1,
                spread_state(slot),
                Box::new(probe),
            )
        })
        .collect();
    for _ in 0..10 {
        app.update();
    }
    assert_eq!(
        updates.load(Ordering::Relaxed),
        0,
        "RL planes went through update()"
    );
    for e in planes {
        let inputs = app.world().get::<ControlInputs>(e).unwrap();
        assert_ne!(
            bits(inputs),
            bits(&ControlInputs::default()),
            "policy never ran"
        );
    }
}
