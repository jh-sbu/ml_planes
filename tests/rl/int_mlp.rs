//! IntMLP level hold: the burn module, its flat inference kernel, and the controller.

use burn::backend::NdArray;
use burn::module::Module;
use burn::nn::LinearConfig;
use burn::record::{DefaultFileRecorder, FullPrecisionSettings};
use burn::tensor::{Tensor, TensorData};

use ml_planes::controllers::{
    ControllerTargets, FlightController, IntMlpLevelHoldController, ModelLoadError,
};
use ml_planes::plane::{ControllerContext, PlaneId, PHYSICS_DT};
use ml_planes::training::int_mlp::{
    int_mlp_features, IntMlpBlock, IntMlpWeights, IntegratorState, INT_MLP_HIDDEN,
    INT_MLP_INPUT_DIM,
};
use ml_planes::training::int_mlp_model::IntMlpPolicy;
use ml_planes::training::level_hold_env::level_hold_observation;
use ml_planes::training::{ActorCritic, DemonstrationEnv, LevelHoldEnv, TrainingEnv};

type B = NdArray;

fn temp_stem(tag: &str) -> String {
    std::env::temp_dir()
        .join(format!("ml_planes_int_mlp_{tag}_{}", std::process::id()))
        .to_str()
        .unwrap()
        .to_string()
}

/// Deterministic pseudo-random inputs in [-1.5, 1.5] (no RNG dependency).
fn inputs(n: usize) -> Vec<[f32; INT_MLP_INPUT_DIM]> {
    let mut s = 0x2545_f491_4f6c_dd1d_u64;
    (0..n)
        .map(|_| {
            std::array::from_fn(|_| {
                s = s
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                ((s >> 40) as f32 / (1u64 << 24) as f32) * 3.0 - 1.5
            })
        })
        .collect()
}

/// A seeded network with every block perturbed, so the forward-parity check
/// exercises the skip path and biases (all zero at init).
fn perturbed_weights(seed: u64) -> IntMlpWeights {
    let device = Default::default();
    let mut w = IntMlpPolicy::<B>::new_seeded(&device, seed)
        .to_weights()
        .unwrap();
    for (i, v) in w.as_mut_slice().iter_mut().enumerate() {
        *v += ((i * 7919 % 97) as f32 / 97.0 - 0.5) * 0.2;
    }
    w
}

#[test]
fn burn_forward_matches_the_flat_kernel() {
    let device = Default::default();
    let w = perturbed_weights(3);
    let model = IntMlpPolicy::<B>::from_weights(&w, &device);
    let xs = inputs(32);
    let flat: Vec<f32> = xs.iter().flatten().copied().collect();
    let pre = model
        .forward(Tensor::<B, 2>::from_data(
            TensorData::new(flat, vec![xs.len(), INT_MLP_INPUT_DIM]),
            &device,
        ))
        .into_data()
        .to_vec::<f32>()
        .unwrap();
    for (row, x) in xs.iter().enumerate() {
        let expected = w.forward_pre_tanh(x);
        for a in 0..4 {
            let got = pre[row * 4 + a];
            assert!(
                (got - expected[a]).abs() < 1e-5,
                "row {row} action {a}: burn {got} vs flat {}",
                expected[a]
            );
        }
    }
}

#[test]
fn weights_round_trip_through_the_burn_module() {
    let device = Default::default();
    let w = perturbed_weights(4);
    let back = IntMlpPolicy::<B>::from_weights(&w, &device)
        .to_weights()
        .unwrap();
    assert_eq!(back, w);
}

#[test]
fn init_follows_the_experiment_recipe_and_is_seed_deterministic() {
    let device = Default::default();
    let w = IntMlpPolicy::<B>::new_seeded(&device, 11)
        .to_weights()
        .unwrap();
    assert_eq!(
        w,
        IntMlpPolicy::<B>::new_seeded(&device, 11)
            .to_weights()
            .unwrap(),
        "same seed must give the same init"
    );
    for block in [
        IntMlpBlock::Fc1Bias,
        IntMlpBlock::Fc2Bias,
        IntMlpBlock::OutBias,
        IntMlpBlock::SkipWeight,
        IntMlpBlock::SkipBias,
    ] {
        assert!(
            w.block(block).iter().all(|v| *v == 0.0),
            "{block:?} not zero"
        );
    }
    // Orthogonal with gain g on an [out, in] matrix: the smaller Gram matrix is g²·I.
    let gram_check = |block: IntMlpBlock, gain: f32| {
        let (rows, cols) = block.shape();
        let m = w.block(block);
        let k = rows.min(cols);
        for i in 0..k {
            for j in 0..k {
                let dot: f32 = if cols <= rows {
                    (0..rows).map(|r| m[r * cols + i] * m[r * cols + j]).sum()
                } else {
                    (0..cols).map(|c| m[i * cols + c] * m[j * cols + c]).sum()
                };
                let want = if i == j { gain * gain } else { 0.0 };
                assert!(
                    (dot - want).abs() < 1e-3 * gain.max(1.0) * gain.max(1.0),
                    "{block:?} gram[{i},{j}] = {dot}, want {want}"
                );
            }
        }
    };
    gram_check(IntMlpBlock::Fc1Weight, std::f32::consts::SQRT_2);
    gram_check(IntMlpBlock::Fc2Weight, std::f32::consts::SQRT_2);
    gram_check(IntMlpBlock::OutWeight, 0.01);
}

#[test]
fn saved_checkpoint_loads_into_a_controller_flying_the_same_network() {
    let device = Default::default();
    let w = perturbed_weights(5);
    let stem = temp_stem("roundtrip");
    IntMlpPolicy::<B>::from_weights(&w, &device)
        .save_file(
            &stem,
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
        )
        .unwrap();

    let mut env = LevelHoldEnv::new(1000.0, 110.0, crate::common::generic_jet_config());
    env.reset();
    let mut ctrl = IntMlpLevelHoldController::load(&stem, 1000.0, 110.0).expect("load");
    assert_eq!(ctrl.weights(), &w);

    let ctx = ControllerContext::empty_for(PlaneId::TEST);
    let mut integrators = IntegratorState::default();
    for step in 0..200 {
        let state = env.current_state();
        let obs = level_hold_observation(&state, 1000.0, 110.0);
        integrators.step(&obs, PHYSICS_DT);
        let a = w.action(&int_mlp_features(&obs, &integrators));
        let u = ctrl.update(&state, &ctx, PHYSICS_DT);
        assert_eq!(ctrl.integrators(), integrators, "step {step}");
        assert_eq!(u.elevator, a[0], "step {step}");
        assert!(
            (u.throttle - (a[1] + 1.0) / 2.0).abs() < 1e-6,
            "step {step}"
        );
        assert_eq!(u.aileron, a[2], "step {step}");
        assert_eq!(u.rudder, a[3], "step {step}");
        if env
            .step(&[u.elevator, u.throttle * 2.0 - 1.0, u.aileron, u.rudder])
            .done()
        {
            break;
        }
    }
}

#[test]
fn a_checkpoint_with_the_wrong_input_width_is_rejected() {
    let device = Default::default();
    let stale = IntMlpPolicy::<B> {
        fc1: LinearConfig::new(13, INT_MLP_HIDDEN).init(&device),
        fc2: LinearConfig::new(INT_MLP_HIDDEN, INT_MLP_HIDDEN).init(&device),
        out: LinearConfig::new(INT_MLP_HIDDEN, 4).init(&device),
        skip: LinearConfig::new(13, 4).init(&device),
    };
    let stem = temp_stem("stale");
    stale
        .save_file(
            &stem,
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
        )
        .unwrap();
    match IntMlpLevelHoldController::load(&stem, 1000.0, 100.0) {
        Err(ModelLoadError::DimensionMismatch { expected, found }) => {
            assert_eq!((expected, found), (INT_MLP_INPUT_DIM, 13));
        }
        other => panic!("expected DimensionMismatch, got {:?}", other.map(|_| ())),
    }
}

#[test]
fn an_actor_critic_checkpoint_is_not_an_int_mlp() {
    let device = Default::default();
    let stem = temp_stem("actor_critic");
    ActorCritic::<B>::new(&device, 13)
        .save_file(
            &stem,
            &DefaultFileRecorder::<FullPrecisionSettings>::default(),
        )
        .unwrap();
    assert!(IntMlpLevelHoldController::load(&stem, 1000.0, 100.0).is_err());
}

#[test]
fn a_changed_target_resets_only_its_own_integrator() {
    let mut ctrl = IntMlpLevelHoldController::from_weights(perturbed_weights(6), 1000.0, 100.0);
    let mut env = LevelHoldEnv::new(1000.0, 100.0, crate::common::generic_jet_config());
    env.alt_spawn_offset_range = 40.0..=40.0;
    env.airspeed_spawn_offset_range = 5.0..=5.0;
    env.reset();
    let ctx = ControllerContext::empty_for(PlaneId::TEST);
    for _ in 0..20 {
        ctrl.update(&env.current_state(), &ctx, PHYSICS_DT);
    }
    let before = ctrl.integrators();
    assert!(before.altitude != 0.0 && before.speed != 0.0, "{before:?}");
    let state = env.current_state();

    ctrl.apply_targets(
        &ControllerTargets::LevelHold {
            altitude: 1000.0,
            airspeed: 100.0,
        },
        &state,
    );
    assert_eq!(ctrl.integrators(), before, "re-applying identical targets");

    ctrl.apply_targets(&ControllerTargets::None, &state);
    assert_eq!(ctrl.integrators(), before, "mismatched variant is a no-op");

    ctrl.apply_targets(
        &ControllerTargets::LevelHold {
            altitude: 1200.0,
            airspeed: 100.0,
        },
        &state,
    );
    assert_eq!(ctrl.integrators().altitude, 0.0);
    assert_eq!(ctrl.integrators().speed, before.speed);
    assert_eq!(
        ctrl.targets(),
        ControllerTargets::LevelHold {
            altitude: 1200.0,
            airspeed: 100.0
        }
    );
}
