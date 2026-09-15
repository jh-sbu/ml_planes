//! Training pipeline for IntMLP level hold: DAgger from the inversion controller, then
//! evolution strategies (ES) on the deterministic evaluation objective.
//!
//! Promoted from `experiments/nn_arch` (`bc_dagger.py`, `es.py`); defaults are that
//! experiment's values. Why these two stages and not PPO: from-scratch PPO never learned
//! to use the integrator inputs, and PPO fine-tuning a good clone eroded it, because the
//! gains that matter (millimetres of tail altitude) are invisible to action-noise
//! advantage estimates. DAgger gets exact labels — the inversion controller is a pure
//! function of the observation and the same two integrators the learner carries — and ES
//! perturbs weights and scores every perturbation on the same episodes.
//!
//! Seeds: training episodes start at [`TRAIN_SEED_BASE`], disjoint from the benchmark
//! (42+i), the holdout (100000+i·104729) and validation ([`VALIDATION_SEED`]).

use burn::module::AutodiffModule;
use burn::optim::{AdamConfig, GradientsParams, Optimizer};
use burn::tensor::backend::AutodiffBackend;
use burn::tensor::{Tensor, TensorData};

use crate::controllers::InversionLevelHoldController;
use crate::plane::PHYSICS_DT;
use crate::training::eval_metrics::MetricFamily;
use crate::training::flight_env::inputs_to_direct_action;
use crate::training::int_mlp::{
    int_mlp_features, IntMlpWeights, IntegratorState, INT_MLP_ACTION_DIM, INT_MLP_INPUT_DIM,
};
use crate::training::int_mlp_model::IntMlpPolicy;
use crate::training::{
    DemonstrationEnv, EvalReport, EvalRun, LevelHoldEnv, TrainingEnv, TAIL_FRACTION,
};

/// First pre-reset seed of any training episode.
pub const TRAIN_SEED_BASE: u64 = 3_000_000;
/// Validation (model-selection) seeds: `VALIDATION_SEED + i * VALIDATION_SEED_STRIDE`.
pub const VALIDATION_SEED: u64 = 900_000;
pub const VALIDATION_SEED_STRIDE: u64 = 7919;
/// The inversion controller's benchmark tails — the bars the ES penalty is anchored to.
pub const INVERSION_BENCH_TAIL_ALTITUDE_M: f64 = 0.005_578;
pub const INVERSION_BENCH_TAIL_SPEED_MPS: f64 = 0.152_694;

// ---------------------------------------------------------------------------
// ES optimizer
// ---------------------------------------------------------------------------

/// Centred rank weights: the worst member gets −0.5, the best +0.5.
pub fn centred_ranks(fitness: &[f64]) -> Vec<f32> {
    let n = fitness.len();
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| fitness[a].total_cmp(&fitness[b]));
    let denom = (n.max(2) - 1) as f32;
    let mut weights = vec![0.0; n];
    for (rank, &i) in order.iter().enumerate() {
        weights[i] = rank as f32 / denom - 0.5;
    }
    weights
}

/// Rank-weighted ES gradient estimate `Σ w_i ε_i / (P σ)`.
pub fn es_gradient(noise: &[Vec<f32>], rank_weights: &[f32], sigma: f32) -> Vec<f32> {
    assert_eq!(
        noise.len(),
        rank_weights.len(),
        "one rank weight per member"
    );
    let dim = noise.first().map_or(0, Vec::len);
    let scale = 1.0 / (noise.len() as f32 * sigma);
    let mut grad = vec![0.0; dim];
    for (eps, &w) in noise.iter().zip(rank_weights) {
        for (g, e) in grad.iter_mut().zip(eps) {
            *g += w * e;
        }
    }
    grad.iter_mut().for_each(|g| *g *= scale);
    grad
}

/// Bias-corrected Adam, returning the step to ADD (gradient ascent).
#[derive(Debug, Clone)]
pub struct Adam {
    m: Vec<f32>,
    v: Vec<f32>,
    t: i32,
}

impl Adam {
    pub const BETA1: f32 = 0.9;
    pub const BETA2: f32 = 0.999;
    pub const EPSILON: f32 = 1e-8;

    pub fn new(dim: usize) -> Self {
        Self {
            m: vec![0.0; dim],
            v: vec![0.0; dim],
            t: 0,
        }
    }

    pub fn step(&mut self, grad: &[f32], learning_rate: f32) -> Vec<f32> {
        self.t += 1;
        let bc1 = 1.0 - Self::BETA1.powi(self.t);
        let bc2 = 1.0 - Self::BETA2.powi(self.t);
        grad.iter()
            .zip(self.m.iter_mut().zip(self.v.iter_mut()))
            .map(|(&g, (m, v))| {
                *m = Self::BETA1 * *m + (1.0 - Self::BETA1) * g;
                *v = Self::BETA2 * *v + (1.0 - Self::BETA2) * g * g;
                learning_rate * (*m / bc1) / ((*v / bc2).sqrt() + Self::EPSILON)
            })
            .collect()
    }
}

/// Antithetic, rank-shaped ES with Adam. `ask` samples a population around `theta`;
/// `tell` consumes one fitness per member (higher is better) and moves `theta`.
#[derive(Debug, Clone)]
pub struct EsOptimizer {
    pub theta: Vec<f32>,
    pub sigma: f32,
    pub learning_rate: f32,
    adam: Adam,
    rng: SplitMix64,
    noise: Vec<Vec<f32>>,
}

impl EsOptimizer {
    pub fn new(theta: Vec<f32>, sigma: f32, learning_rate: f32, seed: u64) -> Self {
        Self {
            adam: Adam::new(theta.len()),
            theta,
            sigma,
            learning_rate,
            rng: SplitMix64(seed),
            noise: Vec::new(),
        }
    }

    /// `population` (even) perturbed parameter vectors: `θ + σε` then `θ − σε` halves.
    pub fn ask(&mut self, population: usize) -> Vec<Vec<f32>> {
        assert!(
            population >= 2 && population % 2 == 0,
            "antithetic ES needs an even population, got {population}"
        );
        let dim = self.theta.len();
        let half: Vec<Vec<f32>> = (0..population / 2)
            .map(|_| (0..dim).map(|_| self.rng.normal()).collect())
            .collect();
        let mirrored = half
            .iter()
            .map(|e| e.iter().map(|v| -v).collect::<Vec<f32>>());
        self.noise = half.iter().cloned().chain(mirrored).collect();
        self.noise
            .iter()
            .map(|eps| {
                self.theta
                    .iter()
                    .zip(eps)
                    .map(|(t, e)| t + self.sigma * e)
                    .collect()
            })
            .collect()
    }

    pub fn tell(&mut self, fitness: &[f64]) {
        assert_eq!(
            fitness.len(),
            self.noise.len(),
            "tell() needs one fitness per member of the last ask()"
        );
        let grad = es_gradient(&self.noise, &centred_ranks(fitness), self.sigma);
        let step = self.adam.step(&grad, self.learning_rate);
        for (t, d) in self.theta.iter_mut().zip(step) {
            *t += d;
        }
    }
}

// ---------------------------------------------------------------------------
// Fitness
// ---------------------------------------------------------------------------

/// `return − pen·max(0, ln(tail_alt/(margin·bar_alt))) − pen·max(0, ln(tail_spd/(margin·bar_spd)))
/// − failure_penalty·failure_fraction`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FitnessConfig {
    pub penalty: f64,
    pub margin: f64,
    pub bar_tail_altitude: f64,
    pub bar_tail_speed: f64,
    pub failure_penalty: f64,
}

impl Default for FitnessConfig {
    fn default() -> Self {
        Self {
            penalty: 20.0,
            margin: 0.8,
            bar_tail_altitude: INVERSION_BENCH_TAIL_ALTITUDE_M,
            bar_tail_speed: INVERSION_BENCH_TAIL_SPEED_MPS,
            failure_penalty: 1000.0,
        }
    }
}

/// One deterministic episode's bookkeeping.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct EpisodeTally {
    pub total_return: f64,
    /// Σ |altitude error| over the tail window [m].
    pub tail_altitude_sum: f64,
    /// Σ |airspeed error| over the tail window [m/s].
    pub tail_speed_sum: f64,
    /// Steps in the tail window for this episode's budget.
    pub tail_window: u32,
    pub failed: bool,
}

/// Aggregate over one member's episodes.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MemberStats {
    /// Over ALL episodes, failed ones included.
    pub mean_return: f64,
    /// Over surviving episodes only; a failure is charged via `failure_fraction`.
    pub tail_altitude: f64,
    pub tail_speed: f64,
    pub failure_fraction: f64,
}

pub fn member_stats(tallies: &[EpisodeTally]) -> MemberStats {
    let n = tallies.len().max(1) as f64;
    let survivors: Vec<&EpisodeTally> = tallies.iter().filter(|t| !t.failed).collect();
    let per_survivor = |f: fn(&EpisodeTally) -> f64| {
        if survivors.is_empty() {
            0.0
        } else {
            survivors
                .iter()
                .map(|t| f(t) / t.tail_window.max(1) as f64)
                .sum::<f64>()
                / survivors.len() as f64
        }
    };
    MemberStats {
        mean_return: tallies.iter().map(|t| t.total_return).sum::<f64>() / n,
        tail_altitude: per_survivor(|t| t.tail_altitude_sum),
        tail_speed: per_survivor(|t| t.tail_speed_sum),
        failure_fraction: (tallies.len() - survivors.len()) as f64 / n,
    }
}

pub fn es_fitness(stats: &MemberStats, cfg: &FitnessConfig) -> f64 {
    let hinge = |value: f64, bar: f64| (value.max(1e-9) / (cfg.margin * bar)).ln().max(0.0);
    stats.mean_return
        - cfg.penalty * hinge(stats.tail_altitude, cfg.bar_tail_altitude)
        - cfg.penalty * hinge(stats.tail_speed, cfg.bar_tail_speed)
        - cfg.failure_penalty * stats.failure_fraction
}

/// The same statistics read off a shared-`EvalRun` report (validation).
pub fn report_stats(report: &EvalReport) -> MemberStats {
    let row = |key: &str| {
        report
            .rows
            .iter()
            .find(|r| r.key == key)
            .map_or(0.0, |r| r.value as f64)
    };
    MemberStats {
        mean_return: report.mean_return as f64,
        tail_altitude: row("mean_tail_abs_altitude_m"),
        tail_speed: row("mean_tail_abs_speed_mps"),
        failure_fraction: 1.0 - report.success_rate as f64,
    }
}

// ---------------------------------------------------------------------------
// Rollouts
// ---------------------------------------------------------------------------

/// Fly one deterministic episode: `set_rng_seed(pre_reset_seed)`, `reset`, then at most
/// `steps` steps, integrators advanced before each forward pass.
pub fn rollout_episode(
    weights: &IntMlpWeights,
    env: &mut LevelHoldEnv,
    pre_reset_seed: u64,
    steps: u32,
) -> EpisodeTally {
    env.set_rng_seed(pre_reset_seed);
    let (mut obs, _) = env.reset();
    // Same window as `EvalRun`: the final `TAIL_FRACTION` of the step budget.
    let tail_window = (steps as f32 * TAIL_FRACTION) as u32;
    let tail_start = steps.saturating_sub(tail_window);
    let mut tally = EpisodeTally {
        tail_window,
        ..EpisodeTally::default()
    };
    let mut integrators = IntegratorState::default();
    for t in 0..steps {
        integrators.step(&obs, PHYSICS_DT);
        let action = weights.action(&int_mlp_features(&obs, &integrators));
        let outcome = env.step(&action);
        tally.total_return += outcome.reward as f64;
        if t >= tail_start {
            tally.tail_altitude_sum += (outcome.obs[0] * 200.0).abs() as f64;
            tally.tail_speed_sum += (outcome.obs[1] * 50.0).abs() as f64;
        }
        if outcome.done() {
            tally.failed = !outcome.truncated();
            break;
        }
        obs = outcome.obs;
    }
    tally
}

/// Deterministic evaluation through the shared `EvalRun`, episode `i` pre-reset seeded
/// `seed + i * stride` — the protocol `evaluate_policy --seed/--seed-stride` uses.
pub fn evaluate_weights(
    weights: &IntMlpWeights,
    template: &LevelHoldEnv,
    episodes: usize,
    seed: u64,
    stride: u64,
    steps: u32,
) -> EvalReport {
    let mut env = template.clone();
    env.max_episode_steps = steps;
    let mut run = EvalRun::new(MetricFamily::LevelHold, episodes, steps, 1);
    for i in 0..episodes {
        env.set_rng_seed(seed.wrapping_add((i as u64).wrapping_mul(stride)));
        let (mut obs, _) = env.reset();
        let mut integrators = IntegratorState::default();
        for _ in 0..steps {
            integrators.step(&obs, PHYSICS_DT);
            let action = weights.action(&int_mlp_features(&obs, &integrators));
            let outcome = env.step(&action);
            run.record(0, &outcome.obs, outcome.reward)
                .expect("single-slot evaluation");
            let done = outcome.done();
            obs = outcome.obs;
            if done {
                break;
            }
        }
        run.finish(0, &obs).expect("episode had at least one step");
    }
    run.report()
}

// ---------------------------------------------------------------------------
// ES stage
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct EsConfig {
    pub generations: u32,
    pub population: usize,
    pub episodes_per_member: usize,
    pub sigma: f32,
    pub learning_rate: f32,
    pub fitness: FitnessConfig,
    pub episode_steps: u32,
    pub validate_every: u32,
    pub validation_episodes: usize,
    pub threads: usize,
    pub seed: u64,
}

impl Default for EsConfig {
    fn default() -> Self {
        Self {
            generations: 500,
            population: 48,
            episodes_per_member: 24,
            sigma: 0.001,
            learning_rate: 3e-4,
            fitness: FitnessConfig::default(),
            episode_steps: 3200,
            validate_every: 10,
            validation_episodes: 64,
            threads: 1,
            seed: 0,
        }
    }
}

/// Per-generation progress handed to the caller (for logging and checkpointing).
pub struct EsProgress<'a> {
    pub generation: u32,
    pub mean_fitness: f64,
    pub max_fitness: f64,
    /// `(report, validation fitness)` on validation generations.
    pub validation: Option<(&'a EvalReport, f64)>,
    pub weights: &'a IntMlpWeights,
}

pub fn run_es(
    init: IntMlpWeights,
    template: &LevelHoldEnv,
    cfg: &EsConfig,
    on_generation: impl FnMut(EsProgress<'_>),
) -> IntMlpWeights {
    let mut on_generation = on_generation;
    let mut env_template = template.clone();
    env_template.max_episode_steps = cfg.episode_steps;
    let mut es = EsOptimizer::new(
        init.into_vec(),
        cfg.sigma,
        cfg.learning_rate,
        cfg.seed ^ 0x9E37_79B9_7F4A_7C15,
    );
    let threads = cfg.threads.max(1);

    for generation in 1..=cfg.generations {
        // Common random numbers: every member flies the same episodes this generation.
        let seed_base =
            TRAIN_SEED_BASE + 7_000_000 + cfg.seed * 1_000_000 + generation as u64 * 1009;
        let members = es.ask(cfg.population);
        let chunk = members.len().div_ceil(threads);
        let fitness: Vec<f64> = std::thread::scope(|scope| {
            let workers: Vec<_> = members
                .chunks(chunk)
                .map(|group| {
                    let template = &env_template;
                    scope.spawn(move || {
                        let mut env = template.clone();
                        group
                            .iter()
                            .map(|params| {
                                let weights = IntMlpWeights::from_vec(params.clone())
                                    .expect("ES members keep the IntMLP layout");
                                let tallies: Vec<EpisodeTally> = (0..cfg.episodes_per_member)
                                    .map(|e| {
                                        rollout_episode(
                                            &weights,
                                            &mut env,
                                            seed_base + e as u64 * 7,
                                            cfg.episode_steps,
                                        )
                                    })
                                    .collect();
                                es_fitness(&member_stats(&tallies), &cfg.fitness)
                            })
                            .collect::<Vec<f64>>()
                    })
                })
                .collect();
            workers
                .into_iter()
                .flat_map(|w| w.join().expect("ES rollout worker panicked"))
                .collect()
        });
        es.tell(&fitness);

        let weights = IntMlpWeights::from_vec(es.theta.clone()).expect("IntMLP layout");
        let validation = (generation % cfg.validate_every.max(1) == 0
            || generation == cfg.generations)
            .then(|| {
                let report = evaluate_weights(
                    &weights,
                    &env_template,
                    cfg.validation_episodes,
                    VALIDATION_SEED,
                    VALIDATION_SEED_STRIDE,
                    cfg.episode_steps,
                );
                let vfit = es_fitness(&report_stats(&report), &cfg.fitness);
                (report, vfit)
            });
        on_generation(EsProgress {
            generation,
            mean_fitness: fitness.iter().sum::<f64>() / fitness.len() as f64,
            max_fitness: fitness.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            validation: validation.as_ref().map(|(r, f)| (r, *f)),
            weights: &weights,
        });
    }
    IntMlpWeights::from_vec(es.theta).expect("IntMLP layout")
}

// ---------------------------------------------------------------------------
// DAgger stage
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct DaggerConfig {
    pub iterations: usize,
    pub envs: usize,
    pub episode_steps: u32,
    pub epochs: usize,
    pub minibatch: usize,
    pub learning_rate: f64,
    /// Per-iteration learning-rate multiplier: iteration `k` fits at `lr · decay^k`.
    pub learning_rate_decay: f64,
    /// Std of the Gaussian action noise on learner rollouts (iterations ≥ 1).
    pub noise_std: f32,
    pub seed: u64,
}

impl Default for DaggerConfig {
    fn default() -> Self {
        Self {
            iterations: 6,
            envs: 128,
            episode_steps: 3200,
            epochs: 8,
            minibatch: 4096,
            learning_rate: 2e-3,
            learning_rate_decay: 0.6,
            noise_std: 0.05,
            seed: 0,
        }
    }
}

pub struct DaggerProgress<'a> {
    pub iteration: usize,
    /// Aggregated dataset size after this iteration's rollout.
    pub samples: usize,
    pub first_epoch_mse: f32,
    pub last_epoch_mse: f32,
    pub weights: &'a IntMlpWeights,
}

pub fn run_dagger<B: AutodiffBackend>(
    template: &LevelHoldEnv,
    cfg: &DaggerConfig,
    device: &B::Device,
    on_iteration: impl FnMut(DaggerProgress<'_>),
) -> IntMlpPolicy<B> {
    let mut on_iteration = on_iteration;
    let mut env_template = template.clone();
    env_template.max_episode_steps = cfg.episode_steps;
    let mut model = IntMlpPolicy::<B>::new_seeded(device, cfg.seed);
    let mut optimizer = AdamConfig::new()
        .with_epsilon(1e-8)
        .init::<B, IntMlpPolicy<B>>();
    let mut rng = SplitMix64(cfg.seed ^ 0xDA66_E125_EED5_0000);
    let mut features: Vec<[f32; INT_MLP_INPUT_DIM]> = Vec::new();
    let mut labels: Vec<[f32; INT_MLP_ACTION_DIM]> = Vec::new();

    for iteration in 0..cfg.iterations {
        // Iteration 0 flies the expert; later iterations fly the (noisy) learner and
        // label the states IT visits — the distribution shift plain BC never sees.
        let learner = (iteration > 0).then(|| {
            model
                .valid()
                .to_weights()
                .expect("a module built by IntMlpPolicy keeps its layout")
        });
        for i in 0..cfg.envs {
            let mut env = env_template.clone();
            env.set_rng_seed(
                TRAIN_SEED_BASE + cfg.seed * 100_000 + iteration as u64 * 10_000 + i as u64 * 131,
            );
            let (mut obs, _) = env.reset();
            let mut integrators = IntegratorState::default();
            for _ in 0..cfg.episode_steps {
                integrators.step(&obs, PHYSICS_DT);
                let x = int_mlp_features(&obs, &integrators);
                // Exact label: the inversion controller at this state, with THIS
                // learner's integrators standing in for its own.
                let expert =
                    InversionLevelHoldController::new(env.target_altitude, env.target_airspeed)
                        .command_with_integrals(
                            &env.current_state(),
                            integrators.altitude as f64,
                            integrators.speed as f64,
                        );
                let label = inputs_to_direct_action(&expert);
                features.push(x);
                labels.push(label);
                let action = match &learner {
                    None => label,
                    Some(weights) => weights
                        .action(&x)
                        .map(|a| (a + cfg.noise_std * rng.normal()).clamp(-1.0, 1.0)),
                };
                let outcome = env.step(&action);
                if outcome.done() {
                    obs = env.reset().0;
                    integrators = IntegratorState::default();
                } else {
                    obs = outcome.obs;
                }
            }
        }

        let lr = cfg.learning_rate * cfg.learning_rate_decay.powi(iteration as i32);
        let mut order: Vec<usize> = (0..features.len()).collect();
        let (mut first_epoch_mse, mut last_epoch_mse) = (f32::NAN, f32::NAN);
        for epoch in 0..cfg.epochs {
            rng.shuffle(&mut order);
            let (mut weighted_loss, mut seen) = (0.0_f64, 0_usize);
            for batch in order.chunks(cfg.minibatch.max(1)) {
                let b = batch.len();
                let x: Vec<f32> = batch.iter().flat_map(|&k| features[k]).collect();
                let y: Vec<f32> = batch.iter().flat_map(|&k| labels[k]).collect();
                let x = Tensor::<B, 2>::from_data(
                    TensorData::new(x, vec![b, INT_MLP_INPUT_DIM]),
                    device,
                );
                let y = Tensor::<B, 2>::from_data(
                    TensorData::new(y, vec![b, INT_MLP_ACTION_DIM]),
                    device,
                );
                let loss = (model.forward(x).tanh() - y).powf_scalar(2.0).mean();
                let value = loss.clone().into_data().to_vec::<f32>().expect("loss")[0];
                weighted_loss += value as f64 * b as f64;
                seen += b;
                let grads = GradientsParams::from_grads(loss.backward(), &model);
                model = optimizer.step(lr, model, grads);
            }
            let mse = (weighted_loss / seen.max(1) as f64) as f32;
            if epoch == 0 {
                first_epoch_mse = mse;
            }
            last_epoch_mse = mse;
        }

        let weights = model
            .valid()
            .to_weights()
            .expect("a module built by IntMlpPolicy keeps its layout");
        on_iteration(DaggerProgress {
            iteration,
            samples: features.len(),
            first_epoch_mse,
            last_epoch_mse,
            weights: &weights,
        });
    }
    model
}

// ---------------------------------------------------------------------------
// Deterministic RNG (no `rand` dependency in the library)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct SplitMix64(u64);

impl SplitMix64 {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in the open interval (0, 1).
    fn uniform(&mut self) -> f64 {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// Standard normal via Box–Muller.
    fn normal(&mut self) -> f32 {
        let (u1, u2) = (self.uniform(), self.uniform());
        ((-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()) as f32
    }

    /// Fisher–Yates.
    fn shuffle<T>(&mut self, items: &mut [T]) {
        for i in (1..items.len()).rev() {
            let j = (self.next_u64() % (i as u64 + 1)) as usize;
            items.swap(i, j);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::training::TrainingEnv;

    fn template() -> LevelHoldEnv {
        LevelHoldEnv::with_target_ranges(
            500.0..=5000.0,
            90.0..=140.0,
            crate::plane::config::fixture_jet_config(),
            crate::training::LevelHoldRewardConfig::default(),
        )
    }

    #[test]
    fn centred_ranks_span_minus_half_to_plus_half() {
        assert_eq!(centred_ranks(&[3.0, 1.0, 2.0]), vec![0.5, -0.5, 0.0]);
        assert_eq!(centred_ranks(&[-1.0, 7.0]), vec![-0.5, 0.5]);
    }

    #[test]
    fn es_gradient_is_the_rank_weighted_noise_sum() {
        let noise = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        let g = es_gradient(&noise, &[0.5, -0.5], 0.1);
        assert!(
            (g[0] - 2.5).abs() < 1e-6 && (g[1] + 2.5).abs() < 1e-6,
            "{g:?}"
        );
    }

    #[test]
    fn adam_first_step_moves_each_coordinate_by_the_learning_rate() {
        let mut adam = Adam::new(2);
        let step = adam.step(&[2.0, -0.1], 0.01);
        assert!((step[0] - 0.01).abs() < 1e-6, "{step:?}");
        assert!((step[1] + 0.01).abs() < 1e-6, "{step:?}");
    }

    #[test]
    fn antithetic_population_mirrors_about_theta() {
        let mut es = EsOptimizer::new(vec![1.0, -2.0, 0.5], 0.1, 0.01, 7);
        let pop = es.ask(4);
        assert_eq!(pop.len(), 4);
        for i in 0..2 {
            for d in 0..3 {
                let mid = (pop[i][d] + pop[i + 2][d]) / 2.0;
                assert!((mid - es.theta[d]).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn es_maximises_a_toy_objective() {
        let target = [3.0f32, -1.0, 0.5, 2.0, -2.5];
        let mut es = EsOptimizer::new(vec![0.0; 5], 0.1, 0.05, 11);
        for _ in 0..400 {
            let pop = es.ask(20);
            let fit: Vec<f64> = pop
                .iter()
                .map(|p| {
                    -p.iter()
                        .zip(target)
                        .map(|(x, t)| ((x - t) as f64).powi(2))
                        .sum::<f64>()
                })
                .collect();
            es.tell(&fit);
        }
        for (x, t) in es.theta.iter().zip(target) {
            assert!((x - t).abs() < 0.15, "theta {:?}", es.theta);
        }
    }

    #[test]
    fn fitness_penalises_only_tails_above_the_margin_and_charges_failures() {
        let cfg = FitnessConfig::default();
        let mut s = MemberStats {
            mean_return: -80.0,
            tail_altitude: 0.5 * cfg.margin * cfg.bar_tail_altitude,
            tail_speed: 0.5 * cfg.margin * cfg.bar_tail_speed,
            failure_fraction: 0.0,
        };
        assert_eq!(es_fitness(&s, &cfg), -80.0);
        s.tail_altitude = 2.0 * cfg.margin * cfg.bar_tail_altitude;
        assert!((es_fitness(&s, &cfg) - (-80.0 - 20.0 * 2f64.ln())).abs() < 1e-9);
        s.failure_fraction = 0.25;
        assert!((es_fitness(&s, &cfg) - (-80.0 - 20.0 * 2f64.ln() - 250.0)).abs() < 1e-9);
    }

    #[test]
    fn member_tails_average_survivors_only() {
        let survivor = EpisodeTally {
            total_return: -10.0,
            tail_altitude_sum: 640.0 * 0.01,
            tail_speed_sum: 640.0 * 0.2,
            tail_window: 640,
            failed: false,
        };
        let crashed = EpisodeTally {
            total_return: -30.0,
            tail_altitude_sum: 9999.0,
            tail_speed_sum: 9999.0,
            tail_window: 640,
            failed: true,
        };
        let s = member_stats(&[survivor, crashed]);
        assert!((s.mean_return - -20.0).abs() < 1e-12);
        assert!((s.tail_altitude - 0.01).abs() < 1e-12);
        assert!((s.tail_speed - 0.2).abs() < 1e-12);
        assert_eq!(s.failure_fraction, 0.5);
    }

    #[test]
    fn rollout_matches_the_shared_eval_run_on_the_same_seed() {
        let mut w = IntMlpWeights::zeros();
        // A mild throttle bias keeps the zero policy airborne for the short budget.
        w.block_mut(crate::training::int_mlp::IntMlpBlock::SkipBias)[1] = 0.2;
        let steps = 400;
        let mut env = template();
        env.max_episode_steps = steps;
        let tally = rollout_episode(&w, &mut env, 1234, steps);
        let report = evaluate_weights(&w, &template(), 1, 1234, 1, steps);
        let stats = report_stats(&report);
        assert!(!tally.failed && report.success_rate == 1.0);
        assert!((tally.total_return - report.mean_return as f64).abs() < 1e-3);
        let tail_steps = tally.tail_window as f64;
        assert!((tally.tail_altitude_sum / tail_steps - stats.tail_altitude).abs() < 1e-3);
        assert!((tally.tail_speed_sum / tail_steps - stats.tail_speed).abs() < 1e-3);
        let _ = env.observation_dim();
    }

    #[test]
    fn dagger_smoke_fits_the_expert() {
        use burn::backend::{Autodiff, NdArray};
        let cfg = DaggerConfig {
            iterations: 2,
            envs: 4,
            episode_steps: 64,
            epochs: 4,
            minibatch: 64,
            ..DaggerConfig::default()
        };
        let mut seen = Vec::new();
        let model = run_dagger::<Autodiff<NdArray>>(&template(), &cfg, &Default::default(), |p| {
            seen.push((p.iteration, p.samples, p.first_epoch_mse, p.last_epoch_mse));
        });
        assert_eq!(seen.len(), 2);
        assert_eq!((seen[0].1, seen[1].1), (4 * 64, 2 * 4 * 64));
        for (_, _, first, last) in &seen {
            assert!(first.is_finite() && last.is_finite());
        }
        assert!(seen[0].3 < seen[0].2, "fit did not reduce mse: {seen:?}");
        let w = model.to_weights().unwrap();
        assert!(w.as_slice().iter().all(|v| v.is_finite()));
    }

    #[test]
    fn es_smoke_runs_and_validates() {
        let mut w = IntMlpWeights::zeros();
        w.block_mut(crate::training::int_mlp::IntMlpBlock::SkipBias)[1] = 0.2;
        let cfg = EsConfig {
            generations: 2,
            population: 4,
            episodes_per_member: 2,
            episode_steps: 64,
            validate_every: 2,
            validation_episodes: 2,
            threads: 2,
            ..EsConfig::default()
        };
        let mut generations = Vec::new();
        let out = run_es(w.clone(), &template(), &cfg, |p| {
            generations.push((p.generation, p.validation.is_some()));
        });
        assert_eq!(generations, vec![(1, false), (2, true)]);
        assert_ne!(out, w, "ES must move theta");
        assert!(out.as_slice().iter().all(|v| v.is_finite()));
    }
}
