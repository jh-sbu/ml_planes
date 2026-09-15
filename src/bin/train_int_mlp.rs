//! Train an IntMLP level-hold policy: DAgger from `InversionLevelHoldController`, then
//! evolution strategies on the deterministic evaluation objective.
//!
//!   cargo run --release --no-default-features --features training --bin train_int_mlp -- \
//!     --output int_mlp_level_hold --threads 12
//!
//! Checkpoints go to `models/int_mlp_level_hold/`:
//!   <stem>_dagger.mpk   the DAgger clone (end of the `dagger` stage)
//!   <stem>.mpk          latest ES weights (rewritten at every validation)
//!   <stem>_best.mpk     best validation fitness so far
//!   <stem>_gNNNN.mpk    snapshot at every validation generation
//!
//! The ES objective anchors its tail penalty to the inversion controller's benchmark
//! tails, and the experiment's winner was generation ~270 of 500 — keep the snapshots
//! and pick by validation (and then `evaluate_policy --arch int_mlp`), not blindly the
//! last one. Defaults are the `experiments/nn_arch` values.
//!
//! Options:
//!   --stage <dagger|es|all>      Stages to run (default all)
//!   --output <stem>              Output stem (default: first free int_mlp_level_hold_N)
//!   --init <path>                es stage only: checkpoint (without .mpk) to start from
//!   --plane-config <path>        Airframe (default generic jet; unreadable is fatal)
//!   --seed <u64>                 Weight init, rollout seeds, ES noise (default 0)
//!   --threads <n>                ES rollout threads (default: available parallelism)
//!   --episode-steps <n>          Episode budget for DAgger, ES and validation (default 3200)
//!   --dagger-iterations <n> (6)  --dagger-envs <n> (128)   --dagger-epochs <n> (8)
//!   --dagger-minibatch <n> (4096) --dagger-lr <f> (2e-3)   --dagger-lr-decay <f> (0.6)
//!   --dagger-noise <f> (0.05)
//!   --es-generations <n> (500)   --es-population <n> (48)  --es-episodes <n> (24)
//!   --es-sigma <f> (0.001)       --es-lr <f> (3e-4)        --es-penalty <f> (20)
//!   --es-margin <f> (0.8)        --es-validate-every <n> (10)
//!   --es-validation-episodes <n> (64)
//!
//! Scoring always uses the shipped `assets/training/level_hold.reward.ron`: the ES
//! fitness is the evaluation return, so a different profile would change what "better"
//! means rather than tune how it is reached.

#[cfg(not(feature = "training"))]
fn main() {
    eprintln!("Build with --features training to train IntMLP level hold.");
}

#[cfg(feature = "training")]
fn main() {
    use burn::backend::{Autodiff, NdArray};
    use burn::module::Module;
    use burn::record::{DefaultFileRecorder, FullPrecisionSettings};
    use ml_planes::training::int_mlp::IntMlpWeights;
    use ml_planes::training::int_mlp_model::IntMlpPolicy;
    use ml_planes::training::int_mlp_train::{
        report_stats, run_dagger, run_es, DaggerConfig, EsConfig, VALIDATION_SEED,
        VALIDATION_SEED_STRIDE,
    };
    use ml_planes::training::reward_config::{load_reward_config, LevelHoldRewardConfig};
    use ml_planes::training::task::{self, EnvSpec, Task};
    use ml_planes::training::EvalReport;
    use std::time::Instant;

    const MODEL_DIR: &str = "models/int_mlp_level_hold";
    const REWARD: &str = "assets/training/level_hold.reward.ron";

    let args: Vec<String> = std::env::args().collect();
    let find = |flag: &str| args.windows(2).find(|w| w[0] == flag).map(|w| w[1].clone());
    fn parsed<T: std::str::FromStr>(value: Option<String>, flag: &str, default: T) -> T {
        match value {
            None => default,
            Some(v) => v.parse().unwrap_or_else(|_| {
                eprintln!("{flag}: could not parse '{v}'");
                std::process::exit(2);
            }),
        }
    }

    let stage = find("--stage").unwrap_or_else(|| "all".to_string());
    if !matches!(stage.as_str(), "dagger" | "es" | "all") {
        eprintln!("--stage must be dagger, es or all (got '{stage}')");
        std::process::exit(2);
    }
    let init = find("--init");
    if stage == "es" && init.is_none() {
        eprintln!("--stage es needs --init <checkpoint path without .mpk>");
        std::process::exit(2);
    }
    let seed: u64 = parsed(find("--seed"), "--seed", 0);
    let threads: usize = parsed(
        find("--threads"),
        "--threads",
        std::thread::available_parallelism().map_or(1, |n| n.get()),
    );
    let episode_steps: u32 = parsed(find("--episode-steps"), "--episode-steps", 3200);

    let d = DaggerConfig::default();
    let dagger = DaggerConfig {
        iterations: parsed(
            find("--dagger-iterations"),
            "--dagger-iterations",
            d.iterations,
        ),
        envs: parsed(find("--dagger-envs"), "--dagger-envs", d.envs),
        episode_steps,
        epochs: parsed(find("--dagger-epochs"), "--dagger-epochs", d.epochs),
        minibatch: parsed(
            find("--dagger-minibatch"),
            "--dagger-minibatch",
            d.minibatch,
        ),
        learning_rate: parsed(find("--dagger-lr"), "--dagger-lr", d.learning_rate),
        learning_rate_decay: parsed(
            find("--dagger-lr-decay"),
            "--dagger-lr-decay",
            d.learning_rate_decay,
        ),
        noise_std: parsed(find("--dagger-noise"), "--dagger-noise", d.noise_std),
        seed,
    };
    let e = EsConfig::default();
    let mut fitness = e.fitness;
    fitness.penalty = parsed(find("--es-penalty"), "--es-penalty", fitness.penalty);
    fitness.margin = parsed(find("--es-margin"), "--es-margin", fitness.margin);
    let es = EsConfig {
        generations: parsed(find("--es-generations"), "--es-generations", e.generations),
        population: parsed(find("--es-population"), "--es-population", e.population),
        episodes_per_member: parsed(
            find("--es-episodes"),
            "--es-episodes",
            e.episodes_per_member,
        ),
        sigma: parsed(find("--es-sigma"), "--es-sigma", e.sigma),
        learning_rate: parsed(find("--es-lr"), "--es-lr", e.learning_rate),
        fitness,
        episode_steps,
        validate_every: parsed(
            find("--es-validate-every"),
            "--es-validate-every",
            e.validate_every,
        ),
        validation_episodes: parsed(
            find("--es-validation-episodes"),
            "--es-validation-episodes",
            e.validation_episodes,
        ),
        threads,
        seed,
    };
    if es.population < 2 || es.population % 2 != 0 {
        eprintln!("--es-population must be an even number >= 2");
        std::process::exit(2);
    }

    let stem = match find("--output") {
        Some(s) => format!("{MODEL_DIR}/{s}"),
        None => (1u32..)
            .map(|n| format!("{MODEL_DIR}/int_mlp_level_hold_{n}"))
            .find(|c| !std::path::Path::new(&format!("{c}.mpk")).exists())
            .expect("unbounded search"),
    };
    std::fs::create_dir_all(MODEL_DIR).expect("create model output dir");

    let plane_config = find("--plane-config")
        .unwrap_or_else(|| ml_planes::training::DEFAULT_PLANE_CONFIG_PATH.to_string());
    let cfg = ml_planes::training::load_plane_config_or_exit(&plane_config);
    let reward: LevelHoldRewardConfig = load_reward_config(REWARD).unwrap_or_else(|err| {
        eprintln!("could not load scoring reward {REWARD}: {err}");
        std::process::exit(2);
    });
    let template = task::level_hold_env(&EnvSpec::defaults_for(Task::LevelHold, cfg), reward);
    println!(
        "plane_config={plane_config} reward={REWARD} seed={seed} threads={threads} out={stem}"
    );

    let device = Default::default();
    let save = |weights: &IntMlpWeights, path: &str| {
        IntMlpPolicy::<NdArray>::from_weights(weights, &device)
            .save_file(
                path,
                &DefaultFileRecorder::<FullPrecisionSettings>::default(),
            )
            .unwrap_or_else(|err| panic!("save {path}.mpk: {err}"));
    };
    let summary = |report: &EvalReport| {
        let s = report_stats(report);
        format!(
            "return {:>9.3}  success {:.3}  tail_alt {:.6} m  tail_spd {:.6} m/s",
            s.mean_return,
            1.0 - s.failure_fraction,
            s.tail_altitude,
            s.tail_speed
        )
    };
    let started = Instant::now();

    let mut weights = match &init {
        Some(path) => {
            let model = IntMlpPolicy::<NdArray>::new(&device)
                .load_file(
                    path,
                    &DefaultFileRecorder::<FullPrecisionSettings>::default(),
                    &device,
                )
                .unwrap_or_else(|err| {
                    eprintln!("could not load --init {path}.mpk: {err}");
                    std::process::exit(2);
                });
            model.to_weights().unwrap_or_else(|err| {
                eprintln!("--init {path}.mpk is not an IntMLP checkpoint: {err}");
                std::process::exit(2);
            })
        }
        None => IntMlpPolicy::<NdArray>::new_seeded(&device, seed)
            .to_weights()
            .expect("fresh IntMLP layout"),
    };

    if stage != "es" && init.is_none() {
        println!(
            "== DAgger: {} iterations x {} envs x {episode_steps} steps",
            dagger.iterations, dagger.envs
        );
        let model = run_dagger::<Autodiff<NdArray>>(&template, &dagger, &Default::default(), |p| {
            let val = ml_planes::training::int_mlp_train::evaluate_weights(
                p.weights,
                &template,
                es.validation_episodes,
                VALIDATION_SEED,
                VALIDATION_SEED_STRIDE,
                episode_steps,
            );
            println!(
                "  it {} samples {:>8} mse {:.3e} -> {:.3e} [{:.0}s] VAL {}",
                p.iteration,
                p.samples,
                p.first_epoch_mse,
                p.last_epoch_mse,
                started.elapsed().as_secs_f64(),
                summary(&val)
            );
        });
        weights = burn::module::AutodiffModule::valid(&model)
            .to_weights()
            .expect("IntMLP layout");
        save(&weights, &format!("{stem}_dagger"));
        println!("  saved {stem}_dagger.mpk");
    }

    if stage != "dagger" {
        println!(
            "== ES: {} generations, population {} x {} episodes, sigma {}, lr {}",
            es.generations, es.population, es.episodes_per_member, es.sigma, es.learning_rate
        );
        let mut best = f64::NEG_INFINITY;
        weights = run_es(weights, &template, &es, |p| match p.validation {
            None => println!(
                "  g{:>4} fit mean {:>9.3} max {:>9.3} [{:.0}s]",
                p.generation,
                p.mean_fitness,
                p.max_fitness,
                started.elapsed().as_secs_f64()
            ),
            Some((report, vfit)) => {
                println!(
                    "  g{:>4} fit mean {:>9.3} max {:>9.3} [{:.0}s] VAL {} vfit {vfit:.3}",
                    p.generation,
                    p.mean_fitness,
                    p.max_fitness,
                    started.elapsed().as_secs_f64(),
                    summary(report)
                );
                save(p.weights, &stem);
                save(p.weights, &format!("{stem}_g{:04}", p.generation));
                if vfit > best {
                    best = vfit;
                    save(p.weights, &format!("{stem}_best"));
                }
            }
        });
        save(&weights, &stem);
        println!("  saved {stem}.mpk (best validation fitness {best:.3} in {stem}_best.mpk)");
    }
    println!("done in {:.0}s", started.elapsed().as_secs_f64());
}
