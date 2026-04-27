//! PPO training binary.  Build and run with:
//!   cargo run --no-default-features --features training --bin train_ppo
//!
//! Trains a PPO level-hold controller for 2 000 000 environment steps and
//! saves the policy to `models/ppo_level_hold.mpk`.
//!
//! Flags:
//!   --plain   Print the traditional metrics table instead of the TUI display.

#[cfg(not(feature = "training"))]
fn main() {
    eprintln!("Build with --features training to run PPO training.");
    eprintln!("  cargo run --no-default-features --features training --bin train_ppo");
}

#[cfg(feature = "training")]
fn main() {
    use std::sync::Arc;
    use std::time::Instant;

    use burn::backend::{Autodiff, Wgpu};
    use burn::train::Interrupter;
    use burn::train::metric::{MetricAttributes, MetricDefinition, MetricId, NumericAttributes};
    use burn::train::renderer::{MetricsRenderer, TrainingProgress};
    use burn::train::renderer::tui::TuiMetricsRenderer;
    use burn::data::dataloader::Progress;
    use bevy::math::Vec3;

    use ml_planes::plane::config::PlaneConfig;
    use ml_planes::training::LevelHoldEnv;
    use ml_planes::training::ppo::PpoTrainer;

    type B = Autodiff<Wgpu>;

    let plain = std::env::args().any(|a| a == "--plain");

    // Generic jet values matching assets/planes/generic_jet.plane.ron
    let cfg = PlaneConfig {
        wing_area: 20.0, mean_chord: 2.0, wing_span: 10.0,
        mass: 5000.0, inertia: Vec3::new(10000.0, 40000.0, 45000.0),
        cl0: 0.1,   cl_alpha: 4.5,  cl_delta_e: 0.4,  cl_max: 1.4,
        cd0: 0.02,  cd_induced: 0.05,
        cm0: -0.02, cm_alpha: 0.6,  cm_q: -14.0,       cm_delta_e: -1.2,
        cl_beta: -0.08, cl_p: -0.45, cl_r: 0.12, cl_delta_a: 0.18,
        cn_beta: 0.10, cn_r: -0.12, cn_delta_r: -0.10,
        thrust_max: 60000.0,
        aileron_limit: 0.4363, elevator_limit: 0.3491, rudder_limit: 0.2618,
    };
    let env = LevelHoldEnv::new(1000.0, 100.0, cfg);

    let device: <B as burn::tensor::backend::Backend>::Device = Default::default();
    let trainer = PpoTrainer::<B>::new(env, device);

    let total_timesteps: usize = 2_000_000;
    let total_iterations = total_timesteps.div_ceil(trainer.rollout_steps);

    // --- metric IDs ---
    let id_mean_return = MetricId::new(Arc::new("mean_return".to_string()));
    let id_ep_len      = MetricId::new(Arc::new("ep_len".to_string()));
    let id_policy_loss = MetricId::new(Arc::new("policy_loss".to_string()));
    let id_value_loss  = MetricId::new(Arc::new("value_loss".to_string()));
    let id_entropy     = MetricId::new(Arc::new("entropy".to_string()));

    let interrupter = Interrupter::new();

    let mut renderer: Box<dyn MetricsRenderer> = if plain {
        Box::new(PlainMetricsRenderer::new(
            total_timesteps,
            id_mean_return.clone(),
            id_ep_len.clone(),
            id_policy_loss.clone(),
            id_value_loss.clone(),
            id_entropy.clone(),
        ))
    } else {
        Box::new(TuiMetricsRenderer::new(interrupter.clone(), None).persistent())
    };

    // Register metrics — must happen before any update_train call.
    let definitions = [
        MetricDefinition {
            metric_id: id_mean_return.clone(),
            name: "Mean Return".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes { unit: None, higher_is_better: true }),
        },
        MetricDefinition {
            metric_id: id_ep_len.clone(),
            name: "Ep Length".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes { unit: Some("steps".into()), higher_is_better: true }),
        },
        MetricDefinition {
            metric_id: id_policy_loss.clone(),
            name: "Policy Loss".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes { unit: None, higher_is_better: false }),
        },
        MetricDefinition {
            metric_id: id_value_loss.clone(),
            name: "Value Loss".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes { unit: None, higher_is_better: false }),
        },
        MetricDefinition {
            metric_id: id_entropy.clone(),
            name: "Entropy".into(),
            description: None,
            attributes: MetricAttributes::Numeric(NumericAttributes { unit: None, higher_is_better: true }),
        },
    ];
    for def in definitions {
        renderer.register_metric(def);
    }

    let mut trainer = trainer;
    let mut steps = 0usize;
    let mut iteration = 0usize;
    let start = Instant::now();

    while steps < total_timesteps {
        let (buffer, mean_return, mean_ep_len) = trainer.collect_rollout();
        steps += buffer.len();
        let metrics = trainer.update(&buffer);
        iteration += 1;

        renderer.update_train(numeric_state(id_mean_return.clone(), format!("{mean_return:.3}"), mean_return as f64));
        renderer.update_train(numeric_state(id_ep_len.clone(), format!("{mean_ep_len:.0}"), mean_ep_len as f64));
        renderer.update_train(numeric_state(id_policy_loss.clone(), format!("{:.4}", metrics.policy_loss), metrics.policy_loss as f64));
        renderer.update_train(numeric_state(id_value_loss.clone(), format!("{:.4}", metrics.value_loss), metrics.value_loss as f64));
        renderer.update_train(numeric_state(id_entropy.clone(), format!("{:.4}", metrics.entropy), metrics.entropy as f64));

        renderer.render_train(TrainingProgress {
            progress: Progress { items_processed: steps, items_total: total_timesteps },
            epoch: iteration,
            epoch_total: total_iterations,
            iteration,
        });

        if interrupter.should_stop() {
            break;
        }
    }

    renderer.on_train_end(None).ok();

    let elapsed_secs = start.elapsed().as_secs();
    println!(
        "Training complete ({steps} steps, {iteration} iterations, elapsed {}).",
        fmt_duration(elapsed_secs),
    );

    std::fs::create_dir_all("models").expect("create models dir");
    trainer.save_policy("models/ppo_level_hold");
}

#[cfg(feature = "training")]
fn numeric_state(
    id: burn::train::metric::MetricId,
    formatted: String,
    value: f64,
) -> burn::train::renderer::MetricState {
    use burn::train::metric::{MetricEntry, NumericEntry, SerializedEntry};
    use burn::train::renderer::MetricState;
    MetricState::Numeric(
        MetricEntry::new(id, SerializedEntry::new(formatted, value.to_string())),
        NumericEntry::Value(value),
    )
}

#[cfg(feature = "training")]
fn fmt_duration(secs: u64) -> String {
    let h = secs / 3600;
    let m = (secs % 3600) / 60;
    let s = secs % 60;
    if h > 0 {
        format!("{}:{:02}:{:02}", h, m, s)
    } else {
        format!("{:02}:{:02}", m, s)
    }
}

/// Plain table renderer — replicates the original metrics printout.
#[cfg(feature = "training")]
struct PlainMetricsRenderer {
    start: std::time::Instant,
    total_timesteps: usize,
    rows_since_header: usize,
    id_mean_return: burn::train::metric::MetricId,
    id_ep_len: burn::train::metric::MetricId,
    id_policy_loss: burn::train::metric::MetricId,
    id_value_loss: burn::train::metric::MetricId,
    id_entropy: burn::train::metric::MetricId,
    mean_return: f64,
    ep_len: f64,
    policy_loss: f64,
    value_loss: f64,
    entropy: f64,
}

#[cfg(feature = "training")]
impl PlainMetricsRenderer {
    fn new(
        total_timesteps: usize,
        id_mean_return: burn::train::metric::MetricId,
        id_ep_len: burn::train::metric::MetricId,
        id_policy_loss: burn::train::metric::MetricId,
        id_value_loss: burn::train::metric::MetricId,
        id_entropy: burn::train::metric::MetricId,
    ) -> Self {
        println!("Starting PPO training — target {} steps", total_timesteps);
        Self {
            start: std::time::Instant::now(),
            total_timesteps,
            rows_since_header: 0,
            id_mean_return,
            id_ep_len,
            id_policy_loss,
            id_value_loss,
            id_entropy,
            mean_return: 0.0,
            ep_len: 0.0,
            policy_loss: 0.0,
            value_loss: 0.0,
            entropy: 0.0,
        }
    }
}

#[cfg(feature = "training")]
impl burn::train::renderer::MetricsRendererTraining for PlainMetricsRenderer {
    fn update_train(&mut self, state: burn::train::renderer::MetricState) {
        use burn::train::renderer::MetricState;
        if let MetricState::Numeric(entry, num) = state {
            let v = num.current();
            if entry.metric_id == self.id_mean_return {
                self.mean_return = v;
            } else if entry.metric_id == self.id_ep_len {
                self.ep_len = v;
            } else if entry.metric_id == self.id_policy_loss {
                self.policy_loss = v;
            } else if entry.metric_id == self.id_value_loss {
                self.value_loss = v;
            } else if entry.metric_id == self.id_entropy {
                self.entropy = v;
            }
        }
    }

    fn update_valid(&mut self, _state: burn::train::renderer::MetricState) {}

    fn render_train(&mut self, item: burn::train::renderer::TrainingProgress) {
        let iteration = item.epoch;
        if iteration % 10 != 0 && iteration > 5 {
            return;
        }

        if self.rows_since_header == 0 {
            println!(
                "{:<6}  {:<11}  {:>5}  {:>8}  {:>9}  {:>9}  {:>8}  {:>6}  {:>8}  {:>8}  {:>8}",
                "iter", "steps", "pct", "steps/s", "elapsed", "eta",
                "mean_ret", "ep_len", "p_loss", "v_loss", "entropy",
            );
        }

        let steps = item.progress.items_processed;
        let elapsed_secs = self.start.elapsed().as_secs_f64();
        let steps_per_sec = steps as f64 / elapsed_secs;
        let pct = 100.0 * steps as f64 / self.total_timesteps as f64;
        let remaining = self.total_timesteps.saturating_sub(steps);
        let eta_secs = if steps_per_sec > 0.0 {
            (remaining as f64 / steps_per_sec) as u64
        } else {
            0
        };

        println!(
            "{:<6}  {:<11}  {:>4.1}%  {:>8.0}  {:>9}  {:>9}  {:>8.3}  {:>6.0}  {:>8.4}  {:>8.4}  {:>8.4}",
            iteration,
            steps,
            pct,
            steps_per_sec,
            fmt_duration(elapsed_secs as u64),
            fmt_duration(eta_secs),
            self.mean_return,
            self.ep_len,
            self.policy_loss,
            self.value_loss,
            self.entropy,
        );

        self.rows_since_header += 1;
        if self.rows_since_header >= 50 {
            self.rows_since_header = 0;
        }
    }

    fn render_valid(&mut self, _item: burn::train::renderer::TrainingProgress) {}
}

#[cfg(feature = "training")]
impl burn::train::renderer::MetricsRendererEvaluation for PlainMetricsRenderer {
    fn update_test(&mut self, _name: burn::train::renderer::EvaluationName, _state: burn::train::renderer::MetricState) {}
    fn render_test(&mut self, _item: burn::train::renderer::EvaluationProgress) {}
}

#[cfg(feature = "training")]
impl burn::train::renderer::MetricsRenderer for PlainMetricsRenderer {
    fn manual_close(&mut self) {}
    fn register_metric(&mut self, _definition: burn::train::metric::MetricDefinition) {}
}
