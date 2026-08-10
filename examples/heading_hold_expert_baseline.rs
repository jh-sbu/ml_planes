//! PID-expert reference rollout for the heading-hold training env.
//!
//! Rolls `HeadingHoldEnv::make_expert()` (the PID `HeadingHoldController` the BC
//! stage imitates) through the same env, tail window, and success definition
//! `evaluate_policy` uses, so an RL checkpoint's numbers can be compared against
//! what the classical controller achieves at the *same* operating point. Answers
//! "is this corner of the envelope hard for the policy, or hard for the plant?".
//!
//! Run with:
//!   cargo run --release --no-default-features --features training \
//!     --example heading_hold_expert_baseline -- \
//!     --episodes 150 --target-speed-range 90 --target-alt-range 5000

#[cfg(not(feature = "training"))]
fn main() {
    eprintln!("Build with --features training (or inference) to run the expert baseline.");
}

#[cfg(feature = "training")]
fn main() {
    use ml_planes::plane::context::ControllerContext;
    use ml_planes::plane::PlaneId;
    use ml_planes::training::heading_hold_env::{
        DEFAULT_TARGET_AIRSPEED_MAX, DEFAULT_TARGET_AIRSPEED_MIN, DEFAULT_TARGET_HEADING_DEG_MAX,
        DEFAULT_TARGET_HEADING_DEG_MIN,
    };
    use ml_planes::training::level_hold_env::{DEFAULT_TARGET_ALT_MAX, DEFAULT_TARGET_ALT_MIN};
    use ml_planes::training::reward_config::HeadingHoldRewardConfig;
    use ml_planes::training::{parse_f32_range, DemonstrationEnv, HeadingHoldEnv, TrainingEnv};

    let args: Vec<String> = std::env::args().collect();
    let arg = |name: &str| -> Option<String> {
        args.windows(2).find(|w| w[0] == name).map(|w| w[1].clone())
    };
    let range = |name: &str, lo: f32, hi: f32| -> std::ops::RangeInclusive<f32> {
        match arg(name) {
            Some(s) => parse_f32_range(&s).unwrap_or_else(|e| {
                eprintln!("{name}: {e}");
                std::process::exit(2);
            }),
            None => lo..=hi,
        }
    };

    let episodes: usize = arg("--episodes")
        .and_then(|s| s.parse().ok())
        .unwrap_or(150);
    let alt_range = range(
        "--target-alt-range",
        DEFAULT_TARGET_ALT_MIN,
        DEFAULT_TARGET_ALT_MAX,
    );
    let speed_range = range(
        "--target-speed-range",
        DEFAULT_TARGET_AIRSPEED_MIN,
        DEFAULT_TARGET_AIRSPEED_MAX,
    );
    let heading_deg = range(
        "--target-heading-range",
        DEFAULT_TARGET_HEADING_DEG_MIN,
        DEFAULT_TARGET_HEADING_DEG_MAX,
    );

    let reward_cfg = HeadingHoldRewardConfig::default();
    let max_steps = reward_cfg.max_episode_steps;
    // Same tail window as `evaluate_policy`: the final 20% of the step budget.
    let tail_start = max_steps - (max_steps as f32 * 0.2) as u32;

    let mut env = HeadingHoldEnv::with_target_ranges(
        heading_deg.start().to_radians()..=heading_deg.end().to_radians(),
        alt_range.clone(),
        speed_range.clone(),
        generic_jet_config(),
        reward_cfg,
    );
    env.max_episode_steps = max_steps;

    let ctx = ControllerContext::empty_for(PlaneId::TEST);
    let dt = ml_planes::plane::PHYSICS_DT;

    let (mut successes, mut tail_n) = (0usize, 0u64);
    let (mut tail_alt, mut tail_spd, mut tail_hdg) = (0.0f64, 0.0f64, 0.0f64);
    let mut tail_beta = 0.0f64;

    for _ in 0..episodes {
        env.reset();
        let mut expert = env.make_expert();
        let mut len = 0u32;
        let mut done = false;
        while !done && len < max_steps {
            let state = env.current_state();
            let inputs = expert.update(&state, &ctx, dt);
            let action = [
                inputs.elevator,
                inputs.throttle * 2.0 - 1.0,
                inputs.aileron,
                inputs.rudder,
            ];
            let (obs, _r, next_done, _info) = env.step(&action);
            len += 1;
            done = next_done;
            if len > tail_start {
                tail_alt += (obs[0] * 200.0).abs() as f64;
                tail_spd += (obs[1] * 50.0).abs() as f64;
                tail_hdg += (obs[13] * 0.5).atan2(obs[14]).abs() as f64;
                tail_beta += (obs[6] * 0.5).abs() as f64;
                tail_n += 1;
            }
        }
        if len >= max_steps {
            successes += 1;
        }
    }

    let n = tail_n.max(1) as f64;
    println!("controller,pid_heading_hold_expert");
    println!("episodes,{episodes}");
    println!("target_alt_range,{}:{}", alt_range.start(), alt_range.end());
    println!(
        "target_speed_range,{}:{}",
        speed_range.start(),
        speed_range.end()
    );
    println!(
        "target_heading_range_deg,{}:{}",
        heading_deg.start(),
        heading_deg.end()
    );
    println!("success_rate,{:.6}", successes as f32 / episodes as f32);
    println!("mean_tail_abs_heading_rad,{:.6}", tail_hdg / n);
    println!("mean_tail_abs_altitude_m,{:.3}", tail_alt / n);
    println!("mean_tail_abs_speed_mps,{:.3}", tail_spd / n);
    // Sideslip: the classical reference for "does this controller crab?" — the PID
    // expert closes β → rudder, so its tail β is what a coordinated policy can reach.
    println!("mean_tail_abs_beta_rad,{:.6}", tail_beta / n);
}

/// Mirrors `assets/planes/generic_jet.plane.ron` (same helper `evaluate_policy` uses).
#[cfg(feature = "training")]
fn generic_jet_config() -> ml_planes::plane::config::PlaneConfig {
    ml_planes::plane::config::PlaneConfig {
        wing_area: 20.0,
        mean_chord: 2.0,
        wing_span: 10.0,
        mass: 5000.0,
        inertia: bevy::math::Vec3::new(10000.0, 40000.0, 45000.0),
        cl0: 0.1,
        cl_alpha: 4.5,
        cl_delta_e: 0.4,
        cl_max: 1.4,
        cd0: 0.02,
        cd_induced: 0.05,
        cm0: -0.02,
        cm_alpha: 0.6,
        cm_q: -14.0,
        cm_delta_e: -1.2,
        cl_beta: 0.08,
        cl_p: -0.45,
        cl_r: -0.12,
        cl_delta_a: 0.18,
        cn_beta: 0.10,
        cn_r: -0.12,
        cn_delta_r: -0.10,
        cy_beta: -0.80,
        cy_p: 0.05,
        cy_r: 0.25,
        cy_delta_r: 0.18,
        thrust_max: 60000.0,
        powerplant: Default::default(),
        aileron_limit: 0.4363,
        elevator_limit: 0.3491,
        rudder_limit: 0.2618,
    }
}
