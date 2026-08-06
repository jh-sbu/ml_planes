//! Parse guards for every shipped `assets/training/*.reward.ron` profile.
//!
//! The reward-config structs deliberately carry no `#[serde(default)]` (see
//! `HeadingHoldRewardConfig`'s doc comment), so adding a field silently invalidates
//! every profile file that predates it — and the training binaries treat a parse
//! failure as a *warning*, falling back to the compiled defaults. A run started that
//! way trains against the task default instead of the profile the operator asked for
//! and still reports "training complete". These tests turn that into a test failure.

use ml_planes::training::reward_config::{
    HeadingHoldRewardConfig, LevelHoldRewardConfig, OrbitRewardConfig,
};
use ml_planes::training::wu_orbit_reward::WuOrbitRewardConfig;

fn profiles_matching(prefix: &str) -> Vec<(String, Vec<u8>)> {
    let dir = std::path::Path::new("assets/training");
    let mut found = Vec::new();
    for entry in std::fs::read_dir(dir).expect("assets/training should exist") {
        let path = entry.expect("readable dir entry").path();
        let name = path.file_name().unwrap().to_string_lossy().into_owned();
        if !name.ends_with(".reward.ron") || !name.starts_with(prefix) {
            continue;
        }
        let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("read {name}: {e}"));
        found.push((name, bytes));
    }
    found
}

#[test]
fn shipped_heading_hold_reward_profiles_parse() {
    let profiles = profiles_matching("heading_hold");
    assert!(
        profiles.len() >= 3,
        "expected the shipped heading_hold profiles, found {}",
        profiles.len()
    );
    for (name, bytes) in profiles {
        let cfg: HeadingHoldRewardConfig =
            ron::de::from_bytes(&bytes).unwrap_or_else(|e| panic!("parse {name}: {e}"));
        assert!(
            cfg.alpha_soft_limit > 0.0 && cfg.alpha_excess_scale > 0.0,
            "{name}: alpha guard scales must be positive even when the weight is 0 \
             (a zero scale divides by zero the moment someone enables the term)"
        );
    }
}

#[test]
fn shipped_level_hold_reward_profiles_parse() {
    let profiles = profiles_matching("level_hold");
    assert!(
        profiles.len() >= 3,
        "expected the shipped level_hold profiles, found {}",
        profiles.len()
    );
    for (name, bytes) in profiles {
        let _cfg: LevelHoldRewardConfig =
            ron::de::from_bytes(&bytes).unwrap_or_else(|e| panic!("parse {name}: {e}"));
    }
}

#[test]
fn shipped_orbit_reward_profiles_parse() {
    for (name, bytes) in profiles_matching("orbit") {
        let _cfg: OrbitRewardConfig =
            ron::de::from_bytes(&bytes).unwrap_or_else(|e| panic!("parse {name}: {e}"));
    }
    for (name, bytes) in profiles_matching("wu_orbit") {
        let _cfg: WuOrbitRewardConfig =
            ron::de::from_bytes(&bytes).unwrap_or_else(|e| panic!("parse {name}: {e}"));
    }
}
