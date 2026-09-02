//! Parse guards for every shipped `assets/training/*.reward.ron` profile.
//!
//! The reward-config structs deliberately carry no `#[serde(default)]` (see
//! `HeadingHoldRewardConfig`'s doc comment), so adding a field silently invalidates
//! every profile file that predates it — and the training binaries treat a parse
//! failure as a *warning*, falling back to the compiled defaults. A run started that
//! way trains against the task default instead of the profile the operator asked for
//! and still reports "training complete". These tests turn that into a test failure.

use ml_planes::controllers::{PlaneTuning, RefuelingTuning};
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

/// Every shipped `.tuning.ron` must carry a parseable `refueling."normal"` profile, and
/// its **outer** block (stations, gates, rates, cascade gains) must match
/// `RefuelingTuning::default()`.
///
/// The `inner` block is deliberately excluded: it is the airframe's own validated
/// level-hold tuning and legitimately differs per plane. The outer block is the station
/// ladder itself, and a file that quietly drifts from the compiled defaults would mean the
/// live sim (which applies the asset) and everything built from
/// `RefuelingTuning::default()` — the scenario path with no inline tuning, and every
/// unit test — are flying different geometry.
#[test]
fn shipped_refueling_profiles_match_the_compiled_station_ladder() {
    let expected = RefuelingTuning::default();
    let mut checked = 0;
    for entry in std::fs::read_dir("assets/planes").expect("assets/planes must exist") {
        let path = entry.expect("dir entry").path();
        if !path.to_string_lossy().ends_with(".tuning.ron") {
            continue;
        }
        let text = std::fs::read_to_string(&path).expect("read tuning file");
        let pt: PlaneTuning = ron::de::from_str(&text)
            .unwrap_or_else(|e| panic!("{} must parse: {e}", path.display()));
        let r = pt
            .get_refueling("normal")
            .unwrap_or_else(|| panic!("{} must define refueling.normal", path.display()));

        let name = path.display();
        assert_eq!(r.astern, expected.astern, "{name}: astern station drifted");
        assert_eq!(
            r.precontact, expected.precontact,
            "{name}: precontact station drifted"
        );
        assert_eq!(
            r.contact, expected.contact,
            "{name}: contact station drifted"
        );
        assert_eq!(r.capture_radius, expected.capture_radius, "{name}");
        assert_eq!(r.contact_radius, expected.contact_radius, "{name}");
        assert_eq!(r.closure_tolerance, expected.closure_tolerance, "{name}");
        assert_eq!(r.dwell_secs, expected.dwell_secs, "{name}");
        assert_eq!(r.abort_radius, expected.abort_radius, "{name}");
        assert_eq!(r.approach_rate, expected.approach_rate, "{name}");
        assert_eq!(r.breakaway_rate, expected.breakaway_rate, "{name}");
        assert_eq!(r.range_kp, expected.range_kp, "{name}");
        assert_eq!(r.range_ki, expected.range_ki, "{name}");
        assert_eq!(r.range_kd, expected.range_kd, "{name}");
        assert_eq!(r.lateral_kp, expected.lateral_kp, "{name}");
        assert_eq!(r.lateral_kd, expected.lateral_kd, "{name}");
        assert_eq!(r.heading_kp, expected.heading_kp, "{name}");
        assert_eq!(r.heading_kd, expected.heading_kd, "{name}");
        checked += 1;
    }
    assert!(
        checked >= 5,
        "expected every shipped airframe, checked {checked}"
    );
}
