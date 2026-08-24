//! Shared CLI argument parsing helpers for the training/eval binaries
//! (`train_ppo`, `train_bc`, `evaluate_policy`).
//!
//! Lives in the library (not a binary) so it is unit-testable and reachable
//! from `evaluate_policy` (which only needs the `inference` feature, not
//! `training`) as well as the two `training`-gated binaries.

use crate::plane::PlaneConfig;
use std::ops::RangeInclusive;

/// Parse a `MIN:MAX` or bare `VALUE` string into an inclusive `f32` range.
///
/// `VALUE` alone yields the degenerate range `VALUE..=VALUE` — used to pin a
/// single fixed target, reproducing the pre-randomization behavior of a
/// task like level-hold.
///
/// Rejects: unparseable numbers, non-finite values, and `min > max`.
pub fn parse_f32_range(s: &str) -> Result<RangeInclusive<f32>, String> {
    let (min, max) = match s.split_once(':') {
        Some((lo, hi)) => {
            let lo: f32 = lo
                .parse()
                .map_err(|_| format!("invalid range bound {lo:?} in {s:?}"))?;
            let hi: f32 = hi
                .parse()
                .map_err(|_| format!("invalid range bound {hi:?} in {s:?}"))?;
            (lo, hi)
        }
        None => {
            let v: f32 = s.parse().map_err(|_| format!("invalid value {s:?}"))?;
            (v, v)
        }
    };
    if !min.is_finite() || !max.is_finite() {
        return Err(format!("range bounds must be finite, got {s:?}"));
    }
    if min > max {
        return Err(format!("range min ({min}) must be <= max ({max}) in {s:?}"));
    }
    Ok(min..=max)
}

/// Default airframe the training/eval binaries fly when `--plane-config` is not
/// given. A filesystem path (relative to the crate root, like the reward/PPO
/// profile paths), **not** a Bevy asset-relative one.
pub const DEFAULT_PLANE_CONFIG_PATH: &str = "assets/planes/generic_jet.plane.ron";

/// Read a `.plane.ron` airframe from disk.
///
/// Thin alias over [`load_ron_config`](crate::training::reward_config::load_ron_config)
/// for call-site readability, mirroring `load_reward_config`. Fallible on purpose:
/// see [`load_plane_config_or_exit`] for why the binaries turn an error into an exit
/// rather than a fallback.
pub fn load_plane_config(path: &str) -> Result<PlaneConfig, Box<dyn std::error::Error>> {
    crate::training::reward_config::load_ron_config(path)
}

/// Load the airframe a training/eval run will fly, or exit(2).
///
/// **Operator-facing only.** The error text embeds `ron`'s parse message, which quotes
/// the offending token from the file — fine for a path typed on a local command line,
/// but see the content-disclosure invariant in CLAUDE.md §7 before reusing this on any
/// path that could arrive from a network peer.
///
/// Deliberately fatal, unlike `load_reward_config`'s warn-and-fall-back-to-defaults:
/// a reward profile is a hyperparameter, but the airframe is the *plant*. A run that
/// silently substituted a different plane would produce a checkpoint fitted to
/// something nobody asked for, and the mistake would only surface much later as
/// unexplained live-sim behaviour.
pub fn load_plane_config_or_exit(path: &str) -> PlaneConfig {
    match load_plane_config(path) {
        // Silent on success: `evaluate_policy`'s stdout is a machine-read
        // `key,value` report, so callers announce the airframe themselves in
        // whatever form their output format wants.
        Ok(cfg) => cfg,
        Err(e) => {
            eprintln!("Cannot load plane config '{path}': {e}");
            eprintln!("Pass a readable .plane.ron via --plane-config, or omit the flag");
            eprintln!("to use the default ({DEFAULT_PLANE_CONFIG_PATH}).");
            std::process::exit(2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loads_the_default_shipped_airframe() {
        let cfg = load_plane_config(DEFAULT_PLANE_CONFIG_PATH).expect("default airframe loads");
        // Spot-check a few fields against assets/planes/generic_jet.plane.ron. This
        // reads the LIVE asset on purpose (unlike the frozen test fixture): the point
        // is that the default path the binaries use actually parses.
        assert_eq!(cfg.mass, 5000.0);
        assert_eq!(cfg.thrust_max, 60000.0);
        assert_eq!(cfg.cl_alpha, 4.5);
    }

    #[test]
    fn missing_plane_config_is_an_error_not_a_fallback() {
        assert!(load_plane_config("assets/planes/no_such_airframe.plane.ron").is_err());
    }

    #[test]
    fn malformed_plane_config_is_an_error() {
        // A .plane.ron missing required fields must fail rather than defaulting them
        // to zero — the whole reason PlaneConfig has no `Default`.
        let dir = std::env::temp_dir().join("ml_planes_cli_test");
        std::fs::create_dir_all(&dir).expect("temp dir");
        let path = dir.join("truncated.plane.ron");
        std::fs::write(&path, "(wing_area: 20.0, mean_chord: 2.0)").expect("write");
        let r = load_plane_config(path.to_str().unwrap());
        std::fs::remove_file(&path).ok();
        assert!(r.is_err(), "a truncated .plane.ron must not parse");
    }

    #[test]
    fn parses_min_max_range() {
        let r = parse_f32_range("500:5000").unwrap();
        assert_eq!(*r.start(), 500.0);
        assert_eq!(*r.end(), 5000.0);
    }

    #[test]
    fn parses_bare_value_as_degenerate_range() {
        let r = parse_f32_range("1000").unwrap();
        assert_eq!(*r.start(), 1000.0);
        assert_eq!(*r.end(), 1000.0);
    }

    #[test]
    fn parses_negative_and_fractional_bounds() {
        let r = parse_f32_range("-10.5:20.25").unwrap();
        assert_eq!(*r.start(), -10.5);
        assert_eq!(*r.end(), 20.25);
    }

    #[test]
    fn rejects_min_greater_than_max() {
        assert!(parse_f32_range("5000:500").is_err());
    }

    #[test]
    fn rejects_garbage() {
        assert!(parse_f32_range("not-a-number").is_err());
        assert!(parse_f32_range("500:not-a-number").is_err());
        assert!(parse_f32_range("").is_err());
    }

    #[test]
    fn rejects_non_finite() {
        assert!(parse_f32_range("nan:100").is_err());
        assert!(parse_f32_range("inf:100").is_err());
        assert!(parse_f32_range("0:inf").is_err());
    }
}
