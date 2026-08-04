//! Shared CLI argument parsing helpers for the training/eval binaries
//! (`train_ppo`, `train_bc`, `evaluate_policy`).
//!
//! Lives in the library (not a binary) so it is unit-testable and reachable
//! from `evaluate_policy` (which only needs the `inference` feature, not
//! `training`) as well as the two `training`-gated binaries.

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

#[cfg(test)]
mod tests {
    use super::*;

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
