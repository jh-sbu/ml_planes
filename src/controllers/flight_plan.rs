//! Preset flight-plan asset: an ordered sequence of legs (waypoint sequences +
//! orbit circles) followed by the L1 guidance controller ([`L1Controller`]).
//!
//! Loaded from `.plan.ron` files via the Bevy asset server (see
//! `plane::plugin::FlightPlanLoader`). The `Default` impls mirror sensible
//! values so unit tests can build plans in code without file I/O.
//!
//! [`L1Controller`]: crate::controllers::l1::L1Controller

use bevy::reflect::Reflect;
use serde::{Deserialize, Serialize};

use crate::controllers::orbit::OrbitDirection;

/// A single leg of a [`FlightPlan`].
///
/// All horizontal coordinates are world-frame XZ in metres (+Y is up); altitude
/// is the absolute target altitude in metres and airspeed is in m/s.
#[derive(Reflect, Serialize, Deserialize, Debug, Clone, PartialEq)]
pub enum FlightPlanLeg {
    /// Fly a straight L1 path toward a world-frame XZ point, advancing to the
    /// next leg once the horizontal distance falls below `capture_radius`
    /// (fly-by capture).
    Waypoint {
        x: f32,
        z: f32,
        altitude: f32,
        airspeed: f32,
        capture_radius: f32,
    },
    /// Loiter on a circle of `radius` around `(center_x, center_z)`. Advances to
    /// the next leg after `turns` full revolutions; `turns: None` loiters
    /// indefinitely (only meaningful as the final leg).
    Orbit {
        center_x: f32,
        center_z: f32,
        radius: f32,
        altitude: f32,
        airspeed: f32,
        direction: OrbitDirection,
        turns: Option<f32>,
    },
}

impl FlightPlanLeg {
    /// Target altitude commanded while flying this leg [m].
    pub fn altitude(&self) -> f32 {
        match self {
            FlightPlanLeg::Waypoint { altitude, .. } | FlightPlanLeg::Orbit { altitude, .. } => {
                *altitude
            }
        }
    }

    /// Target airspeed commanded while flying this leg [m/s].
    pub fn airspeed(&self) -> f32 {
        match self {
            FlightPlanLeg::Waypoint { airspeed, .. } | FlightPlanLeg::Orbit { airspeed, .. } => {
                *airspeed
            }
        }
    }
}

/// An ordered list of legs plus the L1 lateral-guidance tuning parameters.
///
/// The L1 lookahead distance is computed per-tick as
/// `L1 = (1/π) · l1_damping · l1_period · groundspeed`.
#[derive(bevy::asset::Asset, Reflect, Serialize, Deserialize, Debug, Clone)]
pub struct FlightPlan {
    /// Legs flown in order. Empty plans are treated as "hold current state".
    pub legs: Vec<FlightPlanLeg>,
    /// L1 response period [s] — larger is gentler / slower-reacting.
    pub l1_period: f32,
    /// L1 damping ratio (typically ~0.7–0.8).
    pub l1_damping: f32,
}

impl Default for FlightPlan {
    fn default() -> Self {
        Self {
            legs: Vec::new(),
            l1_period: 20.0,
            l1_damping: 0.75,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"(
        l1_period: 18.0,
        l1_damping: 0.7,
        legs: [
            Waypoint( x: 3000.0, z: 0.0, altitude: 1000.0, airspeed: 100.0, capture_radius: 200.0 ),
            Orbit( center_x: 0.0, center_z: 3000.0, radius: 1000.0, altitude: 1200.0,
                   airspeed: 100.0, direction: CounterClockwise, turns: Some(2.0) ),
            Orbit( center_x: 0.0, center_z: 0.0, radius: 800.0, altitude: 1000.0,
                   airspeed: 90.0, direction: Clockwise, turns: None ),
        ],
    )"#;

    #[test]
    fn parses_sample_plan() {
        let plan: FlightPlan = ron::de::from_str(SAMPLE).expect("sample plan should parse");

        assert_eq!(plan.legs.len(), 3);
        assert!((plan.l1_period - 18.0).abs() < 1e-6);
        assert!((plan.l1_damping - 0.7).abs() < 1e-6);

        match &plan.legs[0] {
            FlightPlanLeg::Waypoint {
                x,
                z,
                capture_radius,
                ..
            } => {
                assert_eq!(*x, 3000.0);
                assert_eq!(*z, 0.0);
                assert_eq!(*capture_radius, 200.0);
            }
            other => panic!("leg 0 should be a Waypoint, got {other:?}"),
        }

        match &plan.legs[1] {
            FlightPlanLeg::Orbit {
                direction, turns, ..
            } => {
                assert_eq!(*direction, OrbitDirection::CounterClockwise);
                assert_eq!(*turns, Some(2.0));
            }
            other => panic!("leg 1 should be a finite Orbit, got {other:?}"),
        }

        match &plan.legs[2] {
            FlightPlanLeg::Orbit { turns, .. } => {
                assert_eq!(*turns, None, "final orbit leg should hold forever");
            }
            other => panic!("leg 2 should be a hold-forever Orbit, got {other:?}"),
        }
    }
}
