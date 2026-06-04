//! Shared "active orbit center" extraction for visual overlays.
//!
//! Several controllers circle a fixed world-frame point: [`OrbitController`], the
//! RL orbit variants, and [`L1Controller`] while flying an orbit leg. The visual
//! mode plants a ground "pin" at that center for the selected plane (in 3D and on
//! the map). [`active_orbit_center`] is the single downcast point both overlays
//! share, so the ladder of concrete controller types lives in one place.

use bevy::math::Vec2;

use crate::controllers::traits::FlightController;
use crate::controllers::{FlightPlanLeg, L1Controller, OrbitController};

/// Geometry of the orbit a controller is currently flying, in world-frame XZ.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OrbitMarker {
    /// Orbit center world-frame XZ `(x, z)` [m].
    pub center: Vec2,
    /// Target orbit radius [m].
    pub radius: f32,
}

/// The active orbit center of any orbit-type controller, or `None` when the
/// controller is not circling a point (e.g. LevelHold, Manual, Wingman, or an
/// L1 plan on a waypoint leg / finished).
///
/// Takes `&mut` because [`FlightController`] only exposes `as_any_mut`
/// (`controllers/traits.rs`); the function only reads.
pub fn active_orbit_center(ctrl: &mut dyn FlightController) -> Option<OrbitMarker> {
    if let Some(orbit) = ctrl.as_any_mut().downcast_mut::<OrbitController>() {
        return Some(OrbitMarker {
            center: Vec2::new(orbit.center_x, orbit.center_z),
            radius: orbit.target_radius,
        });
    }

    #[cfg(any(feature = "inference", feature = "training"))]
    {
        use crate::controllers::{
            RlLstmOrbitController, RlOrbitController, RlOrbitResidualController,
        };
        if let Some(rl) = ctrl.as_any_mut().downcast_mut::<RlOrbitController>() {
            let cfg = rl.config();
            return Some(OrbitMarker {
                center: Vec2::new(cfg.center_x, cfg.center_z),
                radius: cfg.target_radius,
            });
        }
        if let Some(rl) = ctrl
            .as_any_mut()
            .downcast_mut::<RlOrbitResidualController>()
        {
            let cfg = rl.config();
            return Some(OrbitMarker {
                center: Vec2::new(cfg.center_x, cfg.center_z),
                radius: cfg.target_radius,
            });
        }
        if let Some(rl) = ctrl.as_any_mut().downcast_mut::<RlLstmOrbitController>() {
            let cfg = rl.config();
            return Some(OrbitMarker {
                center: Vec2::new(cfg.center_x, cfg.center_z),
                radius: cfg.target_radius,
            });
        }
    }

    if let Some(l1) = ctrl.as_any_mut().downcast_mut::<L1Controller>() {
        if let Some(FlightPlanLeg::Orbit {
            center_x,
            center_z,
            radius,
            ..
        }) = l1.active_leg()
        {
            return Some(OrbitMarker {
                center: Vec2::new(*center_x, *center_z),
                radius: *radius,
            });
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controllers::{FlightPlan, LevelHoldController, OrbitDirection};
    use crate::plane::{ControlInputs, FlightState};

    #[test]
    fn orbit_controller_reports_its_center() {
        let mut orbit =
            OrbitController::from_state(&FlightState::default(), &ControlInputs::default());
        orbit.center_x = 123.0;
        orbit.center_z = -456.0;
        orbit.target_radius = 789.0;

        let marker = active_orbit_center(&mut orbit).expect("orbit controller has a center");
        assert_eq!(marker.center, Vec2::new(123.0, -456.0));
        assert_eq!(marker.radius, 789.0);
    }

    #[test]
    fn level_hold_has_no_center() {
        let mut lh = LevelHoldController::new(1000.0, 100.0);
        assert_eq!(active_orbit_center(&mut lh), None);
    }

    #[test]
    fn l1_on_orbit_leg_reports_center_before_update() {
        let plan = FlightPlan {
            legs: vec![FlightPlanLeg::Orbit {
                center_x: 50.0,
                center_z: 60.0,
                radius: 700.0,
                altitude: 1000.0,
                airspeed: 100.0,
                direction: OrbitDirection::Clockwise,
                turns: None,
            }],
            ..Default::default()
        };
        let mut l1 =
            L1Controller::from_plan(&FlightState::default(), plan, &ControlInputs::default());

        let marker = active_orbit_center(&mut l1).expect("L1 orbit leg has a center");
        assert_eq!(marker.center, Vec2::new(50.0, 60.0));
        assert_eq!(marker.radius, 700.0);
    }

    #[test]
    fn l1_on_waypoint_leg_has_no_center() {
        let plan = FlightPlan {
            legs: vec![FlightPlanLeg::Waypoint {
                x: 1000.0,
                z: 0.0,
                altitude: 1000.0,
                airspeed: 100.0,
                capture_radius: 200.0,
            }],
            ..Default::default()
        };
        let mut l1 =
            L1Controller::from_plan(&FlightState::default(), plan, &ControlInputs::default());

        assert_eq!(active_orbit_center(&mut l1), None);
    }
}
