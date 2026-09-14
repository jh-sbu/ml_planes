//! Promotion of the frozen Python force/moment-inversion level-hold cascade.
//!
//! Calibrated for the generic jet, using its nominal aerodynamic coefficients.
//! Fuel mass and density are scheduled from the same observations as RL level
//! hold. This is a separate controller: the established PID cascade and its
//! consumers keep their existing gains and behavior.
//!
//! The benchmark assumes instantaneous actuators. The winning tuning is
//! aggressive and sensitive to actuator lag; see experiments/level_hold_python.

use serde::{Deserialize, Serialize};

use crate::controllers::{ControllerTargets, FlightController};
use crate::plane::{ControlInputs, ControllerContext, FlightState};
use crate::training::level_hold_env::level_hold_observation;

/// Frozen gains selected by the Python experiment; integral arithmetic uses f64.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct InversionLevelHoldGains {
    pub hp: f64,
    pub hd: f64,
    pub hi: f64,
    /// Requested vertical acceleration clamp, not a structural load limit.
    pub azmax: f64,
    pub pp: f64,
    pub pd: f64,
    pub sp: f64,
    pub si: f64,
    pub rp: f64,
    pub rd: f64,
    pub bp: f64,
    pub bd: f64,
}

impl Default for InversionLevelHoldGains {
    fn default() -> Self {
        Self {
            hp: 1.2065224237076453,
            hd: 1.4611030199005073,
            hi: 0.17625833501373706,
            azmax: 48.524517949227636,
            pp: 42.0472393705356,
            pd: 2.09260140386139,
            sp: 1.1445518959947942,
            si: 0.02777178209146401,
            rp: 5.0,
            rd: 4.0,
            bp: 1.5,
            bd: 3.0,
        }
    }
}

/// Generic-jet level hold using model inversion and two bounded integral states.
/// Create a fresh instance at episode reset. Does not claim a bumpless handoff.
#[derive(Debug, Clone)]
pub struct InversionLevelHoldController {
    pub target_altitude: f32,
    pub target_airspeed: f32,
    pub gains: InversionLevelHoldGains,
    altitude_integral: f64,
    speed_integral: f64,
}

impl InversionLevelHoldController {
    pub fn new(target_altitude: f32, target_airspeed: f32) -> Self {
        Self {
            target_altitude,
            target_airspeed,
            gains: InversionLevelHoldGains::default(),
            altitude_integral: 0.0,
            speed_integral: 0.0,
        }
    }

    pub fn from_state(state: &FlightState) -> Self {
        Self::new(state.altitude, state.airspeed)
    }
}

impl FlightController for InversionLevelHoldController {
    fn update(&mut self, state: &FlightState, _ctx: &ControllerContext, dt: f32) -> ControlInputs {
        // Reuse the observation definition so Python/native control see identical
        // normalization, density, attitude conventions and fuel information.
        let obs = level_hold_observation(state, self.target_altitude, self.target_airspeed);
        let h = (obs[0] * 200.0) as f64;
        let ev = (obs[1] * 50.0) as f64;
        let alpha = (obs[2] * 0.5) as f64;
        let q = obs[3] as f64;
        let roll = (obs[4] * 0.5) as f64;
        let p = obs[5] as f64;
        let beta = (obs[6] * 0.5) as f64;
        let r = obs[7] as f64;
        let theta = (obs[8] * 0.5) as f64;
        let vv = (obs[9] * 30.0) as f64;
        let mass = (5000.0 + 2000.0 * obs[10]) as f64;
        let rho = obs[11] as f64;
        let v = (obs[12] * 100.0).max(30.0) as f64;
        let qs = 0.5 * 1.225 * rho * v * v * 20.0;
        let gamma = (vv / v).clamp(-0.8, 0.8).asin();
        let k = self.gains;

        self.altitude_integral = (self.altitude_integral + h * dt as f64).clamp(-30.0, 30.0);
        self.speed_integral = (self.speed_integral + ev * dt as f64).clamp(-20.0, 20.0);
        let az = (-k.hp * h - k.hd * vv - k.hi * self.altitude_integral).clamp(-k.azmax, k.azmax);
        let cl_des = mass * (9.81 + az) / qs / (gamma.cos() * roll.cos()).max(0.6);
        // Lift and zero pitching moment jointly determine trim alpha/elevator.
        let alpha_des =
            ((cl_des - 0.1 + 0.4 * 0.02 / 1.2) / (4.5 + 0.4 * 0.6 / 1.2)).clamp(-0.15, 0.26);
        let theta_des = alpha_des + gamma;
        // Body-positive pitch rate is nose DOWN.
        let qdot = -k.pp * (theta_des - theta) - k.pd * q;
        let de = ((-0.02 + 0.6 * alpha - 14.0 * q / v - 40000.0 * qdot / (qs * 2.0)) / 1.2)
            .clamp(-0.3491, 0.3491);
        let cl = (0.1 + 4.5 * alpha + 0.4 * de).clamp(-1.4, 1.4);
        let drag = qs * (0.02 + 0.05 * cl * cl);
        let acc = -k.sp * ev - k.si * self.speed_integral;
        let thrust =
            (drag + mass * (acc + 9.81 * gamma.sin())) / (alpha.cos() * beta.cos()).max(0.5);
        let throttle = (thrust / (60000.0 * rho)).clamp(0.0, 1.0);
        let pdot = -k.rp * roll - k.rd * p;
        let da =
            (10000.0 * pdot / (qs * 10.0) - 0.08 * beta + 0.45 * p * 5.0 / v + 0.12 * r * 5.0 / v)
                / 0.18;
        let rdot = k.bp * beta - k.bd * r;
        let dr = (0.10 * beta - 0.12 * r * 5.0 / v - 45000.0 * rdot / (qs * 10.0)) / 0.10;
        ControlInputs {
            elevator: (de / 0.3491).clamp(-1.0, 1.0) as f32,
            throttle: throttle as f32,
            aileron: (da / 0.4363).clamp(-1.0, 1.0) as f32,
            rudder: (dr / 0.2618).clamp(-1.0, 1.0) as f32,
        }
    }

    fn name(&self) -> &'static str {
        "InversionLevelHold"
    }

    fn targets(&self) -> ControllerTargets {
        ControllerTargets::LevelHold {
            altitude: self.target_altitude,
            airspeed: self.target_airspeed,
        }
    }

    fn apply_targets(&mut self, targets: &ControllerTargets, _state: &FlightState) {
        if let ControllerTargets::LevelHold { altitude, airspeed } = *targets {
            if altitude != self.target_altitude {
                self.altitude_integral = 0.0;
            }
            if airspeed != self.target_airspeed {
                self.speed_integral = 0.0;
            }
            self.target_altitude = altitude;
            self.target_airspeed = airspeed;
        }
    }

    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}
