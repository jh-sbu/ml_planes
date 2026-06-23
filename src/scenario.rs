//! Multi-plane scenario description, loaded from `.scenario.ron` files.
//!
//! A scenario lists one or more planes, each with an initial state, a plane
//! config, and a controller. Controllers that depend on other planes (e.g.
//! `Wingman`, which follows a leader) reference their peers by `name`; names
//! are resolved to stable [`PlaneId`]s during [`Scenario::resolve`].
//!
//! Loading is pure `ron::de` (no Bevy asset server), mirroring the reward-config
//! pattern in `training/reward_config.rs`. The `examples/observe_state.rs` binary
//! is the primary consumer: it resolves a scenario, builds one controller per
//! plane via [`ResolvedScenario::build_controller`], spawns the entities, and
//! streams per-plane CSV.
//!
//! RL controller specs (`RlLevelHold`, `RlOrbit`, …) always parse so a scenario
//! (e.g. `default.scenario.ron`) loads in every build, but they only *build* on a
//! native `--features inference` build; otherwise [`ResolvedScenario::build_controller`]
//! returns `Err` and the live spawner skips that plane.

use std::collections::HashSet;
use std::f32::consts::FRAC_PI_2;
use std::path::Path;

use bevy::math::{Quat, Vec3};
use serde::Deserialize;

use crate::controllers::tuning::{HeadingHoldTuning, LevelHoldTuning, OrbitTuning};
use crate::controllers::{
    AscentController, ControllerKind, FlightController, FlightPlan, FormationOffset,
    HeadingHoldController, L1Controller, LevelHoldController, ManualController, OrbitController,
    OrbitDirection, OrbitParams, WingmanController,
};
use crate::plane::{ControlInputs, FlightState, PlaneId};

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
use crate::controllers::{
    RlLevelHoldController, RlLstmOrbitConfig, RlLstmOrbitController, RlOrbitConfig,
    RlOrbitController, RlOrbitResidualConfig, RlOrbitResidualController,
};

const DEFAULT_STEPS: usize = 640;
const DEFAULT_INTERVAL: usize = 10;
const DEFAULT_SPEED: f32 = 100.0;
const DEFAULT_POSITION: Vec3 = Vec3::new(0.0, 500.0, 0.0);
/// Default formation slot: 20 m behind, 15 m right, same altitude (matches
/// `FormationOffset::default`).
const DEFAULT_OFFSET: Vec3 = Vec3::new(-20.0, 15.0, 0.0);

// ---------------------------------------------------------------------------
// Serde model

/// Top-level scenario file (`*.scenario.ron`).
#[derive(Debug, Clone, Deserialize)]
pub struct Scenario {
    /// Total simulation steps at 64 Hz. `None` → [`DEFAULT_STEPS`].
    #[serde(default)]
    pub steps: Option<usize>,
    /// Print every Nth step. `None` → [`DEFAULT_INTERVAL`].
    #[serde(default)]
    pub interval: Option<usize>,
    /// Planes spawned for this scenario, in order. The first plane is assigned
    /// `PlaneId(1)`, the second `PlaneId(2)`, and so on.
    pub planes: Vec<PlaneSpec>,
}

/// One plane in a scenario.
#[derive(Debug, Clone, Deserialize)]
pub struct PlaneSpec {
    /// Unique name; used by peers (e.g. a wingman's `leader`) to reference it.
    pub name: String,
    /// Path to a `.plane.ron` config. `None` → the embedded `generic_jet`.
    #[serde(default)]
    pub config: Option<String>,
    /// World-frame spawn position [m]. `None` → [`DEFAULT_POSITION`].
    #[serde(default)]
    pub position: Option<(f32, f32, f32)>,
    /// World-frame velocity [m/s]. `None` → `DEFAULT_SPEED` along the heading.
    #[serde(default)]
    pub velocity: Option<(f32, f32, f32)>,
    /// Initial heading [deg] in the `atan2(z, x)` convention (0 = +X, 90 = +Z),
    /// matching the reported `yaw_deg` and the heading-hold controller. Yawed onto
    /// the level base attitude; also rotates the default velocity when `velocity`
    /// is omitted. `None` → 0 (+X).
    #[serde(default)]
    pub heading_deg: Option<f32>,
    /// Body-frame angular velocity [rad/s]. `None` → zero.
    #[serde(default)]
    pub angular_velocity: Option<(f32, f32, f32)>,
    /// Starting fuel/charge as a fraction of the powerplant's capacity, in
    /// `[0, 1]`. `None` → full tank (1.0), matching the live spawner. The
    /// fraction is multiplied by `Powerplant::capacity()` where the config is
    /// loaded (observe_state) to set the initial `consumable_remaining` and the
    /// loaded mass.
    #[serde(default)]
    pub fuel_fraction: Option<f32>,
    /// The controller flown by this plane.
    pub controller: ControllerSpec,
}

/// Controller selector for a scenario plane. Variant fields mirror the
/// corresponding controller constructors. RL variants require `--features
/// inference`.
#[derive(Debug, Clone, Deserialize)]
pub enum ControllerSpec {
    LevelHold {
        altitude: f32,
        airspeed: f32,
        #[serde(default)]
        tuning: Option<LevelHoldTuning>,
    },
    Orbit {
        #[serde(default)]
        center_x: Option<f32>,
        #[serde(default)]
        center_z: Option<f32>,
        radius: f32,
        direction: OrbitDirection,
        altitude: f32,
        airspeed: f32,
        #[serde(default)]
        tuning: Option<OrbitTuning>,
    },
    HeadingHold {
        heading_deg: f32,
        altitude: f32,
        airspeed: f32,
        #[serde(default)]
        tuning: Option<HeadingHoldTuning>,
    },
    Ascent {
        target_altitude: f32,
    },
    Wingman {
        leader: String,
        #[serde(default)]
        offset: Option<(f32, f32, f32)>,
    },
    FlightPlan {
        plan: String,
    },
    Manual,
    // The RL variants are intentionally NOT feature-gated: the default
    // `visual` build has no `inference`, but its `default.scenario.ron` still
    // lists the RL demo planes. Keeping the variants always-parseable lets the
    // file load everywhere; `build_controller` returns `Err` (→ the live spawner
    // skips the plane) when RL can't actually be loaded. Their fields are all
    // plain types, so no inference-only dependency leaks in.
    RlLevelHold {
        model: String,
        altitude: f32,
        airspeed: f32,
    },
    RlOrbit {
        model: String,
        #[serde(default)]
        center_x: Option<f32>,
        #[serde(default)]
        center_z: Option<f32>,
        radius: f32,
        direction: OrbitDirection,
        altitude: f32,
        airspeed: f32,
    },
    RlOrbitResidual {
        model: String,
        #[serde(default)]
        center_x: Option<f32>,
        #[serde(default)]
        center_z: Option<f32>,
        radius: f32,
        direction: OrbitDirection,
        altitude: f32,
        airspeed: f32,
        #[serde(default = "default_residual_scale")]
        residual_scale: f32,
    },
    RlLstmOrbit {
        model: String,
        #[serde(default)]
        center_x: Option<f32>,
        #[serde(default)]
        center_z: Option<f32>,
        radius: f32,
        direction: OrbitDirection,
        altitude: f32,
        airspeed: f32,
    },
}

fn default_residual_scale() -> f32 {
    0.3
}

impl ControllerSpec {
    /// The [`ControllerKind`] this spec spawns as, so a scenario-spawned plane
    /// displays and cycles correctly in the HUD/map. Mirrors the variant mapping
    /// in [`ResolvedScenario::build_controller`].
    pub fn kind(&self) -> ControllerKind {
        match self {
            ControllerSpec::LevelHold { .. } => ControllerKind::LevelHold,
            ControllerSpec::Orbit { .. } => ControllerKind::Orbit,
            ControllerSpec::HeadingHold { .. } => ControllerKind::HeadingHold,
            ControllerSpec::Ascent { .. } => ControllerKind::Ascent,
            ControllerSpec::Wingman { .. } => ControllerKind::Wingman,
            ControllerSpec::FlightPlan { .. } => ControllerKind::FlightPlan,
            ControllerSpec::Manual => ControllerKind::Manual,
            ControllerSpec::RlLevelHold { .. } => ControllerKind::RlLevelHold,
            ControllerSpec::RlOrbit { .. } => ControllerKind::RlOrbit,
            ControllerSpec::RlOrbitResidual { .. } => ControllerKind::RlOrbitResidual,
            ControllerSpec::RlLstmOrbit { .. } => ControllerKind::RlLstmOrbit,
        }
    }

    /// The body-frame formation slot for a `Wingman` spec (resolved with the
    /// default offset when omitted); `None` for every other controller. Used by
    /// the live spawner to attach the `FormationOffset` component.
    pub fn formation_offset(&self) -> Option<Vec3> {
        match self {
            ControllerSpec::Wingman { offset, .. } => Some(
                offset
                    .map(|(x, y, z)| Vec3::new(x, y, z))
                    .unwrap_or(DEFAULT_OFFSET),
            ),
            _ => None,
        }
    }

    /// The RL model stem (`.mpk` stripped) for an RL spec; `None` otherwise. Used
    /// by the live spawner to attach `SelectedModel` for HUD model cycling.
    #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
    pub fn rl_model_stem(&self) -> Option<String> {
        match self {
            ControllerSpec::RlLevelHold { model, .. }
            | ControllerSpec::RlOrbit { model, .. }
            | ControllerSpec::RlOrbitResidual { model, .. }
            | ControllerSpec::RlLstmOrbit { model, .. } => Some(strip_mpk(model)),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Resolved model

/// Orbit geometry resolved for output diagnostics (radial/heading error columns).
#[derive(Debug, Clone, Copy)]
pub struct OrbitDiag {
    pub center_x: f32,
    pub center_z: f32,
    pub radius: f32,
    pub direction: OrbitDirection,
}

/// A plane with its `PlaneId` assigned and initial state computed.
#[derive(Debug, Clone)]
pub struct ResolvedPlane {
    pub id: PlaneId,
    pub name: String,
    pub config: Option<String>,
    pub position: Vec3,
    /// World-frame velocity [m/s].
    pub velocity: Vec3,
    pub attitude: Quat,
    /// Body-frame angular velocity [rad/s].
    pub angular_velocity: Vec3,
    /// Initial flight state (air data already computed).
    pub state: FlightState,
    /// Starting fuel/charge as a fraction of powerplant capacity `[0, 1]`, carried
    /// from the scenario. `None` → full tank. Applied (× `Powerplant::capacity()`)
    /// where the `.plane.ron` config is loaded, since `resolve()` has only the path.
    pub fuel_fraction: Option<f32>,
    /// Resolved orbit geometry for orbit-like controllers; `None` otherwise.
    pub orbit_diag: Option<OrbitDiag>,
    pub spec: ControllerSpec,
}

/// A fully resolved scenario: plane ids assigned, initial states computed,
/// names resolvable. Controllers are built lazily via [`build_controller`].
#[derive(Debug, Clone)]
pub struct ResolvedScenario {
    pub steps: usize,
    pub interval: usize,
    pub planes: Vec<ResolvedPlane>,
}

// ---------------------------------------------------------------------------
// Loading

impl Scenario {
    /// Parse a scenario from a RON string.
    ///
    /// The `implicit_some` extension is enabled so optional fields can be
    /// written as bare values (`steps: 1920`, `position: (0, 500, 0)`) rather
    /// than `Some(...)`.
    pub fn from_ron_str(src: &str) -> Result<Self, String> {
        let opts = ron::Options::default()
            .with_default_extension(ron::extensions::Extensions::IMPLICIT_SOME);
        opts.from_str(src)
            .map_err(|e| format!("failed to parse scenario RON: {e}"))
    }

    /// Read and parse a scenario file.
    pub fn from_path(path: &Path) -> Result<Self, String> {
        let bytes = std::fs::read_to_string(path)
            .map_err(|e| format!("cannot read scenario '{}': {e}", path.display()))?;
        Self::from_ron_str(&bytes)
    }

    /// Assign `PlaneId`s, compute each plane's initial [`FlightState`], and
    /// validate inter-plane references.
    pub fn resolve(&self) -> Result<ResolvedScenario, String> {
        if self.planes.is_empty() {
            return Err("scenario has no planes".to_string());
        }

        let mut seen = HashSet::new();
        for p in &self.planes {
            if !seen.insert(p.name.as_str()) {
                return Err(format!("duplicate plane name '{}'", p.name));
            }
        }

        let mut planes = Vec::with_capacity(self.planes.len());
        for (idx, spec) in self.planes.iter().enumerate() {
            let id = PlaneId(idx as u32 + 1);
            let attitude = initial_attitude(spec.heading_deg);
            let velocity = initial_velocity(spec.velocity, spec.heading_deg);
            let position = spec
                .position
                .map(|(x, y, z)| Vec3::new(x, y, z))
                .unwrap_or(DEFAULT_POSITION);
            let angular_velocity = spec
                .angular_velocity
                .map(|(x, y, z)| Vec3::new(x, y, z))
                .unwrap_or(Vec3::ZERO);

            let mut state = FlightState {
                position,
                velocity,
                attitude,
                angular_velocity,
                ..FlightState::default()
            };
            state.update_air_data();

            let orbit_diag = orbit_diag_for(&spec.controller, position, velocity);

            planes.push(ResolvedPlane {
                id,
                name: spec.name.clone(),
                config: spec.config.clone(),
                position,
                velocity,
                attitude,
                angular_velocity,
                state,
                fuel_fraction: spec.fuel_fraction,
                orbit_diag,
                spec: spec.controller.clone(),
            });
        }

        // Validate wingman leader references resolve.
        for p in &planes {
            if let ControllerSpec::Wingman { leader, .. } = &p.spec {
                if !planes.iter().any(|q| &q.name == leader) {
                    return Err(format!(
                        "wingman '{}' references unknown leader '{}'",
                        p.name, leader
                    ));
                }
            }
        }

        Ok(ResolvedScenario {
            steps: self.steps.unwrap_or(DEFAULT_STEPS),
            interval: self.interval.unwrap_or(DEFAULT_INTERVAL),
            planes,
        })
    }
}

impl ResolvedScenario {
    /// Look up a plane by name, returning its id and a clone of its initial state.
    pub fn lookup(&self, name: &str) -> Option<(PlaneId, FlightState)> {
        self.planes
            .iter()
            .find(|p| p.name == name)
            .map(|p| (p.id, p.state.clone()))
    }

    /// Build the boxed controller for `planes[idx]`, resolving any peer
    /// references (e.g. a wingman's leader) and loading any referenced assets
    /// (flight plans, RL model files).
    pub fn build_controller(&self, idx: usize) -> Result<Box<dyn FlightController>, String> {
        let plane = &self.planes[idx];
        let state = &plane.state;
        let prev = ControlInputs::default();

        match &plane.spec {
            ControllerSpec::LevelHold {
                altitude,
                airspeed,
                tuning,
            } => {
                let t = tuning.clone().unwrap_or_default();
                let mut c = LevelHoldController::with_tuning(state, &t, &prev);
                c.target_altitude = *altitude;
                c.target_airspeed = *airspeed;
                Ok(Box::new(c))
            }
            ControllerSpec::Orbit {
                altitude,
                airspeed,
                tuning,
                ..
            } => {
                let diag = plane
                    .orbit_diag
                    .expect("orbit controller must have resolved orbit_diag");
                let t = tuning.clone().unwrap_or_default();
                let mut c = OrbitController::with_tuning(state, &t, &prev);
                c.apply_params(
                    &OrbitParams {
                        center_x: diag.center_x,
                        center_z: diag.center_z,
                        target_radius: diag.radius,
                        target_altitude: *altitude,
                        target_airspeed: *airspeed,
                        direction: diag.direction,
                    },
                    *airspeed,
                );
                Ok(Box::new(c))
            }
            ControllerSpec::HeadingHold {
                heading_deg,
                altitude,
                airspeed,
                tuning,
            } => {
                let t = tuning.clone().unwrap_or_default();
                let mut c = HeadingHoldController::with_tuning(state, &t, &prev);
                c.target_heading = heading_deg.to_radians();
                c.inner.target_altitude = *altitude;
                c.inner.target_airspeed = *airspeed;
                Ok(Box::new(c))
            }
            ControllerSpec::Ascent { target_altitude } => {
                Ok(Box::new(AscentController::new(state, *target_altitude)))
            }
            ControllerSpec::Wingman { leader, offset } => {
                let (leader_id, leader_state) = self
                    .lookup(leader)
                    .ok_or_else(|| format!("unknown leader '{leader}'"))?;
                let offset_body = offset
                    .map(|(x, y, z)| Vec3::new(x, y, z))
                    .unwrap_or(DEFAULT_OFFSET);
                Ok(Box::new(WingmanController::new(
                    leader_id,
                    &leader_state,
                    state,
                    FormationOffset { offset_body },
                )))
            }
            ControllerSpec::FlightPlan { plan } => {
                let plan = load_flight_plan(plan)?;
                Ok(Box::new(L1Controller::from_plan(state, plan, &prev)))
            }
            ControllerSpec::Manual => Ok(Box::new(ManualController::new())),
            #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
            ControllerSpec::RlLevelHold {
                model,
                altitude,
                airspeed,
            } => RlLevelHoldController::load(&strip_mpk(model), *altitude, *airspeed)
                .map(|c| Box::new(c) as Box<dyn FlightController>)
                .map_err(|e| format!("failed to load RL model '{model}': {e}")),
            #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
            ControllerSpec::RlOrbit {
                model,
                altitude,
                airspeed,
                ..
            } => {
                let diag = plane.orbit_diag.expect("rl_orbit must have orbit_diag");
                let config = RlOrbitConfig {
                    center_x: diag.center_x,
                    center_z: diag.center_z,
                    target_radius: diag.radius,
                    target_altitude: *altitude,
                    target_airspeed: *airspeed,
                    direction: diag.direction,
                };
                RlOrbitController::load(&strip_mpk(model), config)
                    .map(|c| Box::new(c) as Box<dyn FlightController>)
                    .map_err(|e| format!("failed to load RL model '{model}': {e}"))
            }
            #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
            ControllerSpec::RlOrbitResidual {
                model,
                altitude,
                airspeed,
                residual_scale,
                ..
            } => {
                let diag = plane
                    .orbit_diag
                    .expect("rl_orbit_residual must have orbit_diag");
                let config = RlOrbitResidualConfig {
                    center_x: diag.center_x,
                    center_z: diag.center_z,
                    target_radius: diag.radius,
                    target_altitude: *altitude,
                    target_airspeed: *airspeed,
                    direction: diag.direction,
                    residual_scale: *residual_scale,
                };
                RlOrbitResidualController::load(&strip_mpk(model), config, state, None)
                    .map(|c| Box::new(c) as Box<dyn FlightController>)
                    .map_err(|e| format!("failed to load RL model '{model}': {e}"))
            }
            #[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
            ControllerSpec::RlLstmOrbit {
                model,
                altitude,
                airspeed,
                ..
            } => {
                let diag = plane
                    .orbit_diag
                    .expect("rl_lstm_orbit must have orbit_diag");
                let config = RlLstmOrbitConfig {
                    center_x: diag.center_x,
                    center_z: diag.center_z,
                    target_radius: diag.radius,
                    target_altitude: *altitude,
                    target_airspeed: *airspeed,
                    direction: diag.direction,
                };
                RlLstmOrbitController::load(&strip_mpk(model), config)
                    .map(|c| Box::new(c) as Box<dyn FlightController>)
                    .map_err(|e| format!("failed to load RL model '{model}': {e}"))
            }
            // RL specs always parse (see the enum), but they can only be *built*
            // on a native build with `--features inference`: the non-inference
            // build has no `burn` backend, and the wasm build's `Rl*Controller`
            // loaders are `load_bytes`-only (no filesystem). In every other config
            // we return `Err` so the live spawner skips the plane (and
            // `observe_state` reports a clear error) rather than failing to parse.
            #[cfg(not(all(feature = "inference", not(target_arch = "wasm32"))))]
            ControllerSpec::RlLevelHold { .. }
            | ControllerSpec::RlOrbit { .. }
            | ControllerSpec::RlOrbitResidual { .. }
            | ControllerSpec::RlLstmOrbit { .. } => Err(
                "RL controllers from .scenario.ron require a native build with --features inference"
                    .into(),
            ),
        }
    }
}

// ---------------------------------------------------------------------------
// CSV output

/// Header for the per-plane CSV emitted by `observe_state` in scenario mode.
/// One row per plane per sampled step. The orbit-diagnostic columns
/// (`radial_error_m`..`bank_ff_rad`) are populated for orbit-like controllers and
/// left blank otherwise. The trailing `fuel_remaining` column is the powerplant
/// consumable left (kg for jets, kWh for electric) — appended last so existing
/// positional column indices stay stable for skills that parse by position.
pub const CSV_HEADER: &str = "step,time_s,plane,pos_x,altitude_m,pos_z,airspeed_ms,alpha_deg,beta_deg,roll_deg,pitch_deg,yaw_deg,pitch_rate,roll_rate,yaw_rate,elevator,throttle,aileron,rudder,radial_error_m,heading_error_rad,bank_ff_rad,fuel_remaining";

/// Format one CSV row for a plane at a given step. `orbit_diag` drives the three
/// trailing orbit columns; pass the plane's [`ResolvedPlane::orbit_diag`].
pub fn csv_row(
    step: usize,
    name: &str,
    state: &FlightState,
    inputs: &ControlInputs,
    orbit_diag: Option<OrbitDiag>,
) -> String {
    let av = state.angular_velocity; // body frame (p, q, r) = (x, y, z)
    let orbit_cols = match orbit_diag {
        Some(d) => {
            let terms = crate::controllers::orbit_observation_terms(
                state,
                d.center_x,
                d.center_z,
                d.radius,
                d.direction,
            );
            format!(
                "{:.3},{:.4},{:.4}",
                terms.radial_error, terms.heading_error, terms.bank_ff
            )
        }
        None => ",,".to_string(),
    };
    format!(
        "{},{:.3},{},{:.2},{:.2},{:.2},{:.2},{:.3},{:.3},{:.3},{:.3},{:.3},{:.4},{:.4},{:.4},{:.3},{:.3},{:.3},{:.3},{},{:.1}",
        step,
        step as f32 / 64.0,
        name,
        state.position.x,
        state.altitude,
        state.position.z,
        state.airspeed,
        state.alpha.to_degrees(),
        state.beta.to_degrees(),
        roll_angle(state.attitude).to_degrees(),
        pitch_angle(state.attitude).to_degrees(),
        yaw_angle(state.attitude).to_degrees(),
        av.y, // q = pitch rate
        av.x, // p = roll rate
        av.z, // r = yaw rate
        inputs.elevator,
        inputs.throttle,
        inputs.aileron,
        inputs.rudder,
        orbit_cols,
        state.consumable_remaining, // kg (jet) / kWh (electric); inf if unmodelled
    )
}

/// Bank angle [rad] from attitude (matches `orbit.rs`).
fn roll_angle(attitude: Quat) -> f32 {
    let right_world = attitude * Vec3::Y;
    let up_world = attitude * Vec3::Z;
    right_world.y.atan2(up_world.y)
}

/// Pitch angle [rad] from attitude (matches `orbit.rs`).
fn pitch_angle(attitude: Quat) -> f32 {
    let fwd = attitude * Vec3::X;
    fwd.y.atan2((fwd.x * fwd.x + fwd.z * fwd.z).sqrt())
}

/// Heading [rad] in the world XZ plane from the body-forward axis.
fn yaw_angle(attitude: Quat) -> f32 {
    let fwd = attitude * Vec3::X;
    fwd.z.atan2(fwd.x)
}

// ---------------------------------------------------------------------------
// Helpers

/// Level base attitude (body +Z up → world +Y up), yawed by `heading_deg`.
///
/// The yaw is negated so `heading_deg` matches the `atan2(z, x)` heading
/// convention used by [`yaw_angle`] and `HeadingHoldController::heading_from_state`:
/// `heading_deg = 90` points the nose toward +Z (not -Z).
fn initial_attitude(heading_deg: Option<f32>) -> Quat {
    let base = Quat::from_rotation_x(-FRAC_PI_2);
    match heading_deg {
        Some(d) => Quat::from_rotation_y(-d.to_radians()) * base,
        None => base,
    }
}

/// Explicit world-frame velocity, or `DEFAULT_SPEED` along the heading.
///
/// Uses the same negated-yaw convention as [`initial_attitude`] so the spawn
/// velocity direction matches the reported `heading_deg`.
fn initial_velocity(velocity: Option<(f32, f32, f32)>, heading_deg: Option<f32>) -> Vec3 {
    match velocity {
        Some((x, y, z)) => Vec3::new(x, y, z),
        None => {
            let yaw = Quat::from_rotation_y(-heading_deg.unwrap_or(0.0).to_radians());
            yaw * Vec3::new(DEFAULT_SPEED, 0.0, 0.0)
        }
    }
}

/// Resolve orbit geometry for orbit-like controllers (used both to build the
/// controller and to emit diagnostic CSV columns). `None` for non-orbit specs.
fn orbit_diag_for(spec: &ControllerSpec, position: Vec3, velocity: Vec3) -> Option<OrbitDiag> {
    let (cx, cz, radius, direction) = match spec {
        ControllerSpec::Orbit {
            center_x,
            center_z,
            radius,
            direction,
            ..
        } => (*center_x, *center_z, *radius, *direction),
        ControllerSpec::RlOrbit {
            center_x,
            center_z,
            radius,
            direction,
            ..
        }
        | ControllerSpec::RlOrbitResidual {
            center_x,
            center_z,
            radius,
            direction,
            ..
        }
        | ControllerSpec::RlLstmOrbit {
            center_x,
            center_z,
            radius,
            direction,
            ..
        } => (*center_x, *center_z, *radius, *direction),
        _ => return None,
    };
    let (center_x, center_z) = resolve_orbit_center(cx, cz, radius, direction, position, velocity);
    Some(OrbitDiag {
        center_x,
        center_z,
        radius,
        direction,
    })
}

/// Resolve the orbit center: explicit coordinates where given, otherwise placed
/// perpendicular to the plane's velocity at `radius` (matching
/// `OrbitController::from_state` so auto-centering is bumpless).
fn resolve_orbit_center(
    user_x: Option<f32>,
    user_z: Option<f32>,
    radius: f32,
    direction: OrbitDirection,
    position: Vec3,
    velocity: Vec3,
) -> (f32, f32) {
    let speed_xz = (velocity.x.powi(2) + velocity.z.powi(2)).sqrt();
    let (hx, hz) = if speed_xz > 1.0 {
        (velocity.x / speed_xz, velocity.z / speed_xz)
    } else {
        (1.0, 0.0)
    };
    let (perp_x, perp_z) = match direction {
        OrbitDirection::CounterClockwise => (hz, -hx),
        OrbitDirection::Clockwise => (-hz, hx),
    };
    let auto_x = position.x + perp_x * radius;
    let auto_z = position.z + perp_z * radius;
    (user_x.unwrap_or(auto_x), user_z.unwrap_or(auto_z))
}

fn load_flight_plan(path: &str) -> Result<FlightPlan, String> {
    let bytes = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot read flight plan '{path}': {e}"))?;
    ron::de::from_str(&bytes).map_err(|e| format!("cannot parse flight plan '{path}': {e}"))
}

#[cfg(all(feature = "inference", not(target_arch = "wasm32")))]
fn strip_mpk(path: &str) -> String {
    path.strip_suffix(".mpk").unwrap_or(path).to_string()
}

// ---------------------------------------------------------------------------
// Tests

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plane::ControllerContext;

    const WINGMAN_SCENARIO: &str = r#"(
        steps: 1920,
        interval: 64,
        planes: [
            (
                name: "leader",
                position: (0.0, 1000.0, 0.0),
                controller: LevelHold(
                    altitude: 1000.0,
                    airspeed: 100.0,
                    tuning: (
                        alt_kp: 0.01, alt_ki: 0.12, alt_kd: 0.04,
                        pitch_kp: 1.0, pitch_kd: 0.5,
                        spd_kp: 0.01, spd_ki: 0.06,
                        throttle_ff_gain: 0.7,
                    ),
                ),
            ),
            (
                name: "wingman",
                position: (-20.0, 1000.0, 15.0),
                controller: Wingman(leader: "leader", offset: (-20.0, 15.0, 0.0)),
            ),
        ],
    )"#;

    #[test]
    fn parses_wingman_scenario() {
        let s = Scenario::from_ron_str(WINGMAN_SCENARIO).expect("parse");
        assert_eq!(s.steps, Some(1920));
        assert_eq!(s.interval, Some(64));
        assert_eq!(s.planes.len(), 2);
        assert_eq!(s.planes[0].name, "leader");
        assert!(matches!(
            s.planes[0].controller,
            ControllerSpec::LevelHold { .. }
        ));
        assert!(matches!(
            s.planes[1].controller,
            ControllerSpec::Wingman { .. }
        ));
    }

    #[test]
    fn resolve_assigns_sequential_ids_from_one() {
        let s = Scenario::from_ron_str(WINGMAN_SCENARIO).unwrap();
        let r = s.resolve().unwrap();
        assert_eq!(r.steps, 1920);
        assert_eq!(r.interval, 64);
        assert_eq!(r.planes[0].id, PlaneId(1));
        assert_eq!(r.planes[1].id, PlaneId(2));
    }

    #[test]
    fn resolve_computes_air_data() {
        let s = Scenario::from_ron_str(WINGMAN_SCENARIO).unwrap();
        let r = s.resolve().unwrap();
        let leader = &r.planes[0];
        assert!((leader.state.altitude - 1000.0).abs() < 1e-3);
        // Default velocity is DEFAULT_SPEED along +X.
        assert!((leader.state.airspeed - DEFAULT_SPEED).abs() < 1e-3);
        assert!(leader.state.alpha.abs() < 1e-3);
    }

    #[test]
    fn heading_deg_yaws_initial_state() {
        let src = r#"(planes: [(
            name: "p", heading_deg: 90.0,
            controller: Manual,
        )])"#;
        let r = Scenario::from_ron_str(src).unwrap().resolve().unwrap();
        let p = &r.planes[0];
        let v = p.state.velocity;
        // heading_deg must match the atan2(z, x) convention used by yaw_angle and
        // HeadingHoldController::heading_from_state: heading 90° → flying +Z.
        assert!(v.x.abs() < 1.0, "vx={}", v.x);
        assert!(v.z > 0.0, "vz={} (heading 90° should fly +Z)", v.z);
        assert!((v.length() - DEFAULT_SPEED).abs() < 1e-3);
        // Reported yaw_deg must equal the requested heading, not its negation.
        assert!(
            (yaw_angle(p.attitude).to_degrees() - 90.0).abs() < 1e-3,
            "yaw_deg={}",
            yaw_angle(p.attitude).to_degrees()
        );
    }

    #[test]
    fn empty_planes_is_error() {
        let r = Scenario::from_ron_str("(planes: [])").unwrap().resolve();
        assert!(r.is_err());
    }

    #[test]
    fn duplicate_names_is_error() {
        let src = r#"(planes: [
            (name: "a", controller: Manual),
            (name: "a", controller: Manual),
        ])"#;
        let r = Scenario::from_ron_str(src).unwrap().resolve();
        assert!(r.is_err());
    }

    #[test]
    fn wingman_missing_leader_is_error() {
        let src = r#"(planes: [
            (name: "w", controller: Wingman(leader: "nope")),
        ])"#;
        let r = Scenario::from_ron_str(src).unwrap().resolve();
        assert!(r.is_err());
    }

    #[test]
    fn wingman_resolves_leader_id() {
        let s = Scenario::from_ron_str(WINGMAN_SCENARIO).unwrap();
        let r = s.resolve().unwrap();
        let ctrl = r.build_controller(1).expect("build wingman");
        assert_eq!(ctrl.name(), "Wingman");
    }

    #[test]
    fn builds_each_non_rl_controller_with_finite_first_tick() {
        let src = r#"(planes: [
            (name: "lh", controller: LevelHold(altitude: 500.0, airspeed: 100.0)),
            (name: "orb", controller: Orbit(radius: 1000.0, direction: CounterClockwise, altitude: 500.0, airspeed: 100.0)),
            (name: "hh", controller: HeadingHold(heading_deg: 30.0, altitude: 500.0, airspeed: 100.0)),
            (name: "asc", controller: Ascent(target_altitude: 2000.0)),
            (name: "man", controller: Manual),
        ])"#;
        let r = Scenario::from_ron_str(src).unwrap().resolve().unwrap();
        let expected = ["LevelHold", "Orbit", "HeadingHold", "Ascent", "Manual"];
        for (idx, name) in expected.iter().enumerate() {
            let mut ctrl = r.build_controller(idx).expect("build");
            assert_eq!(&ctrl.name(), name);
            let ctx = ControllerContext::empty_for(r.planes[idx].id);
            let out = ctrl.update(&r.planes[idx].state, &ctx, 1.0 / 64.0);
            assert!(out.elevator.is_finite() && out.throttle.is_finite());
        }
    }

    #[test]
    fn orbit_diag_present_only_for_orbit() {
        let src = r#"(planes: [
            (name: "lh", controller: LevelHold(altitude: 500.0, airspeed: 100.0)),
            (name: "orb", controller: Orbit(radius: 800.0, direction: Clockwise, altitude: 500.0, airspeed: 100.0)),
        ])"#;
        let r = Scenario::from_ron_str(src).unwrap().resolve().unwrap();
        assert!(r.planes[0].orbit_diag.is_none());
        let diag = r.planes[1].orbit_diag.expect("orbit diag");
        assert_eq!(diag.radius, 800.0);
    }

    // RL specs always parse + resolve so `default.scenario.ron` loads in every
    // build; without a native inference build, only the *controller build* fails
    // (so the live spawner skips the plane), not the parse.
    #[cfg(not(all(feature = "inference", not(target_arch = "wasm32"))))]
    #[test]
    fn rl_variant_parses_but_build_controller_fails_without_inference() {
        let src = r#"(planes: [
            (name: "rl", controller: RlLevelHold(model: "m", altitude: 500.0, airspeed: 100.0)),
        ])"#;
        let resolved = Scenario::from_ron_str(src)
            .expect("RL spec parses")
            .resolve()
            .expect("RL scenario resolves");
        assert_eq!(resolved.planes[0].spec.kind(), ControllerKind::RlLevelHold);
        assert!(
            resolved.build_controller(0).is_err(),
            "RL controller cannot be built without a native inference build"
        );
    }
}
