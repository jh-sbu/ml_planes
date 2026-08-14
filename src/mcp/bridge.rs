//! The command bridge (write path) — plain Rust, no `rmcp` (kept quarantined in `service.rs`).
//!
//! rmcp write tools push a [`ControlRequest`] onto a `crossbeam-channel`; the Bevy
//! [`drain_control_requests`] system drains it each frame and fires the matching
//! `commands.client_trigger(..)` — the exact client→server call the visual client makes in
//! `src/ui/lifecycle_panel.rs` / `ui/hud.rs` / `ui/time_control.rs`. This is the **single**
//! client→server send path in the MCP binary.
//!
//! The channel is the write-path analogue of the read path's [`SnapshotHandle`]
//! (`crate::mcp::snapshot`): created once in `main`, one end handed to the rmcp
//! [`PlanesService`], the other consumed by a system in [`McpBridgePlugin`].
//!
//! [`SnapshotHandle`]: crate::mcp::snapshot::SnapshotHandle
//! [`PlanesService`]: crate::mcp::service::PlanesService
//! [`McpBridgePlugin`]: crate::mcp::McpBridgePlugin

use bevy::prelude::*;
use bevy_replicon::prelude::ClientTriggerExt;
use crossbeam_channel::{unbounded, Receiver, Sender};

use crate::controllers::{ControllerKind, ControllerTargets, OrbitDirection};
use crate::mcp::snapshot::RequestedSimSpeed;
use crate::net::{
    RemovePlaneNetCommand, SetControllerTargetsCommand, SetSimSpeedCommand,
    SetTuningProfileCommand, SpawnPlaneNetCommand, SwitchControllerCommand,
};
use crate::plane::PlaneId;
use crate::sim_speed::SimSpeed;
use crate::training::SpawnSpec;

#[cfg(feature = "inference")]
use crate::net::SetModelCommand;

/// A mutating request from an rmcp tool, drained on the Bevy thread into a
/// `client_trigger`. One variant per client→server command in `net::protocol`.
#[derive(Debug, Clone)]
pub enum ControlRequest {
    /// Spawn a new plane (`SpawnPlaneNetCommand`).
    Spawn {
        config_path: String,
        kind: ControllerKind,
        spec: SpawnSpec,
    },
    /// Remove a plane by id (`RemovePlaneNetCommand`).
    Remove { plane: PlaneId },
    /// Switch a plane's active controller (`SwitchControllerCommand`).
    SwitchController {
        plane: PlaneId,
        kind: ControllerKind,
    },
    /// Select a named PID tuning profile (`SetTuningProfileCommand`).
    SetTuningProfile { plane: PlaneId, profile: String },
    /// Set the authoritative playback speed (`SetSimSpeedCommand`).
    SetSimSpeed { speed: SimSpeed },
    /// Edit a plane's controller setpoints (`SetControllerTargetsCommand`).
    SetControllerTargets {
        plane: PlaneId,
        targets: ControllerTargets,
    },
    /// Select a trained RL model by file stem (`SetModelCommand`). Inference-only, like the
    /// command it maps to.
    #[cfg(feature = "inference")]
    SetModel { plane: PlaneId, model_stem: String },
}

/// Sending half of the command channel — cloned into the rmcp [`PlanesService`].
///
/// [`PlanesService`]: crate::mcp::service::PlanesService
#[derive(Clone)]
pub struct ControlSender(pub Sender<ControlRequest>);

/// Receiving half of the command channel — a Bevy `Resource` drained by
/// [`drain_control_requests`]. `Clone` (crossbeam receivers are cloneable) so
/// [`McpBridgePlugin`] can insert it from `&self`.
///
/// [`McpBridgePlugin`]: crate::mcp::McpBridgePlugin
#[derive(Resource, Clone)]
pub struct ControlReceiver(pub Receiver<ControlRequest>);

/// Build a fresh unbounded command channel: the `ControlSender` goes to the rmcp service, the
/// `ControlReceiver` into the Bevy app.
pub fn control_channel() -> (ControlSender, ControlReceiver) {
    let (tx, rx) = unbounded();
    (ControlSender(tx), ControlReceiver(rx))
}

/// `Update` system: drain every queued [`ControlRequest`] and fire its client→server command.
///
/// Non-blocking (`try_recv` until empty). `SetSimSpeed` also records the value into
/// [`RequestedSimSpeed`] so `get_sim_status` can echo the MCP's last request (the server's true
/// speed is not replicated — see `snapshot.rs`).
pub fn drain_control_requests(
    receiver: Res<ControlReceiver>,
    mut requested_speed: ResMut<RequestedSimSpeed>,
    mut commands: Commands,
) {
    while let Ok(req) = receiver.0.try_recv() {
        match req {
            ControlRequest::Spawn {
                config_path,
                kind,
                spec,
            } => {
                commands.client_trigger(SpawnPlaneNetCommand {
                    config_path,
                    kind,
                    spec,
                });
            }
            ControlRequest::Remove { plane } => {
                commands.client_trigger(RemovePlaneNetCommand { plane });
            }
            ControlRequest::SwitchController { plane, kind } => {
                commands.client_trigger(SwitchControllerCommand { plane, kind });
            }
            ControlRequest::SetTuningProfile { plane, profile } => {
                commands.client_trigger(SetTuningProfileCommand { plane, profile });
            }
            ControlRequest::SetSimSpeed { speed } => {
                requested_speed.0 = Some(speed);
                commands.client_trigger(SetSimSpeedCommand { speed });
            }
            ControlRequest::SetControllerTargets { plane, targets } => {
                commands.client_trigger(SetControllerTargetsCommand { plane, targets });
            }
            #[cfg(feature = "inference")]
            ControlRequest::SetModel { plane, model_stem } => {
                commands.client_trigger(SetModelCommand { plane, model_stem });
            }
        }
    }
}

/// The controller kinds an MCP client may spawn / switch to, by their serde variant name.
///
/// Mirrors the visual spawn UI's `SPAWNABLE_KINDS` (`src/ui/lifecycle_panel.rs`): only kinds
/// whose generic `ControllerKind::build()` is self-sufficient (no leader entity, `.plan.ron`
/// asset, or RL model needed). `Wingman` / `FlightPlan` are deliberately excluded — `build()`
/// silently substitutes `LevelHold` / `Orbit` for them. RL kinds appear only under `inference`.
pub fn spawnable_kind_names() -> &'static [&'static str] {
    #[cfg(not(feature = "inference"))]
    {
        &["Manual", "LevelHold", "HeadingHold", "Ascent", "Orbit"]
    }
    #[cfg(feature = "inference")]
    {
        &[
            "Manual",
            "LevelHold",
            "HeadingHold",
            "Ascent",
            "Orbit",
            "RlLevelHold",
            "RlHeadingHold",
            "RlOrbit",
            "RlOrbitResidual",
            "RlLstmOrbit",
        ]
    }
}

/// Parse a controller-kind name (serde variant identifier) into a spawnable [`ControllerKind`].
///
/// Backs both `spawn_plane` and `switch_controller`. `Wingman` / `FlightPlan` are
/// rejected with a descriptive error because the generic builder mis-substitutes them; unknown
/// names likewise return an `Err` (never a panic). RL kinds parse only under `inference`.
pub fn parse_spawnable_controller_kind(name: &str) -> Result<ControllerKind, String> {
    match name {
        "Manual" => Ok(ControllerKind::Manual),
        "LevelHold" => Ok(ControllerKind::LevelHold),
        "HeadingHold" => Ok(ControllerKind::HeadingHold),
        "Ascent" => Ok(ControllerKind::Ascent),
        "Orbit" => Ok(ControllerKind::Orbit),
        #[cfg(feature = "inference")]
        "RlLevelHold" => Ok(ControllerKind::RlLevelHold),
        #[cfg(feature = "inference")]
        "RlHeadingHold" => Ok(ControllerKind::RlHeadingHold),
        #[cfg(feature = "inference")]
        "RlOrbit" => Ok(ControllerKind::RlOrbit),
        #[cfg(feature = "inference")]
        "RlOrbitResidual" => Ok(ControllerKind::RlOrbitResidual),
        #[cfg(feature = "inference")]
        "RlLstmOrbit" => Ok(ControllerKind::RlLstmOrbit),
        "Wingman" | "FlightPlan" => Err(format!(
            "controller kind `{name}` cannot be spawned or switched over MCP: its generic \
             builder substitutes a different controller (Wingman→LevelHold, FlightPlan→Orbit), \
             so the plane would fly the wrong one. Choose a self-sufficient kind: {}.",
            spawnable_kind_names().join(", ")
        )),
        other => Err(format!(
            "unknown controller kind `{other}`; expected one of: {}",
            spawnable_kind_names().join(", ")
        )),
    }
}

/// Parse a [`SimSpeed`] name (serde variant identifier) into the enum.
///
/// Backs the `set_sim_speed` tool. Keyed on the exact variant names replicon serializes
/// (`SimSpeed` derives `Serialize`/`Deserialize` under `net`); unknown names return an `Err`
/// (never a panic). Note `"X1"` — not `"1x"`, which is the display label (`SimSpeed::label`).
pub fn parse_sim_speed(name: &str) -> Result<SimSpeed, String> {
    match name {
        "Paused" => Ok(SimSpeed::Paused),
        "X1" => Ok(SimSpeed::X1),
        "X5" => Ok(SimSpeed::X5),
        "X10" => Ok(SimSpeed::X10),
        other => Err(format!(
            "unknown sim speed `{other}`; expected one of: Paused, X1, X5, X10"
        )),
    }
}

/// Parse an [`OrbitDirection`] name (serde variant identifier, or its `CW`/`CCW` HUD-label
/// alias) into the enum.
///
/// Backs the `direction` field of `set_controller_targets`. Unknown names return an `Err`
/// (never a panic).
pub fn parse_orbit_direction(name: &str) -> Result<OrbitDirection, String> {
    match name {
        "CW" | "Clockwise" => Ok(OrbitDirection::Clockwise),
        "CCW" | "CounterClockwise" => Ok(OrbitDirection::CounterClockwise),
        other => Err(format!(
            "unknown orbit direction `{other}`; expected one of: CW, CCW"
        )),
    }
}

/// Merge a partial edit (only the fields the caller actually supplied) onto `current`'s
/// `ControllerTargets` variant.
///
/// Backs the `set_controller_targets` tool. Only the fields that apply to `current`'s variant
/// take effect — e.g. `radius` is silently ignored against a `LevelHold` plane, mirroring
/// `FlightController::apply_targets`'s own mismatched-variant tolerance — so the caller can
/// change one field of a plane's setpoints without restating every other one. Returns `Err`
/// if `current` is `None` (the plane's controller has nothing settable, e.g. Manual or
/// FlightPlan) or `direction_name` fails to parse.
#[allow(clippy::too_many_arguments)]
pub fn merge_controller_targets(
    current: ControllerTargets,
    altitude: Option<f32>,
    airspeed: Option<f32>,
    heading_deg: Option<f32>,
    center_x: Option<f32>,
    center_z: Option<f32>,
    radius: Option<f32>,
    direction_name: Option<&str>,
    leader_id: Option<u32>,
) -> Result<ControllerTargets, String> {
    match current {
        ControllerTargets::None => {
            Err("this plane's controller has no editable targets (Manual/FlightPlan)".to_string())
        }
        ControllerTargets::LevelHold {
            altitude: alt,
            airspeed: spd,
        } => Ok(ControllerTargets::LevelHold {
            altitude: altitude.unwrap_or(alt),
            airspeed: airspeed.unwrap_or(spd),
        }),
        ControllerTargets::Ascent { altitude: alt } => Ok(ControllerTargets::Ascent {
            altitude: altitude.unwrap_or(alt),
        }),
        ControllerTargets::HeadingHold {
            heading,
            altitude: alt,
            airspeed: spd,
        } => Ok(ControllerTargets::HeadingHold {
            heading: heading_deg.map(f32::to_radians).unwrap_or(heading),
            altitude: altitude.unwrap_or(alt),
            airspeed: airspeed.unwrap_or(spd),
        }),
        ControllerTargets::Orbit(mut params) => {
            if let Some(v) = center_x {
                params.center_x = v;
            }
            if let Some(v) = center_z {
                params.center_z = v;
            }
            if let Some(v) = radius {
                params.target_radius = v;
            }
            if let Some(v) = altitude {
                params.target_altitude = v;
            }
            if let Some(v) = airspeed {
                params.target_airspeed = v;
            }
            if let Some(name) = direction_name {
                params.direction = parse_orbit_direction(name)?;
            }
            Ok(ControllerTargets::Orbit(params))
        }
        ControllerTargets::Wingman { leader } => Ok(ControllerTargets::Wingman {
            leader: leader_id.map(PlaneId).unwrap_or(leader),
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controllers::OrbitParams;

    fn sample_orbit_params() -> OrbitParams {
        OrbitParams {
            center_x: 0.0,
            center_z: 0.0,
            target_radius: 1000.0,
            target_altitude: 1000.0,
            target_airspeed: 100.0,
            direction: OrbitDirection::CounterClockwise,
        }
    }

    #[test]
    fn parse_orbit_direction_accepts_cw_and_ccw() {
        assert_eq!(
            parse_orbit_direction("CW").unwrap(),
            OrbitDirection::Clockwise
        );
        assert_eq!(
            parse_orbit_direction("CCW").unwrap(),
            OrbitDirection::CounterClockwise
        );
    }

    #[test]
    fn parse_orbit_direction_rejects_unknown() {
        assert!(parse_orbit_direction("sideways").is_err());
    }

    #[test]
    fn merge_controller_targets_overrides_level_hold_fields() {
        let current = ControllerTargets::LevelHold {
            altitude: 1000.0,
            airspeed: 80.0,
        };
        let merged = merge_controller_targets(
            current,
            Some(1600.0),
            None,
            None,
            None,
            None,
            None,
            None,
            None,
        )
        .unwrap();
        assert_eq!(
            merged,
            ControllerTargets::LevelHold {
                altitude: 1600.0,
                airspeed: 80.0,
            },
            "unspecified airspeed should keep its current value"
        );
    }

    #[test]
    fn merge_controller_targets_ascent_overrides_altitude() {
        let current = ControllerTargets::Ascent { altitude: 1000.0 };
        let merged = merge_controller_targets(
            current,
            Some(2000.0),
            None,
            None,
            None,
            None,
            None,
            None,
            None,
        )
        .unwrap();
        assert_eq!(merged, ControllerTargets::Ascent { altitude: 2000.0 });
    }

    #[test]
    fn merge_controller_targets_heading_hold_converts_degrees_to_radians() {
        let current = ControllerTargets::HeadingHold {
            heading: 0.0,
            altitude: 1000.0,
            airspeed: 80.0,
        };
        let merged = merge_controller_targets(
            current,
            None,
            None,
            Some(90.0),
            None,
            None,
            None,
            None,
            None,
        )
        .unwrap();
        match merged {
            ControllerTargets::HeadingHold { heading, .. } => {
                assert!(
                    (heading - std::f32::consts::FRAC_PI_2).abs() < 1e-5,
                    "heading={heading}"
                );
            }
            other => panic!("expected HeadingHold, got {other:?}"),
        }
    }

    #[test]
    fn merge_controller_targets_orbit_overrides_geometry_and_direction() {
        let current = ControllerTargets::Orbit(sample_orbit_params());
        let merged = merge_controller_targets(
            current,
            Some(1800.0),
            Some(120.0),
            None,
            Some(50.0),
            Some(60.0),
            Some(3000.0),
            Some("CW"),
            None,
        )
        .unwrap();
        assert_eq!(
            merged,
            ControllerTargets::Orbit(OrbitParams {
                center_x: 50.0,
                center_z: 60.0,
                target_radius: 3000.0,
                target_altitude: 1800.0,
                target_airspeed: 120.0,
                direction: OrbitDirection::Clockwise,
            })
        );
    }

    #[test]
    fn merge_controller_targets_orbit_rejects_bad_direction() {
        let current = ControllerTargets::Orbit(sample_orbit_params());
        assert!(merge_controller_targets(
            current,
            None,
            None,
            None,
            None,
            None,
            None,
            Some("sideways"),
            None
        )
        .is_err());
    }

    #[test]
    fn merge_controller_targets_wingman_overrides_leader() {
        let current = ControllerTargets::Wingman { leader: PlaneId(1) };
        let merged =
            merge_controller_targets(current, None, None, None, None, None, None, None, Some(9))
                .unwrap();
        assert_eq!(merged, ControllerTargets::Wingman { leader: PlaneId(9) });
    }

    #[test]
    fn merge_controller_targets_errors_on_none_variant() {
        assert!(merge_controller_targets(
            ControllerTargets::None,
            Some(1000.0),
            None,
            None,
            None,
            None,
            None,
            None,
            None
        )
        .is_err());
    }

    #[test]
    fn parses_the_self_sufficient_pid_kinds() {
        assert_eq!(
            parse_spawnable_controller_kind("Manual").unwrap(),
            ControllerKind::Manual
        );
        assert_eq!(
            parse_spawnable_controller_kind("LevelHold").unwrap(),
            ControllerKind::LevelHold
        );
        assert_eq!(
            parse_spawnable_controller_kind("HeadingHold").unwrap(),
            ControllerKind::HeadingHold
        );
        assert_eq!(
            parse_spawnable_controller_kind("Ascent").unwrap(),
            ControllerKind::Ascent
        );
        assert_eq!(
            parse_spawnable_controller_kind("Orbit").unwrap(),
            ControllerKind::Orbit
        );
    }

    #[test]
    fn rejects_wingman_and_flight_plan_with_a_reason() {
        for name in ["Wingman", "FlightPlan"] {
            let err = parse_spawnable_controller_kind(name).unwrap_err();
            assert!(
                err.contains("substitutes"),
                "rejection for {name} should explain the mis-substitution, got: {err}"
            );
        }
    }

    #[test]
    fn rejects_unknown_names() {
        let err = parse_spawnable_controller_kind("Nonsense").unwrap_err();
        assert!(err.contains("unknown controller kind"));
        assert!(err.contains("Nonsense"));
    }

    #[cfg(feature = "inference")]
    #[test]
    fn parses_rl_kinds_under_inference() {
        assert_eq!(
            parse_spawnable_controller_kind("RlOrbit").unwrap(),
            ControllerKind::RlOrbit
        );
        assert_eq!(
            parse_spawnable_controller_kind("RlLstmOrbit").unwrap(),
            ControllerKind::RlLstmOrbit
        );
    }

    #[cfg(not(feature = "inference"))]
    #[test]
    fn rl_kinds_are_unknown_without_inference() {
        assert!(parse_spawnable_controller_kind("RlOrbit").is_err());
    }

    #[test]
    fn parses_sim_speed_variant_names() {
        assert_eq!(parse_sim_speed("Paused").unwrap(), SimSpeed::Paused);
        assert_eq!(parse_sim_speed("X1").unwrap(), SimSpeed::X1);
        assert_eq!(parse_sim_speed("X5").unwrap(), SimSpeed::X5);
        assert_eq!(parse_sim_speed("X10").unwrap(), SimSpeed::X10);
    }

    #[test]
    fn rejects_unknown_sim_speed_names() {
        // The display label "1x" is not the serde variant name — must not round-trip.
        let err = parse_sim_speed("1x").unwrap_err();
        assert!(err.contains("unknown sim speed"));
        assert!(err.contains("1x"));
    }

    #[test]
    fn channel_round_trips_a_request() {
        let (tx, rx) = control_channel();
        tx.0.send(ControlRequest::Remove { plane: PlaneId(3) })
            .unwrap();
        match rx.0.try_recv().unwrap() {
            ControlRequest::Remove { plane } => assert_eq!(plane, PlaneId(3)),
            other => panic!("unexpected request: {other:?}"),
        }
    }
}
