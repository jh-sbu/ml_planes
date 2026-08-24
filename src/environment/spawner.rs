use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::controllers::{
    ActiveController, ControllerKind, ControllerTargets, ControllerTelemetry, FlightController,
    PlaneTuning, SelectedTuningProfile,
};
use crate::plane::{
    ControlInputs, FlightState, NextPlaneId, PlaneConfig, PlaneConfigHandle, PlaneId, PlaneIndex,
    PlaneTuningHandle, PlaneTuningPath, SpawnedPlane,
};
use crate::training::SpawnSpec;

#[cfg(feature = "visual")]
use super::visual::PhysicsInterp;

/// Fired (via `Commands::trigger`) when a plane entity contacts the ground collider.
/// In Bevy 0.18 events are observer-based: listen with `app.add_observer(|on: On<PlaneGroundContactEvent>| …)`.
#[derive(Event, Debug, Clone)]
pub struct PlaneGroundContactEvent(pub Entity);

/// Resolve a `SpawnSpec` into the concrete initial `FlightState` a spawned plane
/// starts with, applying the same defaults `spawn_plane` uses. Shared so callers
/// that need to construct a controller *before* spawning (e.g. the lifecycle
/// spawn command) build it from the exact state the plane will hold.
pub fn initial_state_from_spec(spec: &SpawnSpec) -> FlightState {
    let position = spec.position.unwrap_or(Vec3::new(0.0, 500.0, 0.0));
    // -π/2 around X maps body +Z (cockpit up) → world +Y (up).
    let attitude = spec
        .attitude
        .unwrap_or(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2));
    let linvel = spec.velocity.unwrap_or(Vec3::new(100.0, 0.0, 0.0));
    let angvel_body = spec.angular_velocity.unwrap_or(Vec3::ZERO);

    let mut s = FlightState {
        position,
        velocity: linvel,
        attitude,
        angular_velocity: angvel_body,
        ..Default::default()
    };
    s.update_air_data();
    s
}

/// Request a plane, allocating the next available `PlaneId` from `ids`
/// (auto-incremented). Thin wrapper around [`spawn_plane_with_id`] for the common
/// case where the caller doesn't need to know the id up front.
///
/// Like `spawn_plane_with_id`, this returns a [`PendingPlaneSpawn`] that does **not**
/// simulate yet — everything is built from `config_path` once that asset loads (see
/// [`finalize_pending_spawns`]).
pub fn spawn_plane(
    commands: &mut Commands,
    ids: &mut NextPlaneId,
    asset_server: &AssetServer,
    config_path: &str,
    spec: &SpawnSpec,
    controller: Box<dyn FlightController>,
    kind: ControllerKind,
) -> SpawnedPlane {
    let plane_id = PlaneId(ids.0);
    ids.0 += 1;
    spawn_plane_with_id(
        commands,
        plane_id,
        asset_server,
        config_path,
        spec,
        controller,
        kind,
    )
}

/// A spawn request waiting on its `.plane.ron` asset.
///
/// Spawning is asset-driven: the entity exists immediately (so callers can attach
/// per-kind extras and hold onto its `Entity`), but it carries **only** this component
/// until `finalize_pending_spawns` has the real [`PlaneConfig`] in hand and can build
/// `FlightState`, the fuel load, and the Rapier body from it in one step.
///
/// Deliberately none of `PlaneId`, `FlightState`, `ActiveController` or `ControllerKind`
/// live here: every plane-facing system keys off one of those, so a pending entity is
/// invisible to controllers, physics, the MCP snapshot, and — critically —
/// `mark_planes_replicated`, which would otherwise ship a half-formed plane to clients.
#[derive(Component)]
pub struct PendingPlaneSpawn {
    pub plane_id: PlaneId,
    pub spec: SpawnSpec,
    pub kind: ControllerKind,
    /// `Box<dyn FlightController>` is not `Clone`, so finalization `take()`s it.
    pub controller: Option<Box<dyn FlightController>>,
    pub config: Handle<PlaneConfig>,
    /// Sanitized, assets-relative. Kept for the `.tuning.ron` sibling lookup and logs.
    pub config_path: String,
}

/// Request a plane under an explicit, caller-assigned `PlaneId`. Use this instead of
/// [`spawn_plane`] when the id must be known (or reserved) before controllers are
/// built — e.g. a scenario whose wingmen reference their leader by id, where the id
/// has to be settled before `WingmanController::leader_id` is baked in.
///
/// `config_path` is the `.plane.ron` asset this plane is built from. The returned
/// entity is a [`PendingPlaneSpawn`] and does **not** simulate yet; it becomes a real
/// plane once the asset loads (see `finalize_pending_spawns`). There is deliberately no
/// `cfg` parameter and no fallback airframe: mass, inertia, and aerodynamics all come
/// from the same single read of the same file, so they cannot disagree.
pub fn spawn_plane_with_id(
    commands: &mut Commands,
    plane_id: PlaneId,
    asset_server: &AssetServer,
    config_path: &str,
    spec: &SpawnSpec,
    controller: Box<dyn FlightController>,
    kind: ControllerKind,
) -> SpawnedPlane {
    // Validate before handing the path to Bevy. `FileAssetReader` does a bare
    // `root_path.join(path)`, so an absolute path escapes the asset root outright.
    // A rejected path falls back to the default *path* — which is then loaded like
    // any other, rather than to a hardcoded struct.
    let safe_config_path =
        sanitize_asset_path(config_path).unwrap_or_else(|| DEFAULT_PLANE_CONFIG.to_string());
    let handle: Handle<PlaneConfig> = asset_server.load(safe_config_path.clone());

    let entity = commands
        .spawn(PendingPlaneSpawn {
            plane_id,
            spec: spec.clone(),
            kind,
            controller: Some(controller),
            config: handle,
            config_path: safe_config_path,
        })
        .id();

    SpawnedPlane {
        entity,
        id: plane_id,
    }
}

/// Build the real plane once its `.plane.ron` asset is available.
///
/// Runs in `PreUpdate`, which in Bevy 0.18 precedes `RunFixedMainLoop`, so a plane
/// finalized this frame still takes its first Rapier step in the same frame and no
/// system ever observes a partially-built plane.
///
/// Readiness is `Assets<PlaneConfig>::get`, **not** `AssetServer::is_loaded`, so a test
/// can satisfy a pending spawn by inserting the asset directly instead of depending on
/// IO-pool timing.
pub fn finalize_pending_spawns(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    configs: Res<Assets<PlaneConfig>>,
    mut pending: Query<(Entity, &mut PendingPlaneSpawn)>,
) {
    for (entity, mut pending) in pending.iter_mut() {
        let Some(cfg) = configs.get(&pending.config) else {
            // A terminal load failure means this plane can never be built. Drop the
            // request rather than substituting a different airframe.
            //
            // The error text is deliberately NOT echoed: `config_path` can arrive from
            // any peer that can reach the UDP port, and `ron`'s parse errors quote the
            // offending token from the file (CLAUDE.md §7). `src/bin/server.rs` runs
            // without a `LogPlugin`, so this `eprintln!` is the only line that reaches
            // stderr there.
            if asset_server.load_state(&pending.config).is_failed() {
                eprintln!(
                    "plane config '{}' failed to load; not spawning plane {}. \
                     (Check the file exists under the asset root — bevy resolves that \
                     from BEVY_ASSET_ROOT, else CARGO_MANIFEST_DIR, else the directory \
                     holding the executable, NOT the working directory.)",
                    pending.config_path, pending.plane_id.0
                );
                commands.entity(entity).despawn();
            }
            continue;
        };

        let controller = pending
            .controller
            .take()
            .expect("a pending spawn is finalized exactly once");
        let plane_id = pending.plane_id;

        let mut state = initial_state_from_spec(&pending.spec);
        // Opt this plane into the fuel model: fill the tank to the requested fraction of
        // capacity (default full). `consume_fuel` then burns it down.
        state.consumable_remaining =
            cfg.powerplant.capacity() * pending.spec.fuel_fraction.unwrap_or(1.0);
        let position = state.position;
        let attitude = state.attitude;
        let linvel = state.velocity;
        // Body-frame angular velocity → world frame for Rapier.
        let angvel_world = attitude.mul_vec3(state.angular_velocity);

        commands
            .entity(entity)
            .remove::<PendingPlaneSpawn>()
            .insert((
                RigidBody::Dynamic,
                Collider::cuboid(3.0, 0.5, 1.0),
                // The collider is for ground contact only; all mass comes from the
                // explicit `AdditionalMassProperties` below (which Rapier *adds* to the
                // collider-derived mass — default density 1.0 would sneak in ~12 kg the
                // training integrator never sees).
                ColliderMassProperties::Mass(0.0),
                Velocity {
                    linvel,
                    angvel: angvel_world,
                },
                ExternalForce::default(),
                AdditionalMassProperties::MassProperties(MassProperties {
                    local_center_of_mass: Vec3::ZERO,
                    // Empty mass + fuel load (jets); empty mass alone for electric.
                    // `update_plane_mass` keeps this current as fuel burns.
                    mass: cfg
                        .powerplant
                        .effective_mass(cfg.mass, state.consumable_remaining),
                    principal_inertia: cfg.inertia,
                    principal_inertia_local_frame: Quat::IDENTITY,
                }),
                state,
                ControlInputs::default(),
                ActiveController(controller),
                // Replicated controller-telemetry / -targets views, populated each tick by
                // `sync_controller_telemetry` / `sync_controller_targets`; replicon carries
                // them to the client HUD (status display and editable setpoints
                // respectively — nested here since the bundle is already at Bevy's
                // 15-element tuple-`Bundle` ceiling).
                (ControllerTelemetry::default(), ControllerTargets::default()),
                // Inserted here rather than at request time on purpose: this is the
                // `Changed<ControllerKind>` tick that drives the tuning and RL-model
                // rebuild systems in `sim_control.rs`, and it must fire on a plane that
                // already has its `FlightState` and `ActiveController`.
                pending.kind,
                // `PlaneIndex` is the display/cycle ordinal used by the camera, map,
                // and HUD. Deriving it from the already-unique, monotonic `PlaneId`
                // guarantees every spawned plane is indexed automatically — callers
                // never have to insert it by hand.
                (plane_id, PlaneIndex(plane_id.0)),
                PlaneConfigHandle(pending.config.clone()),
                Transform::from_translation(position).with_rotation(attitude),
            ));

        // Attach the per-config tuning pool so `apply_initial_tuning` applies the
        // airframe's PID gains once the asset loads. Skipped when the config ships no
        // `.tuning.ron` sibling, leaving `build()` defaults.
        if let Some(tuning_path) = tuning_path_for_config(&pending.config_path) {
            let tuning_handle: Handle<PlaneTuning> = asset_server.load(tuning_path.clone());
            commands.entity(entity).insert((
                PlaneTuningHandle(tuning_handle),
                SelectedTuningProfile("normal".to_string()),
                // Replicated companion to the handle so a networked client can rebuild
                // its own `PlaneTuningHandle` and enumerate profiles.
                PlaneTuningPath(tuning_path),
            ));
        }

        #[cfg(feature = "visual")]
        commands.entity(entity).insert(PhysicsInterp {
            prev_pos: position,
            prev_rot: attitude,
            curr_pos: position,
            curr_rot: attitude,
        });
    }
}

/// Derive the `.tuning.ron` sibling for a `.plane.ron` config path, returning it
/// only if the file actually ships on disk (under `assets/`). A spawned plane
/// carrying this handle + a `SelectedTuningProfile` gets its per-config PID gains
/// applied by the visual `apply_initial_tuning` system; without it the plane
/// silently flies on `LevelHoldController` defaults. Returns `None` when no
/// sibling exists so callers skip the handle gracefully.
pub fn tuning_path_for_config(config_path: &str) -> Option<String> {
    // Validate before the `.exists()` probe: the resulting `PlaneTuningPath` is
    // a *replicated* component, so probing an unvalidated path would hand a
    // client a file-existence oracle that crosses the wire.
    let safe_path = sanitize_asset_path(config_path)?;
    let tuning_path = format!("{}.tuning.ron", safe_path.strip_suffix(".plane.ron")?);
    std::path::Path::new(&format!("assets/{tuning_path}"))
        .exists()
        .then_some(tuning_path)
}

/// Asset-relative `.plane.ron` used whenever a caller-supplied path is rejected.
pub const DEFAULT_PLANE_CONFIG: &str = "planes/generic_jet.plane.ron";

/// Validate a caller-supplied asset path, returning the canonical
/// `assets/`-relative form or `None` if it is not one.
///
/// **Security boundary.** `config_path` reaches here straight off the wire —
/// `SpawnPlaneNetCommand` (and the MCP `spawn_plane` tool) carry an unvalidated
/// `String` that ends up in both `fs::read("assets/" + p)` and
/// `AssetServer::load(p)`. Neither sink normalizes: Bevy's `FileAssetReader`
/// does a bare `root_path.join(path)`, so an *absolute* path escapes the asset
/// root entirely there, and `..` escapes both.
///
/// Rejects rather than sanitizes. Deliberately **no `canonicalize`**: it touches the
/// filesystem, fails on paths that do not exist yet, and would make this
/// function its own file-existence oracle. A pure lexical check is enough.
pub fn sanitize_asset_path(path: &str) -> Option<String> {
    // Screened before component parsing: `:` and `#` are how Bevy's `AssetPath`
    // selects an asset *source* (`remote://…`) and a sub-asset label, a
    // namespace escape the plain `fs::read` sink does not even have. A
    // backslash is a separator on Windows, so a `/`-only component walk would
    // let `..\..\` through.
    if path.is_empty()
        || path.contains('\0')
        || path.contains('\\')
        || path.contains(':')
        || path.contains('#')
    {
        return None;
    }

    let mut parts: Vec<&str> = Vec::new();
    for component in std::path::Path::new(path).components() {
        match component {
            std::path::Component::Normal(s) => parts.push(s.to_str()?),
            // ParentDir / CurDir / RootDir / Prefix are all rejected outright.
            _ => return None,
        }
    }

    // Callers pass paths relative to `assets/`, so a redundant leading
    // `assets/` is ambiguous rather than harmless — keep one canonical form.
    if parts.is_empty() || parts[0] == "assets" {
        return None;
    }
    Some(parts.join("/"))
}

// Ground-contact detection reads the Rapier context, absent on the thin networked
// client, which never collides locally.
#[cfg(any(not(feature = "net"), feature = "server"))]
pub fn detect_ground_contact(
    plane_query: Query<Entity, With<FlightState>>,
    rapier_context: ReadRapierContext,
    ground_query: Query<Entity, (With<Collider>, Without<FlightState>)>,
    mut commands: Commands,
) {
    let Ok(ctx) = rapier_context.single() else {
        return;
    };
    for ground in ground_query.iter() {
        for plane in plane_query.iter() {
            if ctx.contact_pair(plane, ground).is_some() {
                commands.trigger(PlaneGroundContactEvent(plane));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The default a rejected path falls back to is an asset *path*, loaded like any
    /// other — not a hardcoded struct. This is what lets the spawn path have no
    /// built-in airframe at all; the end-to-end behaviour lives in
    /// `tests/core/lifecycle.rs::spawn_defers_until_the_plane_config_asset_is_available`.
    #[test]
    fn default_plane_config_is_a_real_loadable_asset() {
        assert_eq!(
            sanitize_asset_path(DEFAULT_PLANE_CONFIG),
            Some(DEFAULT_PLANE_CONFIG.to_string()),
            "the fallback path must itself survive sanitization"
        );
        assert!(
            std::path::Path::new(&format!("assets/{DEFAULT_PLANE_CONFIG}")).exists(),
            "the fallback airframe must ship, or a rejected path spawns nothing"
        );
    }

    #[test]
    fn tuning_path_derives_sibling_when_present() {
        // The cargo jet ships a `.tuning.ron` next to its `.plane.ron`.
        assert_eq!(
            tuning_path_for_config("planes/cargo_jet.plane.ron"),
            Some("planes/cargo_jet.tuning.ron".to_string()),
            "should map .plane.ron to its existing .tuning.ron sibling"
        );
    }

    #[test]
    fn tuning_path_is_none_without_sibling() {
        // No `.tuning.ron` ships for this name → no tuning handle to attach.
        assert_eq!(
            tuning_path_for_config("planes/no_such_airframe.plane.ron"),
            None,
            "a config without a tuning sibling yields no tuning path"
        );
    }

    #[test]
    fn sanitize_accepts_a_clean_relative_path() {
        assert_eq!(
            sanitize_asset_path("planes/generic_jet.plane.ron"),
            Some("planes/generic_jet.plane.ron".to_string()),
            "an ordinary asset-relative path round-trips unchanged"
        );
    }

    #[test]
    fn sanitize_rejects_parent_dir_traversal() {
        // The reported vulnerability: `assets/` + this escapes the asset root.
        for path in [
            "../../../../etc/hostname",
            "planes/../../../etc/passwd",
            "planes/../../etc/x.plane.ron",
            "..",
        ] {
            assert_eq!(
                sanitize_asset_path(path),
                None,
                "`..` must be rejected outright, not normalized away: {path}"
            );
        }
    }

    #[test]
    fn sanitize_rejects_absolute_paths() {
        // `AssetServer::load` -> `root_path.join("/etc/passwd")` leaves the asset
        // root entirely; a lexical reject is the only thing standing in the way.
        for path in ["/etc/passwd", "/planes/generic_jet.plane.ron"] {
            assert_eq!(
                sanitize_asset_path(path),
                None,
                "absolute paths escape the asset root via Path::join: {path}"
            );
        }
    }

    #[test]
    fn sanitize_rejects_backslash_and_nul() {
        // A `/`-only component walk would let a Windows-style traversal through.
        assert_eq!(sanitize_asset_path(r"..\..\etc\passwd"), None);
        assert_eq!(sanitize_asset_path("planes\\x.plane.ron"), None);
        assert_eq!(sanitize_asset_path("planes/x\0.plane.ron"), None);
    }

    #[test]
    fn sanitize_rejects_bevy_asset_path_metacharacters() {
        // `AssetPath` parses `source://` and `#label`, a namespace escape the
        // plain `fs::read` sink does not have.
        assert_eq!(sanitize_asset_path("remote://planes/x.plane.ron"), None);
        assert_eq!(sanitize_asset_path("planes/x.plane.ron#label"), None);
    }

    #[test]
    fn sanitize_rejects_redundant_assets_prefix_and_cur_dir() {
        // One canonical form: callers pass paths relative to `assets/`.
        assert_eq!(sanitize_asset_path("assets/planes/x.plane.ron"), None);
        assert_eq!(sanitize_asset_path("./planes/x.plane.ron"), None);
        assert_eq!(sanitize_asset_path(""), None);
    }

    /// Resolves to a real asset if traversal is not rejected, avoiding a
    /// false-positive test caused by a nonexistent path.
    const TRAVERSAL_TO_REAL_FILE: &str = "planes/../planes/cargo_jet.plane.ron";

    #[test]
    fn sanitize_rejects_traversal_even_back_into_assets() {
        // Must not resolve `..` at all — `AssetServer::load`'s bare `root_path.join`
        // would happily walk out of the asset root and back in.
        assert_eq!(sanitize_asset_path(TRAVERSAL_TO_REAL_FILE), None);
        assert_eq!(sanitize_asset_path("../../../../etc/hostname"), None);
    }

    #[test]
    fn tuning_path_is_none_for_traversal_path() {
        // `tuning_path_for_config`'s `.exists()` probe feeds the *replicated*
        // `PlaneTuningPath`, so an unvalidated path is an over-the-wire oracle.
        assert_eq!(
            tuning_path_for_config(TRAVERSAL_TO_REAL_FILE),
            None,
            "a traversal path must not be probed on disk"
        );
    }
}
