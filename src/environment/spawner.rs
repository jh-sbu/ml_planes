use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

use crate::controllers::{
    ActiveController, ControllerKind, ControllerTelemetry, FlightController, PlaneTuning,
    SelectedTuningProfile,
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

/// Mass/inertia (and control limits) for the generic-jet airframe, used to seed
/// Rapier `AdditionalMassProperties` at spawn time. Aerodynamic coefficients are
/// left at zero — the async-loaded `.plane.ron` handle drives aero forces once
/// the asset is ready. Shared by `main.rs`'s startup spawns and the runtime
/// `SpawnPlaneCommand` so both seed identical body mass.
pub fn generic_jet_spawn_config() -> PlaneConfig {
    PlaneConfig {
        wing_area: 20.0,
        mean_chord: 2.0,
        wing_span: 10.0,
        mass: 5000.0,
        inertia: Vec3::new(10000.0, 40000.0, 45000.0),
        cl0: 0.0,
        cl_alpha: 0.0,
        cl_delta_e: 0.0,
        cl_max: 1.4,
        cd0: 0.0,
        cd_induced: 0.0,
        cm0: 0.0,
        cm_alpha: 0.0,
        cm_q: 0.0,
        cm_delta_e: 0.0,
        cl_beta: 0.0,
        cl_p: 0.0,
        cl_r: 0.0,
        cl_delta_a: 0.0,
        cn_beta: 0.0,
        cn_r: 0.0,
        cn_delta_r: 0.0,
        thrust_max: 0.0,
        powerplant: Default::default(),
        aileron_limit: 0.4363,
        elevator_limit: 0.3491,
        rudder_limit: 0.2618,
    }
}

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

/// Spawn a physics-ready plane entity and assign it a stable `PlaneId`.
///
/// `ids` allocates the next available id (auto-incremented).
/// `config_path` is the `.plane.ron` asset loaded asynchronously to drive
/// aerodynamic forces once ready; `cfg` is used immediately for Rapier
/// `AdditionalMassProperties` (mass/inertia) at spawn time. Pass a `cfg` whose
/// mass/inertia match `config_path` — they describe the same airframe.
pub fn spawn_plane(
    commands: &mut Commands,
    ids: &mut NextPlaneId,
    asset_server: &AssetServer,
    config_path: &str,
    spec: &SpawnSpec,
    controller: Box<dyn FlightController>,
    kind: ControllerKind,
    cfg: &PlaneConfig,
) -> SpawnedPlane {
    let plane_id = PlaneId(ids.0);
    ids.0 += 1;
    let mut state = initial_state_from_spec(spec);
    // Opt this plane into the fuel model: fill the tank to the requested fraction of
    // capacity (default full). The integrator / `consume_fuel` then burn it down.
    state.consumable_remaining = cfg.powerplant.capacity() * spec.fuel_fraction.unwrap_or(1.0);
    let position = state.position;
    let attitude = state.attitude;
    let linvel = state.velocity;
    // Body-frame angular velocity → world frame for Rapier.
    let angvel_world = attitude.mul_vec3(state.angular_velocity);

    let handle: Handle<PlaneConfig> = asset_server.load(config_path.to_string());

    let entity = commands
        .spawn((
            RigidBody::Dynamic,
            Collider::cuboid(3.0, 0.5, 1.0),
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
            // Replicated controller-telemetry view, populated each tick by
            // `sync_controller_telemetry`; replicon carries it to the client HUD.
            ControllerTelemetry::default(),
            kind,
            // `PlaneIndex` is the display/cycle ordinal used by the camera, map,
            // and HUD. Deriving it from the already-unique, monotonic `PlaneId`
            // guarantees every spawned plane is indexed automatically — callers
            // never have to insert it by hand.
            (plane_id, PlaneIndex(plane_id.0)),
            PlaneConfigHandle(handle),
            Transform::from_translation(position).with_rotation(attitude),
        ))
        .id();

    // Attach the per-config tuning pool so the visual `apply_initial_tuning`
    // system applies the airframe's PID gains once the asset loads. Skipped when
    // the config ships no `.tuning.ron` sibling, leaving `build()` defaults.
    if let Some(tuning_path) = tuning_path_for_config(config_path) {
        let tuning_handle: Handle<PlaneTuning> = asset_server.load(tuning_path.clone());
        commands.entity(entity).insert((
            PlaneTuningHandle(tuning_handle),
            SelectedTuningProfile("normal".to_string()),
            // Replicated companion to the handle so a networked client can rebuild
            // its own `PlaneTuningHandle` and enumerate profiles (Phase 6).
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

    SpawnedPlane {
        entity,
        id: plane_id,
    }
}

/// Load the `PlaneConfig` for a spawn `config_path` (asset-relative, e.g.
/// `planes/cargo_jet.plane.ron`) directly from disk so the Rapier body gets the
/// correct mass/inertia/fuel *at spawn*, before any aero applies. Falls back to
/// the generic-jet spawn config (with a warning) if the file is missing or
/// invalid, keeping the `N` hotkey and bad paths robust.
///
/// Reading synchronously here — rather than waiting on the async asset handle —
/// is what keeps `cfg` (mass/inertia/fuel) consistent with `config_path`'s
/// aerodynamics from the first physics step. A mismatched (e.g. generic) inertia
/// under a heavy airframe's aero moments diverges to a non-finite state.
/// Derive the `.tuning.ron` sibling for a `.plane.ron` config path, returning it
/// only if the file actually ships on disk (under `assets/`). A spawned plane
/// carrying this handle + a `SelectedTuningProfile` gets its per-config PID gains
/// applied by the visual `apply_initial_tuning` system; without it the plane
/// silently flies on `LevelHoldController` defaults. Returns `None` when no
/// sibling exists so callers skip the handle gracefully.
pub fn tuning_path_for_config(config_path: &str) -> Option<String> {
    let tuning_path = format!("{}.tuning.ron", config_path.strip_suffix(".plane.ron")?);
    std::path::Path::new(&format!("assets/{tuning_path}"))
        .exists()
        .then_some(tuning_path)
}

pub fn load_spawn_config(config_path: &str) -> PlaneConfig {
    let disk_path = format!("assets/{config_path}");
    match std::fs::read(&disk_path) {
        Ok(bytes) => match ron::de::from_bytes::<PlaneConfig>(&bytes) {
            Ok(cfg) => cfg,
            Err(e) => {
                eprintln!("spawn config '{disk_path}' parse failed ({e}); using generic jet");
                generic_jet_spawn_config()
            }
        },
        Err(e) => {
            eprintln!("spawn config '{disk_path}' read failed ({e}); using generic jet");
            generic_jet_spawn_config()
        }
    }
}

// Ground-contact detection reads the Rapier context, absent on the thin networked
// client (which never collides locally — see `plans/client_server.md` Phase 4).
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

    #[test]
    fn load_spawn_config_reads_real_mass_and_inertia() {
        // The cargo jet is a heavy airframe; seeding the body from the real
        // config (not the generic fallback) is what keeps aero/inertia consistent.
        let cfg = load_spawn_config("planes/cargo_jet.plane.ron");
        assert_eq!(cfg.mass, 128000.0, "should read the cargo jet's dry mass");
        assert!(
            (cfg.inertia.y - 2.46e7).abs() < 1.0,
            "should read the cargo jet's heavy Iyy, got {}",
            cfg.inertia.y
        );
    }

    #[test]
    fn load_spawn_config_falls_back_on_missing_file() {
        let cfg = load_spawn_config("planes/does_not_exist.plane.ron");
        assert_eq!(
            cfg.mass,
            generic_jet_spawn_config().mass,
            "missing config falls back to the generic-jet spawn config"
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
}
