use bevy::prelude::*;
use bevy_rapier3d::prelude::{AdditionalMassProperties, ExternalForce, Velocity};

use crate::aerodynamics::{compute_aero_forces, engine_thrust};
use crate::controllers::{ActiveController, ControllerTelemetry};
use crate::plane::context::{ControllerContext, PlaneSnapshot};
use crate::plane::plugin::PlaneConfigHandle;
use crate::plane::{ControlInputs, FlightState, PlaneConfig, PlaneId};

/// System 1: Read Rapier writeback → populate FlightState.
/// angvel from Rapier is world-frame; rotate to body frame before storing.
pub fn sync_flight_state(mut query: Query<(&Transform, &Velocity, &mut FlightState)>) {
    for (transform, vel, mut state) in query.iter_mut() {
        state.position = transform.translation;
        state.velocity = vel.linvel;
        state.attitude = transform.rotation;
        // transform.rotation is body→world; inverse converts world→body.
        state.angular_velocity = transform.rotation.inverse().mul_vec3(vel.angvel);
        state.update_air_data();
    }
}

/// System 2: Tick each plane's FlightController → ControlInputs.
///
/// Two-phase via ParamSet to avoid overlapping queries on the same component set:
///   Phase 1 (immutable): build a snapshot of every plane's current state.
///   Phase 2 (mutable):   call each controller with the full context.
pub fn run_flight_controllers(
    time: Res<Time<Fixed>>,
    mut params: ParamSet<(
        Query<(&PlaneId, &FlightState)>,
        Query<(
            &PlaneId,
            &FlightState,
            &mut ActiveController,
            &mut ControlInputs,
        )>,
    )>,
) {
    let dt = time.delta_secs();

    let snaps: std::sync::Arc<[PlaneSnapshot]> = params
        .p0()
        .iter()
        .map(|(id, st)| PlaneSnapshot {
            id: *id,
            state: st.clone(),
        })
        .collect::<Vec<_>>()
        .into();

    for (id, state, mut ctrl, mut inputs) in params.p1().iter_mut() {
        let ctx = ControllerContext {
            own_id: *id,
            planes: std::sync::Arc::clone(&snaps),
        };
        *inputs = ctrl.0.update(state, &ctx, dt);
        inputs.clamp();
    }
}

/// Snapshot each controller's read-only telemetry into its replicated
/// [`ControllerTelemetry`] component, after the controller has run this tick.
///
/// `ControllerTelemetry` lives server-side only as a derived view; replicon carries
/// it to clients (whose thin build never steps the controller) so the HUD can show
/// orbit radial error, L1 leg/status, wingman/ascent state. The `PartialEq` guard
/// keeps replicon's change detection quiet when nothing moved.
pub fn sync_controller_telemetry(
    mut query: Query<(&ActiveController, &FlightState, &mut ControllerTelemetry)>,
) {
    for (ctrl, state, mut telemetry) in query.iter_mut() {
        let next = ctrl.0.telemetry(state);
        if *telemetry != next {
            *telemetry = next;
        }
    }
}

/// System 3: Compute aero forces (body frame) → rotate to world → ExternalForce.
/// ExternalForce is NOT auto-cleared by Rapier; overwrite unconditionally.
/// Silently skips entities whose PlaneConfig asset is not yet loaded.
pub fn apply_aerodynamic_forces(
    plane_configs: Res<Assets<PlaneConfig>>,
    mut query: Query<(
        &FlightState,
        &ControlInputs,
        &PlaneConfigHandle,
        &mut ExternalForce,
    )>,
) {
    for (state, inputs, config_handle, mut ext_force) in query.iter_mut() {
        let Some(cfg) = plane_configs.get(&config_handle.0) else {
            continue;
        };

        let aero = compute_aero_forces(state, inputs, cfg);
        // state.attitude == Transform.rotation (body→world quaternion).
        let force_world = state.attitude.mul_vec3(aero.force_body);
        let torque_world = state.attitude.mul_vec3(aero.torque_body);

        *ext_force = ExternalForce {
            force: force_world,
            torque: torque_world,
        };
    }
}

/// FixedUpdate: burn each plane's consumable for the thrust it produces this tick.
/// Uses the shared [`engine_thrust`] / [`Powerplant::burn_rate`] primitives so the
/// live sim and the training integrator agree. A non-finite (unmodelled) tank stays
/// non-finite. Silently skips planes whose `PlaneConfig` asset is not yet loaded.
pub fn consume_fuel(
    time: Res<Time<Fixed>>,
    plane_configs: Res<Assets<PlaneConfig>>,
    mut query: Query<(&mut FlightState, &ControlInputs, &PlaneConfigHandle)>,
) {
    let dt = time.delta_secs();
    for (mut state, inputs, config_handle) in query.iter_mut() {
        let Some(cfg) = plane_configs.get(&config_handle.0) else {
            continue;
        };
        let thrust = engine_thrust(&state, inputs, cfg);
        let burn = cfg.powerplant.burn_rate(thrust) * dt;
        state.consumable_remaining = (state.consumable_remaining - burn).max(0.0);
    }
}

/// FixedUpdate: keep the Rapier body mass in sync with remaining fuel for
/// mass-contributing (jet) powerplants, so the airframe gets lighter as it burns.
/// Electric planes are left untouched (constant mass). Runs before the physics step.
pub fn update_plane_mass(
    plane_configs: Res<Assets<PlaneConfig>>,
    mut query: Query<(
        &FlightState,
        &PlaneConfigHandle,
        &mut AdditionalMassProperties,
    )>,
) {
    for (state, config_handle, mut mass_props) in query.iter_mut() {
        let Some(cfg) = plane_configs.get(&config_handle.0) else {
            continue;
        };
        if !cfg.powerplant.contributes_mass() {
            continue;
        }
        if let AdditionalMassProperties::MassProperties(mp) = mass_props.as_mut() {
            mp.mass = cfg
                .powerplant
                .effective_mass(cfg.mass, state.consumable_remaining);
        }
    }
}
