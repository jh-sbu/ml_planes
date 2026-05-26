use bevy::prelude::*;
use bevy_rapier3d::prelude::{ExternalForce, Velocity};

use crate::aerodynamics::compute_aero_forces;
use crate::controllers::ActiveController;
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
