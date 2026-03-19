use bevy::prelude::*;
use bevy_rapier3d::prelude::{ExternalForce, Velocity};

use crate::aerodynamics::compute_aero_forces;
use crate::controllers::ActiveController;
use crate::plane::{ControlInputs, FlightState, PlaneConfig};
use crate::plane::plugin::PlaneConfigHandle;

/// System 1: Read Rapier writeback → populate FlightState.
/// angvel from Rapier is world-frame; rotate to body frame before storing.
pub fn sync_flight_state(
    mut query: Query<(&Transform, &Velocity, &mut FlightState)>,
) {
    for (transform, vel, mut state) in query.iter_mut() {
        state.position         = transform.translation;
        state.velocity         = vel.linvel;
        state.attitude         = transform.rotation;
        // transform.rotation is body→world; inverse converts world→body.
        state.angular_velocity = transform.rotation.inverse().mul_vec3(vel.angvel);
        state.update_air_data();
    }
}

/// System 2: Tick each plane's FlightController → ControlInputs.
/// Uses Time<Fixed> so dt matches Rapier's fixed timestep exactly.
pub fn run_flight_controllers(
    time: Res<Time<Fixed>>,
    mut query: Query<(&FlightState, &mut ActiveController, &mut ControlInputs)>,
) {
    let dt = time.delta_secs();
    for (state, mut controller, mut inputs) in query.iter_mut() {
        *inputs = controller.0.update(state, dt);
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
        let Some(cfg) = plane_configs.get(&config_handle.0) else { continue };

        let aero = compute_aero_forces(state, inputs, cfg);
        // state.attitude == Transform.rotation (body→world quaternion).
        let force_world  = state.attitude.mul_vec3(aero.force_body);
        let torque_world = state.attitude.mul_vec3(aero.torque_body);

        *ext_force = ExternalForce { force: force_world, torque: torque_world };
    }
}
