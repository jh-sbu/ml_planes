use bevy::math::Vec3;
use bevy::reflect::Reflect;
use serde::{Deserialize, Serialize};

/// Runtime-loaded plane configuration asset.
/// Field names must exactly match `assets/planes/*.plane.ron`.
#[cfg_attr(feature = "visual", derive(bevy::asset::Asset))]
#[derive(Reflect, Serialize, Deserialize, Debug, Clone)]
pub struct PlaneConfig {
    // Geometry
    pub wing_area: f32,       // S  [m²]
    pub mean_chord: f32,      // c̄  [m]
    pub wing_span: f32,       // b  [m]
    // Mass / Inertia
    pub mass: f32,            // [kg]
    pub inertia: Vec3,        // Ixx, Iyy, Izz  [kg·m²]
    // Longitudinal aero
    pub cl0: f32,
    pub cl_alpha: f32,
    pub cl_delta_e: f32,
    pub cl_max: f32,
    pub cd0: f32,
    pub cd_induced: f32,
    pub cm0: f32,
    pub cm_alpha: f32,
    pub cm_q: f32,
    pub cm_delta_e: f32,
    // Lateral-directional aero
    pub cl_beta: f32,
    pub cl_p: f32,
    pub cl_r: f32,
    pub cl_delta_a: f32,
    pub cn_beta: f32,
    pub cn_r: f32,
    pub cn_delta_r: f32,
    // Engine
    pub thrust_max: f32,      // [N]
    // Control limits
    pub aileron_limit: f32,   // [rad]
    pub elevator_limit: f32,  // [rad]
    pub rudder_limit: f32,    // [rad]
}
