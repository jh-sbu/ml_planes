use bevy::pbr::Material;
use bevy::prelude::*;
use bevy::render::render_resource::AsBindGroup;
use bevy::shader::ShaderRef;

#[derive(Component, Default)]
pub struct GroundPlane;

#[derive(Asset, TypePath, AsBindGroup, Clone, Default)]
pub struct GridMaterial {}

impl Material for GridMaterial {
    fn fragment_shader() -> ShaderRef {
        "shaders/ground_grid.wgsl".into()
    }
}

pub fn follow_camera(
    camera_query: Query<&Transform, With<Camera3d>>,
    mut ground_query: Query<&mut Transform, (With<GroundPlane>, Without<Camera3d>)>,
) {
    let Ok(cam) = camera_query.single() else {
        return;
    };
    let Ok(mut ground) = ground_query.single_mut() else {
        return;
    };
    let snap = 10.0_f32;
    ground.translation.x = (cam.translation.x / snap).round() * snap;
    ground.translation.z = (cam.translation.z / snap).round() * snap;
}
