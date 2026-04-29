use bevy::prelude::*;
use bevy::input::mouse::AccumulatedMouseMotion;
use std::f32::consts::PI;

use super::mode::CameraMode;
use crate::plane::{FlightState, PlaneIndex};

#[derive(Component)]
pub struct FreeLookCamera {
    pub yaw: f32,
    pub pitch: f32,
}

impl Default for FreeLookCamera {
    fn default() -> Self {
        Self { yaw: 0.0, pitch: 0.0 }
    }
}

pub fn spawn_camera(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 10.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
        FreeLookCamera::default(),
    ));
}

pub fn cycle_camera_mode(
    keys: Res<ButtonInput<KeyCode>>,
    mut mode: ResMut<CameraMode>,
    planes: Query<(Entity, &PlaneIndex), With<FlightState>>,
) {
    let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    let forward = keys.just_pressed(KeyCode::Tab) && !shift;
    let backward = keys.just_pressed(KeyCode::Tab) && shift;

    if !forward && !backward {
        return;
    }

    let mut pairs: Vec<(Entity, u32)> = planes.iter().map(|(e, idx)| (e, idx.0)).collect();
    pairs.sort_by_key(|&(_, i)| i);
    let sorted: Vec<Entity> = pairs.into_iter().map(|(e, _)| e).collect();

    *mode = if forward {
        match *mode {
            CameraMode::FreeLook => sorted.first().copied().map(CameraMode::Follow).unwrap_or(CameraMode::FreeLook),
            CameraMode::Follow(current) => {
                let idx = sorted.iter().position(|&e| e == current);
                match idx {
                    Some(i) if i + 1 < sorted.len() => CameraMode::Follow(sorted[i + 1]),
                    _ => CameraMode::FreeLook,
                }
            }
        }
    } else {
        match *mode {
            CameraMode::FreeLook => sorted.last().copied().map(CameraMode::Follow).unwrap_or(CameraMode::FreeLook),
            CameraMode::Follow(current) => {
                let idx = sorted.iter().position(|&e| e == current);
                match idx {
                    Some(0) | None => CameraMode::FreeLook,
                    Some(i) => CameraMode::Follow(sorted[i - 1]),
                }
            }
        }
    };
}

pub fn update_free_look_camera(
    mode: Res<CameraMode>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    accumulated: Res<AccumulatedMouseMotion>,
    mut query: Query<(&mut Transform, &mut FreeLookCamera), With<Camera3d>>,
) {
    if !matches!(*mode, CameraMode::FreeLook) {
        return;
    }
    if !mouse_buttons.pressed(MouseButton::Right) {
        return;
    }

    let Ok((mut transform, mut look)) = query.single_mut() else { return };

    const SENSITIVITY: f32 = 0.003;
    look.yaw -= accumulated.delta.x * SENSITIVITY;
    look.pitch -= accumulated.delta.y * SENSITIVITY;
    look.pitch = look.pitch.clamp(-(PI / 2.0 - 0.01), PI / 2.0 - 0.01);

    transform.rotation = Quat::from_rotation_y(look.yaw) * Quat::from_rotation_x(look.pitch);
}

pub fn update_follow_camera(
    mode: Res<CameraMode>,
    target_query: Query<&Transform, Without<Camera3d>>,
    mut camera_query: Query<&mut Transform, With<Camera3d>>,
) {
    let CameraMode::Follow(target_entity) = *mode else { return };

    let Ok(target_transform) = target_query.get(target_entity) else { return };
    let Ok(mut cam_transform) = camera_query.single_mut() else { return };

    let target_pos = target_transform.translation;
    let target_rot = target_transform.rotation;

    let offset = target_rot * Vec3::new(0.0, 10.0, 30.0);
    let camera_pos = target_pos + offset;

    cam_transform.translation = cam_transform.translation.lerp(camera_pos, 0.1);
    cam_transform.look_at(target_pos, Vec3::Y);
}
