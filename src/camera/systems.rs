use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use std::f32::consts::{FRAC_PI_2, PI};

use super::mode::CameraMode;
use crate::plane::{FlightState, PlaneIndex};

#[derive(Component)]
pub struct FreeLookCamera {
    pub yaw: f32,
    pub pitch: f32,
}

impl Default for FreeLookCamera {
    fn default() -> Self {
        Self {
            yaw: 0.0,
            pitch: 0.0,
        }
    }
}

#[derive(Component)]
pub struct FollowCamera {
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
    /// Exponentially smoothed position of the follow target. Initialized to
    /// Vec3::ZERO and snap-set to the physics target on the first valid frame
    /// (detected by distance > 500 m from target), so the camera never jumps
    /// from the world origin to the target plane.
    pub smoothed_pos: Vec3,
    /// Exponentially smoothed rotation of the follow target.
    pub smoothed_rot: Quat,
}

impl Default for FollowCamera {
    fn default() -> Self {
        // Matches the previous hardcoded body-frame offset Vec3(0, 10, 30)
        Self {
            yaw: 0.0,
            pitch: 10.0_f32.atan2(30.0),
            distance: (10.0_f32.powi(2) + 30.0_f32.powi(2)).sqrt(),
            smoothed_pos: Vec3::ZERO,
            smoothed_rot: Quat::IDENTITY,
        }
    }
}

pub fn spawn_camera(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 10.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
        FreeLookCamera::default(),
        FollowCamera::default(),
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
            CameraMode::FreeLook => sorted
                .first()
                .copied()
                .map(CameraMode::Follow)
                .unwrap_or(CameraMode::FreeLook),
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
            CameraMode::FreeLook => sorted
                .last()
                .copied()
                .map(CameraMode::Follow)
                .unwrap_or(CameraMode::FreeLook),
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

    let Ok((mut transform, mut look)) = query.single_mut() else {
        return;
    };

    const SENSITIVITY: f32 = 0.003;
    look.yaw -= accumulated.delta.x * SENSITIVITY;
    look.pitch -= accumulated.delta.y * SENSITIVITY;
    look.pitch = look.pitch.clamp(-(PI / 2.0 - 0.01), PI / 2.0 - 0.01);

    transform.rotation = Quat::from_rotation_y(look.yaw) * Quat::from_rotation_x(look.pitch);
}

pub fn handle_follow_camera_input(
    mode: Res<CameraMode>,
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    accumulated_motion: Res<AccumulatedMouseMotion>,
    accumulated_scroll: Res<AccumulatedMouseScroll>,
    mut query: Query<&mut FollowCamera, With<Camera3d>>,
) {
    if !matches!(*mode, CameraMode::Follow(_)) {
        return;
    }
    let Ok(mut follow) = query.single_mut() else {
        return;
    };

    let scroll = accumulated_scroll.delta.y;
    if scroll != 0.0 {
        follow.distance = (follow.distance - scroll * follow.distance * 0.1).clamp(5.0, 300.0);
    }

    if mouse_buttons.pressed(MouseButton::Right) {
        const SENSITIVITY: f32 = 0.003;
        follow.yaw += accumulated_motion.delta.x * SENSITIVITY;
        follow.pitch -= accumulated_motion.delta.y * SENSITIVITY;
        follow.pitch = follow.pitch.clamp(-(FRAC_PI_2 - 0.01), FRAC_PI_2 - 0.01);
    }
}

pub fn update_follow_camera(
    mode: Res<CameraMode>,
    time: Res<Time>,
    target_query: Query<&Transform, Without<Camera3d>>,
    mut camera_query: Query<(&mut Transform, &mut FollowCamera), With<Camera3d>>,
) {
    let CameraMode::Follow(target_entity) = *mode else {
        return;
    };

    let Ok(target_transform) = target_query.get(target_entity) else {
        return;
    };
    let Ok((mut cam_transform, mut follow)) = camera_query.single_mut() else {
        return;
    };

    let target_pos = target_transform.translation;
    let target_rot = target_transform.rotation;

    // Snap-initialize on the first frame (smoothed_pos starts at Vec3::ZERO which
    // is far from any flying plane) so the camera doesn't sweep across the world.
    if follow.smoothed_pos.distance(target_pos) > 500.0 {
        follow.smoothed_pos = target_pos;
        follow.smoothed_rot = target_rot;
    }

    // Smooth the target pose at tau=12 rad/s — fast enough to track turns,
    // slow enough to filter the 64 Hz fixed-step snaps from Rapier.
    let alpha = 1.0 - (-12.0_f32 * time.delta_secs()).exp();
    follow.smoothed_pos = follow.smoothed_pos.lerp(target_pos, alpha);
    follow.smoothed_rot = follow.smoothed_rot.slerp(target_rot, alpha);

    let horiz = follow.distance * follow.pitch.cos();
    let vert = follow.distance * follow.pitch.sin();
    // Body frame: −X = behind (nose), +Y = right wing, +Z = up
    let local_offset = Vec3::new(-horiz * follow.yaw.cos(), horiz * follow.yaw.sin(), vert);
    let offset = follow.smoothed_rot * local_offset;
    cam_transform.translation = follow.smoothed_pos + offset;
    cam_transform.look_at(follow.smoothed_pos, Vec3::Y);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    fn orbit_offset(yaw: f32, pitch: f32, distance: f32) -> Vec3 {
        let horiz = distance * pitch.cos();
        let vert = distance * pitch.sin();
        Vec3::new(-horiz * yaw.cos(), horiz * yaw.sin(), vert)
    }

    #[test]
    fn default_offset_is_behind_and_above() {
        let follow = FollowCamera::default();
        let offset = orbit_offset(follow.yaw, follow.pitch, follow.distance);
        assert!((offset.x + 30.0).abs() < 0.1, "x={} expected≈-30", offset.x);
        assert!(offset.y.abs() < 0.1, "y={} expected≈0", offset.y);
        assert!((offset.z - 10.0).abs() < 0.1, "z={} expected≈10", offset.z);
    }

    #[test]
    fn yaw_90_puts_camera_to_right() {
        let follow = FollowCamera {
            yaw: FRAC_PI_2,
            ..FollowCamera::default()
        };
        let offset = orbit_offset(follow.yaw, follow.pitch, follow.distance);
        assert!(offset.x.abs() < 0.1, "x={} expected≈0", offset.x);
        assert!(offset.y > 0.0, "y={} expected>0 (right side)", offset.y);
    }
}
