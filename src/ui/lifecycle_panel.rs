//! Bottom-left **Planes** panel + spawn/remove hotkeys (visual mode only).
//!
//! Lists every live plane with a Remove button and offers a small spawn form
//! (controller kind + `.plane.ron` path) that places a new plane ahead of the
//! camera. Both the panel buttons and the `N` / `Delete` hotkeys fan out to the
//! headless-safe [`SpawnPlaneCommand`] / [`RemovePlaneCommand`] events, so all
//! the actual lifecycle logic lives in `environment::lifecycle`.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

use crate::camera::CameraMode;
use crate::controllers::ControllerKind;
use crate::environment::{RemovePlaneCommand, SpawnPlaneCommand};
use crate::plane::{FlightState, PlaneIndex};
use crate::training::SpawnSpec;
use crate::ui::map::MapState;

const DEFAULT_CONFIG: &str = "planes/generic_jet.plane.ron";

/// Controller kinds whose `build()` is self-sufficient (no leader, plan, or
/// model needed), so they can be spawned directly from the panel.
#[cfg(not(feature = "inference"))]
const SPAWNABLE_KINDS: &[ControllerKind] = &[
    ControllerKind::Manual,
    ControllerKind::LevelHold,
    ControllerKind::Ascent,
    ControllerKind::HeadingHold,
    ControllerKind::Orbit,
];

/// As above, plus the ML controllers (only when an inference/training backend is
/// compiled in). A runtime-spawned RL plane starts on the PID fallback that
/// `build()` produces; `apply_rl_controller_switch` upgrades it to the trained
/// model the same frame, or reverts the kind to its PID baseline if no model is
/// available.
#[cfg(feature = "inference")]
const SPAWNABLE_KINDS: &[ControllerKind] = &[
    ControllerKind::Manual,
    ControllerKind::LevelHold,
    ControllerKind::Ascent,
    ControllerKind::HeadingHold,
    ControllerKind::Orbit,
    ControllerKind::RlLevelHold,
    ControllerKind::RlOrbit,
    ControllerKind::RlOrbitResidual,
    ControllerKind::RlLstmOrbit,
];

/// Distance ahead of the camera to place a freshly-spawned plane.
const SPAWN_AHEAD_M: f32 = 400.0;
/// Floor altitude so a plane spawned with a downward-pointing camera still
/// starts safely above the ground death-plane.
const MIN_SPAWN_ALT_M: f32 = 500.0;
/// Initial airspeed for a panel/hotkey-spawned plane.
const SPAWN_SPEED_MS: f32 = 100.0;

/// Spawn-form state for the Planes panel (selected kind + config path).
#[derive(Resource)]
pub struct PlanePanelState {
    pub selected_kind: ControllerKind,
    pub config_path: String,
}

impl Default for PlanePanelState {
    fn default() -> Self {
        Self {
            selected_kind: ControllerKind::LevelHold,
            config_path: DEFAULT_CONFIG.to_string(),
        }
    }
}

/// Position + velocity for a plane placed ahead of the camera, flying level
/// along the camera's horizontal heading and never below the altitude floor.
fn spawn_pose_ahead(camera: &Transform) -> (Vec3, Vec3) {
    let fwd = camera.forward().as_vec3();
    let horiz = Vec3::new(fwd.x, 0.0, fwd.z).normalize_or_zero();
    // Degenerate case: camera looking straight up/down → default to world +X.
    let dir = if horiz.length_squared() > 0.0 {
        horiz
    } else {
        Vec3::X
    };
    let mut pos = camera.translation + dir * SPAWN_AHEAD_M;
    pos.y = pos.y.max(MIN_SPAWN_ALT_M);
    (pos, dir * SPAWN_SPEED_MS)
}

/// Draws the bottom-left "Planes" panel: a live roster with Remove buttons and a
/// spawn form. Buttons fire lifecycle commands; no lifecycle logic lives here.
pub fn draw_plane_panel(
    map: Res<MapState>,
    mut contexts: EguiContexts,
    mut state: ResMut<PlanePanelState>,
    mut pending: ResMut<crate::ui::file_load::PendingLoads>,
    mut commands: Commands,
    planes: Query<(Entity, &PlaneIndex, &ControllerKind, &FlightState)>,
    camera: Query<&Transform, With<Camera3d>>,
) {
    // The full-screen map replaces the 3D view (and this panel) while open.
    if map.open {
        return;
    }
    let Ok(ctx) = contexts.ctx_mut() else { return };

    egui::Window::new("Planes")
        .anchor(egui::Align2::LEFT_BOTTOM, egui::vec2(10.0, -10.0))
        .collapsible(true)
        .show(ctx, |ui| {
            let mut rows: Vec<(Entity, u32, ControllerKind, f32, f32)> = planes
                .iter()
                .map(|(e, idx, kind, st)| (e, idx.0, *kind, st.altitude, st.airspeed))
                .collect();
            rows.sort_by_key(|r| r.1);
            for (entity, idx, kind, alt, spd) in rows {
                ui.horizontal(|ui| {
                    ui.label(format!("#{idx}  {}  {alt:.0} m  {spd:.0} m/s", kind.name()));
                    if ui.small_button("Remove").clicked() {
                        commands.trigger(RemovePlaneCommand(entity));
                    }
                });
            }

            ui.separator();

            egui::ComboBox::from_label("Kind")
                .selected_text(state.selected_kind.name())
                .show_ui(ui, |ui| {
                    for kind in SPAWNABLE_KINDS {
                        ui.selectable_value(&mut state.selected_kind, *kind, kind.name());
                    }
                });
            ui.horizontal(|ui| {
                ui.label("Config:");
                ui.text_edit_singleline(&mut state.config_path);
                if ui.button("Browse…").clicked() {
                    crate::ui::file_load::spawn_plane_config_load(&mut pending);
                }
            });
            if ui.button("Spawn ahead of camera").clicked() {
                if let Ok(cam) = camera.single() {
                    let (pos, vel) = spawn_pose_ahead(cam);
                    commands.trigger(SpawnPlaneCommand {
                        spec: SpawnSpec {
                            position: Some(pos),
                            velocity: Some(vel),
                            ..Default::default()
                        },
                        kind: state.selected_kind,
                        config_path: state.config_path.clone(),
                    });
                }
            }
        });
}

/// `N` spawns a default plane ahead of the camera; `Delete` removes the followed
/// plane. Both are suppressed while egui has keyboard focus (e.g. editing the
/// config-path field) so typing doesn't trigger them.
pub fn plane_lifecycle_hotkeys(
    keys: Res<ButtonInput<KeyCode>>,
    mut contexts: EguiContexts,
    mut commands: Commands,
    mode: Res<CameraMode>,
    camera: Query<&Transform, With<Camera3d>>,
) {
    if let Ok(ctx) = contexts.ctx_mut() {
        if ctx.wants_keyboard_input() {
            return;
        }
    }

    if keys.just_pressed(KeyCode::KeyN) {
        if let Ok(cam) = camera.single() {
            let (pos, vel) = spawn_pose_ahead(cam);
            commands.trigger(SpawnPlaneCommand::at(pos, vel, ControllerKind::LevelHold));
        }
    }
    if keys.just_pressed(KeyCode::Delete) {
        if let CameraMode::Follow(entity) = *mode {
            commands.trigger(RemovePlaneCommand(entity));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spawn_pose_is_ahead_level_and_at_camera_altitude() {
        // Camera at 600 m looking along world +X.
        let cam = Transform::from_xyz(0.0, 600.0, 0.0).looking_to(Vec3::X, Vec3::Y);
        let (pos, vel) = spawn_pose_ahead(&cam);
        assert!(pos.x > 300.0, "plane should be ahead along +X, got {pos:?}");
        assert!((pos.y - 600.0).abs() < 1.0, "stays at camera altitude");
        assert!(vel.y.abs() < 1e-3, "spawns in level flight");
        assert!((vel.length() - SPAWN_SPEED_MS).abs() < 1e-3, "spawn speed");
    }

    #[test]
    fn spawn_pose_respects_altitude_floor() {
        let cam = Transform::from_xyz(0.0, 100.0, 0.0).looking_to(Vec3::X, Vec3::Y);
        let (pos, _) = spawn_pose_ahead(&cam);
        assert!(
            pos.y >= MIN_SPAWN_ALT_M,
            "never below the floor, got {}",
            pos.y
        );
    }

    #[test]
    fn spawnable_kinds_include_pid_baseline() {
        for k in [
            ControllerKind::Manual,
            ControllerKind::LevelHold,
            ControllerKind::Ascent,
            ControllerKind::HeadingHold,
            ControllerKind::Orbit,
        ] {
            assert!(
                SPAWNABLE_KINDS.contains(&k),
                "{} should be spawnable",
                k.name()
            );
        }
    }

    #[cfg(feature = "inference")]
    #[test]
    fn spawnable_kinds_include_rl_when_ml_enabled() {
        for k in [
            ControllerKind::RlLevelHold,
            ControllerKind::RlOrbit,
            ControllerKind::RlOrbitResidual,
            ControllerKind::RlLstmOrbit,
        ] {
            assert!(
                SPAWNABLE_KINDS.contains(&k),
                "{} should be spawnable when ML is enabled",
                k.name()
            );
        }
    }

    #[test]
    fn spawn_pose_handles_vertical_camera() {
        // Looking straight down: horizontal heading is degenerate → default +X.
        let cam = Transform::from_xyz(0.0, 800.0, 0.0).looking_to(Vec3::NEG_Y, Vec3::X);
        let (_, vel) = spawn_pose_ahead(&cam);
        assert!(
            vel.y.abs() < 1e-3,
            "still level even with a vertical camera"
        );
        assert!(vel.length() > 0.0, "non-zero spawn velocity");
    }
}
