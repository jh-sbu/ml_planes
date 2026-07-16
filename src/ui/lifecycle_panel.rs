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
#[cfg(not(feature = "net"))]
use crate::environment::{RemovePlaneCommand, SpawnPlaneCommand};
use crate::plane::{FlightState, PlaneId, PlaneIndex};
use crate::training::SpawnSpec;
use crate::ui::map::MapState;

#[cfg(feature = "net")]
use crate::net::{RemovePlaneNetCommand, SpawnPlaneNetCommand};
#[cfg(feature = "net")]
use bevy_replicon::prelude::ClientTriggerExt;

/// Spawn a plane: the local-sim build fires the headless [`SpawnPlaneCommand`]
/// observer; the networked client sends a [`SpawnPlaneNetCommand`] so the server
/// (authoritative) spawns the replicated plane.
#[cfg(not(feature = "net"))]
fn send_spawn(commands: &mut Commands, spec: SpawnSpec, kind: ControllerKind, config_path: String) {
    commands.trigger(SpawnPlaneCommand {
        spec,
        kind,
        config_path,
    });
}

#[cfg(feature = "net")]
fn send_spawn(commands: &mut Commands, spec: SpawnSpec, kind: ControllerKind, config_path: String) {
    commands.client_trigger(SpawnPlaneNetCommand {
        config_path,
        kind,
        spec,
    });
}

/// Remove a plane by entity (local) / id (networked).
#[cfg(not(feature = "net"))]
fn send_remove(commands: &mut Commands, entity: Entity, _plane: PlaneId) {
    commands.trigger(RemovePlaneCommand(entity));
}

#[cfg(feature = "net")]
fn send_remove(commands: &mut Commands, _entity: Entity, plane: PlaneId) {
    commands.client_trigger(RemovePlaneNetCommand { plane });
}

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
///
/// Note: 100 m/s is below the sustainable level-flight envelope of a *loaded*
/// heavy jet (the 218 t cargo jet holds level only at ~110-120 m/s and above — see
/// the validated envelope in `assets/planes/cargo_jet.tuning.ron`). A cargo/tanker
/// spawned here will trim into a slow controlled descent; this is physical, not a
/// tuning bug. Lighter airframes (generic/business jet) hold fine at 100 m/s.
const SPAWN_SPEED_MS: f32 = 100.0;

/// Fraction of the screen height the roster list may occupy, and the absolute
/// bounds it is clamped to. 0.35 leaves the panel (roster + spawn form) clear of
/// the ~250 px top-left "Flight Data" HUD even on a short 720 px window.
const ROSTER_SCREEN_FRAC: f32 = 0.35;
/// Keeps a few rows visible on a short window rather than collapsing to nothing.
const ROSTER_MIN_H: f32 = 80.0;
/// Hard ceiling: the panel is anchored bottom-left and grows upward, so an
/// unbounded roster (large scenarios) would run off-screen and cover the
/// top-left "Flight Data" HUD.
const ROSTER_MAX_H: f32 = 320.0;

/// Max height of the scrollable roster list for a given screen height. The
/// spawn form is drawn outside the scroll area, so the window as a whole stays
/// somewhat taller than this.
fn roster_max_height(screen_height: f32) -> f32 {
    (screen_height * ROSTER_SCREEN_FRAC).clamp(ROSTER_MIN_H, ROSTER_MAX_H)
}

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

/// One roster line: a live plane as the panel needs to display it.
struct PlaneRow {
    entity: Entity,
    index: u32,
    kind: ControllerKind,
    altitude: f32,
    airspeed: f32,
    plane: PlaneId,
}

/// What the user asked for this frame. [`plane_panel_ui`] only reports these;
/// [`draw_plane_panel`] turns them into lifecycle commands, which keeps the
/// layout testable without an ECS world.
enum PanelAction {
    Remove(Entity, PlaneId),
    Spawn,
    Browse,
}

/// Draws the panel and reports the window rect plus any requested actions.
/// Pure UI: no `Commands`, so it can be laid out in a headless egui context.
fn plane_panel_ui(
    ctx: &egui::Context,
    rows: &[PlaneRow],
    state: &mut PlanePanelState,
) -> (Option<egui::Rect>, Vec<PanelAction>) {
    let roster_h = roster_max_height(ctx.content_rect().height());
    let mut actions = Vec::new();

    let response = egui::Window::new("Planes")
        .anchor(egui::Align2::LEFT_BOTTOM, egui::vec2(10.0, -10.0))
        .collapsible(true)
        .show(ctx, |ui| {
            // Capped + scrollable: the window is bottom-anchored and grows
            // upward, so an unbounded roster would cover the top-left HUD.
            egui::ScrollArea::vertical()
                .max_height(roster_h)
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    for row in rows {
                        ui.horizontal(|ui| {
                            ui.label(format!(
                                "#{}  {}  {:.0} m  {:.0} m/s",
                                row.index,
                                row.kind.name(),
                                row.altitude,
                                row.airspeed
                            ));
                            if ui.small_button("Remove").clicked() {
                                actions.push(PanelAction::Remove(row.entity, row.plane));
                            }
                        });
                    }
                });

            ui.separator();

            // Outside the scroll area, so the spawn form stays reachable no
            // matter how many planes are live.
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
                    actions.push(PanelAction::Browse);
                }
            });
            if ui.button("Spawn ahead of camera").clicked() {
                actions.push(PanelAction::Spawn);
            }
        });

    (response.map(|r| r.response.rect), actions)
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
    planes: Query<(Entity, &PlaneIndex, &ControllerKind, &FlightState, &PlaneId)>,
    camera: Query<&Transform, With<Camera3d>>,
) {
    // The full-screen map replaces the 3D view (and this panel) while open.
    if map.open {
        return;
    }
    let Ok(ctx) = contexts.ctx_mut() else { return };

    let mut rows: Vec<PlaneRow> = planes
        .iter()
        .map(|(entity, idx, kind, st, plane)| PlaneRow {
            entity,
            index: idx.0,
            kind: *kind,
            altitude: st.altitude,
            airspeed: st.airspeed,
            plane: *plane,
        })
        .collect();
    rows.sort_by_key(|r| r.index);

    let (_, actions) = plane_panel_ui(ctx, &rows, &mut state);

    for action in actions {
        match action {
            PanelAction::Remove(entity, plane) => send_remove(&mut commands, entity, plane),
            PanelAction::Browse => crate::ui::file_load::spawn_plane_config_load(&mut pending),
            PanelAction::Spawn => {
                if let Ok(cam) = camera.single() {
                    let (pos, vel) = spawn_pose_ahead(cam);
                    send_spawn(
                        &mut commands,
                        SpawnSpec {
                            position: Some(pos),
                            velocity: Some(vel),
                            ..Default::default()
                        },
                        state.selected_kind,
                        state.config_path.clone(),
                    );
                }
            }
        }
    }
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
    // Only the networked Delete path needs to resolve the followed entity → id.
    #[cfg(feature = "net")] planes: Query<&PlaneId>,
) {
    if let Ok(ctx) = contexts.ctx_mut() {
        if ctx.wants_keyboard_input() {
            return;
        }
    }

    if keys.just_pressed(KeyCode::KeyN) {
        if let Ok(cam) = camera.single() {
            let (pos, vel) = spawn_pose_ahead(cam);
            send_spawn(
                &mut commands,
                SpawnSpec {
                    position: Some(pos),
                    velocity: Some(vel),
                    ..Default::default()
                },
                ControllerKind::LevelHold,
                DEFAULT_CONFIG.to_string(),
            );
        }
    }
    if keys.just_pressed(KeyCode::Delete) {
        if let CameraMode::Follow(entity) = *mode {
            #[cfg(feature = "net")]
            if let Ok(plane) = planes.get(entity) {
                send_remove(&mut commands, entity, *plane);
            }
            #[cfg(not(feature = "net"))]
            send_remove(&mut commands, entity, PlaneId(0));
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

    /// Lays the real panel out in a headless egui context and returns the
    /// window's rect, so the anchored bottom-left growth can be asserted
    /// without a GPU or a window manager.
    fn layout_panel(screen: egui::Vec2, plane_count: u32) -> egui::Rect {
        let ctx = egui::Context::default();
        let mut state = PlanePanelState::default();
        let rows: Vec<PlaneRow> = (0..plane_count)
            .map(|i| PlaneRow {
                entity: Entity::PLACEHOLDER,
                index: i,
                kind: ControllerKind::LevelHold,
                altitude: 1000.0,
                airspeed: 100.0,
                plane: PlaneId(i),
            })
            .collect();

        let mut rect = None;
        // egui settles anchored/auto-sized windows over a couple of frames.
        for _ in 0..3 {
            let input = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(egui::pos2(0.0, 0.0), screen)),
                ..Default::default()
            };
            let _ = ctx.run(input, |ctx| {
                rect = plane_panel_ui(ctx, &rows, &mut state).0;
            });
        }
        rect.expect("panel window should be laid out")
    }

    /// The bug: with a large scenario the bottom-anchored roster grew upward off
    /// the top of the screen and covered the top-left "Flight Data" HUD.
    #[test]
    fn panel_with_a_hundred_planes_stays_on_screen_and_clear_of_the_hud() {
        let screen = egui::vec2(1280.0, 720.0);
        let rect = layout_panel(screen, 100);

        assert!(
            rect.top() > 0.0,
            "panel must not run off the top of the screen, got top={}",
            rect.top()
        );
        // The HUD is anchored LEFT_TOP; keep the panel well clear of that band.
        assert!(
            rect.top() > screen.y * 0.4,
            "panel must stay clear of the top-left HUD, got top={} on a {}px screen",
            rect.top(),
            screen.y
        );
        assert!(
            rect.height() < screen.y,
            "panel must be shorter than the screen, got {}",
            rect.height()
        );
    }

    #[test]
    fn panel_height_is_capped_rather_than_growing_with_plane_count() {
        let screen = egui::vec2(1280.0, 720.0);
        let small = layout_panel(screen, 4);
        let huge = layout_panel(screen, 100);
        assert!(
            huge.height() <= small.height() + ROSTER_MAX_H,
            "100 planes ({}) must not grow unboundedly past 4 planes ({})",
            huge.height(),
            small.height()
        );
        assert!(
            huge.height() <= roster_max_height(screen.y) + 200.0,
            "panel height should stay near the roster cap plus the spawn form, got {}",
            huge.height()
        );
    }

    /// A handful of planes should still render a compact panel rather than
    /// reserving the full capped height.
    #[test]
    fn panel_shrinks_to_fit_a_small_roster() {
        let screen = egui::vec2(1280.0, 720.0);
        let small = layout_panel(screen, 2);
        let huge = layout_panel(screen, 100);
        assert!(
            small.height() < huge.height(),
            "2 planes ({}) should be shorter than 100 planes ({})",
            small.height(),
            huge.height()
        );
    }

    #[test]
    fn roster_height_capped_on_a_tall_screen() {
        let h = roster_max_height(1080.0);
        assert!(
            h <= ROSTER_MAX_H,
            "roster must stay capped so it cannot reach the top-left HUD, got {h}"
        );
    }

    #[test]
    fn roster_height_never_collapses_on_a_short_screen() {
        let h = roster_max_height(200.0);
        assert!(
            h >= ROSTER_MIN_H,
            "roster must stay usable on a short screen, got {h}"
        );
    }

    #[test]
    fn roster_height_grows_with_screen_but_leaves_room_for_the_spawn_form() {
        assert!(
            roster_max_height(1440.0) >= roster_max_height(720.0),
            "taller screens allow at least as many visible rows"
        );
        for screen in [400.0, 720.0, 1080.0, 1440.0, 2160.0] {
            let h = roster_max_height(screen);
            assert!(
                h < screen,
                "roster must leave room for the spawn form at screen height {screen}, got {h}"
            );
        }
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
