//! Main menu + scenario selection (visual mode only).
//!
//! The app boots into [`AppState::MainMenu`] instead of dropping straight into a
//! hardcoded scene. "Start Scenario" opens [`AppState::ScenarioSelect`], which
//! lists the `*.scenario.ron` files under `assets/scenarios/`; choosing one
//! transitions to [`AppState::InGame`], where [`spawn_selected_scenario`] spawns
//! every plane via [`spawn_resolved_scenario`]. `Esc` returns to the menu, and
//! [`despawn_in_game_planes`] tears the scene down so a different scenario starts
//! clean.
//!
//! State gating uses Bevy `States` + `run_if(in_state(..))`. The gameplay HUD/sim
//! systems (registered in [`crate::ui::plugin`] and `main.rs`) are gated to
//! `InGame` so the menu is free of in-flight UI.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts, EguiPrimaryContextPass};

// Local scenario spawning is compiled out when networking is on: the dedicated
// server owns scenario spawning and the client renders the replicated result.
#[cfg(not(feature = "net"))]
use std::path::Path;

use crate::camera::CameraMode;
#[cfg(not(feature = "net"))]
use crate::environment::spawn_resolved_scenario;
#[cfg(not(feature = "net"))]
use crate::plane::NextPlaneId;
use crate::plane::PlaneId;
#[cfg(not(feature = "net"))]
use crate::scenario::Scenario;
use crate::ui::map::MapState;
use crate::ui::notifications::Notifications;

/// Top-level screen the app is currently showing.
#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AppState {
    /// Title screen with Start Scenario / Train / Quit.
    #[default]
    MainMenu,
    /// Pick a `.scenario.ron` to fly.
    ScenarioSelect,
    /// A scenario is loaded and flying.
    InGame,
}

/// One selectable scenario discovered under `assets/scenarios/`.
#[derive(Debug, Clone)]
pub struct ScenarioEntry {
    /// Display name (filename with the `.scenario.ron` suffix stripped).
    pub name: String,
    /// Path passed to [`Scenario::from_path`].
    pub path: String,
}

/// The scenarios offered on the selection screen, rescanned on entry.
#[derive(Resource, Default)]
pub struct ScenarioList(pub Vec<ScenarioEntry>);

/// The scenario chosen on the selection screen, consumed on `OnEnter(InGame)`.
#[derive(Resource, Default)]
pub struct SelectedScenario(pub Option<String>);

const SCENARIO_DIR: &str = "assets/scenarios";
const SCENARIO_SUFFIX: &str = ".scenario.ron";

/// Rescan `assets/scenarios/` for `*.scenario.ron`, sorted with `default` first.
/// On wasm (no filesystem listing) the list falls back to the embedded default.
fn scan_scenarios(mut list: ResMut<ScenarioList>) {
    list.0 = discover_scenarios();
}

#[cfg(not(target_arch = "wasm32"))]
fn discover_scenarios() -> Vec<ScenarioEntry> {
    let mut entries: Vec<ScenarioEntry> = match std::fs::read_dir(SCENARIO_DIR) {
        Ok(dir) => dir
            .flatten()
            .filter_map(|f| {
                let file = f.file_name().into_string().ok()?;
                let name = file.strip_suffix(SCENARIO_SUFFIX)?.to_string();
                Some(ScenarioEntry {
                    path: format!("{SCENARIO_DIR}/{file}"),
                    name,
                })
            })
            .collect(),
        Err(e) => {
            warn!("cannot list {SCENARIO_DIR}: {e}");
            Vec::new()
        }
    };
    // `default` first, then alphabetical, so the familiar demo is the top choice.
    entries.sort_by(|a, b| match (a.name == "default", b.name == "default") {
        (true, false) => std::cmp::Ordering::Less,
        (false, true) => std::cmp::Ordering::Greater,
        _ => a.name.cmp(&b.name),
    });
    entries
}

#[cfg(target_arch = "wasm32")]
fn discover_scenarios() -> Vec<ScenarioEntry> {
    vec![ScenarioEntry {
        name: "default".to_string(),
        path: format!("{SCENARIO_DIR}/default{SCENARIO_SUFFIX}"),
    }]
}

/// Title screen: Start Scenario, an optional Train placeholder, and Quit.
fn draw_main_menu(
    mut contexts: EguiContexts,
    mut next: ResMut<NextState<AppState>>,
    mut notes: ResMut<Notifications>,
    mut exit: MessageWriter<AppExit>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.vertical_centered(|ui| {
            ui.add_space(120.0);
            ui.heading("ml_planes");
            ui.add_space(40.0);
            if ui.button("Start Scenario").clicked() {
                next.set(AppState::ScenarioSelect);
            }
            ui.add_space(8.0);
            #[cfg(feature = "training")]
            {
                if ui.button("Train").clicked() {
                    // Placeholder: training is not wired into the visual app yet.
                    notes.push("Training is not available from the menu yet.");
                }
                ui.add_space(8.0);
            }
            // Suppress the unused-variable warning when the Train button (the
            // only `notes` user here) is compiled out.
            let _ = &mut notes;
            if ui.button("Quit").clicked() {
                exit.write(AppExit::Success);
            }
        });
    });
}

/// Scenario picker: one button per discovered scenario, plus Back.
fn draw_scenario_select(
    mut contexts: EguiContexts,
    list: Res<ScenarioList>,
    mut selected: ResMut<SelectedScenario>,
    mut next: ResMut<NextState<AppState>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.vertical_centered(|ui| {
            ui.add_space(80.0);
            ui.heading("Select Scenario");
            ui.add_space(24.0);
            if list.0.is_empty() {
                ui.label("No scenarios found under assets/scenarios/.");
            }
            for entry in &list.0 {
                if ui.button(&entry.name).clicked() {
                    selected.0 = Some(entry.path.clone());
                    next.set(AppState::InGame);
                }
                ui.add_space(4.0);
            }
            ui.add_space(24.0);
            if ui.button("Back").clicked() {
                next.set(AppState::MainMenu);
            }
        });
    });
}

/// `OnEnter(InGame)`: load + resolve + spawn the chosen scenario. Planes whose
/// controller can't be built (e.g. a missing RL model) are skipped with a HUD
/// notification rather than aborting the whole scene.
#[cfg(not(feature = "net"))]
fn spawn_selected_scenario(
    mut commands: Commands,
    mut ids: ResMut<NextPlaneId>,
    asset_server: Res<AssetServer>,
    selected: Res<SelectedScenario>,
    mut notes: ResMut<Notifications>,
) {
    let Some(path) = selected.0.as_deref() else {
        notes.push("No scenario selected.");
        return;
    };
    let resolved = match Scenario::from_path(Path::new(path)).and_then(|s| s.resolve()) {
        Ok(r) => r,
        Err(e) => {
            notes.push(format!("Scenario '{path}' failed: {e}"));
            return;
        }
    };
    let result = spawn_resolved_scenario(&mut commands, &mut ids, &asset_server, &resolved);
    for msg in result.skipped {
        notes.push(msg);
    }
}

/// `OnExit(InGame)`: despawn every plane and drop the camera back to free-look so
/// returning to the menu (and picking another scenario) starts from a clean slate.
fn despawn_in_game_planes(
    mut commands: Commands,
    planes: Query<Entity, With<PlaneId>>,
    mut camera_mode: ResMut<CameraMode>,
) {
    for entity in &planes {
        commands.entity(entity).despawn();
    }
    *camera_mode = CameraMode::FreeLook;
}

/// In-game overlay: a small "Menu" window (top-right) with a single **Main Menu**
/// button for players who don't know the `Esc` hotkey. Setting the state triggers
/// the same `OnExit(InGame)` teardown as `Esc`. Hidden while the full-screen map is
/// open (mirrors [`crate::ui::lifecycle_panel::draw_plane_panel`]).
fn draw_in_game_menu(
    map: Res<MapState>,
    mut contexts: EguiContexts,
    mut next: ResMut<NextState<AppState>>,
) {
    if map.open {
        return;
    }
    let Ok(ctx) = contexts.ctx_mut() else { return };
    egui::Window::new("Menu")
        .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-10.0, 10.0))
        .collapsible(true)
        .resizable(false)
        .show(ctx, |ui| {
            if ui.button("Main Menu").clicked() {
                next.set(AppState::MainMenu);
            }
        });
}

/// `Esc` while flying returns to the main menu (unless egui has keyboard focus,
/// e.g. a text field). `M` is already the map toggle, so Esc is used here.
fn menu_escape_to_main(
    keys: Res<ButtonInput<KeyCode>>,
    mut contexts: EguiContexts,
    mut next: ResMut<NextState<AppState>>,
) {
    let wants_kb = contexts
        .ctx_mut()
        .map(|c| c.wants_keyboard_input())
        .unwrap_or(false);
    if !wants_kb && keys.just_pressed(KeyCode::Escape) {
        next.set(AppState::MainMenu);
    }
}

/// Registers the menu state machine, its resources, and the menu/scenario UI plus
/// the scene spawn/teardown hooks. Gameplay systems are gated to `InGame`
/// elsewhere (see [`crate::ui::plugin`] and `main.rs`).
pub struct MenuPlugin;

impl Plugin for MenuPlugin {
    fn build(&self, app: &mut App) {
        app.init_state::<AppState>()
            .init_resource::<ScenarioList>()
            .init_resource::<SelectedScenario>()
            .add_systems(OnEnter(AppState::MainMenu), scan_scenarios)
            .add_systems(OnExit(AppState::InGame), despawn_in_game_planes)
            .add_systems(
                EguiPrimaryContextPass,
                (
                    draw_main_menu.run_if(in_state(AppState::MainMenu)),
                    draw_scenario_select.run_if(in_state(AppState::ScenarioSelect)),
                    draw_in_game_menu.run_if(in_state(AppState::InGame)),
                ),
            )
            .add_systems(
                Update,
                menu_escape_to_main.run_if(in_state(AppState::InGame)),
            );

        // Spawn the chosen scenario locally only in non-networked builds (WASM /
        // local-sim). With `net`, planes come from the authoritative server, so the
        // client must not spawn its own (see `plans/client_server.md` Phase 4).
        #[cfg(not(feature = "net"))]
        app.add_systems(OnEnter(AppState::InGame), spawn_selected_scenario);
    }
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests {
    use super::*;

    #[test]
    fn discover_scenarios_lists_default_first() {
        let entries = discover_scenarios();
        assert!(
            !entries.is_empty(),
            "assets/scenarios should contain at least the default scenario"
        );
        assert_eq!(
            entries[0].name, "default",
            "the default scenario is offered first"
        );
        assert!(
            entries.iter().any(|e| e.name == "orbit"),
            "other shipped scenarios are discovered too"
        );
    }
}
