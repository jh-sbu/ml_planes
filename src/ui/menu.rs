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

// Networked client flow (Start New Server / Connect to Server). The dedicated
// server owns physics + spawning; the client only opens a transport and renders
// the replicated result (see `plans/client_server.md` Phase 5).
#[cfg(feature = "net")]
use std::net::{Ipv4Addr, SocketAddr};
#[cfg(feature = "net")]
use std::process::Child;

#[cfg(feature = "net")]
use bevy_replicon::prelude::ClientState;
#[cfg(feature = "net")]
use bevy_replicon_renet::netcode::NetcodeClientTransport;
#[cfg(feature = "net")]
use bevy_replicon_renet::RenetClient;

#[cfg(feature = "net")]
use crate::net::{start_renet_client, ConnectTarget, DEFAULT_PORT};

/// Top-level screen the app is currently showing.
#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AppState {
    /// Title screen. Non-net: Start Scenario / Train / Quit. Net: Start New
    /// Server / Connect to Server / Quit.
    #[default]
    MainMenu,
    /// Pick a `.scenario.ron`. Non-net: to fly locally. Net: to host on a freshly
    /// launched local server.
    ScenarioSelect,
    /// Net only: enter a `host:port` to join an existing server.
    #[cfg(feature = "net")]
    ConnectEntry,
    /// Net only: the client transport is open; waiting for
    /// [`ClientState::Connected`] (or a timeout back to [`AppState::MainMenu`]).
    #[cfg(feature = "net")]
    Connecting,
    /// A scenario is flying. Non-net: locally simulated. Net: connected and
    /// rendering replicated state.
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

/// Net only: a local `ml_planes_server` we launched (Start New Server). `None` for
/// a remote join — only a server we spawned is killed on teardown.
#[cfg(feature = "net")]
#[derive(Resource, Default)]
pub struct LocalServer(pub Option<Child>);

/// Net only: the `host:port` text buffer on the Connect-to-Server screen.
#[cfg(feature = "net")]
#[derive(Resource)]
pub struct ConnectForm {
    pub address: String,
}

#[cfg(feature = "net")]
impl Default for ConnectForm {
    fn default() -> Self {
        Self {
            address: format!("127.0.0.1:{DEFAULT_PORT}"),
        }
    }
}

/// Net only: wall-clock deadline (`Time::elapsed_secs_f64`) by which the client
/// must reach [`ClientState::Connected`], else the attempt fails back to the menu.
#[cfg(feature = "net")]
#[derive(Resource, Default)]
pub struct ConnectDeadline(pub f64);

/// How long to wait for a connection before giving up (seconds).
#[cfg(feature = "net")]
const CONNECT_TIMEOUT_SECS: f64 = 5.0;

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

/// Title screen (non-net / local-sim build): Start Scenario, an optional Train
/// placeholder, and Quit.
#[cfg(not(feature = "net"))]
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

/// Title screen (networked client): Start New Server (host a fresh local server)
/// / Connect to Server (join a host:port), an optional Train placeholder, and Quit.
#[cfg(feature = "net")]
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
            if ui.button("Start New Server").clicked() {
                next.set(AppState::ScenarioSelect);
            }
            ui.add_space(8.0);
            if ui.button("Connect to Server").clicked() {
                next.set(AppState::ConnectEntry);
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

/// Scenario picker (non-net / local-sim): one button per discovered scenario, plus
/// Back. Choosing one flies it locally.
#[cfg(not(feature = "net"))]
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

/// Scenario picker (Start New Server): choosing a scenario launches a local
/// `ml_planes_server` child process hosting it, then connects this client to it.
#[cfg(feature = "net")]
fn draw_scenario_select(
    mut contexts: EguiContexts,
    list: Res<ScenarioList>,
    mut selected: ResMut<SelectedScenario>,
    mut next: ResMut<NextState<AppState>>,
    mut commands: Commands,
    mut local_server: ResMut<LocalServer>,
    mut notes: ResMut<Notifications>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    let mut chosen: Option<String> = None;
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.vertical_centered(|ui| {
            ui.add_space(80.0);
            ui.heading("Start New Server");
            ui.add_space(24.0);
            if list.0.is_empty() {
                ui.label("No scenarios found under assets/scenarios/.");
            }
            for entry in &list.0 {
                if ui.button(&entry.name).clicked() {
                    chosen = Some(entry.path.clone());
                }
                ui.add_space(4.0);
            }
            ui.add_space(24.0);
            if ui.button("Back").clicked() {
                next.set(AppState::MainMenu);
            }
        });
    });

    let Some(path) = chosen else { return };
    match launch_local_server(&path, DEFAULT_PORT) {
        Ok(child) => {
            local_server.0 = Some(child);
            selected.0 = Some(path);
            let addr = SocketAddr::from((Ipv4Addr::LOCALHOST, DEFAULT_PORT));
            commands.insert_resource(ConnectTarget(addr));
            next.set(AppState::Connecting);
        }
        Err(e) => {
            notes.push(format!("Could not start server: {e}"));
        }
    }
}

/// Spawn an `ml_planes_server` child process hosting `scenario_path` on `port`.
/// Resolves the server binary next to the current executable so the installed/
/// `target/<profile>` layout is used (the child inherits this process's CWD, so the
/// `assets/...` scenario path resolves the same way the client sees it).
#[cfg(feature = "net")]
fn launch_local_server(scenario_path: &str, port: u16) -> std::io::Result<Child> {
    let exe = std::env::current_exe()?
        .with_file_name(format!("ml_planes_server{}", std::env::consts::EXE_SUFFIX));
    std::process::Command::new(exe)
        .arg("--scenario")
        .arg(scenario_path)
        .arg("--port")
        .arg(port.to_string())
        .spawn()
}

/// Parse a `host:port` string into a [`SocketAddr`], resolving DNS names. Returns
/// the first resolved address, or `None` if the input is empty/unparseable.
#[cfg(feature = "net")]
fn parse_addr(raw: &str) -> Option<SocketAddr> {
    use std::net::ToSocketAddrs;
    raw.trim().to_socket_addrs().ok()?.next()
}

/// Connect-to-Server screen: a `host:port` field plus Connect / Back. Connect opens
/// a transport to the entered address and transitions to [`AppState::Connecting`].
#[cfg(feature = "net")]
fn draw_connect_entry(
    mut contexts: EguiContexts,
    mut form: ResMut<ConnectForm>,
    mut next: ResMut<NextState<AppState>>,
    mut commands: Commands,
    mut notes: ResMut<Notifications>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.vertical_centered(|ui| {
            ui.add_space(100.0);
            ui.heading("Connect to Server");
            ui.add_space(24.0);
            ui.label("Address (host:port)");
            ui.text_edit_singleline(&mut form.address);
            ui.add_space(16.0);
            if ui.button("Connect").clicked() {
                match parse_addr(&form.address) {
                    Some(addr) => {
                        commands.insert_resource(ConnectTarget(addr));
                        next.set(AppState::Connecting);
                    }
                    None => notes.push(format!("Invalid address: '{}'", form.address)),
                }
            }
            ui.add_space(8.0);
            if ui.button("Back").clicked() {
                next.set(AppState::MainMenu);
            }
        });
    });
}

/// `OnEnter(Connecting)`: arm the connection timeout. The transport itself is opened
/// by [`start_renet_client`] (registered alongside this on the same transition).
#[cfg(feature = "net")]
fn arm_connect_deadline(time: Res<Time>, mut deadline: ResMut<ConnectDeadline>) {
    deadline.0 = time.elapsed_secs_f64() + CONNECT_TIMEOUT_SECS;
}

/// Connecting screen: a status line plus Cancel. Cancel tears the attempt down and
/// returns to the main menu.
#[cfg(feature = "net")]
fn draw_connecting(
    mut contexts: EguiContexts,
    target: Option<Res<ConnectTarget>>,
    mut next: ResMut<NextState<AppState>>,
    mut commands: Commands,
    mut local_server: ResMut<LocalServer>,
    client: Option<ResMut<RenetClient>>,
) {
    let Ok(ctx) = contexts.ctx_mut() else { return };
    let addr = target.map(|t| t.0.to_string()).unwrap_or_default();
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.vertical_centered(|ui| {
            ui.add_space(140.0);
            ui.heading("Connecting…");
            ui.add_space(12.0);
            ui.label(addr);
            ui.add_space(24.0);
            if ui.button("Cancel").clicked() {
                teardown_connection(&mut commands, &mut local_server, client);
                next.set(AppState::MainMenu);
            }
        });
    });
}

/// `Update` while [`AppState::Connecting`]: advance to [`AppState::InGame`] once
/// replicon reports [`ClientState::Connected`], or fail back to the menu (tearing
/// the attempt down) when the deadline passes.
#[cfg(feature = "net")]
fn poll_connecting(
    time: Res<Time>,
    deadline: Res<ConnectDeadline>,
    client_state: Res<State<ClientState>>,
    mut next: ResMut<NextState<AppState>>,
    mut commands: Commands,
    mut local_server: ResMut<LocalServer>,
    mut notes: ResMut<Notifications>,
    client: Option<ResMut<RenetClient>>,
) {
    if *client_state.get() == ClientState::Connected {
        next.set(AppState::InGame);
        return;
    }
    if time.elapsed_secs_f64() > deadline.0 {
        teardown_connection(&mut commands, &mut local_server, client);
        notes.push("Connection failed: server did not respond.");
        next.set(AppState::MainMenu);
    }
}

/// Disconnect the client transport and kill any local server we launched. Used on
/// leaving the game and on cancelling/timing-out a connection attempt.
#[cfg(feature = "net")]
fn teardown_connection(
    commands: &mut Commands,
    local_server: &mut LocalServer,
    client: Option<ResMut<RenetClient>>,
) {
    if let Some(mut client) = client {
        client.disconnect();
        commands.remove_resource::<RenetClient>();
        commands.remove_resource::<NetcodeClientTransport>();
    }
    if let Some(mut child) = local_server.0.take() {
        let _ = child.kill();
        let _ = child.wait();
    }
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

/// `OnExit(InGame)` (non-net): despawn every plane and drop the camera back to
/// free-look so returning to the menu starts from a clean slate.
#[cfg(not(feature = "net"))]
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

/// `OnExit(InGame)` (net): disconnect the client transport, kill any local server we
/// launched, despawn the replicated planes, and reset the camera. Replicon also
/// removes replicated entities on disconnect, but the explicit despawn keeps the
/// client view clean regardless of timing.
#[cfg(feature = "net")]
fn despawn_in_game_planes(
    mut commands: Commands,
    planes: Query<Entity, With<PlaneId>>,
    mut camera_mode: ResMut<CameraMode>,
    mut local_server: ResMut<LocalServer>,
    client: Option<ResMut<RenetClient>>,
) {
    teardown_connection(&mut commands, &mut local_server, client);
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

        // Networked connection flow (Start New Server / Connect to Server):
        // host:port entry, transport open + timeout, and connecting status. The
        // transport is opened by `start_renet_client` (reused from `--connect`).
        #[cfg(feature = "net")]
        {
            app.init_resource::<LocalServer>()
                .init_resource::<ConnectForm>()
                .init_resource::<ConnectDeadline>()
                .add_systems(
                    OnEnter(AppState::Connecting),
                    (start_renet_client, arm_connect_deadline),
                )
                .add_systems(
                    EguiPrimaryContextPass,
                    (
                        draw_connect_entry.run_if(in_state(AppState::ConnectEntry)),
                        draw_connecting.run_if(in_state(AppState::Connecting)),
                    ),
                )
                .add_systems(
                    Update,
                    poll_connecting.run_if(in_state(AppState::Connecting)),
                );
        }
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

    #[cfg(feature = "net")]
    #[test]
    fn parse_addr_accepts_host_port_and_rejects_garbage() {
        let addr = parse_addr("127.0.0.1:5555").expect("ip:port should parse");
        assert_eq!(addr.port(), 5555);
        assert!(addr.ip().is_loopback());
        // Surrounding whitespace is tolerated.
        assert!(parse_addr("  127.0.0.1:5555  ").is_some());
        // Missing port / nonsense yields None rather than panicking.
        assert!(parse_addr("not-an-address").is_none());
        assert!(parse_addr("").is_none());
    }
}
