use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use std::f32::consts::PI;

use crate::camera::CameraMode;
use crate::controllers::{ActiveController, AscentController, ControllerKind, PlaneTuning, SelectedTuningProfile};
use crate::plane::{ControlInputs, FlightState, PlaneTuningHandle};

pub fn draw_flight_hud(
    mode: Res<CameraMode>,
    mut contexts: EguiContexts,
    mut plane_query: Query<(
        &FlightState,
        &ControlInputs,
        &mut ControllerKind,
        &mut ActiveController,
        Option<&mut SelectedTuningProfile>,
        Option<&PlaneTuningHandle>,
    )>,
    all_planes: Query<Entity, With<FlightState>>,
    tuning_assets: Res<Assets<PlaneTuning>>,
) {
    // Determine which entity to display
    let result = match *mode {
        CameraMode::Follow(entity) => plane_query.get_mut(entity).ok(),
        CameraMode::FreeLook => plane_query.iter_mut().next(),
    };

    let Some((state, inputs, mut kind, mut controller, profile, tuning_handle)) = result else { return };

    let Ok(ctx) = contexts.ctx_mut() else { return };

    let mut sorted_planes: Vec<Entity> = all_planes.iter().collect();
    sorted_planes.sort();
    let camera_label = match *mode {
        CameraMode::FreeLook => "Camera: Free Look".to_string(),
        CameraMode::Follow(entity) => {
            let n = sorted_planes.iter().position(|&e| e == entity).map(|i| i + 1).unwrap_or(0);
            format!("Camera: Follow Plane {}", n)
        }
    };

    egui::Window::new("Flight Data")
        .anchor(egui::Align2::LEFT_TOP, egui::vec2(10.0, 10.0))
        .collapsible(false)
        .show(ctx, |ui| {
            ui.label(&camera_label);
            ui.separator();
            let knots = state.airspeed * 1.944;
            ui.label(format!("Airspeed:  {:.1} m/s  ({:.0} kts)", state.airspeed, knots));
            ui.label(format!("Altitude:  {:.1} m", state.altitude));

            let alpha_deg = state.alpha * 180.0 / PI;
            let beta_deg = state.beta * 180.0 / PI;
            ui.label(format!("Alpha:     {:.1}°", alpha_deg));
            ui.label(format!("Beta:      {:.1}°", beta_deg));

            let av = state.angular_velocity;
            let p_deg = av.x * 180.0 / PI;
            let q_deg = av.y * 180.0 / PI;
            let r_deg = av.z * 180.0 / PI;
            ui.label(format!("p/q/r:     {:.1} / {:.1} / {:.1} °/s", p_deg, q_deg, r_deg));

            ui.separator();

            ui.label("Controls:");
            add_surface_bar(ui, "Aileron", inputs.aileron, -1.0..=1.0);
            add_surface_bar(ui, "Elevator", inputs.elevator, -1.0..=1.0);
            add_surface_bar(ui, "Rudder", inputs.rudder, -1.0..=1.0);
            add_surface_bar(ui, "Throttle", inputs.throttle, 0.0..=1.0);

            ui.separator();
            let current = *kind;
            let mut selected = current;
            egui::ComboBox::from_label("Controller")
                .selected_text(selected.name())
                .show_ui(ui, |ui| {
                    for &k in ControllerKind::ALL {
                        ui.selectable_value(&mut selected, k, k.name());
                    }
                });
            if selected != current {
                *kind = selected;
            }
            ui.label("(C to cycle)");

            if *kind == ControllerKind::Ascent {
                if let Some(ascent) = controller.0.as_any_mut().downcast_mut::<AscentController>() {
                    let status = if ascent.complete { "Complete" } else { "Climbing" };
                    ui.label(format!("Target: {:.0} m — {}", ascent.target_altitude, status));
                }
            }

            if *kind == ControllerKind::LevelHold {
                if let (Some(ref mut profile), Some(handle)) = (profile, tuning_handle) {
                    if let Some(pt) = tuning_assets.get(&handle.0) {
                        let mut profiles: Vec<&str> =
                            pt.level_hold.keys().map(|s| s.as_str()).collect();
                        profiles.sort();
                        let current_profile = profile.0.clone();
                        let mut selected_profile = current_profile.clone();
                        egui::ComboBox::from_label("Tune Profile")
                            .selected_text(&selected_profile)
                            .show_ui(ui, |ui| {
                                for &p in &profiles {
                                    ui.selectable_value(&mut selected_profile, p.to_string(), p);
                                }
                            });
                        if selected_profile != current_profile {
                            profile.0 = selected_profile;
                        }
                    }
                }
            }
        });
}

fn add_surface_bar(ui: &mut egui::Ui, label: &str, value: f32, range: std::ops::RangeInclusive<f32>) {
    ui.horizontal(|ui| {
        ui.label(format!("{:>8}:", label));
        let mut v = value;
        ui.add(egui::Slider::new(&mut v, range).show_value(true));
    });
}
