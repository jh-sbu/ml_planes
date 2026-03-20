use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};
use std::f32::consts::PI;

use crate::camera::CameraMode;
use crate::controllers::ActiveController;
use crate::plane::{ControlInputs, FlightState};

pub fn draw_flight_hud(
    mode: Res<CameraMode>,
    mut contexts: EguiContexts,
    plane_query: Query<(&FlightState, &ControlInputs, &ActiveController)>,
) {
    // Determine which entity to display
    let result = match *mode {
        CameraMode::Follow(entity) => plane_query.get(entity).ok(),
        CameraMode::FreeLook => plane_query.iter().next(),
    };

    let Some((state, inputs, controller)) = result else { return };

    let Ok(ctx) = contexts.ctx_mut() else { return };

    egui::Window::new("Flight Data")
        .anchor(egui::Align2::LEFT_TOP, egui::vec2(10.0, 10.0))
        .collapsible(false)
        .show(ctx, |ui| {
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
            ui.label(format!("Controller: {}", controller.0.name()));
        });
}

fn add_surface_bar(ui: &mut egui::Ui, label: &str, value: f32, range: std::ops::RangeInclusive<f32>) {
    ui.horizontal(|ui| {
        ui.label(format!("{:>8}:", label));
        let mut v = value;
        ui.add(egui::Slider::new(&mut v, range).show_value(true));
    });
}
