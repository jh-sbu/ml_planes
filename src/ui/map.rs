//! Toggleable full-screen top-down **map** overlay (visual mode only).
//!
//! Press `M` to replace the 3D view with a 2D plan-view showing every plane's
//! world-frame XZ position and heading, plus the geometry of any active L1
//! [`FlightPlan`](crate::controllers::FlightPlan) (waypoints + orbit circles).
//! The view auto-fits all planes on open and supports drag-to-pan / scroll-to-zoom.
//!
//! The map projects world space onto the screen with `+X → right` and
//! `+Z → down`, so world north (`-Z`) is up. All projection math lives on
//! [`MapState`] as egui-free, unit-tested methods using glam [`Vec2`]; the
//! [`draw_map`] system is a thin egui shell on top.
//!
//! Interactive editing of flight plans is **out of scope** here — but the
//! [`MapState::screen_to_world`] hook is the seam a future editor will use to
//! turn pointer clicks into waypoint coordinates.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

use crate::camera::CameraMode;
use crate::controllers::{
    active_orbit_center, ActiveController, ControllerKind, FlightPlanLeg, L1Controller,
};
use crate::plane::{FlightState, PlaneIndex};

/// Lower bound on zoom (most zoomed-in): 5 cm per pixel.
const MIN_MPP: f32 = 0.05;
/// Upper bound on zoom (most zoomed-out): 5 km per pixel.
const MAX_MPP: f32 = 5000.0;
/// Fallback zoom when the planes are coincident (or there are none) [m/px].
const DEFAULT_MPP: f32 = 5.0;

/// Persistent pan/zoom state for the map overlay.
#[derive(Resource)]
pub struct MapState {
    /// Whether the map is currently shown (replacing the 3D view).
    pub open: bool,
    /// World-frame XZ point rendered at the centre of the view: `(x, z)` [m].
    pub center: Vec2,
    /// Zoom: world metres covered by one screen pixel.
    pub meters_per_pixel: f32,
    /// Set when the map is opened so the next frame auto-fits all planes.
    needs_fit: bool,
}

impl Default for MapState {
    fn default() -> Self {
        Self {
            open: false,
            center: Vec2::ZERO,
            meters_per_pixel: DEFAULT_MPP,
            needs_fit: false,
        }
    }
}

impl MapState {
    /// Project a world-frame XZ point (`x`, `z`) to a screen pixel, given the
    /// pixel coordinate of the view centre.
    pub fn world_to_screen(&self, world_xz: Vec2, view_center_px: Vec2) -> Vec2 {
        view_center_px + (world_xz - self.center) / self.meters_per_pixel
    }

    /// Inverse of [`world_to_screen`](Self::world_to_screen): screen pixel →
    /// world-frame XZ point. The seam a future flight-plan editor uses to turn
    /// a pointer click into waypoint coordinates.
    pub fn screen_to_world(&self, screen_px: Vec2, view_center_px: Vec2) -> Vec2 {
        self.center + (screen_px - view_center_px) * self.meters_per_pixel
    }

    /// Centre and zoom the view so every point fits within `view_size_px`
    /// minus a fractional margin on each edge. No-op for an empty slice.
    pub fn fit_to_points(&mut self, points: &[Vec2], view_size_px: Vec2, margin_frac: f32) {
        if points.is_empty() {
            return;
        }
        let mut min = points[0];
        let mut max = points[0];
        for &p in points {
            min = min.min(p);
            max = max.max(p);
        }
        self.center = (min + max) * 0.5;

        let span = max - min;
        let usable = view_size_px * (1.0 - margin_frac).max(0.05);
        let mpp_x = if usable.x > 0.0 {
            span.x / usable.x
        } else {
            0.0
        };
        let mpp_y = if usable.y > 0.0 {
            span.y / usable.y
        } else {
            0.0
        };
        let mpp = mpp_x.max(mpp_y);
        self.meters_per_pixel = if mpp < 1e-3 {
            DEFAULT_MPP
        } else {
            mpp.clamp(MIN_MPP, MAX_MPP)
        };
    }

    /// Multiply the zoom by `factor` while keeping the world point currently
    /// under `cursor_px` fixed on screen (`factor > 1` zooms out).
    pub fn zoom_about(&mut self, cursor_px: Vec2, view_center_px: Vec2, factor: f32) {
        let world_under_cursor = self.screen_to_world(cursor_px, view_center_px);
        self.meters_per_pixel = (self.meters_per_pixel * factor).clamp(MIN_MPP, MAX_MPP);
        // Re-anchor the centre so `world_under_cursor` maps back to `cursor_px`.
        self.center = world_under_cursor - (cursor_px - view_center_px) * self.meters_per_pixel;
    }
}

/// Map color for a plane glyph, keyed by its active controller.
fn kind_color(kind: ControllerKind) -> egui::Color32 {
    match kind {
        ControllerKind::LevelHold | ControllerKind::Ascent | ControllerKind::HeadingHold => {
            egui::Color32::from_rgb(90, 160, 255) // blue
        }
        ControllerKind::Wingman => egui::Color32::from_rgb(90, 220, 120), // green
        ControllerKind::Orbit => egui::Color32::from_rgb(255, 90, 90),    // red
        ControllerKind::RlOrbit
        | ControllerKind::RlOrbitResidual
        | ControllerKind::RlLstmOrbit
        | ControllerKind::RlLevelHold => egui::Color32::from_rgb(190, 110, 255), // purple
        ControllerKind::FlightPlan => egui::Color32::from_rgb(255, 170, 60), // orange
        ControllerKind::Manual => egui::Color32::from_rgb(200, 200, 200), // grey
    }
}

/// Ground-track forward direction (world XZ, unit) — velocity when moving,
/// else body forward; defaults to north (`-Z`) when both are degenerate.
fn ground_forward(state: &FlightState) -> Vec2 {
    let v = Vec2::new(state.velocity.x, state.velocity.z);
    if v.length() > 1.0 {
        return v.normalize();
    }
    let fwd = state.attitude * Vec3::X;
    let f = Vec2::new(fwd.x, fwd.z);
    if f.length() > 1e-3 {
        f.normalize()
    } else {
        Vec2::new(0.0, -1.0)
    }
}

/// Per-plane snapshot collected for rendering.
struct PlaneRender {
    entity: Entity,
    pos: Vec2,
    forward: Vec2,
    index: u32,
    kind: ControllerKind,
    altitude: f32,
    airspeed: f32,
    /// `(legs, active_leg_index)` for a plane flying an L1 flight plan.
    plan: Option<(Vec<FlightPlanLeg>, usize)>,
    /// Active orbit center (world XZ) if this plane is circling a point.
    orbit_center: Option<Vec2>,
}

/// Full-screen top-down map overlay. Toggled with `M`; while open it covers the
/// 3D view with an opaque panel and renders plane positions + flight plans.
pub fn draw_map(
    mut map: ResMut<MapState>,
    mut camera_mode: ResMut<CameraMode>,
    keys: Res<ButtonInput<KeyCode>>,
    mut contexts: EguiContexts,
    mut planes: Query<(
        Entity,
        &FlightState,
        &PlaneIndex,
        &ControllerKind,
        &mut ActiveController,
    )>,
) {
    if keys.just_pressed(KeyCode::KeyM) {
        map.open = !map.open;
        if map.open {
            map.needs_fit = true;
        }
    }
    if !map.open {
        return;
    }

    let Ok(ctx) = contexts.ctx_mut() else { return };

    // Snapshot every plane (clone plan legs so the query borrow ends here).
    let mut rends: Vec<PlaneRender> = Vec::new();
    for (entity, state, index, kind, mut ctrl) in planes.iter_mut() {
        let plan = if *kind == ControllerKind::FlightPlan {
            ctrl.0
                .as_any_mut()
                .downcast_mut::<L1Controller>()
                .map(|l1| (l1.plan.legs.clone(), l1.leg_index))
        } else {
            None
        };
        let orbit_center = active_orbit_center(ctrl.0.as_mut()).map(|m| m.center);
        rends.push(PlaneRender {
            entity,
            pos: Vec2::new(state.position.x, state.position.z),
            forward: ground_forward(state),
            index: index.0,
            kind: *kind,
            altitude: state.altitude,
            airspeed: state.airspeed,
            plan,
            orbit_center,
        });
    }
    rends.sort_by_key(|r| r.index);

    let frame = egui::Frame {
        fill: egui::Color32::from_rgb(12, 16, 24),
        ..Default::default()
    };
    egui::CentralPanel::default().frame(frame).show(ctx, |ui| {
        let (response, painter) =
            ui.allocate_painter(ui.available_size(), egui::Sense::click_and_drag());
        let rect = response.rect;
        let view_center = Vec2::new(rect.center().x, rect.center().y);

        // --- Auto-fit on first frame after opening ---
        if map.needs_fit {
            let pts: Vec<Vec2> = rends.iter().map(|r| r.pos).collect();
            map.fit_to_points(&pts, Vec2::new(rect.width(), rect.height()), 0.2);
            map.needs_fit = false;
        }

        // --- Pan (left-drag) ---
        if response.dragged() {
            let d = response.drag_delta();
            let mpp = map.meters_per_pixel;
            map.center -= Vec2::new(d.x, d.y) * mpp;
        }

        // --- Zoom (scroll about the cursor) ---
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll != 0.0 {
            if let Some(cursor) = response.hover_pos() {
                // Scroll up (positive) zooms in → fewer metres per pixel.
                let factor = (1.0 - scroll * 0.0015).clamp(0.5, 2.0);
                map.zoom_about(Vec2::new(cursor.x, cursor.y), view_center, factor);
            }
        }

        let w2s = |w: Vec2| -> egui::Pos2 {
            let s = map.world_to_screen(w, view_center);
            egui::pos2(s.x, s.y)
        };

        // --- Click a plane to follow it ---
        if response.clicked() {
            if let Some(click) = response.interact_pointer_pos() {
                if let Some(r) = rends.iter().min_by(|a, b| {
                    let da = (w2s(a.pos) - click).length();
                    let db = (w2s(b.pos) - click).length();
                    da.total_cmp(&db)
                }) {
                    if (w2s(r.pos) - click).length() < 14.0 {
                        *camera_mode = CameraMode::Follow(r.entity);
                    }
                }
            }
        }

        draw_grid(&painter, rect, &map, view_center);

        // --- Flight plans (read-only) ---
        for r in &rends {
            if let Some((legs, active)) = &r.plan {
                draw_flight_plan(&painter, &w2s, legs, *active, kind_color(r.kind));
            }
        }

        // --- Planes ---
        let hover = response.hover_pos();
        let followed = match *camera_mode {
            CameraMode::Follow(e) => Some(e),
            CameraMode::FreeLook => None,
        };
        for r in &rends {
            if followed == Some(r.entity) {
                painter.circle_stroke(
                    w2s(r.pos),
                    14.0,
                    egui::Stroke::new(2.0, egui::Color32::WHITE),
                );
                if let Some(center) = r.orbit_center {
                    draw_orbit_pin(&painter, w2s(center));
                }
            }
            draw_plane_glyph(&painter, &w2s, r);
        }

        // --- Hover tooltip ---
        if let Some(h) = hover {
            for r in &rends {
                let p = w2s(r.pos);
                if (egui::vec2(p.x - h.x, p.y - h.y)).length() < 12.0 {
                    let text = format!(
                        "Plane {}  ({})\nalt {:.0} m   spd {:.0} m/s\nx {:.0}  z {:.0}",
                        r.index,
                        r.kind.name(),
                        r.altitude,
                        r.airspeed,
                        r.pos.x,
                        r.pos.y,
                    );
                    draw_tooltip(&painter, egui::pos2(p.x + 14.0, p.y), &text);
                    break;
                }
            }
        }

        draw_scale_bar(&painter, rect, map.meters_per_pixel);
        draw_north(&painter, rect);

        painter.text(
            egui::pos2(rect.left() + 10.0, rect.bottom() - 10.0),
            egui::Align2::LEFT_BOTTOM,
            "M: close   click: follow   drag: pan   scroll: zoom",
            egui::FontId::proportional(13.0),
            egui::Color32::from_gray(140),
        );
    });
}

/// Faint world grid with brighter origin axes.
fn draw_grid(painter: &egui::Painter, rect: egui::Rect, map: &MapState, view_center: Vec2) {
    // Pick a grid spacing that renders ~80–200 px apart.
    let target_px = 120.0;
    let raw = target_px * map.meters_per_pixel;
    let spacing = nice_distance(raw);
    let step_px = spacing / map.meters_per_pixel;
    if step_px < 4.0 {
        return;
    }

    let grid = egui::Stroke::new(1.0, egui::Color32::from_rgb(30, 38, 50));
    let axis = egui::Stroke::new(1.5, egui::Color32::from_rgb(70, 90, 110));

    // World coords at the screen edges.
    let top_left = map.screen_to_world(Vec2::new(rect.left(), rect.top()), view_center);
    let bot_right = map.screen_to_world(Vec2::new(rect.right(), rect.bottom()), view_center);

    let x0 = (top_left.x / spacing).floor() * spacing;
    let mut x = x0;
    while x <= bot_right.x {
        let sx = view_center.x + (x - map.center.x) / map.meters_per_pixel;
        let stroke = if x.abs() < spacing * 0.5 { axis } else { grid };
        painter.line_segment(
            [egui::pos2(sx, rect.top()), egui::pos2(sx, rect.bottom())],
            stroke,
        );
        x += spacing;
    }
    let z0 = (top_left.y / spacing).floor() * spacing;
    let mut z = z0;
    while z <= bot_right.y {
        let sy = view_center.y + (z - map.center.y) / map.meters_per_pixel;
        let stroke = if z.abs() < spacing * 0.5 { axis } else { grid };
        painter.line_segment(
            [egui::pos2(rect.left(), sy), egui::pos2(rect.right(), sy)],
            stroke,
        );
        z += spacing;
    }
}

/// Draw a plane's flight-plan legs; the active leg is drawn brighter.
fn draw_flight_plan(
    painter: &egui::Painter,
    w2s: &impl Fn(Vec2) -> egui::Pos2,
    legs: &[FlightPlanLeg],
    active: usize,
    base: egui::Color32,
) {
    let dim = base.linear_multiply(0.45);
    let mut prev_wp: Option<Vec2> = None;
    for (i, leg) in legs.iter().enumerate() {
        let is_active = i == active;
        let col = if is_active { base } else { dim };
        let stroke = egui::Stroke::new(if is_active { 2.5 } else { 1.5 }, col);
        match leg {
            FlightPlanLeg::Waypoint { x, z, .. } => {
                let wp = Vec2::new(*x, *z);
                let p = w2s(wp);
                if let Some(prev) = prev_wp {
                    painter.line_segment([w2s(prev), p], stroke);
                }
                painter.circle_filled(p, if is_active { 5.0 } else { 4.0 }, col);
                prev_wp = Some(wp);
            }
            FlightPlanLeg::Orbit {
                center_x,
                center_z,
                radius,
                ..
            } => {
                let center = Vec2::new(*center_x, *center_z);
                let c = w2s(center);
                let edge = w2s(center + Vec2::new(*radius, 0.0));
                let r_px = (edge.x - c.x).abs();
                painter.circle_stroke(c, r_px, stroke);
                painter.circle_filled(c, 3.0, col);
                prev_wp = None;
            }
        }
    }
}

/// Draw a single plane as a heading-oriented triangle plus its index label.
fn draw_plane_glyph(painter: &egui::Painter, w2s: &impl Fn(Vec2) -> egui::Pos2, r: &PlaneRender) {
    let col = kind_color(r.kind);
    let c = w2s(r.pos);
    let dir = egui::vec2(r.forward.x, r.forward.y);
    let dir = if dir.length() > 1e-3 {
        dir.normalized()
    } else {
        egui::vec2(0.0, -1.0)
    };
    let perp = egui::vec2(-dir.y, dir.x);
    let size = 9.0;
    let tip = c + dir * size;
    let left = c - dir * size * 0.6 + perp * size * 0.6;
    let right = c - dir * size * 0.6 - perp * size * 0.6;
    painter.add(egui::Shape::convex_polygon(
        vec![tip, left, right],
        col,
        egui::Stroke::new(1.0, egui::Color32::BLACK),
    ));
    painter.text(
        c + perp * 0.0 - dir * (size + 8.0),
        egui::Align2::CENTER_CENTER,
        format!("{}", r.index),
        egui::FontId::proportional(12.0),
        col,
    );
}

/// Draw a map "pin" whose tip sits exactly at `at` (the orbit center): a short
/// vertical stem topped by a filled head, in a bright colour so it reads as a
/// dropped pin distinct from flight-plan dots.
fn draw_orbit_pin(painter: &egui::Painter, at: egui::Pos2) {
    let col = egui::Color32::from_rgb(255, 230, 50);
    let stem = 14.0;
    let head = egui::pos2(at.x, at.y - stem);
    painter.line_segment([at, head], egui::Stroke::new(2.0, egui::Color32::BLACK));
    painter.line_segment([at, head], egui::Stroke::new(1.5, col));
    painter.circle(head, 5.0, col, egui::Stroke::new(1.5, egui::Color32::BLACK));
    // Mark the exact center point.
    painter.circle_filled(at, 1.5, egui::Color32::BLACK);
}

/// A small dark tooltip box anchored at `at` (its left-centre).
fn draw_tooltip(painter: &egui::Painter, at: egui::Pos2, text: &str) {
    let galley = painter.layout_no_wrap(
        text.to_owned(),
        egui::FontId::proportional(12.0),
        egui::Color32::WHITE,
    );
    let pad = egui::vec2(6.0, 4.0);
    let rect = egui::Rect::from_min_size(
        at - egui::vec2(0.0, galley.size().y * 0.5) - pad,
        galley.size() + pad * 2.0,
    );
    painter.rect_filled(
        rect,
        3.0,
        egui::Color32::from_rgba_unmultiplied(0, 0, 0, 220),
    );
    painter.galley(rect.min + pad, galley, egui::Color32::WHITE);
}

/// Bottom-right scale bar showing a round ground distance.
fn draw_scale_bar(painter: &egui::Painter, rect: egui::Rect, mpp: f32) {
    let target_px = 150.0;
    let dist = nice_distance(target_px * mpp);
    let len_px = dist / mpp;
    let y = rect.bottom() - 24.0;
    let x1 = rect.right() - 20.0;
    let x0 = x1 - len_px;
    let stroke = egui::Stroke::new(2.0, egui::Color32::from_gray(200));
    painter.line_segment([egui::pos2(x0, y), egui::pos2(x1, y)], stroke);
    painter.line_segment([egui::pos2(x0, y - 5.0), egui::pos2(x0, y + 5.0)], stroke);
    painter.line_segment([egui::pos2(x1, y - 5.0), egui::pos2(x1, y + 5.0)], stroke);
    let label = if dist >= 1000.0 {
        format!("{:.0} km", dist / 1000.0)
    } else {
        format!("{:.0} m", dist)
    };
    painter.text(
        egui::pos2((x0 + x1) * 0.5, y - 8.0),
        egui::Align2::CENTER_BOTTOM,
        label,
        egui::FontId::proportional(12.0),
        egui::Color32::from_gray(200),
    );
}

/// Top-right north arrow (world `-Z` is up on the map).
fn draw_north(painter: &egui::Painter, rect: egui::Rect) {
    let x = rect.right() - 30.0;
    let top = egui::pos2(x, rect.top() + 24.0);
    let bottom = egui::pos2(x, rect.top() + 50.0);
    let stroke = egui::Stroke::new(2.0, egui::Color32::from_gray(200));
    painter.line_segment([bottom, top], stroke);
    painter.line_segment([top, top + egui::vec2(-4.0, 7.0)], stroke);
    painter.line_segment([top, top + egui::vec2(4.0, 7.0)], stroke);
    painter.text(
        top - egui::vec2(0.0, 6.0),
        egui::Align2::CENTER_BOTTOM,
        "N",
        egui::FontId::proportional(13.0),
        egui::Color32::from_gray(220),
    );
}

/// Round `raw` up to a 1/2/5 × 10ⁿ "nice" value for grid/scale spacing.
fn nice_distance(raw: f32) -> f32 {
    if raw <= 0.0 {
        return 1.0;
    }
    let pow = 10f32.powf(raw.log10().floor());
    let frac = raw / pow;
    let nice = if frac <= 1.0 {
        1.0
    } else if frac <= 2.0 {
        2.0
    } else if frac <= 5.0 {
        5.0
    } else {
        10.0
    };
    nice * pow
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state() -> MapState {
        MapState {
            open: true,
            center: Vec2::new(100.0, -50.0),
            meters_per_pixel: 4.0,
            needs_fit: false,
        }
    }

    #[test]
    fn world_screen_round_trip() {
        let m = state();
        let view_center = Vec2::new(640.0, 360.0);
        let world = Vec2::new(250.0, 175.0);
        let screen = m.world_to_screen(world, view_center);
        let back = m.screen_to_world(screen, view_center);
        assert!(
            (back - world).length() < 1e-3,
            "round trip failed: {back:?}"
        );
    }

    #[test]
    fn center_maps_to_view_center() {
        let m = state();
        let view_center = Vec2::new(640.0, 360.0);
        let s = m.world_to_screen(m.center, view_center);
        assert!((s - view_center).length() < 1e-4);
    }

    #[test]
    fn fit_centers_on_midpoint_and_contains_points() {
        let mut m = state();
        let view = Vec2::new(1000.0, 800.0);
        let pts = [
            Vec2::new(-200.0, -200.0),
            Vec2::new(800.0, 600.0),
            Vec2::new(300.0, 100.0),
        ];
        m.fit_to_points(&pts, view, 0.2);

        // Centre is the bounding-box midpoint.
        assert!((m.center - Vec2::new(300.0, 200.0)).length() < 1e-3);

        // Every point projects inside the view rect.
        let view_center = view * 0.5;
        for &p in &pts {
            let s = m.world_to_screen(p, view_center);
            assert!(s.x >= 0.0 && s.x <= view.x, "x out of view: {}", s.x);
            assert!(s.y >= 0.0 && s.y <= view.y, "y out of view: {}", s.y);
        }
    }

    #[test]
    fn fit_single_point_uses_default_zoom() {
        let mut m = state();
        m.fit_to_points(&[Vec2::new(42.0, 7.0)], Vec2::new(800.0, 600.0), 0.2);
        assert!((m.center - Vec2::new(42.0, 7.0)).length() < 1e-4);
        assert!((m.meters_per_pixel - DEFAULT_MPP).abs() < 1e-4);
    }

    #[test]
    fn fit_empty_is_noop() {
        let mut m = state();
        let before = (m.center, m.meters_per_pixel);
        m.fit_to_points(&[], Vec2::new(800.0, 600.0), 0.2);
        assert_eq!(before, (m.center, m.meters_per_pixel));
    }

    #[test]
    fn zoom_keeps_world_under_cursor_fixed() {
        let mut m = state();
        let view_center = Vec2::new(640.0, 360.0);
        let cursor = Vec2::new(900.0, 200.0);
        let world_before = m.screen_to_world(cursor, view_center);
        m.zoom_about(cursor, view_center, 0.5);
        let world_after = m.screen_to_world(cursor, view_center);
        assert!(
            (world_before - world_after).length() < 1e-2,
            "cursor anchor moved: {world_before:?} -> {world_after:?}"
        );
        assert!((m.meters_per_pixel - 2.0).abs() < 1e-4);
    }

    #[test]
    fn zoom_respects_limits() {
        let mut m = state();
        let vc = Vec2::new(640.0, 360.0);
        for _ in 0..100 {
            m.zoom_about(vc, vc, 0.1);
        }
        assert!(m.meters_per_pixel >= MIN_MPP - 1e-6);
        for _ in 0..100 {
            m.zoom_about(vc, vc, 10.0);
        }
        assert!(m.meters_per_pixel <= MAX_MPP + 1e-3);
    }

    #[test]
    fn nice_distance_rounds_to_1_2_5() {
        assert_eq!(nice_distance(0.9), 1.0);
        assert_eq!(nice_distance(1.5), 2.0);
        assert_eq!(nice_distance(3.0), 5.0);
        assert_eq!(nice_distance(7.0), 10.0);
        assert_eq!(nice_distance(150.0), 200.0);
        assert_eq!(nice_distance(900.0), 1000.0);
    }
}
