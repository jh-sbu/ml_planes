//! Sign-convention guards for every shipped `.plane.ron` airframe.
//!
//! This codebase's body frame mirrors NED about the roll and pitch axes
//! (+Z up instead of down): positive roll torque = +Y wing UP, positive pitch
//! torque = nose DOWN. Stability derivatives copied from standard (NED)
//! references must therefore have some signs flipped. These tests pin the
//! frame-adjusted signs for all shipped assets so the next NED-copied value
//! is caught at test time instead of flying with inverted coupling.

use ml_planes::plane::PlaneConfig;

/// Parse every `assets/planes/*.plane.ron` into `(name, config)` pairs.
fn shipped_plane_configs() -> Vec<(String, PlaneConfig)> {
    let dir = std::path::Path::new("assets/planes");
    let mut configs = Vec::new();
    for entry in std::fs::read_dir(dir).expect("assets/planes should exist") {
        let path = entry.expect("readable dir entry").path();
        let name = path.file_name().unwrap().to_string_lossy().into_owned();
        if !name.ends_with(".plane.ron") {
            continue;
        }
        let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("read {name}: {e}"));
        let cfg: PlaneConfig =
            ron::de::from_bytes(&bytes).unwrap_or_else(|e| panic!("parse {name}: {e}"));
        configs.push((name, cfg));
    }
    assert!(
        configs.len() >= 5,
        "expected the shipped airframes, found {}",
        configs.len()
    );
    configs
}

#[test]
fn lateral_derivative_signs_match_frame_convention() {
    for (name, cfg) in shipped_plane_configs() {
        // Dihedral: beta > 0 (wind from +Y) must raise the windward wing → positive
        // roll torque in this frame.
        assert!(
            cfg.cl_beta > 0.0,
            "{name}: cl_beta={} must be positive (stable dihedral; NED sign is flipped here)",
            cfg.cl_beta
        );
        // Yaw→roll coupling: r > 0 raises the outer (-Y) wing → negative roll torque.
        assert!(
            cfg.cl_r < 0.0,
            "{name}: cl_r={} must be negative (NED sign is flipped here)",
            cfg.cl_r
        );
        // Weathervane: nose turns into the wind. Yaw sense is unchanged vs NED.
        assert!(
            cfg.cn_beta > 0.0,
            "{name}: cn_beta={} must be positive (weathervane stability)",
            cfg.cn_beta
        );
        // Rate dampings oppose their rate in any sign convention.
        assert!(cfg.cl_p < 0.0, "{name}: cl_p={} must damp roll", cfg.cl_p);
        assert!(cfg.cn_r < 0.0, "{name}: cn_r={} must damp yaw", cfg.cn_r);
    }
}

#[test]
fn longitudinal_derivative_signs_match_frame_convention() {
    for (name, cfg) in shipped_plane_configs() {
        // Static pitch stability: alpha > 0 needs a nose-down restoring moment,
        // which is POSITIVE pitch torque in this frame (NED sign is flipped).
        assert!(
            cfg.cm_alpha > 0.0,
            "{name}: cm_alpha={} must be positive (nose-down restoring; NED sign is flipped here)",
            cfg.cm_alpha
        );
        assert!(cfg.cm_q < 0.0, "{name}: cm_q={} must damp pitch", cfg.cm_q);
    }
}
