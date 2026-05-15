// Canary: asserts that orbit.normal in generic_jet.tuning.ron matches OrbitTuning::default()
// (which reflects OrbitController::from_state() hardcoded values).
//
// The orbit controller is spawned with from_state() gains; the named profile is applied only
// after the asset loads and apply_initial_tuning fires. If the RON values diverge from the
// defaults, apply_initial_tuning would produce a different controller than what was live during
// the first frames — a latent inconsistency. This test catches that drift early.
use ml_planes::controllers::{OrbitTuning, PlaneTuning};

#[test]
fn orbit_normal_inner_matches_spawn_defaults() {
    let text = std::fs::read_to_string("assets/planes/generic_jet.tuning.ron")
        .expect("assets/planes/generic_jet.tuning.ron must exist");
    let pt: PlaneTuning = ron::de::from_str(&text).expect("tuning file must parse");
    let orbit = pt
        .get_orbit("normal")
        .expect("orbit.normal profile must exist");
    let expected = OrbitTuning::default();

    assert_eq!(
        orbit.radial_kp, expected.radial_kp,
        "orbit.normal.radial_kp diverged from OrbitController defaults"
    );
    assert_eq!(
        orbit.radial_kd, expected.radial_kd,
        "orbit.normal.radial_kd diverged"
    );
    assert_eq!(
        orbit.heading_kp, expected.heading_kp,
        "orbit.normal.heading_kp diverged"
    );
    assert_eq!(
        orbit.heading_kd, expected.heading_kd,
        "orbit.normal.heading_kd diverged"
    );
    assert_eq!(
        orbit.inner.alt_kp, expected.inner.alt_kp,
        "orbit.normal.inner.alt_kp diverged from LevelHoldController defaults"
    );
    assert_eq!(
        orbit.inner.alt_ki, expected.inner.alt_ki,
        "orbit.normal.inner.alt_ki diverged"
    );
    assert_eq!(
        orbit.inner.alt_kd, expected.inner.alt_kd,
        "orbit.normal.inner.alt_kd diverged"
    );
    assert_eq!(
        orbit.inner.pitch_kp, expected.inner.pitch_kp,
        "orbit.normal.inner.pitch_kp diverged"
    );
    assert_eq!(
        orbit.inner.pitch_kd, expected.inner.pitch_kd,
        "orbit.normal.inner.pitch_kd diverged"
    );
    assert_eq!(
        orbit.inner.throttle_ff_gain, expected.inner.throttle_ff_gain,
        "orbit.normal.inner.throttle_ff_gain diverged"
    );
}
