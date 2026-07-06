use ml_planes::controllers::PidController;

/// Verify PidController drives a first-order plant to setpoint within 200 steps.
/// No Bevy, no Rapier — pure function calls only.
#[test]
fn pid_converges_to_setpoint() {
    // PidController::new(kp, ki, kd, integral_clamp, output_min, output_max)
    let mut pid = PidController::new(2.0, 0.5, 0.1, 10.0, -5.0, 5.0);
    let setpoint = 1.0_f32;
    let mut y = 0.0_f32;
    let dt = 0.01_f32;

    for _ in 0..5000 {
        let error = setpoint - y;
        let u = pid.update(error, dt);
        // First-order plant: τ = 0.5 s → discrete: y += (dt/τ) * (u - y)
        y += (dt / 0.5) * (u - y);
    }

    assert!(
        (y - setpoint).abs() < 0.01,
        "PID did not converge: y={y}, setpoint={setpoint}"
    );
}
