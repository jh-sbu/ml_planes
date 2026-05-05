pub struct PidController {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral_clamp: f32,
    pub output_min: f32,
    pub output_max: f32,
    integral: f32,
    prev_error: Option<f32>,
}

impl PidController {
    pub fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        integral_clamp: f32,
        output_min: f32,
        output_max: f32,
    ) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_clamp,
            output_min,
            output_max,
            integral: 0.0,
            prev_error: None,
        }
    }

    pub fn update(&mut self, error: f32, dt: f32) -> f32 {
        self.integral =
            (self.integral + error * dt).clamp(-self.integral_clamp, self.integral_clamp);

        let derivative = match self.prev_error {
            Some(prev) => (error - prev) / dt,
            None => 0.0,
        };
        self.prev_error = Some(error);

        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        output.clamp(self.output_min, self.output_max)
    }

    /// Derivative-on-measurement variant.
    ///
    /// `measured_rate` is `d(process_variable)/dt` as observed from the sensor —
    /// e.g., pitch rate `q` for the alpha loop, roll rate `p` for the roll loop.
    /// The derivative contribution is `-kd * measured_rate`, which damps motion
    /// toward the setpoint without spiking when the setpoint itself changes.
    ///
    /// Use this instead of `update` for any inner loop whose setpoint varies
    /// frame-to-frame (output of an outer PID), or for any loop with direct
    /// access to the relevant body rate.
    pub fn update_dom(&mut self, error: f32, measured_rate: f32, dt: f32) -> f32 {
        self.integral =
            (self.integral + error * dt).clamp(-self.integral_clamp, self.integral_clamp);
        let output = self.kp * error + self.ki * self.integral - self.kd * measured_rate;
        output.clamp(self.output_min, self.output_max)
    }

    /// Pre-load the integrator to a specific value (clamped).
    ///
    /// Use at controller engagement to seed the integral so the initial output
    /// matches trim without waiting for the integrator to wind up.
    pub fn seed_integral(&mut self, value: f32) {
        self.integral = value.clamp(-self.integral_clamp, self.integral_clamp);
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn p_only() {
        let mut pid = PidController::new(2.5, 0.0, 0.0, 100.0, -100.0, 100.0);
        let out = pid.update(1.0, 0.1);
        assert!((out - 2.5).abs() < 1e-6, "out={}", out);
    }

    #[test]
    fn integral_accumulates() {
        let mut pid = PidController::new(0.0, 1.0, 0.0, 1000.0, -1000.0, 1000.0);
        let dt = 0.1;
        let error = 1.0;
        let steps = 5;
        let mut out = 0.0;
        for _ in 0..steps {
            out = pid.update(error, dt);
        }
        // integral = error * dt * steps = 0.5, output = ki * integral = 0.5
        let expected = 1.0 * dt * steps as f32;
        assert!(
            (out - expected).abs() < 1e-5,
            "out={} expected={}",
            out,
            expected
        );
    }

    #[test]
    fn integral_windup_clamped() {
        let clamp = 5.0;
        let mut pid = PidController::new(0.0, 1.0, 0.0, clamp, -1000.0, 1000.0);
        for _ in 0..1000 {
            pid.update(10.0, 0.1);
        }
        // output = ki * integral, integral clamped to 5.0
        let out = pid.update(10.0, 0.1);
        assert!(out <= clamp + 1e-5, "out={} should be <= {}", out, clamp);
    }

    #[test]
    fn output_clamped_to_limits() {
        let mut pid = PidController::new(100.0, 0.0, 0.0, 100.0, -1.0, 1.0);
        let out = pid.update(5.0, 0.1);
        assert!(
            (out - 1.0).abs() < 1e-6,
            "out={} should be clamped to 1.0",
            out
        );

        let out = pid.update(-5.0, 0.1);
        assert!(
            (out - (-1.0)).abs() < 1e-6,
            "out={} should be clamped to -1.0",
            out
        );
    }

    #[test]
    fn derivative_zero_on_first_call() {
        let mut pid = PidController::new(0.0, 0.0, 10.0, 100.0, -1000.0, 1000.0);
        let out = pid.update(5.0, 0.1);
        assert!(
            (out).abs() < 1e-6,
            "first call derivative should be 0, out={}",
            out
        );
    }

    #[test]
    fn dom_no_kick_on_setpoint_step() {
        // DoM must not spike when the error jumps due to a setpoint change.
        // kp=0, ki=0, kd=1 — output is purely the derivative term.
        // Measured rate is 0; error jumps from 0 to 10 (setpoint step).
        let mut pid = PidController::new(0.0, 0.0, 1.0, 100.0, -1000.0, 1000.0);
        let out = pid.update_dom(10.0, 0.0, 0.1);
        assert!(out.abs() < 1e-6, "DoM kick on setpoint step: out={}", out);
    }

    #[test]
    fn dom_damps_measured_rate() {
        // kp=0, ki=0, kd=2 — output = -kd * measured_rate = -6.0.
        let mut pid = PidController::new(0.0, 0.0, 2.0, 100.0, -1000.0, 1000.0);
        let out = pid.update_dom(0.0, 3.0, 0.1);
        assert!((out - (-6.0)).abs() < 1e-5, "out={}", out);
    }

    #[test]
    fn reset_clears_state() {
        let mut pid = PidController::new(1.0, 1.0, 1.0, 100.0, -1000.0, 1000.0);
        // Build up some state
        for _ in 0..10 {
            pid.update(1.0, 0.1);
        }
        pid.reset();

        // After reset: integral=0, prev_error=None
        // p_only output (kp*e=1.0), ki*0=0, kd*0=0 (no prev_error) → 1.0
        let out = pid.update(1.0, 0.1);
        assert!(
            (out - 1.0 - 1.0 * 0.1).abs() < 1e-5,
            "after reset out={}",
            out
        );

        // Specifically check derivative is zero by using kp=0, ki=0, kd=1
        let mut pid2 = PidController::new(0.0, 0.0, 1.0, 100.0, -1000.0, 1000.0);
        pid2.update(5.0, 0.1); // set prev_error
        pid2.reset();
        let out2 = pid2.update(5.0, 0.1); // after reset, derivative should be 0
        assert!(
            (out2).abs() < 1e-6,
            "derivative after reset should be 0, out={}",
            out2
        );
    }
}
