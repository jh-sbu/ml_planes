---
description: Run the headless observe-state binary to trace FlightState evolution for debugging
---

Build and run the `observe_state` example binary, then analyse the CSV output to help diagnose flight dynamics issues.

## Step 1 — run the binary

```bash
cargo run --example observe_state --no-default-features -- [OPTIONS]
```

| Flag | Default | Meaning |
|---|---|---|
| `--steps N` | 600 | Simulation steps at 60 Hz (600 = 10 s) |
| `--controller level_hold\|manual` | `level_hold` | Controller attached to the plane |
| `--interval N` | 10 | Print every Nth step |

## Step 2 — output columns

```
step, time_s, altitude_m, airspeed_ms, alpha_deg, beta_deg,
pitch_rate [rad/s], roll_rate [rad/s], yaw_rate [rad/s],
elevator [-1,1], throttle [0,1], aileron [-1,1], rudder [-1,1]
```

Spawn conditions: altitude 500 m, airspeed 100 m/s, level flight, generic jet config.
`level_hold` targets altitude 500 m and airspeed 100 m/s.
`manual` applies zero inputs — reveals free (uncontrolled) dynamics.

## Step 3 — common workflows

```bash
# Default 10 s run with level-hold controller
cargo run --example observe_state --no-default-features

# Capture to file for external plotting
cargo run --example observe_state --no-default-features -- --steps 3600 > run.csv

# Every step, 5 s, zero inputs — observe natural phugoid mode
cargo run --example observe_state --no-default-features -- --steps 300 --controller manual --interval 1
```

## Step 4 — what to look for

Run the binary with the Bash tool and read the output. Highlight any of the following:

- **Altitude drift** — `altitude_m` leaves 500 m and doesn't return → trim offset, check integral gains
- **Pitch oscillation** — `pitch_rate` and `alpha_deg` cycling → phase-margin issue, reduce `alpha_pid.kd`
- **Controller saturation** — `elevator` pinned at ±1 → outer loop commanding an alpha target the inner loop can't track
- **Airspeed decay** — `airspeed_ms` falling toward stall → throttle PID not responding
- **NaN / Inf** — physics blow-up; check aero coefficient signs or time step
- **Immediate zeros** — all state fields are 0 in early rows → FixedUpdate hasn't fired yet (normal for step 0; state populates by step 1–2)

Summarise findings and suggest which gain or coefficient to investigate.
