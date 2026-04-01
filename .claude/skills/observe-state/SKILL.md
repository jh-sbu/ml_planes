---
description: Run the headless observe-state binary to trace FlightState evolution for debugging
---

Verify that the level-hold controller converges across the flight envelope of a given plane config.

**Requires a plane config argument.** If the user did not supply a `.ron` path, stop immediately and print:

```
Usage: /observe-state <path/to/plane.plane.ron>
Example: /observe-state assets/planes/generic_jet.plane.ron
```

Otherwise, proceed with the steps below using the supplied path as `PLANE`.

---

## Step 1 — Read the plane config

Use the Read tool to open `PLANE`. Extract the following to inform the scenario matrix:

- `thrust_max` — maximum thrust (N)
- `mass` — aircraft mass (kg)
- `cl_max` — maximum lift coefficient
- `wing_area` (S) — reference area (m²)

Estimate a rough cruise speed range (the airspeed where lift ≈ weight at moderate α).
Use these values to anchor the scenario altitudes and airspeeds to the actual aircraft.

---

## Step 2 — Choose the scenario matrix

Pick **5 (altitude, airspeed) pairs** that span the plane's envelope. Typical targets for a
generic jet (adjust proportionally based on Step 1 estimates):

| # | Altitude (m) | Airspeed (m/s) | Rationale |
|---|---|---|---|
| A | 200 | 80 | Low / slow |
| B | 500 | 100 | Mid / cruise |
| C | 1000 | 130 | High / fast |
| D | 2000 | 110 | High alt / cruise |
| E | 300 | 70 | Low / near-stall |

Reject any pair where the required lift-to-weight ratio at that airspeed would demand `CL > cl_max * 0.85` (too close to stall) or where `thrust_max` is clearly insufficient to sustain level flight at that speed.

---

## Step 3 — Run each scenario

For each (alt, airspeed) pair, run the binary with **3600 steps (60 s at 60 Hz)**:

```bash
cargo run --example observe_state --no-default-features -- \
  --plane PLANE \
  --steps 3600 \
  --interval 60 \
  --controller level_hold \
  --altitude TARGET_ALT \
  --airspeed TARGET_AIRSPEED \
  [--alt-kp F] [--alt-ki F] [--alt-kd F] \
  [--alpha-kp F] [--alpha-kd F] \
  [--spd-kp F] [--spd-ki F]
```

The plane spawns **at the target altitude and airspeed** (steady-state test, not a recovery test).
The `--interval 60` flag prints one row per second, giving 60 rows total per run.

---

## Step 4 — Output columns

```
step, time_s, altitude_m, airspeed_ms, alpha_deg, beta_deg,
pitch_rate [rad/s], roll_rate [rad/s], yaw_rate [rad/s],
elevator [-1,1], throttle [0,1], aileron [-1,1], rudder [-1,1]
```

---

## Step 5 — Check convergence per scenario

Examine the **last 30 rows** of each run (final 30 s of the 60 s window).

A scenario **passes** if, for every row in that window:

- `|altitude_m - TARGET_ALT| ≤ 10 m`
- `|airspeed_ms - TARGET_AIRSPEED| ≤ 2 m/s`

For each scenario record:
- worst altitude error (m) in the final 30 rows
- worst airspeed error (m/s) in the final 30 rows
- PASS or FAIL

Also flag these secondary pathologies even on a passing run:
- **Elevator saturation** — `elevator` pinned at ±1.0 → outer loop overcommanding
- **Pitch oscillation** — `pitch_rate` cycling with amplitude > 0.05 rad/s in the final 30 rows
- **NaN / Inf** — physics blow-up; abort immediately and report

---

## Step 6 — Report results

Print a summary table:

```
Scenario  Alt(m)  Speed(m/s)  Worst ΔAlt(m)  Worst ΔSpd(m/s)  Result
A         200     80          3.2            0.8              PASS
B         500     100         1.1            0.3              PASS
...
```

Then either:

**All pass →**

```
All 5 scenarios converged over 60 s.

Exact reproduction command (scenario B as reference):
  cargo run --example observe_state --no-default-features -- \
    --plane PLANE --steps 3600 --interval 60 \
    --altitude 500 --airspeed 100 \
    [gains used]

Write these PID gains to PLANE as a comment block? (yes/no)
```

**Any fail →** Print which scenario failed, the worst errors, and diagnosis (trim offset, saturation, oscillation). Do not offer write-back. Suggest which gain to adjust and re-run.

---

## Step 7 — Write-back (only if user answers yes)

Append the following comment block at the **bottom of the PLANE file** (after the closing `)`):

```
// --- Tuned PID gains (observe-state YYYY-MM-DD) ---
// --alt-kp F --alt-ki F --alt-kd F
// --alpha-kp F --alpha-kd F
// --spd-kp F --spd-ki F
```

Use today's date. Use the actual gain values passed to (or defaulted by) the binary.
If the file already has a `// --- Tuned PID gains` block, replace it in place.
