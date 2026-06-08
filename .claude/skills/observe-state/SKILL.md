---
description: Run the headless observe-state binary to verify level-hold convergence (PID gains or a trained RL policy) across a plane's flight envelope
---

Verify that the level-hold family converges across the flight envelope. Two input modes are supported:

- **PID branch** — input is a `.plane.ron` config. Uses `--controller level_hold`, tunes/verifies PID gains, and offers a gain write-back at the end.
- **RL branch** — input is a `.mpk` trained model under `models/level_hold/`. Uses `--controller rl_level_hold --model <PATH>`, requires `--features inference`, and reports a verdict without write-back (RL has no gains to update).

**Requires exactly one argument.** If the user did not supply one — or supplied a path with an unrecognised extension — stop immediately and print:

```
Usage: /observe-state <path>
  <path> ending in .plane.ron → verifies PID level-hold convergence
  <path> ending in .mpk       → verifies a trained RL level-hold policy
Examples:
  /observe-state assets/planes/generic_jet.plane.ron
  /observe-state models/level_hold/ppo_level_hold.mpk
```

Otherwise, dispatch to **Step 1A** (PID branch, `.plane.ron`) or **Step 1B** (RL branch, `.mpk`).

> Out of scope: orbit / `rl_orbit*` verification. Those need a different scenario matrix (radius, center, direction). For ad-hoc orbit runs, drive the binary directly per the reference doc at `.agents/skills/observe-state/SKILL.md`.

---

## Step 1A — PID branch: read the plane config

Use the Read tool to open the supplied `.plane.ron`. Extract:

- `thrust_max` — maximum thrust (N)
- `mass` — aircraft mass (kg)
- `cl_max` — maximum lift coefficient
- `wing_area` (S) — reference area (m²)

Estimate a rough cruise speed range (airspeed where lift ≈ weight at moderate α). Use these values to anchor the scenario matrix in Step 2.

Set `PLANE = <supplied path>`. Skip to Step 2.

## Step 1B — RL branch: confirm the model and its training envelope

Confirm the model file exists. Confirm its parent directory is `models/level_hold/` — if not, abort with:

```
RL level-hold verification expects a model under models/level_hold/. Got: <path>
```

The model has no embedded plane config, so the verification uses the built-in `generic_jet` defaults (matches `assets/planes/generic_jet.plane.ron`). Set `MODEL = <supplied path>` and `PLANE = assets/planes/generic_jet.plane.ron`. Read `PLANE` to extract the same four fields as Step 1A — the scenario matrix needs them either way.

---

## Step 2 — Choose the scenario matrix

Pick **5 (altitude, airspeed) pairs** that span the plane's envelope. Typical targets for a generic jet (adjust proportionally based on Step 1 estimates):

| # | Altitude (m) | Airspeed (m/s) | Rationale |
|---|---|---|---|
| A | 200 | 80 | Low / slow |
| B | 500 | 100 | Mid / cruise |
| C | 1000 | 130 | High / fast |
| D | 2000 | 110 | High alt / cruise |
| E | 300 | 70 | Low / near-stall |

Reject any pair where the required lift-to-weight ratio at that airspeed would demand `CL > cl_max * 0.85` (too close to stall) or where `thrust_max` is clearly insufficient to sustain level flight at that speed.

For the RL branch, additionally reject any scenario that sits well outside the typical PPO training envelope (roughly 500–1000 m, 80–130 m/s for the shipped policies) — the model will not have learned controls for it. If you skip a scenario for this reason, say so explicitly in the report.

---

## Step 3 — Run each scenario

For each (alt, airspeed) pair, run the binary for **3840 steps (60 s at 64 Hz)**.

### Step 3A — PID invocation

```bash
cargo run --example observe_state --no-default-features -- \
  --plane PLANE \
  --steps 3840 \
  --interval 64 \
  --controller level_hold \
  --altitude TARGET_ALT \
  --airspeed TARGET_AIRSPEED \
  [--alt-kp F] [--alt-ki F] [--alt-kd F] \
  [--pitch-kp F] [--pitch-kd F] \
  [--spd-kp F] [--spd-ki F]
```

### Step 3B — RL invocation

```bash
cargo run --example observe_state --no-default-features --features inference -- \
  --controller rl_level_hold \
  --model MODEL \
  --steps 3840 \
  --interval 64 \
  --altitude TARGET_ALT \
  --airspeed TARGET_AIRSPEED
```

The `.mpk` extension on `MODEL` is optional. No `--plane` is passed — the default `generic_jet` matches the typical training plane. No gain overrides apply to the RL branch.

In both modes the plane spawns **at the target altitude and airspeed** (steady-state test, not a recovery test). `--interval 64` prints one row per second, giving 60 rows per run.

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
- **Elevator saturation** — `elevator` pinned at ±1.0 → outer loop overcommanding (PID) or policy operating at its action limits (RL)
- **Pitch oscillation** — `pitch_rate` cycling with amplitude > 0.05 rad/s in the final 30 rows
- **NaN / Inf** — physics blow-up; abort immediately and report

For the RL branch, also note: RL inference is deterministic (`mean_action`, no sampling). Two identical commands should produce byte-identical CSV; if they don't, something else is varying (timestep, spawn, build).

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

**PID branch, all pass →**

```
All 5 scenarios converged over 60 s.

Exact reproduction command (scenario B as reference):
  cargo run --example observe_state --no-default-features -- \
    --plane PLANE --steps 3840 --interval 64 \
    --altitude 500 --airspeed 100 \
    [gains used]

Write these PID gains to PLANE as a comment block? (yes/no)
```

Continue to Step 7 if the user answers yes.

**RL branch, all pass →**

```
Model MODEL converged across all 5 envelope scenarios over 60 s.

Exact reproduction command (scenario B as reference):
  cargo run --example observe_state --no-default-features --features inference -- \
    --controller rl_level_hold --model MODEL \
    --steps 3840 --interval 64 \
    --altitude 500 --airspeed 100
```

No write-back step — the model is the source of truth. Stop after the verdict.

**Any fail (either branch) →** Print which scenario failed, the worst errors, and a brief diagnosis (trim offset, saturation, oscillation for PID; out-of-distribution operating point, action saturation, or genuinely bad policy for RL). Do not offer write-back. For PID, suggest which gain to adjust and re-run. For RL, suggest narrowing the envelope or re-training.

---

## Step 7 — Write-back (PID branch only, if user answered yes)

Append the following comment block at the **bottom of the PLANE file** (after the closing `)`):

```
// --- Tuned PID gains (observe-state YYYY-MM-DD) ---
// --alt-kp F --alt-ki F --alt-kd F
// --pitch-kp F --pitch-kd F
// --spd-kp F --spd-ki F
```

Use today's date. Use the actual gain values passed to (or defaulted by) the binary. If the file already has a `// --- Tuned PID gains` block, replace it in place.
