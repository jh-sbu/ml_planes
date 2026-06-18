---
description: Iteratively tune PID gains for a plane config and write back the best result
---

Auto-tune PID gains for a given plane config and controller. Accepts an optional controller
argument (default: `level_hold`).

**Requires a plane config argument.** If the user did not supply a `.ron` path, stop immediately and print:

```
Usage: /tune <path/to/plane.plane.ron> [controller]
Example: /tune assets/planes/generic_jet.plane.ron
Example: /tune assets/planes/generic_jet.plane.ron level_hold
```

Otherwise, proceed with the steps below using the supplied path as `PLANE` and the controller
name as `CONTROLLER` (default `level_hold` if omitted).

---

## How candidates are run (scenario-driven)

`observe_state` is driven by a `.scenario.ron` file — there is no gain-flag
interface. For each candidate gain set, **write a temporary scenario** to
`target/tune.scenario.ron` (gitignored) with the gains inline under the
controller's `tuning` field, then run:

```bash
cargo run --example observe_state --no-default-features -- --scenario target/tune.scenario.ron
```

Set `steps` per phase (1800 = coarse grid, 3840 = full 60 s validation) and
`interval: 64` (one row per second). The loader enables `implicit_some`, so write
bare values (`tuning: (...)`), not `Some(...)`.

> **Fuel / loaded mass.** `observe_state` now models the powerplant: a plane
> spawns with `fuel_fraction × capacity` of fuel and flies at the corresponding
> **loaded mass** (`effective_mass(dry_mass, fuel)`), burning down over the run.
> `fuel_fraction` defaults to `1.0` (full tank) when omitted — so tunes are done at
> the heaviest, deployed condition. Add `fuel_fraction: F` (0–1) to a plane to tune
> at a different load. The dry `mass` field in `.plane.ron` is the **empty** mass;
> for jets the flown mass is `mass + fuel_fraction × capacity_kg` (electric burns no
> mass). Use this **loaded mass** everywhere a Phase-1 formula says "mass" (see
> Phase 1).

**Level-hold template** (`STEPS`, `TARGET_ALT`, `TARGET_AIRSPEED`, and the 7 gains):

```ron
(
    steps: STEPS,
    interval: 64,
    planes: [(
        name: "p",
        config: "PLANE",
        position: (0.0, TARGET_ALT, 0.0),
        // Spawn AT target airspeed (+X). Required — omitting velocity defaults to
        // 100 m/s and corrupts every non-100 envelope point with a speed transient.
        velocity: (TARGET_AIRSPEED, 0.0, 0.0),
        // Optional — fraction of fuel/charge capacity [0,1]; omit = full tank.
        // fuel_fraction: 1.0,
        controller: LevelHold(
            altitude: TARGET_ALT,
            airspeed: TARGET_AIRSPEED,
            tuning: (
                alt_kp: F, alt_ki: F, alt_kd: F,
                pitch_kp: F, pitch_kd: F,
                spd_kp: F, spd_ki: F,
                throttle_ff_gain: 0.7,
            ),
        ),
    )],
)
```

**Orbit template** (adds `RADIUS`; 4 outer + 7 inner gains):

```ron
(
    steps: STEPS,
    interval: 64,
    planes: [(
        name: "p",
        config: "PLANE",
        position: (0.0, TARGET_ALT, 0.0),
        velocity: (TARGET_AIRSPEED, 0.0, 0.0),   // spawn at target airspeed (+X)
        // fuel_fraction: 1.0,   // optional [0,1]; omit = full tank (loaded mass)
        controller: Orbit(
            radius: RADIUS,
            direction: CounterClockwise,
            altitude: TARGET_ALT,
            airspeed: TARGET_AIRSPEED,
            tuning: (
                radial_kp: F, radial_kd: F, heading_kp: F, heading_kd: F,
                inner: (
                    alt_kp: F, alt_ki: F, alt_kd: F,
                    pitch_kp: F, pitch_kd: F,
                    spd_kp: F, spd_ki: F,
                    throttle_ff_gain: 0.7,
                ),
            ),
        ),
    )],
)
```

**Heading-hold template** (spawn level facing +X = heading 0; `TARGET_HEADING_DEG`
is the commanded heading; 2 outer + 7 inner gains):

```ron
(
    steps: STEPS,
    interval: 64,
    planes: [(
        name: "p",
        config: "PLANE",
        position: (0.0, TARGET_ALT, 0.0),
        velocity: (TARGET_AIRSPEED, 0.0, 0.0),   // spawn at target airspeed, heading 0 (+X)
        // fuel_fraction: 1.0,   // optional [0,1]; omit = full tank (loaded mass)
        controller: HeadingHold(
            heading_deg: TARGET_HEADING_DEG,
            altitude: TARGET_ALT,
            airspeed: TARGET_AIRSPEED,
            tuning: (
                heading_kp: F, heading_kd: F,
                inner: (
                    alt_kp: F, alt_ki: F, alt_kd: F,
                    pitch_kp: F, pitch_kd: F,
                    spd_kp: F, spd_ki: F,
                    throttle_ff_gain: 0.7,
                ),
            ),
        ),
    )],
)
```

**Output columns** (single plane → one row per step):

```
step, time_s, plane, pos_x, altitude_m, pos_z, airspeed_ms, alpha_deg, beta_deg,
roll_deg, pitch_deg, yaw_deg, pitch_rate, roll_rate, yaw_rate,
elevator, throttle, aileron, rudder, radial_error_m, heading_error_rad, bank_ff_rad,
fuel_remaining
```

Level-hold tuning uses `altitude_m`, `airspeed_ms`, `elevator`, `pitch_rate`.
The trailing `fuel_remaining` (kg jet / kWh electric) is informational — it lets you
confirm the tank did not run dry mid-run (a flameout would corrupt the result).
Orbit tuning also uses `radial_error_m` and `heading_error_rad` (blank for
level-hold runs). Step 0 often shows all-zero state — ignore it; evaluate the
final window only.

---

## Controller routing

**If `CONTROLLER` is `level_hold`:** proceed with the phases below.

**If `CONTROLLER` is `orbit`:** the orbit controller is a cascade (inner `LevelHoldController`
+ outer radial/heading PIDs). Proceed with the **Orbit phases** at the bottom of this file.
Skip the level-hold phases entirely.

**If `CONTROLLER` is `heading_hold`:** the heading-hold controller wraps the same inner
`LevelHoldController` as orbit, with a 2-gain outer heading PID. Proceed with the
**Heading-Hold phases** at the bottom of this file. Skip the level-hold phases entirely.

> **Tip:** run `/tune PLANE level_hold` first to get well-tuned inner gains, then copy the
> resulting `LevelHoldTuning` values into `HeadingHoldTuning.inner` in the `.tuning.ron` file.
> Heading-hold outer gains are largely independent of the inner loop.

**If `CONTROLLER` is anything else:** print:

```
Controller "CONTROLLER" is not supported by /tune.
Supported controllers: level_hold, orbit, heading_hold
Orbit tuning guidance: /tune PLANE orbit
Heading-hold tuning:   /tune PLANE heading_hold
```

Stop here.

---

## Phase 1 — Read config and derive physics-based gain estimates

Use the Read tool to open `PLANE`. Extract:

- `thrust_max` — maximum thrust (N)
- `mass` — **dry/empty** aircraft mass (kg)
- `powerplant` — for `JetFuel`, the `capacity_kg`; for `Electric`, no mass contribution
- `wing_area` (S) — reference area (m²)
- `cl0`, `cl_alpha`, `cl_max`
- `cd0`, `cd_induced`
- `cm_delta_e`, `elevator_limit`
- `mean_chord`

**Compute the loaded mass first.** `observe_state` flies the plane at its loaded
mass, not the dry `mass` field. For the default full tank:

```
mass_loaded = mass + capacity_kg          # JetFuel  (use mass + fuel_fraction × capacity_kg if you set fuel_fraction)
mass_loaded = mass                         # Electric (constant mass)
```

Use `mass_loaded` (NOT the dry `mass` field) everywhere "mass" appears below.

**Compute cruise reference values** (air density ρ = 1.225 kg/m³):

```
V_c  = sqrt(2 * mass_loaded * 9.81 / (1.225 * wing_area * (cl0 + cl_alpha * 0.05)))
q_c  = 0.5 * 1.225 * V_c²
CL_c = mass_loaded * 9.81 / (q_c * wing_area)
CD_c = cd0 + cd_induced * CL_c²
T_c  = (q_c * wing_area * CD_c) / thrust_max   [normalised 0–1]
```

**Compute physics-based candidate gains** (Round 0):

```
alt_kp   = 0.5 / V_c
alt_ki   = alt_kp * 0.1
alt_kd   = alt_kp * 2.0
pitch_kp = 0.8 / (|cm_delta_e| * elevator_limit)
pitch_kd = pitch_kp * 0.4
spd_kp   = T_c / (V_c * 2.0)
spd_ki   = spd_kp * 0.5
```

Show the computed values in a brief table before running anything.

**Also choose the 5-scenario matrix** using the same envelope-aware logic as `/observe-state`:
reject any (alt, speed) pair where the required CL > 0.85 × cl_max or where the plane clearly
cannot sustain level flight (T_c > 0.95 at that speed). Typical targets for a generic jet:

| # | Alt (m) | Speed (m/s) | Rationale |
|---|---|---|---|
| A | 200 | 80 | Low / slow |
| B | 500 | 100 | Mid / cruise |
| C | 1000 | 130 | High / fast |
| D | 2000 | 110 | High alt / cruise |
| E | 300 | 70 | Low / near-stall |

---

## Phase 1b — (Optional) Coarse outer-loop grid search

Run this phase at your own discretion — no user prompt needed. It replaces the
physics-based outer-loop estimates with empirically selected ones when those estimates are
likely unreliable.

**Run Phase 1b when any of the following hold:**
- `thrust_max / (mass_loaded × 9.81)` is outside [0.3, 0.8]
- Computed `V_c` is below 50 m/s or above 200 m/s
- Computed `T_c` exceeds 0.85 or is below 0.1
- (Post-Round-0 fallback) Round 0 fails 3+ scenarios **and** failure modes are inconsistent
  across scenarios — different fixes indicated per scenario, suggesting the estimates are
  simply wrong rather than systematically biased

**Skip Phase 1b when** all cruise reference values are within the ranges above or the plane
is close to `generic_jet` defaults.

---

### Grid definition

Keep `pitch_kp` and `pitch_kd` fixed at Phase 1 physics values. Sweep outer loops only:

| Parameter | Base value       | Multipliers       |
|-----------|------------------|-------------------|
| `alt_kp`  | Phase 1 estimate | ×0.5, ×1.0, ×2.0 |
| `alt_ki`  | Phase 1 estimate | ×0.5, ×1.0, ×2.0 |
| `alt_kd`  | Phase 1 estimate | ×0.5, ×1.0, ×2.0 |
| `spd_kp`  | Phase 1 estimate | ×0.5, ×1.0, ×2.0 |
| `spd_ki`  | Phase 1 estimate | ×0.5, ×1.0, ×2.0 |

3^5 = 243 grid points. Evaluate on **scenario B only** (500 m / 100 m/s).

Traverse order: center point (×1.0 all) first, then single-axis perturbations (one param
varied, rest ×1.0), then two-axis, etc. This places the most likely candidates first and
maximises early-exit probability.

### Run (per grid point)

Write the **level-hold template** to `target/tune.scenario.ron` with `STEPS = 1800`,
`TARGET_ALT = 500`, `TARGET_AIRSPEED = 100`, and the grid point's gains, then run it.
Each run produces 29 sampled rows (1800 / 64). Evaluate all of them as the final
window (same pass criteria: `|ΔAlt| ≤ 10 m`, `|ΔSpd| ≤ 2 m/s`).

### Selection

Score each grid point:

```
score = worst |altitude_m − 500| + worst |airspeed_ms − 100|   (over all sampled rows)
```

**Early stopping:** if a grid point passes cleanly (both criteria on all sampled rows), stop
and use it as the winner immediately. Otherwise evaluate all 243 and pick the lowest score.

### Output

```
Phase 1b grid search — scenario B (500 m / 100 m/s), 30-row window
Top 3 candidates:
  Rank  alt_kp  alt_ki  alt_kd  spd_kp  spd_ki  Score  Pass?
  1     F       F       F       F       F       F      YES/NO  ← winner
  2     F       F       F       F       F       F      YES/NO
  3     F       F       F       F       F       F      YES/NO

Winner outer-loop gains (pitch loop unchanged from Phase 1):
  alt_kp F  alt_ki F  alt_kd F
  pitch_kp F  pitch_kd F   [unchanged]
  spd_kp F  spd_ki F
```

Replace the physics-based outer-loop gains with the winner's values. Phase 2 runs these
as Round 0 across all 5 scenarios.

---

## Phase 2 — Baseline run (Round 0)

> If Phase 1b ran, the gains here are the grid-search winner's outer-loop values with
> physics-based `pitch_kp`/`pitch_kd`. Otherwise they are the raw Phase 1 physics estimates.
> The procedure and pass criteria are identical either way.

Run all 5 scenarios with the Round 0 gains computed above. For each scenario write the
**level-hold template** with `STEPS = 3840` (60 s), the scenario's `TARGET_ALT` /
`TARGET_AIRSPEED`, and the Round 0 gains, then run it.

**Pass criteria** (evaluate the last 30 rows of each 60-row run):

- `|altitude_m − TARGET_ALT| ≤ 10 m`
- `|airspeed_ms − TARGET_AIRSPEED| ≤ 2 m/s`

**Pathology flags** (note even on a passing run):

- **Elevator saturation** — `elevator` pinned at ±1.0
- **Pitch oscillation** — `pitch_rate` amplitude > 0.05 rad/s in the final 30 rows
- **NaN / Inf** — abort immediately and report

Print a compact Round 0 table:

```
Round 0 (physics-based estimates)
Scenario  Alt(m)  Speed(m/s)  Worst ΔAlt(m)  Worst ΔSpd(m/s)  Flags  Result
A         200     80          …              …                       PASS/FAIL
…
```

If all 5 pass → jump directly to Phase 5.

---

## Phase 3 — Diagnose failure modes

For each failing or pathological scenario, identify the **dominant failure mode** using this table:

| Symptom | Dominant problem | Fix for next round |
|---|---|---|
| Altitude drifts monotonically (slow, no oscillation) | `alt_ki` too low (trim offset) | × 2 alt_ki |
| Altitude oscillates (pitch_rate cycles > 0.05 rad/s) | `alt_kp` too aggressive | × 0.5 alt_kp, × 1.5 alt_kd |
| Elevator saturated at ±1 for most of run | `alt_kp` or `pitch_kp` overcommanding | × 0.5 pitch_kp, then × 0.5 alt_kp |
| Pitch overshoots then slowly settles | `pitch_kd` too low | × 1.5 pitch_kd |
| Airspeed drifts monotonically | `spd_ki` too low | × 2 spd_ki |
| Airspeed oscillates around target | `spd_ki` too high | × 0.5 spd_ki |
| Airspeed sluggish but stable | `spd_kp` too low | × 2 spd_kp |

**Fix order when multiple loops fail:** innermost first (pitch → altitude → airspeed).

Describe the diagnosis briefly ("Scenario A: elevator saturated in first 10 s, pitch_kp likely
too high → halving pitch_kp for Round 1").

---

## Phase 4 — Iterative refinement (up to 3 rounds)

Repeat the following for rounds 1, 2, and 3 (stop early if all scenarios pass):

1. Apply the fix(es) identified in the previous round's diagnosis to produce new candidate gains.
   Show the updated gain table before running.

2. Run **only the scenarios that are still failing** (not all 5 — saves time): write the
   level-hold template (`STEPS = 3840`) with the new gains and that scenario's operating
   point, then run it.

3. Print a compact result row for each re-run scenario.

4. **Accept or reject the update per loop:**
   - If the scenario now passes or worst error improved → keep the new gains for that loop.
   - If worst error got worse → revert that loop's gains to the previous round.

5. Track the **best candidate** gain set: the one with the lowest sum of worst-case altitude and
   airspeed errors across all scenarios so far (even if not all pass yet).

6. Re-diagnose remaining failures and propose the next fix. If the same fix was tried twice
   without improvement, try the opposite direction (e.g., if halving kp didn't help, try ×1.5
   instead) or move on to adjusting the adjacent loop.

   If you reach Round 2 with 3+ scenarios still failing and no consistent improvement
   direction, and Phase 1b was not already run, stop the refinement loop, run Phase 1b now
   (using its Round-0-failure trigger criteria), then restart Phase 4 from Round 1 with the
   grid-search winner's gains.

---

## Phase 5 — Full validation run

Run all 5 scenarios with the **best-candidate gains** (level-hold template, `STEPS = 3840`).

Print the full summary table:

```
Final validation (best gains after N rounds)
Scenario  Alt(m)  Speed(m/s)  Worst ΔAlt(m)  Worst ΔSpd(m/s)  Flags  Result
A         200     80          …              …                       PASS/FAIL
B         500     100         …              …
C         1000    130         …              …
D         2000    110         …              …
E         300     70          …              …
```

**If all 5 pass:**

```
✓ All scenarios converged (N tuning rounds).

Best gains:
  alt_kp F  alt_ki F  alt_kd F
  pitch_kp F  pitch_kd F
  spd_kp F  spd_ki F

Reproduction (scenario B): generated target/tune.scenario.ron with
  LevelHold(altitude: 500, airspeed: 100, tuning: (…best gains…)); ran
  cargo run --example observe_state --no-default-features -- --scenario target/tune.scenario.ron

Once written to the tuning file, you can verify with a scenario that sets
  config: "PLANE" and a LevelHold controller whose tuning matches the
  level_hold."PROFILE" entry.

Write these gains to the tuning file? If yes, also provide a profile name (default: "normal").
```

**If any scenario still fails after 3 rounds:**

```
Could not converge all scenarios in 3 tuning rounds.
Best result found:
  [table above]

Remaining issues:
  [per-scenario diagnosis of what is still wrong]

Suggested next steps:
  [specific gain direction for each remaining failure]
```

Do not offer write-back if any scenario fails.

---

## Phase 6 — Write-back (only if user answers yes in Phase 5)

Derive the tuning file path by replacing `.plane.ron` with `.tuning.ron` in `PLANE`:

```
TUNING_FILE = PLANE with ".plane.ron" → ".tuning.ron"
Example: assets/planes/generic_jet.plane.ron → assets/planes/generic_jet.tuning.ron
```

Use `PROFILE` = the name the user supplied (default `"normal"`).
Use today's date and the actual winning gain values.
`throttle_ff_gain` is not tuned — preserve the existing value from the file, or default to `0.7` if absent.

---

### 6a — File does not exist

Use **Write** to create `TUNING_FILE`:

```ron
// Tuned PID gains for PLANE_BASENAME
// Profile "PROFILE" — tune YYYY-MM-DD, N rounds
PlaneTuning(
    level_hold: {
        "PROFILE": LevelHoldTuning(
            alt_kp: F,
            alt_ki: F,
            alt_kd: F,
            pitch_kp: F,
            pitch_kd: F,
            spd_kp: F,
            spd_ki: F,
            throttle_ff_gain: 0.7,
        ),
    },
)
```

---

### 6b — File exists, profile already present

Use **Read** to get the file content. Locate the block:

```
"PROFILE": LevelHoldTuning(
    ...
),
```

Use **Edit** to replace the entire `LevelHoldTuning( … )` value for that key with the new values.
Also update (or insert) the date comment for that profile if one exists nearby.

---

### 6c — File exists, profile not present but `level_hold` map is present

Use **Read** to get the file content. Locate the `level_hold: {` line.
Use **Edit** to insert the new profile entry immediately after the opening `{`:

```ron
        "PROFILE": LevelHoldTuning(
            alt_kp: F,
            alt_ki: F,
            alt_kd: F,
            pitch_kp: F,
            pitch_kd: F,
            spd_kp: F,
            spd_ki: F,
            throttle_ff_gain: 0.7,
        ),
```

---

### 6d — File exists but has no `level_hold` field

Use **Edit** to insert `level_hold: { … }` before the final `)` of the `PlaneTuning(` struct:

```ron
    level_hold: {
        "PROFILE": LevelHoldTuning(
            alt_kp: F,
            alt_ki: F,
            alt_kd: F,
            pitch_kp: F,
            pitch_kd: F,
            spd_kp: F,
            spd_ki: F,
            throttle_ff_gain: 0.7,
        ),
    },
```

---

---

# Orbit tuning phases

> These phases run when `CONTROLLER` is `orbit`. They replace Phases 1–6 above entirely.
> The orbit controller is a cascade: all 7 level-hold gains apply to the inner loop;
> `radial_kp`, `radial_kd`, `heading_kp`, `heading_kd` are the outer loop. Use the
> **Orbit template** from the top of this file for every run.

---

## Orbit Phase 1 — Read config and derive physics-based gain estimates

Use the Read tool to open `PLANE`. Extract the same fields as level-hold Phase 1
(`thrust_max`, `mass`, `powerplant`, `wing_area`, `cl0`, `cl_alpha`, `cd0`,
`cd_induced`, `cm_delta_e`, `elevator_limit`).

Compute `mass_loaded` (= `mass + capacity_kg` for JetFuel at full tank, `mass` for
Electric) and the same cruise reference values (`V_c`, `q_c`, `CL_c`, `CD_c`, `T_c`)
using `mass_loaded` — exactly as in level-hold Phase 1.

**Inner-loop candidates** (same formulas as level-hold Phase 1):

```
alt_kp   = 0.5 / V_c
alt_ki   = alt_kp * 0.1
alt_kd   = alt_kp * 2.0
pitch_kp = 0.8 / (|cm_delta_e| * elevator_limit)
pitch_kd = pitch_kp * 0.4
spd_kp   = T_c / (V_c * 2.0)
spd_ki   = spd_kp * 0.5
```

**Outer-loop candidates:**

```
radial_kp  = 0.002 * (100.0 / V_c)   [scales with speed]
radial_kd  = radial_kp * 5.0
heading_kp = 0.7                       [centripetal FF handles steady-state; kp is a correction]
heading_kd = 0.1
```

Show all 11 computed gain values in a brief table before running anything.

**Orbit scenario matrix** (5 scenarios):

| # | Alt (m) | Speed (m/s) | Radius (m) | Rationale |
|---|---|---|---|---|
| A | 500  | 100 | 500  | Tight orbit |
| B | 500  | 100 | 1000 | Standard |
| C | 500  | 100 | 2000 | Wide orbit |
| D | 300  | 80  | 800  | Low/slow |
| E | 1000 | 130 | 1500 | High/fast |

Reject scenarios where `T_c > 0.95` at that speed (cannot sustain level flight).

---

## Orbit Phase 1b — (Optional) Coarse outer-loop grid search

Apply the same trigger criteria as level-hold Phase 1b. When triggered, sweep the
4 outer-loop gains only (inner fixed at Phase 1 physics values):

| Parameter | Base | Multipliers |
|---|---|---|
| `radial_kp`  | Phase 1 | ×0.5, ×1.0, ×2.0 |
| `radial_kd`  | Phase 1 | ×0.5, ×1.0, ×2.0 |
| `heading_kp` | Phase 1 | ×0.5, ×1.0, ×2.0 |
| `heading_kd` | Phase 1 | ×0.5, ×1.0, ×2.0 |

3^4 = 81 grid points evaluated on **scenario B only** (R=1000 m, 500 m / 100 m/s),
using the Orbit template with `STEPS = 1800`.

Score per grid point (over all sampled rows — 29 at 1800 / 64):

```
score = worst |radial_error_m| + worst |heading_error_rad| * 100
```

(The ×100 factor normalises heading radians to roughly the same scale as radial metres.)

Early-stop on a clean pass (all sampled rows within both criteria in Orbit Phase 2).

---

## Orbit Phase 2 — Baseline run (Round 0)

Run all 5 scenarios with the Orbit template (`STEPS = 3840`, `interval: 64`), using each
scenario's altitude / airspeed / radius and the Round 0 gains.

**Pass criteria** (evaluate the last 30 rows of each 60-row run):

- `|altitude_m − TARGET_ALT| ≤ 15 m`
- `|airspeed_ms − TARGET_AIRSPEED| ≤ 3 m/s`
- `|radial_error_m| ≤ 100 m`
- `|heading_error_rad| ≤ 0.3 rad`

**Pathology flags** (note even on a passing run):

- **Aileron saturation** — `aileron` pinned at ±1.0
- **Elevator saturation** — `elevator` pinned at ±1.0
- **NaN / Inf** — abort immediately and report

Print a compact Round 0 table:

```
Round 0 (orbit — physics-based estimates)
Scenario  Alt  Spd  Rad(m)  WorstΔAlt  WorstΔSpd  WorstRadErr  WorstHdgErr  Flags  Result
A         500  100  500     …          …           …            …                   PASS/FAIL
…
```

If all 5 pass → jump directly to Orbit Phase 5.

---

## Orbit Phase 3 — Diagnose failure modes

For each failing scenario, identify the **dominant failure mode**:

| Symptom | Dominant problem | Fix |
|---|---|---|
| `radial_error_m` diverges monotonically | `radial_kp` too low | × 2 radial_kp |
| `radial_error_m` oscillates | `radial_kp` too high | × 0.5 radial_kp, × 1.5 radial_kd |
| `heading_error_rad` large and stable | `heading_kp` too low | × 1.5 heading_kp |
| Bank / roll oscillations (roll_rate cycles) | `heading_kp` too high | × 0.5 heading_kp, × 1.5 heading_kd |
| Altitude drifts monotonically | `alt_ki` too low | × 2 alt_ki |
| Altitude oscillates | `alt_kp` too aggressive | × 0.5 alt_kp, × 1.5 alt_kd |
| Elevator saturated | `pitch_kp` overcommanding | × 0.5 pitch_kp, then × 0.5 alt_kp |
| Airspeed drifts | `spd_ki` too low | × 2 spd_ki |

**Fix order:** inner loop first (altitude/airspeed/pitch), then outer loop (radial, then heading).

---

## Orbit Phase 4 — Iterative refinement (up to 3 rounds)

Same procedure as level-hold Phase 4:

1. Apply fixes from previous diagnosis; show updated gain table.
2. Re-run only the still-failing scenarios with the Orbit template (`STEPS = 3840`).
3. Print result rows; accept/reject per loop.
4. Track best candidate (lowest sum of all 4 worst-case errors across scenarios).
5. Re-diagnose; if same fix tried twice without improvement, try the opposite direction.
6. If Round 2 has 3+ failures and no consistent improvement, run Orbit Phase 1b now
   (if not already run) then restart from Round 1 with the grid-search winner.

---

## Orbit Phase 5 — Full validation run

Run all 5 scenarios with best-candidate gains using the Orbit template.

Print the full summary table:

```
Final validation — orbit (best gains after N rounds)
Scenario  Alt  Spd  Rad(m)  WorstΔAlt  WorstΔSpd  WorstRadErr  WorstHdgErr  Flags  Result
A         500  100  500     …          …           …            …                   PASS/FAIL
…
```

**If all 5 pass:**

```
✓ All orbit scenarios converged (N tuning rounds).

Best gains:
  radial_kp F  radial_kd F
  heading_kp F  heading_kd F
  alt_kp F  alt_ki F  alt_kd F
  pitch_kp F  pitch_kd F
  spd_kp F  spd_ki F

Reproduction (scenario B): generated target/tune.scenario.ron with
  Orbit(radius: 1000, direction: CounterClockwise, altitude: 500, airspeed: 100,
        tuning: (…best gains…)); ran
  cargo run --example observe_state --no-default-features -- --scenario target/tune.scenario.ron

Write these gains to the tuning file? If yes, also provide a profile name (default: "normal").
```

**If any scenario still fails after 3 rounds:** report the best result, per-scenario diagnosis,
and suggested next steps. Do not offer write-back.

---

## Orbit Phase 6 — Write-back (only if user answers yes in Orbit Phase 5)

Derive `TUNING_FILE` the same way as level-hold Phase 6.
`PROFILE` = user-supplied name (default `"normal"`).
`throttle_ff_gain` — preserve from existing file or default to `0.7`.

---

### O6a — File does not exist

Use **Write** to create `TUNING_FILE`:

```ron
// Tuned PID gains for PLANE_BASENAME
// Profile "PROFILE" — orbit tune YYYY-MM-DD, N rounds
PlaneTuning(
    orbit: {
        "PROFILE": OrbitTuning(
            radial_kp:  F,
            radial_kd:  F,
            heading_kp: F,
            heading_kd: F,
            inner: LevelHoldTuning(
                alt_kp:           F,
                alt_ki:           F,
                alt_kd:           F,
                pitch_kp:         F,
                pitch_kd:         F,
                spd_kp:           F,
                spd_ki:           F,
                throttle_ff_gain: 0.7,
            ),
        ),
    },
)
```

---

### O6b — File exists, orbit profile already present

Use **Read** to get the file content. Locate the block:

```
"PROFILE": OrbitTuning(
    ...
),
```

Use **Edit** to replace the entire `OrbitTuning( … )` value for that key with the new values.

---

### O6c — File exists, orbit profile not present but `orbit` map is present

Use **Read** to get the file content. Locate the `orbit: {` line.
Use **Edit** to insert the new profile entry immediately after the opening `{`:

```ron
        "PROFILE": OrbitTuning(
            radial_kp:  F,
            radial_kd:  F,
            heading_kp: F,
            heading_kd: F,
            inner: LevelHoldTuning(
                alt_kp:           F,
                alt_ki:           F,
                alt_kd:           F,
                pitch_kp:         F,
                pitch_kd:         F,
                spd_kp:           F,
                spd_ki:           F,
                throttle_ff_gain: 0.7,
            ),
        ),
```

---

### O6d — File exists but has no `orbit` field

Use **Edit** to insert `orbit: { … }` before the final `)` of the `PlaneTuning(` struct:

```ron
    orbit: {
        "PROFILE": OrbitTuning(
            radial_kp:  F,
            radial_kd:  F,
            heading_kp: F,
            heading_kd: F,
            inner: LevelHoldTuning(
                alt_kp:           F,
                alt_ki:           F,
                alt_kd:           F,
                pitch_kp:         F,
                pitch_kd:         F,
                spd_kp:           F,
                spd_ki:           F,
                throttle_ff_gain: 0.7,
            ),
        ),
    },
```

---

---

# Heading-Hold tuning phases

> These phases run when `CONTROLLER` is `heading_hold`. They replace Phases 1–6 above entirely.
> The heading-hold controller wraps `LevelHoldController` (inner, 7 gains) plus a 2-gain outer
> heading PID (`heading_kp`, `heading_kd`). Tune the inner loop first with `/tune PLANE level_hold`,
> then use these phases to tune only the 2 outer gains. Use the **Heading-hold template** from
> the top of this file for every run.

---

## H1 — Read config and derive physics-based gain estimates

Use the Read tool to open `PLANE`. Extract `mass`, `powerplant`, `wing_area`,
`cl0`, `cl_alpha`. Compute `mass_loaded` (= `mass + capacity_kg` for JetFuel at
full tank, `mass` for Electric) — observe_state flies the loaded mass.

Compute cruise speed:
```
V_c = sqrt(2 * mass_loaded * 9.81 / (1.225 * wing_area * (cl0 + cl_alpha * 0.05)))
```

**Outer-loop candidates:**
```
heading_kp = 0.7    [centripetal kinematics; ≈ constant across speeds]
heading_kd = 0.1    [damps roll oscillation during turn transient]
```

Show the table. Use `heading_kp = 0.7`, `heading_kd = 0.1` as Round 0.

Also determine the **inner gains** to use. If `PLANE.tuning.ron` already has a
`level_hold."normal"` profile, copy those 7 values into `HeadingHoldTuning.inner`.
Otherwise use `LevelHoldTuning::default()` (see `src/controllers/tuning.rs`).

---

## H2 — Scenario matrix

Use 5 heading-step scenarios, all at 1000 m altitude, 80 m/s. The plane spawns level
facing +X (heading 0); `TARGET_HEADING_DEG` is the commanded heading:

| # | Target heading (`TARGET_HEADING_DEG`) | Step | Rationale |
|---|---|---|---|
| A | 0   | 0° | Disturbance hold |
| B | 10  | 10° small step | Small correction |
| C | 30  | 30° medium step | Typical maneuver |
| D | 60  | 60° large step | Max bank excursion |
| E | -30 | 30° opposite | Symmetry check |

For each scenario write the Heading-hold template (`TARGET_ALT = 1000`,
`TARGET_AIRSPEED = 80`, `STEPS = 1280` ≈ 20 s) and run it. Measure from the
`yaw_deg` column:
- **Peak heading error** (deg) — max |yaw_deg − TARGET_HEADING_DEG|
- **Settle time** (s) — `time_s` to reach and stay within ±5°
- **Overshoot** (deg) — exceedance past target before settling

Pass criteria: peak error < 10°, settle time < 15 s, no divergence.

---

## H3 — Iterative refinement (up to 3 rounds)

Diagnose by symptom:

| Symptom | Diagnosis | Adjustment |
|---|---|---|
| Slow convergence, sluggish bank | kp too low | ↑ heading_kp by 20% |
| Overshoot > 10° then oscillates | kp too high or kd too low | ↓ heading_kp 15%, ↑ heading_kd 20% |
| Oscillating roll during turn | kd too low | ↑ heading_kd 20% |
| Heading oscillates around target | kp too high | ↓ heading_kp 15% |
| Roll axis unstable (aileron saturates) | outer loop too fast for inner loop | ↓ heading_kp 25% |

Re-run only the failing scenarios after each change. Stop when all pass or 3 rounds reached.

---

## H4 — Write-back

Use the same write-back logic as Orbit Phase O6 (a/b/c/d), but for `HeadingHoldTuning`:

```ron
    heading_hold: {
        "PROFILE": HeadingHoldTuning(
            heading_kp: F,
            heading_kd: F,
            inner: LevelHoldTuning(
                alt_kp:           F,
                alt_ki:           F,
                alt_kd:           F,
                pitch_kp:         F,
                pitch_kd:         F,
                spd_kp:           F,
                spd_ki:           F,
                throttle_ff_gain: 0.7,
            ),
        ),
    },
```

If the `heading_hold` map already exists, insert a new profile or replace the existing one.
If it does not exist, add `heading_hold: { … },` before the closing `)` of `PlaneTuning(`.
Add a comment above the heading_hold block with the date and brief result summary.
