---
description: Iteratively tune level-hold PID gains for a plane config and write back the best result
---

Auto-tune the level-hold PID gains for a given plane config. Runs physics-based initial estimates,
then iterates (up to 3 rounds) adjusting gains based on observed failure modes until all 5 envelope
scenarios converge — then offers to write the winning gains back to the config file.

**Requires a plane config argument.** If the user did not supply a `.ron` path, stop immediately and print:

```
Usage: /tune <path/to/plane.plane.ron>
Example: /tune assets/planes/generic_jet.plane.ron
```

Otherwise, proceed with the steps below using the supplied path as `PLANE`.

---

## Phase 1 — Read config and derive physics-based gain estimates

Use the Read tool to open `PLANE`. Extract:

- `thrust_max` — maximum thrust (N)
- `mass` — aircraft mass (kg)
- `wing_area` (S) — reference area (m²)
- `cl0`, `cl_alpha`, `cl_max`
- `cd0`, `cd_induced`
- `cm_delta_e`, `elevator_limit`
- `mean_chord`

**Compute cruise reference values** (air density ρ = 1.225 kg/m³):

```
V_c  = sqrt(2 * mass * 9.81 / (1.225 * wing_area * (cl0 + cl_alpha * 0.05)))
q_c  = 0.5 * 1.225 * V_c²
CL_c = mass * 9.81 / (q_c * wing_area)
CD_c = cd0 + cd_induced * CL_c²
T_c  = (q_c * wing_area * CD_c) / thrust_max   [normalised 0–1]
```

**Compute physics-based candidate gains** (Round 0):

```
alt_kp   = 0.5 / V_c
alt_ki   = alt_kp * 0.1
alt_kd   = alt_kp * 2.0
alpha_kp = 0.8 / (|cm_delta_e| * elevator_limit)
alpha_kd = alpha_kp * 0.4
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
- `thrust_max / (mass × 9.81)` is outside [0.3, 0.8]
- Computed `V_c` is below 50 m/s or above 200 m/s
- Computed `T_c` exceeds 0.85 or is below 0.1
- (Post-Round-0 fallback) Round 0 fails 3+ scenarios **and** failure modes are inconsistent
  across scenarios — different fixes indicated per scenario, suggesting the estimates are
  simply wrong rather than systematically biased

**Skip Phase 1b when** all cruise reference values are within the ranges above or the plane
is close to `generic_jet` defaults.

---

### Grid definition

Keep `alpha_kp` and `alpha_kd` fixed at Phase 1 physics values. Sweep outer loops only:

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

### Run command (per grid point)

```bash
cargo run --example observe_state --no-default-features -- \
  --plane PLANE \
  --steps 1800 \
  --interval 60 \
  --controller level_hold \
  --altitude 500 \
  --airspeed 100 \
  --alt-kp ALT_KP --alt-ki ALT_KI --alt-kd ALT_KD \
  --alpha-kp ALPHA_KP --alpha-kd ALPHA_KD \
  --spd-kp SPD_KP --spd-ki SPD_KI
```

Each run produces 30 rows. Evaluate all 30 as the final window
(same pass criteria: `|ΔAlt| ≤ 10 m`, `|ΔSpd| ≤ 2 m/s`).

### Selection

Score each grid point:

```
score = worst |altitude_m − 500| + worst |airspeed_ms − 100|   (over all 30 rows)
```

**Early stopping:** if a grid point passes cleanly (both criteria on all 30 rows), stop
and use it as the winner immediately. Otherwise evaluate all 243 and pick the lowest score.

### Output

```
Phase 1b grid search — scenario B (500 m / 100 m/s), 30-row window
Top 3 candidates:
  Rank  alt_kp  alt_ki  alt_kd  spd_kp  spd_ki  Score  Pass?
  1     F       F       F       F       F       F      YES/NO  ← winner
  2     F       F       F       F       F       F      YES/NO
  3     F       F       F       F       F       F      YES/NO

Winner outer-loop gains (alpha loop unchanged from Phase 1):
  --alt-kp F --alt-ki F --alt-kd F
  --alpha-kp F --alpha-kd F   [unchanged]
  --spd-kp F --spd-ki F
```

Replace the physics-based outer-loop gains with the winner's values. Phase 2 runs these
as Round 0 across all 5 scenarios.

---

## Phase 2 — Baseline run (Round 0)

> If Phase 1b ran, the gains here are the grid-search winner's outer-loop values with
> physics-based `alpha_kp`/`alpha_kd`. Otherwise they are the raw Phase 1 physics estimates.
> The procedure and pass criteria are identical either way.

Run all 5 scenarios with the Round 0 gains computed above (60 s, 3600 steps at 60 Hz):

```bash
cargo run --example observe_state --no-default-features -- \
  --plane PLANE \
  --steps 3600 \
  --interval 60 \
  --controller level_hold \
  --altitude TARGET_ALT \
  --airspeed TARGET_AIRSPEED \
  --alt-kp ALT_KP --alt-ki ALT_KI --alt-kd ALT_KD \
  --alpha-kp ALPHA_KP --alpha-kd ALPHA_KD \
  --spd-kp SPD_KP --spd-ki SPD_KI
```

**Output columns:** `step, time_s, altitude_m, airspeed_ms, alpha_deg, beta_deg, pitch_rate, roll_rate, yaw_rate, elevator, throttle, aileron, rudder`

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
| Elevator saturated at ±1 for most of run | `alt_kp` or `alpha_kp` overcommanding | × 0.5 alpha_kp, then × 0.5 alt_kp |
| Alpha overshoots then slowly settles | `alpha_kd` too low | × 1.5 alpha_kd |
| Airspeed drifts monotonically | `spd_ki` too low | × 2 spd_ki |
| Airspeed oscillates around target | `spd_ki` too high | × 0.5 spd_ki |
| Airspeed sluggish but stable | `spd_kp` too low | × 2 spd_kp |

**Fix order when multiple loops fail:** innermost first (alpha → altitude → airspeed).

Describe the diagnosis briefly ("Scenario A: elevator saturated in first 10 s, alpha_kp likely
too high → halving alpha_kp for Round 1").

---

## Phase 4 — Iterative refinement (up to 3 rounds)

Repeat the following for rounds 1, 2, and 3 (stop early if all scenarios pass):

1. Apply the fix(es) identified in the previous round's diagnosis to produce new candidate gains.
   Show the updated gain table before running.

2. Run **only the scenarios that are still failing** (not all 5 — saves time):

   ```bash
   cargo run --example observe_state --no-default-features -- \
     --plane PLANE \
     --steps 3600 \
     --interval 60 \
     --controller level_hold \
     --altitude TARGET_ALT \
     --airspeed TARGET_AIRSPEED \
     --alt-kp ALT_KP --alt-ki ALT_KI --alt-kd ALT_KD \
     --alpha-kp ALPHA_KP --alpha-kd ALPHA_KD \
     --spd-kp SPD_KP --spd-ki SPD_KI
   ```

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

Run all 5 scenarios with the **best-candidate gains** (60 s, 3600 steps):

```bash
cargo run --example observe_state --no-default-features -- \
  --plane PLANE \
  --steps 3600 \
  --interval 60 \
  --controller level_hold \
  --altitude TARGET_ALT \
  --airspeed TARGET_AIRSPEED \
  --alt-kp ALT_KP --alt-ki ALT_KI --alt-kd ALT_KD \
  --alpha-kp ALPHA_KP --alpha-kd ALPHA_KD \
  --spd-kp SPD_KP --spd-ki SPD_KI
```

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
  --alt-kp F --alt-ki F --alt-kd F
  --alpha-kp F --alpha-kd F
  --spd-kp F --spd-ki F

Exact reproduction command (scenario B as reference):
  cargo run --example observe_state --no-default-features -- \
    --plane PLANE --steps 3600 --interval 60 \
    --altitude 500 --airspeed 100 \
    [gains above]

Once written to the tuning file, the equivalent command will be:
  cargo run --example observe_state --no-default-features -- \
    --plane PLANE --tuning-file TUNING_FILE --profile PROFILE \
    --steps 3600 --interval 60 --altitude 500 --airspeed 100

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

You can continue tuning manually:
  cargo run --example observe_state --no-default-features -- \
    --plane PLANE --steps 3600 --interval 60 \
    --altitude TARGET_ALT --airspeed TARGET_AIRSPEED \
    [best gains found]
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
            alpha_kp: F,
            alpha_kd: F,
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
            alpha_kp: F,
            alpha_kd: F,
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
            alpha_kp: F,
            alpha_kd: F,
            spd_kp: F,
            spd_ki: F,
            throttle_ff_gain: 0.7,
        ),
    },
```
