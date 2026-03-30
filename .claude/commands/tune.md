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

## Phase 2 — Baseline run (Round 0)

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

Write these gains to PLANE as a comment block? (yes/no)
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

Append (or replace an existing block) at the **bottom of `PLANE`**, after the closing `)`:

```
// --- Tuned PID gains (tune YYYY-MM-DD, N rounds) ---
// --alt-kp F --alt-ki F --alt-kd F
// --alpha-kp F --alpha-kd F
// --spd-kp F --spd-ki F
```

Use today's date. Use the actual winning gain values.
If the file already contains a `// --- Tuned PID gains` block, replace it in place (do not append
a second block).
