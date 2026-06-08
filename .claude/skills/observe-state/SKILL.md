---
description: Run the headless observe-state binary to verify level-hold convergence (PID gains or a trained RL policy) or formation-keeping across a plane's flight envelope
---

Verify flight-control behaviour with the scenario-driven `observe_state` example.
The binary takes a single `--scenario PATH` (a `.scenario.ron` file describing
one or more planes) plus optional `--steps` / `--interval` overrides. This skill
generates the scenario files it needs as temporary files under `target/`
(gitignored), runs the binary, and analyses the per-plane CSV.

Three input modes are supported, dispatched on the argument:

- **PID branch** — argument is a `.plane.ron` config. Generates `LevelHold`
  scenarios with inline `tuning`, verifies PID gains across the envelope, and
  offers a gain write-back to the plane file at the end.
- **RL branch** — argument is a `.mpk` trained model under `models/level_hold/`.
  Generates `RlLevelHold` scenarios, runs with `--features inference`, and
  reports a verdict without write-back (RL has no gains to update).
- **Formation branch** — argument is a `.scenario.ron` file containing a
  `Wingman` plane. Runs it as-is and reports a formation-slot-HOLD verdict.

**Requires exactly one argument.** If the user did not supply one — or supplied
a path with an unrecognised extension — stop immediately and print:

```
Usage: /observe-state <path>
  <path> ending in .plane.ron    → verifies PID level-hold convergence
  <path> ending in .mpk          → verifies a trained RL level-hold policy
  <path> ending in .scenario.ron → verifies a multi-plane scenario (e.g. formation)
Examples:
  /observe-state assets/planes/generic_jet.plane.ron
  /observe-state models/level_hold/ppo_level_hold.mpk
  /observe-state assets/scenarios/wingman_formation.scenario.ron
```

Otherwise dispatch to **Step 1A** (`.plane.ron`), **Step 1B** (`.mpk`), or
**Step 1C** (`.scenario.ron`).

> Out of scope for the level-hold envelope sweep: orbit / `rl_orbit*`
> verification. Those need a different scenario matrix (radius, center,
> direction). For ad-hoc orbit or other multi-plane runs, write a `.scenario.ron`
> and drive the binary directly per the reference doc at
> `.agents/skills/observe-state/SKILL.md`.

---

## Scenario file format (shared by all branches)

The binary is driven entirely by `.scenario.ron`. The minimal single-plane
level-hold scenario this skill writes per envelope point is:

```ron
(
    steps: 3840,
    interval: 64,
    planes: [
        (
            name: "p",
            // config omitted → embedded generic_jet. Set `config: "PATH"` to
            // load a specific .plane.ron.
            position: (0.0, TARGET_ALT, 0.0),
            // Spawn AT target airspeed (+X). Required — omitting velocity defaults
            // to 100 m/s, turning a steady-state test into a deceleration transient.
            velocity: (TARGET_AIRSPEED, 0.0, 0.0),
            controller: LevelHold(
                altitude: TARGET_ALT,
                airspeed: TARGET_AIRSPEED,
                // tuning omitted → default gains. Include to test specific gains:
                tuning: (
                    alt_kp: F, alt_ki: F, alt_kd: F,
                    pitch_kp: F, pitch_kd: F,
                    spd_kp: F, spd_ki: F,
                    throttle_ff_gain: 0.7,
                ),
            ),
        ),
    ],
)
```

Optional `Some(...)` is not needed — the loader enables `implicit_some`, so bare
values (`position: (..)`, `tuning: (..)`) parse directly. Write generated files
to `target/observe_<purpose>.scenario.ron`.

The run command is always:

```bash
cargo run --example observe_state --no-default-features -- --scenario <FILE>
# RL branch adds: --features inference
```

---

## Step 1A — PID branch: read the plane config

Use the Read tool to open the supplied `.plane.ron`. Extract:

- `thrust_max` — maximum thrust (N)
- `mass` — aircraft mass (kg)
- `cl_max` — maximum lift coefficient
- `wing_area` (S) — reference area (m²)

Estimate a rough cruise speed range (airspeed where lift ≈ weight at moderate α).
Set `PLANE = <supplied path>` — generated scenarios set `config: "PLANE"`.
Skip to Step 2.

## Step 1B — RL branch: confirm the model and its training envelope

Confirm the model file exists and its parent directory is `models/level_hold/` —
if not, abort with:

```
RL level-hold verification expects a model under models/level_hold/. Got: <path>
```

The model has no embedded plane config, so verification uses the built-in
`generic_jet` defaults (matches `assets/planes/generic_jet.plane.ron`). Set
`MODEL = <supplied path>` and `PLANE = assets/planes/generic_jet.plane.ron`. Read
`PLANE` to extract the same four fields as Step 1A. Generated scenarios omit
`config` (embedded generic_jet) and use a `RlLevelHold` controller. Skip to Step 2.

## Step 1C — Formation branch

Read the supplied `.scenario.ron`. Confirm it contains at least one `Wingman`
plane and the leader it references. Note the leader's name, the wingman's name,
and the wingman's `offset` (body frame). No envelope matrix — run the scenario
as-is. Skip to **Step 5F**.

---

## Step 2 — Choose the scenario matrix (PID and RL branches)

Pick **5 (altitude, airspeed) pairs** that span the plane's envelope. Typical
targets for a generic jet (adjust proportionally based on Step 1 estimates):

| # | Altitude (m) | Airspeed (m/s) | Rationale |
|---|---|---|---|
| A | 200 | 80 | Low / slow |
| B | 500 | 100 | Mid / cruise |
| C | 1000 | 130 | High / fast |
| D | 2000 | 110 | High alt / cruise |
| E | 300 | 70 | Low / near-stall |

Reject any pair where the required lift-to-weight ratio at that airspeed would
demand `CL > cl_max * 0.85` (too close to stall) or where `thrust_max` is clearly
insufficient to sustain level flight at that speed.

For the RL branch, additionally reject any scenario well outside the typical PPO
training envelope (roughly 500–1000 m, 80–130 m/s for the shipped policies) — the
model will not have learned controls for it. State explicitly if you skip one.

---

## Step 3 — Run each scenario

For each (alt, airspeed) pair, write a temp scenario with `steps: 3840`
(60 s at 64 Hz) and `interval: 64` (one row per second → 60 rows), then run it.
The plane spawns **at the target altitude and airspeed** (steady-state test, not
a recovery test).

### Step 3A — PID scenario

Write `target/observe_pid.scenario.ron` with one `LevelHold` plane:
`config: "PLANE"`, `position: (0, TARGET_ALT, 0)`, `velocity: (TARGET_AIRSPEED, 0, 0)`, `controller: LevelHold(altitude: TARGET_ALT, airspeed: TARGET_AIRSPEED, tuning: (…the gains under test…))`.
Then:

```bash
cargo run --example observe_state --no-default-features -- --scenario target/observe_pid.scenario.ron
```

### Step 3B — RL scenario

Write `target/observe_rl.scenario.ron` with one `RlLevelHold` plane (omit
`config`): `position: (0, TARGET_ALT, 0)`, `velocity: (TARGET_AIRSPEED, 0, 0)`, `controller: RlLevelHold(model: "MODEL", altitude: TARGET_ALT, airspeed: TARGET_AIRSPEED)`.
Then:

```bash
cargo run --example observe_state --no-default-features --features inference -- --scenario target/observe_rl.scenario.ron
```

The `.mpk` extension on the model path is optional. No gain overrides apply to RL.

---

## Step 4 — Output columns

```
step, time_s, plane, pos_x, altitude_m, pos_z, airspeed_ms, alpha_deg, beta_deg,
roll_deg, pitch_deg, yaw_deg, pitch_rate, roll_rate, yaw_rate,
elevator, throttle, aileron, rudder, radial_error_m, heading_error_rad, bank_ff_rad
```

One row per plane per sampled step (single-plane envelope runs → one row per
step). The trailing three orbit columns are blank for level-hold planes. Step 0
often shows all-zero state — normal, the state populates by step 1.

---

## Step 5 — Check convergence per scenario (PID and RL branches)

Examine the **last 30 rows** of each run. A scenario **passes** if, for every row:

- `|altitude_m - TARGET_ALT| ≤ 10 m`
- `|airspeed_ms - TARGET_AIRSPEED| ≤ 2 m/s`

Record worst altitude error, worst airspeed error, and PASS/FAIL. Flag secondary
pathologies even on a passing run:
- **Elevator saturation** — `elevator` pinned at ±1.0
- **Pitch oscillation** — `pitch_rate` cycling with amplitude > 0.05 rad/s
- **NaN / Inf** — physics blow-up; abort immediately and report

RL inference is deterministic (`mean_action`, no sampling) — two identical runs
should produce byte-identical CSV; if not, something else is varying.

## Step 5F — Check formation hold (Formation branch)

Run the supplied scenario for its declared steps (or override `--steps 1920`,
`--interval 64`). The output interleaves leader and wingman rows per step.

Evaluate over a **steady-state window only** — the wingman settles into the slot
within the first few seconds, so the hold criterion applies *after* the
transient. Use the **final 15 sampled rows** (≈ final 15 s at `interval: 64`) and
**always discard step 0** (its state is all-zero before the first physics tick).
Do not use "last 30 rows": the shipped scenario emits only 30 rows total
(1920 / 64), so that would include the entire startup transient and falsely fail.

Compute the wingman's slot error in that window. The desired world slot for a
level leader is the leader position plus its body-frame `offset` mapped to world
via `(x,y,z) → (x, z, -y)` (the `from_rotation_x(-π/2)` base attitude). With
leader `yaw_deg` ≈ 0 (flying +X) this reduces to:

```
slot_x = leader.pos_x + offset_x        # offset_x body → world +X (fore-aft)
slot_y = leader.altitude_m + offset_z   # offset_z body → world +Y (vertical)
slot_z = leader.pos_z - offset_y        # offset_y body → world -Z (lateral)
```

(For the shipped `(-20, 15, 0)` offset: slot = leader + (-20, 0, -15).) The
controller uses the leader's *full* attitude (`wingman.rs:97`); the yaw-only
reduction above holds only while the leader is wings-level. If the leader is
turning (`yaw_deg` drifting), rotate the body offset by the leader yaw about
world +Y before adding — and note that bank/pitch make even that an approximation.

The scenario **passes** if, over that steady-state window:
- fore-aft, lateral, and vertical slot errors each stay `≤ 5 m`
- `yaw_deg` of the wingman stays bounded (does not grow monotonically)

`WingmanController` is a pure lateral-position→bank loop with no heading
damping: it HOLDS an on-slot equilibrium but spirals away from a large initial
offset. Treat a monotonically growing `yaw_deg` / `pos_z` as **divergence** and
FAIL, noting the wingman was likely spawned off-slot.

---

## Step 6 — Report results

Print a summary table.

**PID / RL branch:**

```
Scenario  Alt(m)  Speed(m/s)  Worst ΔAlt(m)  Worst ΔSpd(m/s)  Result
A         200     80          3.2            0.8              PASS
...
```

**PID branch, all pass →**

```
All 5 scenarios converged over 60 s.

Exact reproduction (scenario B): generated target/observe_pid.scenario.ron with
LevelHold(altitude: 500, airspeed: 100, tuning: (…gains…)); ran
  cargo run --example observe_state --no-default-features -- --scenario target/observe_pid.scenario.ron

Write these PID gains to PLANE as a comment block? (yes/no)
```

Continue to Step 7 if the user answers yes.

**RL branch, all pass →** report the verdict and the reproduction command (with
`--features inference`). No write-back — the model is the source of truth.

**Formation branch →** report per-axis worst slot error over the final window and
PASS/FAIL. No write-back.

**Any fail →** print which scenario failed, the worst errors, and a brief
diagnosis (trim offset / saturation / oscillation for PID; out-of-distribution
operating point or bad policy for RL; off-slot spawn / spiral for formation).
Do not offer write-back. For PID, suggest which gain to adjust and re-run.

---

## Step 7 — Write-back (PID branch only, if user answered yes)

Append the following comment block at the **bottom of the PLANE file** (after the
closing `)`):

```
// --- Tuned PID gains (observe-state YYYY-MM-DD) ---
// alt_kp F  alt_ki F  alt_kd F
// pitch_kp F  pitch_kd F
// spd_kp F  spd_ki F
```

Use today's date and the actual gain values used in the scenario `tuning` block.
If the file already has a `// --- Tuned PID gains` block, replace it in place.
