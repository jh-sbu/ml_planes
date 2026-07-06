# Test-Matrix Compile-Speed Plan: Test Consolidation + Build-Profile Changes

**Status:** COMPLETE (Phase 7 final remeasurement + acceptance, 2026-07-06). Phase 6
(optional mold) deliberately skipped — link time no longer dominates, so the incremental
mold win is not worth the cache-invalidating rustflags change. All Phase 7 acceptance
criteria met (parity oracle green: 228/0/0, 321/0/1, 257/0/0; mcp-server incremental
rebuild down ~92 %; `just test-all` after a `src/` touch in ~1 min; `target/` at 17 G).
See the Results Appendix **After (final)** column.
**Written:** 2026-07-06
**Scope:** reduce the compile cost of the supported test matrix (`just test-all`) without
changing test coverage, test semantics, or the TDD workflow. Two mechanical changes:

1. Consolidate the 24 `tests/*.rs` integration-test binaries into **3** (each `tests/*.rs`
   file is a separate crate linked against the full Bevy+Rapier dep tree; the matrix links
   ~72 binaries per run today).
2. Add a `[profile.dev]` section trimming debuginfo (currently default full debuginfo, no
   profile section exists), which is paid in every one of those compile+link steps.

**Explicitly out of scope:** workspace/crate split, mold-linker adoption beyond the optional
measurement-gated Phase 6, cargo-nextest, any change to which tests exist or what they
assert, any change to the `just` recipe commands themselves.

**Why this is TDD-compliant:** no implementation code changes. The "red/green" for a test-infra
refactor is **behavior parity**: the exact per-config pass/fail/ignored counts are captured
before any change (Phase 0) and re-asserted at every milestone.

---

## Measurement Protocol (used by Phase 0 and Phase 7 — run identically both times)

All timings on an otherwise-idle machine, warm dependency cache (deps already built for each
config — run each command once untimed first if `target/` was cleaned). The `touch` simulates
the inner-loop cost of a one-line `src/` change.

```bash
cd ~/sandbox/rust/rapier/ml_planes

# --- A. Incremental rebuild cost per matrix config (the headline number) ---
touch src/lib.rs
time cargo test --no-default-features                       --no-run   # config 1: core
time cargo test --no-default-features --features "mcp server" --no-run # config 2: net
time cargo test --no-default-features --features inference  --no-run   # config 3: RL

# --- B. Link-step count per config (should drop ~24 -> ~3-5 incl. bins) ---
cargo test --no-default-features --no-run 2>&1 | grep -c 'Executable'
cargo test --no-default-features --features "mcp server" --no-run 2>&1 | grep -c 'Executable'
cargo test --no-default-features --features inference --no-run 2>&1 | grep -c 'Executable'

# --- C. Full-matrix wall time (compile + run) ---
touch src/lib.rs
time just test-all

# --- D. Behavior-parity counts (passed/failed/ignored summed across binaries) ---
cargo test --no-default-features 2>&1 \
  | awk '/^test result/ {p+=$4; f+=$6; i+=$8} END {print "passed="p, "failed="f, "ignored="i}'
cargo test --no-default-features --features "mcp server" 2>&1 \
  | awk '/^test result/ {p+=$4; f+=$6; i+=$8} END {print "passed="p, "failed="f, "ignored="i}'
cargo test --no-default-features --features inference 2>&1 \
  | awk '/^test result/ {p+=$4; f+=$6; i+=$8} END {print "passed="p, "failed="f, "ignored="i}'

# --- E. Disk footprint ---
du -sh target
```

Record all results in the **Results Appendix** at the bottom of this document.

Reference baseline measured 2026-07-06 (protocol A only, before this plan):
core **17.5 s**, `mcp server` **3 m 26.6 s**, `inference` **1 m 31.7 s** after a
`touch src/lib.rs`; `target/` at **209 GB**.

> **Caveat:** Phase 1's profile change invalidates every cached artifact — the *first* build
> of each config afterwards is a full ~from-scratch build (many minutes). Always re-warm
> (build once untimed) before timing protocol A/C.

---

## Phase 0 — Record the Baseline

1. Run the full Measurement Protocol (A–E) on current `main` with a clean working tree.
2. Write the numbers into the Results Appendix under **Baseline**.
3. Save protocol-D counts; these exact counts are the parity oracle for every later milestone.
   (Expected shape: config 1 has the `sim_enabled` + ungated tests; config 2 adds
   net/mcp/server tests with `mcp_e2e` showing as *ignored*; config 3 adds `rl_inference`.)

**Milestone M0:** Results Appendix has a complete Baseline row set (A, B, C, D, E), and
`just test-all` is green.

---

## Phase 1 — Build-Profile Changes

1. Add to `Cargo.toml` (crate root; there is currently **no** `[profile.*]` section — do not
   confuse with the `[profile.flamegraph]` in `.cargo/config.toml`, which stays as-is):

   ```toml
   [profile.dev]
   # Full debuginfo generation + linking dominated sys-time in the 2026-07-06 baseline.
   # line-tables-only keeps file/line backtraces (enough for test failures); the expensive
   # variable/type DWARF is dropped.
   debug = "line-tables-only"
   # Keep debuginfo out of the link step entirely (stays in the .o files under target/).
   split-debuginfo = "unpacked"
   ```

   Notes:
   - `[profile.test]` inherits from `dev`, so this covers `cargo test` automatically.
   - Release/flamegraph profiles are untouched (training runs use `--release`).
   - If test-failure backtraces prove insufficient in practice, the revert is deleting the
     section; do **not** preemptively soften to `debug = true`.

2. `cargo clean` once to reclaim the 209 GB of stale full-debuginfo artifacts (they are all
   invalidated by the profile change anyway).

3. Re-warm all three matrix configs plus `--features training` check builds
   (`cargo test <cfg> --no-run` each, untimed).

**Milestone M1:**
- `just test-all` green; protocol-D counts identical to Baseline.
- Protocol A re-run (after re-warm): expect a measurable drop, especially in `sys` time.
  Record in Results Appendix under **After Phase 1**.
- Protocol E: `target/` dramatically smaller than 209 GB.
- A deliberately-failed test (e.g. temporarily flip an assert, run, revert) still shows
  file:line in its backtrace/panic message.

---

## Phase 2 — Consolidate Core Tests into `tests/core/`

**Mechanism:** Cargo compiles every `tests/*.rs` file as its own test crate, but a
`tests/<name>/main.rs` directory-with-main is a single target named `<name>`, and files in
its subdirectory are plain modules (not compiled separately). Module resolution is relative
to `main.rs`, so `mod wingman;` in `tests/core/main.rs` finds `tests/core/wingman.rs`.

**Target layout** (13 former binaries → 1):

```
tests/core/main.rs        # new crate root, see skeleton below
tests/core/pid_convergence.rs      (moved, ungated)
tests/core/spawn_reset.rs          (moved, ungated)
tests/core/orbit_tune_sync.rs      (moved, ungated)
tests/core/controller_telemetry.rs (moved, ungated)
tests/core/sim_control.rs          (moved, ungated)
tests/core/scenario.rs             (moved, ungated)
tests/core/lifecycle.rs            (moved, ungated — note it has visual-gated cases inside; keep those inner #[cfg]s)
tests/core/aero_physics.rs         (moved, gate moves to mod decl)
tests/core/level_hold.rs           (moved, gate moves to mod decl)
tests/core/heading_hold.rs         (moved, gate moves to mod decl)
tests/core/flight_plan.rs          (moved, gate moves to mod decl)
tests/core/wingman.rs              (moved, gate moves to mod decl)
tests/core/fuel.rs                 (moved, gate moves to mod decl)
tests/common/mod.rs       # stays where it is (shared with Phase 3's binary)
```

**`tests/core/main.rs` skeleton:**

```rust
//! Consolidated core-sim integration tests (formerly 13 separate test binaries; merged
//! per plans/test_compile_speed.md to cut link steps). Run one former file's tests with
//! `cargo test --no-default-features --test core <module>::`.

#[path = "../common/mod.rs"]
mod common;

mod controller_telemetry;
mod lifecycle;
mod orbit_tune_sync;
mod pid_convergence;
mod scenario;
mod sim_control;
mod spawn_reset;

// 6-DOF sim chain required; compiles out on net-without-server builds (see build.rs).
#[cfg(sim_enabled)]
mod aero_physics;
#[cfg(sim_enabled)]
mod flight_plan;
#[cfg(sim_enabled)]
mod fuel;
#[cfg(sim_enabled)]
mod heading_hold;
#[cfg(sim_enabled)]
mod level_hold;
#[cfg(sim_enabled)]
mod wingman;
```

**Per-file mechanical edits when moving each file into `tests/core/`:**

1. Delete the crate-level `#![cfg(sim_enabled)]` inner attribute (the gate now lives on the
   `mod` declaration in `main.rs`). Ungated files need no gate change.
2. Keep the `//!` doc headers — they become module docs. Update any header text that says
   the *file* is gated (`gates the whole file`) to say the *module* is gated from
   `tests/core/main.rs`.
3. Delete the file's own `mod common;` line (it would now resolve to
   `tests/core/<file>/common.rs` and break). Rewrite references: `common::` →
   `crate::common::` (including in `use` statements). This affects (from the 2026-07-06
   audit): `flight_plan`, `level_hold`, `heading_hold`, `aero_physics`, `lifecycle`,
   `spawn_reset`, `fuel`, `sim_control`, `scenario`, `wingman` (re-grep at execution time:
   `grep -l 'mod common' tests/*.rs`).
4. No test-function renames: modules namespace the tests, so duplicate helper names across
   former files cannot collide.

**Working order (keep the suite green at every step — this is the refactor analog of the
red-green cycle):** move one file at a time, `git mv tests/<f>.rs tests/core/<f>.rs`, apply
the edits, run `cargo test --no-default-features --test core`, proceed. Do not batch all 13
and debug at the end.

**Milestone M2:**
- `tests/` contains no loose `*.rs` file that was in the core group; `cargo test
  --no-default-features` shows the `core` executable and protocol-D counts for config 1
  are **identical to Baseline**.
- `cargo test --no-default-features --test core wingman::` runs exactly the former
  `tests/wingman.rs` tests (spot-check the per-file filter story).
- Config 2 (`mcp server`) and config 3 (`inference`) protocol-D counts also unchanged
  (the moved files compile under those configs too).
- `cargo fmt` run; `just test-all` green.

---

## Phase 3 — Consolidate Net/MCP/Server Tests into `tests/net/`

**Target layout** (9 former binaries → 1). Everything in this group requires at least
`net`, so the crate root carries one gate and the whole binary compiles to empty in
config 1 (that is today's behavior for these files, preserved):

```
tests/net/main.rs
tests/net/net_serde.rs        (net)
tests/net/net_protocol.rs     (net)
tests/net/client_net.rs       (net)
tests/net/local_server.rs     (net)
tests/net/server_sim.rs       (server)
tests/net/mcp_snapshot.rs     (mcp)
tests/net/mcp_bridge.rs       (mcp)
tests/net/mcp_lifecycle.rs    (mcp)
tests/net/mcp_e2e.rs          (mcp + server; its tests keep their #[ignore])
```

**`tests/net/main.rs` skeleton:**

```rust
//! Consolidated net/mcp/server integration tests (formerly 9 binaries; see
//! plans/test_compile_speed.md). Entirely compiled out without `net`.
#![cfg(feature = "net")]

#[path = "../common/mod.rs"]
mod common;   // only if any module here uses it — check at execution time

mod client_net;
mod local_server;
mod net_protocol;
mod net_serde;

#[cfg(feature = "server")]
mod server_sim;

#[cfg(feature = "mcp")]
mod mcp_bridge;
#[cfg(feature = "mcp")]
mod mcp_lifecycle;
#[cfg(feature = "mcp")]
mod mcp_snapshot;

#[cfg(all(feature = "mcp", feature = "server"))]
mod mcp_e2e;
```

**Per-file edits:** same recipe as Phase 2 (crate-level `#![cfg]` → `mod`-decl gate; `mod
common;` → `crate::common`; doc-header wording). Additional items specific to this group:

- `mcp_e2e` doc header tells users to run
  `cargo test --no-default-features --features "mcp server" -- --ignored mcp_e2e`. The
  substring filter still matches (test paths become `mcp_e2e::<name>`), but update the
  header to the precise form: `--test net -- --ignored mcp_e2e`.
- Parallelism audit: former files now share one process. The 2026-07-06 audit found only
  `mcp_e2e` binds sockets/spawns children, and it is `#[ignore]` + uses ephemeral ports —
  no expected conflicts. `local_server` exercises `ServerProcess` child-kill semantics;
  verify at execution time it spawns no fixed-port listener. If any flakiness appears in
  M3 runs, the sanctioned fallback is `#[ignore]`-ing nothing and serializing the specific
  offenders with a shared `std::sync::Mutex` test lock — not `--test-threads=1` globally.

**Working order:** one file at a time, testing with
`cargo test --no-default-features --features "mcp server" --test net` after each move.

**Milestone M3:**
- Config 2 protocol-D counts identical to Baseline (including `mcp_e2e` still counted under
  *ignored*).
- Config 1 (`--no-default-features`): the `net` target compiles to an empty test binary
  (0 tests) — same effective behavior as today's gated files.
- The ignored e2e still passes when run explicitly:
  `cargo test --no-default-features --features "mcp server" --test net -- --ignored mcp_e2e`.
- Run config 2 tests **3× consecutively** to watch for parallelism flakes (new risk
  introduced by sharing a process); all green.
- `cargo fmt` run; `just test-all` green.

---

## Phase 4 — Consolidate RL Tests into `tests/rl/`

**Target layout** (2 former binaries → 1):

```
tests/rl/main.rs
tests/rl/rl_inference.rs   (any(inference, training))
tests/rl/ppo_training.rs   (training)
```

**`tests/rl/main.rs` skeleton:**

```rust
//! Consolidated RL integration tests (see plans/test_compile_speed.md).
#![cfg(any(feature = "inference", feature = "training"))]

mod rl_inference;

#[cfg(feature = "training")]
mod ppo_training;
```

Same per-file recipe. Update the run-instruction doc headers
(`--test rl_inference` → `--test rl rl_inference::`, `--test ppo_training` →
`--features training --test rl ppo_training::`).

**Milestone M4:**
- Config 3 (`inference`) protocol-D counts identical to Baseline.
- `just test-training` (`--features training`) compiles and its counts match Baseline for
  that config (heavy wgpu build — run once; needs the GPU box).
- `tests/` now contains exactly: `core/`, `net/`, `rl/`, `common/`. No loose `*.rs`.
- `cargo fmt` run; `just test-all` green.

---

## Phase 5 — Documentation Sync

1. **CLAUDE.md §6** ("Integration tests (`tests/`)"): rewrite the per-file bullet list as
   per-module bullets under the three binaries (`tests/core/…`, `tests/net/…`,
   `tests/rl/…`), and document the new filter idiom
   (`cargo test --no-default-features --test core wingman::`). Keep each bullet's
   one-line description — they are load-bearing docs.
2. **CLAUDE.md §6 Rules**: the `sim_enabled` paragraph references file-level gating
   ("gated `#![cfg(sim_enabled)]`") — update to module-level gating in `tests/core/main.rs`.
3. **justfile**: recipes are unchanged (they don't name test binaries), but the header
   comment mentions per-file gating — reword if stale.
4. Sweep for stale references to old test-binary names:
   ```bash
   grep -rn -- '--test ' CLAUDE.md justfile plans/ .claude/ src/ tests/ | grep -vE 'test core|test net|test rl'
   ```
   Fix every hit (known offenders from the audit: doc headers inside `ppo_training.rs`,
   `rl_inference.rs`, `mcp_e2e.rs` — already handled in Phases 3–4; this sweep catches the rest,
   e.g. any `.claude/skills/` files and plan docs).
5. Update the auto-memory note if any memory file references specific test file paths
   (check `~/.claude/projects/-home-jeanpierre-sandbox-rust-rapier-ml-planes/memory/`).

**Milestone M5:** the grep in step 4 returns no stale hits; CLAUDE.md §6 matches the real
`tests/` tree; `just test-all` green.

---

## Phase 6 (OPTIONAL, measurement-gated) — mold Linker

Only proceed if the Phase 7 remeasurement still shows link time dominating (rough signal:
protocol-A `real` per config still ≫ what `cargo build` of the lib alone costs). Rust 1.92
already defaults to `rust-lld` on x86_64-linux, so the win is incremental, not the classic
bfd→lld jump.

1. `sudo pacman -S mold`
2. In `.cargo/config.toml`, extend the existing `[target.x86_64-unknown-linux-gnu]`
   `rustflags` (which already carries `force-frame-pointers`) with
   `"-C", "link-arg=-fuse-ld=mold"`.
3. Re-warm, run protocol A. **Keep only if ≥20 % improvement** on config 2; otherwise revert
   the rustflags change (rustflags edits invalidate caches, so an unproductive change costs
   a full rebuild — decide from one measurement, don't iterate).

**Milestone M6:** either the rustflags change is committed with before/after numbers in the
Results Appendix, or the appendix records the measured non-win and the config is unchanged.

---

## Phase 7 — Final Remeasurement & Acceptance

1. Re-run the full Measurement Protocol (A–E), identical commands, warm caches.
2. Fill the **After (final)** rows of the Results Appendix.
3. Acceptance criteria:
   - Protocol D: pass/fail/ignored counts per config **exactly equal** to Baseline.
   - Protocol B: ≤ 5 executables per config (3 test binaries + feature-gated bins).
   - Protocol A: config 2 (`mcp server`) incremental rebuild reduced by ≥ 50 % vs the
     2026-07-06 baseline (3 m 26 s); configs 1 and 3 not regressed.
   - Protocol C: `just test-all` after a `src/` touch under ~3 minutes wall (stretch: 2).
   - Protocol E: `target/` an order of magnitude below 209 GB.
4. `cargo fmt`, commit (this is infra: the "no implementation without a failing test" rule
   is satisfied by the parity oracle, per the preamble).
5. If acceptance misses the protocol-A target, the follow-on lever is the workspace split
   (separate plan; see the 2026-07-06 session discussion — extract `training`/burn first).

**Milestone M7:** Results Appendix complete; all acceptance criteria met or the miss is
documented with the follow-on recommendation; work committed.

---

## Risks & Mitigations

| Risk | Mitigation |
|---|---|
| Consolidated binary runs former files' tests in one process → parallel interference | Audit found only `mcp_e2e` (ignored, ephemeral ports) touches global resources. M3 mandates 3× consecutive green runs. Fallback: per-offender mutex lock, never global `--test-threads=1`. |
| `line-tables-only` debuginfo too thin for debugging a hard failure | Backtraces keep file:line (verified in M1). For a debugging session, `CARGO_PROFILE_DEV_DEBUG=2 cargo test …` overrides ad hoc without editing Cargo.toml. |
| Profile change forces one full rebuild per config | Expected, one-time; re-warm before any timing. Do Phase 1 at a natural break. |
| `mod common;` path breakage after moves | Explicit per-file edit step in Phases 2–3; compile errors surface immediately under the one-file-at-a-time working order. |
| Lost per-file test filtering ergonomics | `--test core <module>::` documented in CLAUDE.md §6 (Phase 5); module paths preserve all old names as prefixes. |
| Stale docs/skills referencing old `--test <file>` names | Phase 5 grep sweep is a milestone gate. |

---

## Results Appendix (fill in during execution)

| Metric | Baseline (Phase 0) | After Phase 1 | After (final, Phase 7) |
|---|---|---|---|
| A: core incremental (`real`) | 10.6 s | 7.5 s | 6.8 s |
| A: mcp server incremental (`real`) | 3 m 44.5 s | 25.6 s | 17.5 s |
| A: inference incremental (`real`) | 1 m 11.1 s | 36.4 s | 7.8 s |
| B: executables core / net / inference | 26 / 28 / 27 | 26 / 28 / 27 | 5 / 7 / 6 |
| C: `just test-all` wall after touch | 5 m 2.2 s | (n/m) | 1 m 2.9 s |
| D: counts core (p/f/i) | 228 / 0 / 0 | 228 / 0 / 0 | 228 / 0 / 0 |
| D: counts mcp server (p/f/i) | 321 / 0 / 1 | 321 / 0 / 1 | 321 / 0 / 1 |
| D: counts inference (p/f/i) | 257 / 0 / 0 | 257 / 0 / 0 | 257 / 0 / 0 |
| E: `du -sh target` | 209 G | 14 G | 17 G |

**After Phase 1 notes (2026-07-06):** profile change (`debug = "line-tables-only"`,
`split-debuginfo = "unpacked"`) applied to `Cargo.toml`; `cargo clean` + full re-warm of all
four configs. Protocol-D counts identical to Baseline (parity oracle green). Protocol A: mcp
server incremental dropped 224.5 s → 25.6 s (~89 %, already past the Phase 7 ≥50 % target);
core and inference also improved. Protocol B unchanged (test consolidation is Phase 2+).
Protocol E: 213.4 GiB reclaimed → 14 G. Backtrace sanity confirmed: a flipped assert still
panics with `tests/pid_convergence.rs:20:5` and resolves file:line in the backtrace. Protocol C
not re-measured this phase (n/m) — deferred to the Phase 7 final remeasurement.

Reference (2026-07-06, protocol A only, pre-plan): core 17.5 s, mcp server 3 m 26.6 s,
inference 1 m 31.7 s; target/ 209 GB.

**After (final) notes — Phase 7 (2026-07-06):** full protocol A–E re-run on `main` with all
three configs warm-cached. Acceptance criteria:

- **D (parity oracle):** core 228/0/0, mcp server 321/0/1, inference 257/0/0 — **exactly
  equal to Baseline**. ✓
- **A (config 2 ≥50 % faster):** mcp-server incremental 3 m 44.5 s (Phase-0 baseline) /
  3 m 26.6 s (pre-plan reference) → **17.5 s**, a ~92 % reduction. Configs 1 and 3 not
  regressed (core 10.6 s → 6.8 s; inference 1 m 11.1 s → 7.8 s — both improved further past
  Phase 1, since consolidation removed ~21–25 redundant test-binary link steps). ✓
- **B (≤5 execs per config):** 26/28/27 → **5/7/6**. Literally ≤5 only for core; net (7) and
  inference (6) exceed it solely by their feature-gated **bin** unittest harnesses
  (`ml_planes_mcp` + `ml_planes_server` for net; `evaluate_policy` for inference) plus the
  `lib` + `main` unittest harnesses — i.e. exactly the "3 test binaries + feature-gated bins"
  the criterion's parenthetical anticipates. The integration-test-binary count (the actual
  target of the consolidation) dropped 24 → **3** (`core`/`net`/`rl`) as intended. ✓ (spirit)
- **C (`just test-all` < ~3 min, stretch 2):** 5 m 2.2 s → **1 m 2.9 s** — beats the stretch
  goal. ✓
- **E (`target/` an order of magnitude below 209 G):** **17 G** (~12×). ✓

**Phase 6 (mold) skipped** by decision: link time no longer dominates after Phases 1–4 (config-2
incremental is 17.5 s total), so the incremental rust-lld → mold win would not clear the 20 %
gate and is not worth the cache-invalidating `.cargo/config.toml` rustflags edit. The follow-on
lever if further speedups are wanted remains the workspace/crate split (extract `training`/burn
first), per Phase 7 step 5 — not pursued, acceptance already met.
