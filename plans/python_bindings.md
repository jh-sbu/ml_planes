# Plan: Library Prerequisites for the Python Training-Env Bindings

## Context

The PyO3 scaffold is in place and green (`just py-test`: 2 passed). `bindings/python` is a
separate, non-workspace crate (`ml_planes_py`) that path-depends on `ml_planes` with
`default-features = false`; `pyproject.toml` drives maturin with `python-source = "python"`
and `module-name = "ml_planes._core"`; `python/ml_planes/__init__.py` re-exports from the
private extension module. Today the whole surface is one `physics_dt()` passthrough proving
the crate links (`bindings/python/src/lib.rs`).

The hard prerequisites already landed. `TrainingEnv` (`src/training/env.rs:82`) requires an
**absolute** `set_rng_seed`, which is what a Gymnasium-style `reset(seed=…)` needs; all five
envs are `Clone + Send + Sync + 'static`; `PHYSICS_DT` is shared with the live sim; the
scaffold's build/import/link wiring is pinned by pytest. **Nothing is blocking.** What remains
is a short list of gaps in the *Rust library*, all of which must close before a wrapper can be
written without duplicating logic on the Python side — which is precisely the divergence
CLAUDE.md §3's constraint (b) exists to prevent.

This plan covers those prerequisites and ends with a single unexported proof slice that
demonstrates them sufficient. The full binding surface (all five envs, a `VecEnv` wrapper, a
Gymnasium adapter) is a **follow-on plan**, not this one, and Phase 5 ships no public Python
API — see its preamble.

### Decisions locked with the user

- **Dynamic dispatch is required** for the one-Python-class-over-five-Rust-envs problem
  (Phase 1). Not an enum dispatcher, not five separate `#[pyclass]` types.
- **`StepInfo.extra` gets a full implementation** across all five envs (Phase 2) — it is not
  left as the dead field it is today, and not deferred to the binding plan.
- **The `reset()` return shape / `SpawnSpec` → Python mapping is DEFERRED** (see "Deferred
  decisions" below). Phase 5's slice is written so that it does not lock the decision in.

### Non-goals (this plan)

- No policy interchange (`.mpk` ↔ PyTorch checkpoints). Still undecided, still separate.
- No Gymnasium `Env`/`VectorEnv` adapter class, no `gymnasium` dependency.
- No Python wrapper for `VecEnv` (Phase 4 prepares the Rust side; the wrapper is follow-on).
- No behavior change to the Rust `train_ppo` / `train_bc` / `evaluate_policy` path. Phase 3 is
  a **pure refactor** and is tested as one.
- No `pyo3` anywhere in the root crate's dependency graph. `just test-all` must stay green on
  a machine with no Python toolchain installed (CLAUDE.md §3, §6).
- No rendering, no Bevy `App`, no GPU. The `training::` envs are Bevy-free by construction.

---

## Gap Summary

| # | Gap | Phase |
|---|---|---|
| 1 | Nothing supports `dyn TrainingEnv`; `#[pyclass]` cannot be generic | 1 |
| 2 | `StepInfo.extra` is declared, never written, never read | 2 |
| 3 | The task registry (name → env/paths/defaults) lives in three binaries | 3 |
| 4 | `VecEnv::step_batch` is fixed-width `&[[f32; 4]]`; no per-index absolute seeding | 4 |
| 5 | `reset()` returns `(Observation, SpawnSpec)`; Gymnasium returns `(obs, info)` | **deferred** |
| 6 | No plan doc; no test enforcing "same env on both sides" | this doc; 5 |

---

## Phase 1 — Dynamic Dispatch for `TrainingEnv`

`grep` finds zero `dyn TrainingEnv` in the crate. A `#[pyclass]` cannot be generic, so one
Python `Env` class covering five Rust env types needs `Box<dyn TrainingEnv>` to *be* a
`TrainingEnv`. The trait is already object-safe — every method takes `&self`/`&mut self`, no
generics, no `Self` returns — and `Send + Sync + 'static` are supertraits, so `dyn TrainingEnv`
carries them and `Box<dyn TrainingEnv>` is `Send + Sync` without extra bounds.

1. `src/training/env.rs` — add `impl TrainingEnv for Box<dyn TrainingEnv>`, forwarding every
   method through `(**self)`.
2. **Forward `offset_rng_seed` explicitly**, even though it has a default body. Without the
   explicit arm, the default impl runs *on the box* and routes through the box's
   `rng_seed`/`set_rng_seed`, silently bypassing any override an inner env supplies. No env
   overrides it today; the point is that one could, and the bug would be invisible.
3. `VecEnv<Box<dyn TrainingEnv>>` then compiles with no change to `src/training/vec_env.rs`.

**Do not** add a `clone_box` / `dyn-clone` shim to make `Box<dyn TrainingEnv>` satisfy
`PpoTrainer<B, E>`'s `E: TrainingEnv + Clone` bound (`src/training/ppo/trainer.rs:56`). The
Rust trainers stay monomorphized over concrete env types — that is a deliberate split, not an
oversight: dynamic dispatch exists for the *bindings*, and putting a vtable call in the Rust
PPO hot loop buys nothing. Record this in the impl's doc comment so it is not "fixed" later.

**TDD:** in `env.rs`'s `mod tests`, a minimal fake env (counts steps, echoes seed):
- each method forwards through the box (obs/action dims, reset, step, seed round-trip);
- an env that **overrides** `offset_rng_seed` has its override honored through the box — this
  is the test that fails without item 2 above;
- `VecEnv<Box<dyn TrainingEnv>>` holding two *different* concrete env types (`LevelHoldEnv`
  and `OrbitEnv`, boxed) resets and `step_batch`es without panicking, and reports each env's
  own `observation_dim`.

**Milestone 1:** `Box<dyn TrainingEnv>: TrainingEnv`, heterogeneous `VecEnv` works, trainer
bounds untouched. `cargo test --no-default-features` green; `cargo fmt`.

---

## Phase 2 — Implement `StepInfo.extra`

`StepInfo.extra` (`src/training/env.rs:20`) is written by nothing and read by nothing. It is
the natural Gymnasium `info` channel for the per-step diagnostics a PyTorch loop would log,
and every env already computes the values internally while building its reward.

### 2a. Change the container type

`HashMap<String, f32>` would allocate ~10 `String`s and hash them on **every step** of a
multi-million-step training run. Replace it with an insertion-ordered
`Vec<(&'static str, f32)>`: no hashing, one small allocation, keys are `&'static str` so
there is nothing to allocate per key. The field is public and currently unread (`grep`
confirms), so this is a free change today and an expensive one later.

Add a `StepInfo::get(&self, key: &str) -> Option<f32>` helper so callers and tests read by
name without caring about the representation.

### 2b. Populate it, per env

Values are **raw and signed**, in SI units and radians — not the reward's absolute,
scaled terms. A logger wants the sign; the reward functions take `.abs()` themselves. Document
the whole vocabulary in one block comment in `env.rs`, and give each env a test asserting it
publishes exactly its documented key set.

- **`LevelHoldEnv`** — `altitude_error`, `airspeed_error`, `roll`, `beta`, `target_altitude`,
  `target_airspeed`.
- **`HeadingHoldEnv`** — the level-hold set, plus `heading_error`, `target_heading`,
  `roll_rate`, `bank_excess`, `alpha_excess`. (The last two mirror the soft-limit terms in
  `compute_reward`; both are 0 when inside the limit, which is the useful signal.)
- **`OrbitEnv`** — `radial_error`, `heading_error` (pure tangent), `guidance_heading_error`,
  `bank_ff`, `altitude_error`, `airspeed_error`, `roll`, `beta`, `roll_rate`, `pitch_rate`,
  `yaw_rate`. All available on the `OrbitObservationTerms` the step already builds
  (`src/controllers/orbit.rs:60`).
- **`ResidualOrbitEnv`** — the orbit set, plus the baseline the residual rides on:
  `pid_elevator`, `pid_throttle`, `pid_aileron`, `pid_rudder`. Separating the PID baseline
  from the applied delta is the whole debugging story for this env and is currently
  unobservable from outside.
- **`WuOrbitEnv`** — the orbit set, plus `r_tt`, `r_ps`, `r_rs` (the multiplicative reward
  factors), `heading_error_dot`, and `curriculum_stage` as an f32 ordinal (0 = Coarse,
  1 = HeadingFine, 2 = Full — document the mapping; the container is numeric by design).

`episode_step` stays a struct field and is **not** duplicated into `extra`.

### 2c. Prove the cost

Population is unconditional. Measure `train_ppo` throughput (steps/s over a fixed
`--total-timesteps`, release build, `level_hold`, fixed `--seed`) before and after, and record
both numbers in the commit message. **Contingency, to be applied only if the measurement shows
a regression worth the complexity:** add `TrainingEnv::set_info_enabled(bool)` defaulting to
*on*, and have `PpoTrainer`/`LstmPpoTrainer` switch it off for their env pools. Do not
implement the flag speculatively — it is a second code path and its whole justification is a
number nobody has yet.

**TDD:** per-env unit tests that step once and assert (i) the exact key set, (ii) that a known
deviation shows up with the right sign — e.g. spawn a `LevelHoldEnv` 50 m below target and
assert `altitude_error` is ≈ −50 and *not* +50, which is the assertion an `.abs()`-copied
implementation fails. Plus a `StepInfo::get` unit test for a missing key.

**Milestone 2:** every env publishes documented, signed, per-step diagnostics; throughput
measured and recorded. Tests green; `cargo fmt`.

---

## Phase 3 — Lift the Task Registry into the Library

The task-name mapping exists **three times**, all of it private to binaries:
`src/bin/train_ppo.rs:284` (`enum Task` + `reward_config_path`/`default_ppo_config_path`/
`model_dir`/`default_stem`, and the `match task` env construction at `:458`–`:640`),
`src/bin/train_bc.rs:197` (a second `enum Task`), and `src/bin/evaluate_policy.rs:95` (a third
copy, as bare string matching). The orbit geometry is hardcoded inside those arms —
`(1000.0, 100.0, 1000.0)` for `orbit`/`residual_orbit` (`train_ppo.rs:576`, `:603`) and
`(1000.0, 100.0, 3000.0)` for `lstm_orbit` (`:632`).

A Python `make_env("orbit", …)` has nowhere to call. It would have to re-implement the
mapping — constraint (b) again — or the bindings could expose only per-type constructors with
no task-name entry point at all.

### 3a. New `src/training/task.rs`

- `pub enum Task { LevelHold, HeadingHold, Orbit, ResidualOrbit, LstmOrbit }` with `ALL`,
  `parse(&str)`, `as_str()`, and the four path/name accessors moved verbatim from `train_ppo`.
- Default envelope accessors: `default_target_alt_range()`, `default_target_speed_range()`,
  `default_target_heading_range()` (wrapping the existing
  `level_hold_env::DEFAULT_TARGET_*` / `heading_hold_env::DEFAULT_TARGET_*` constants, which
  already live in the library), and `default_orbit_geometry() -> (f32, f32, f32)` for the
  three literals above. **Name the heading accessor for its unit** — the shipped constants are
  `DEFAULT_TARGET_HEADING_DEG_MIN`/`_MAX` in *degrees*, converted to radians at the CLI
  boundary (`train_ppo.rs:238`), while the env stores radians. `EnvSpec` holds radians; a
  `_deg` suffix on the accessor is the only thing standing between a Python caller and a
  policy trained on a 57×-too-wide envelope.
- `pub struct EnvSpec` — the resolved construction inputs: `plane_config: PlaneConfig`,
  target ranges, orbit geometry, and the already-loaded reward config. `EnvSpec` holds
  *resolved values*, not paths: reward-config loading is CLI presentation (it prints "Loaded
  reward config from …" and warns-then-defaults on failure) and stays in the binaries.

### 3b. Two construction layers

- **Concrete, one per task:** `task::level_hold_env(&EnvSpec) -> LevelHoldEnv`,
  `task::orbit_env(&EnvSpec) -> OrbitEnv`, etc. The binaries call these, so
  `run_training_loop_bc::<B, _>` stays monomorphized and `PpoTrainer`'s `E: Clone` bound is
  still satisfied.
- **Erased, one total:** `task::make_env(Task, &EnvSpec) -> Box<dyn TrainingEnv>` — a thin
  wrapper that calls the same five concrete constructors and boxes the result. Depends on
  Phase 1. This is the bindings' single entry point, and because it delegates rather than
  duplicates, a Python-built env cannot diverge from a Rust-built one.

### 3c. Refactor the three binaries

Delete both private `Task` enums and `evaluate_policy`'s string matching; import
`ml_planes::training::task::Task`. Move only parse / paths / defaults / env construction.
**Leave in the binaries:** reward-config loading and its warn-and-default fallback,
`log_fields`, `open_log`, `save_path_for`, and all CLI parsing and printing.

**TDD:** new `tests/core/training_task.rs` (+ `mod training_task;` in `tests/core/main.rs`,
ungated — no sim chain needed):
- all five names parse round-trip via `as_str()`; an unknown name errors with the existing
  message;
- every `reward_config_path()` and `default_ppo_config_path()` points at a file that exists
  under `assets/training/` — this also catches a profile renamed out from under a binary;
- **behavior-preservation pins:** every default range and the orbit geometry equal the values
  the binaries used before this refactor, asserted against literals. A changed default here is
  a silent change to what every future `train_ppo` run trains on, so it must fail a test, not
  a review;
- `make_env(task, &spec).observation_dim()` equals the per-task obs-dim constant for all five
  tasks, and `action_dim()` is 4.

Then re-run `just test-training` — the binaries are `training`-gated and `test-all` does not
compile them.

**Milestone 3:** one task registry in the library; three binaries consume it; defaults pinned.
`just test-all` + `just test-training` green; `cargo fmt`.

---

## Phase 4 — `VecEnv` Shape for Batched Callers

Two mismatches with what a batched Python caller needs (`src/training/vec_env.rs`).

1. **Fixed-width actions.** `step_batch(&mut self, actions: &[[f32; 4]])` (`:67`) contradicts
   the trait's `&[f32]` + `action_dim()`. It is correct today — all five envs return 4 — but
   it is the wrong shape for an `(N, action_dim)` numpy array. Change it to a **flat**
   `&[f32]` chunked by `action_dim()`, with a length assertion (`len == n * action_dim`), so a
   contiguous buffer marshals with no per-env allocation. There are exactly two callers
   (`ppo/trainer.rs:245`, `ppo/lstm_trainer.rs:265`); update both rather than keeping a second
   `step_batch_flat` path that will drift.
2. **No per-index absolute seeding.** `set_rng_seed(base)` (`:43`) seeds the whole pool at
   `ENV_SEED_STRIDE` spacing. Gymnasium's `reset(seed=[s0, s1, …])` needs per-sub-env absolute
   control. Add `set_rng_seed_at(i, seed)` and `set_rng_seeds(&[u64])`. **Leave
   `set_rng_seed(base)`'s stride semantics exactly as they are** — CLAUDE.md pins that the
   trainers seed through `offset_rng_seed` on cloned templates specifically so existing
   `--seed` values keep reproducing their past runs.

**TDD:** flat `step_batch` produces outcomes identical to the old array form for the same
actions (pin against a hand-built expected sequence, not against the old signature); a
mis-sized action buffer panics with a clear message; `set_rng_seed_at` changes exactly one
sub-env's seed and leaves its neighbors' untouched; `set_rng_seeds` + `reset_all` reproduces
the same episodes across two independently constructed pools.

**Milestone 4:** `VecEnv` takes flat batched actions and per-index absolute seeds; trainer
behavior unchanged (`ppo::trainer::tests::seeded_env_pool_is_offset_from_the_template_not_absolute`
still green). `just test-training` green; `cargo fmt`.

---

## Phase 5 — Proof Slice (not the binding surface) + Rust/Python Parity Test

One env reachable from Python, and the test that makes constraint (b) enforceable rather than
aspirational. This is the gate that proves Phases 1–4 were sufficient — **it is not the
binding**. The binding surface (all five envs, `VecEnv`, a Gymnasium adapter, and the
`reset()`-shape decision below) remains the follow-on plan's, and nothing here may pre-empt it.

Concretely, the slice stays **provisional and unexported**: the class lives in `_core` only,
is not re-exported from `python/ml_planes/__init__.py`, and carries a docstring saying it is a
parity harness whose signatures may change without notice. Publishing it as public API would
silently settle exactly the question "Deferred decisions" leaves open — a public
`Env.reset() -> Vec<f32>` *is* the reset-shape decision, and the follow-on plan would then have
to break it or wrap around it rather than decide it freely.

### 5a. The `_Env` proof class

`bindings/python/src/lib.rs` — marshalling only, no simulation logic:

- `#[pyclass] struct _Env { inner: Box<dyn TrainingEnv> }`, constructed from
  `training::task::make_env` with a task name and a `.plane.ron` path (loaded through the
  existing `training::cli::load_plane_config`). Underscore-prefixed on purpose: it is the
  harness, not the API.
- `reset(&mut self) -> Vec<f32>` — **observation only.** See "Deferred decisions": returning
  the observation alone commits to nothing about how `SpawnSpec` or a reset-time `info` will
  eventually be surfaced.
- `step(&mut self, action: Vec<f32>) -> (Vec<f32>, f32, bool, bool, PyObject)` —
  `(obs, reward, terminated, truncated, info)`. `terminated` is
  `end == Some(TerminationReason::Failure)`; `truncated` is `StepOutcome::truncated()`. Note
  that `done()` maps to *neither* on its own — Gymnasium wants the two split, which is exactly
  what `end` already encodes, and collapsing them here would reintroduce the value-target bug
  CLAUDE.md warns about at the Python boundary. `info` is a dict built from Phase 2's `extra`
  plus `episode_step`.
- `observation_dim()`, `action_dim()`, `seed(u64)` (→ `set_rng_seed`), `rng_seed()`.
- **No re-export from `python/ml_planes/__init__.py`.** The parity test reaches it as
  `ml_planes._core._Env`; the public package surface is unchanged by this plan.

### 5b. The parity test

Same task, same seed, same action sequence → **bit-identical** obs and reward streams on both
sides. A fixture file regenerated by hand is the wrong shape here (it is the stale-`.so`
failure mode again), so the reference is computed at test time:

- `bindings/python/examples/reference_rollout.rs` — uses `ml_planes::training::task::make_env`
  directly, runs a fixed action sequence from a fixed seed, prints JSON (`obs` rows, `reward`,
  `terminated`, `truncated`, `info`) to stdout. It lives in the binding crate because that
  crate already path-depends on `ml_planes` and, per its `Cargo.toml`, is buildable by plain
  `cargo` (maturin is what adds `pyo3/extension-module`).
- The pytest case shells out to
  `cargo run --manifest-path bindings/python/Cargo.toml --example reference_rollout`, with
  `CARGO_TARGET_DIR=target` for the same reason every `py-*` recipe sets it — otherwise the
  binding crate builds a second full copy of the dependency tree under
  `bindings/python/target`.
- Compare with `==` on floats, not `pytest.approx`. Both sides run the identical Rust code on
  the identical inputs; anything but exact equality means they are not, in fact, the same env,
  and a tolerance would hide precisely the divergence this test exists to catch.
- Cover at least one episode that ends in `Failure` and one that ends in `Timeout`, so the
  terminated/truncated split is exercised rather than assumed.

Add a `just py-parity` recipe (or fold it into `py-test`) and note in the justfile comment
that it depends on a fresh `.so`, same as everything else in that lane.

**Milestone 5 (definition of done for this plan):**
- `just test-all` green **on a machine with no Python toolchain installed** — unchanged, and
  the property the whole separate-crate arrangement exists to protect;
- `just test-training` and `just test-visual` green;
- `just py-test` green, including the parity test;
- one env constructible from Python by task name through the unexported `_core._Env` harness,
  producing rollouts bit-identical to Rust — the public `ml_planes` package surface unchanged;
- `cargo fmt`; CLAUDE.md §2/§3 updated to describe the task registry and the `info` vocabulary,
  and to record that the library prerequisites are closed and a parity harness exists.
  **Leave "env wrappers not yet implemented" in place** — it is still true: this plan ships no
  public wrapper, and clearing that note is the follow-on binding plan's milestone, not this
  one's.

---

## Deferred decisions

**`reset()`'s return shape and the `SpawnSpec` mapping (gap 5).** `TrainingEnv::reset` returns
`(Observation, SpawnSpec)`; Gymnasium returns `(obs, info)`. `SpawnSpec` holds bevy `Vec3`/
`Quat`, though field access (`p.x`) marshals fine without adding a bevy dependency to the
binding crate, so this is a shape question, not a dependency one. Options on the table: drop it
(the live spawner is the only real consumer), fold it into the reset `info` dict, or expose it
as a separate read-only accessor.

Deferred to the follow-on binding plan by explicit decision. Phase 5a is written to avoid
prejudging it twice over: `_Env.reset()` returns the observation alone, and `_Env` is not
re-exported, so there is no public Python-visible contract to break whichever way the decision
lands. **Do not** add a partial `SpawnSpec` mapping in
this plan's slice "just to have something".

---

## Risks / Open Questions

- **Phase 3 is a refactor of shipped training binaries.** A default silently changed during
  the lift changes what every future run trains on, and would not show up until a checkpoint
  underperformed for unrelated-looking reasons. The behavior-preservation pins in 3b are the
  mitigation and are not optional.
- **`make_env` erases `CurriculumEnv`.** `Box<dyn TrainingEnv>` loses `WuOrbitEnv`'s
  `advance_curriculum` / `curriculum_stage_name` / `next_stage_threshold`. Phase 2's
  `curriculum_stage` key makes the stage *observable* from Python but not *settable*. The
  follow-on plan decides between a parallel `impl TrainingEnv for Box<dyn CurriculumEnv>` and a
  separate handle type; `lstm_orbit` is simply read-only-staged until then, which should be
  stated in the Python docstring rather than discovered.
- **Phase 2's throughput cost is unmeasured until it is measured.** The `Vec<(&'static str,
  f32)>` choice is expected to be cheap, but "expected" is why 2c requires a number in the
  commit message.
- **`--no-default-features` still compiles bevy and rapier** into the binding crate's
  dependency graph, since `ml_planes` depends on them unconditionally. The `training::` envs
  do not use them; this is build time, not runtime, and is not worth restructuring the root
  crate over. Revisit only if binding build times become an actual complaint.
- **The stale-`.so` trap applies to every phase here.** Phases 1–4 edit `ml_planes`, which
  `maturin develop` does *not* rebuild editably. Any Python-side result observed without a
  `just py-build` in between is reporting the previous state of the crate.

---

## Future Extensions (out of scope here)

- The full binding surface: all five envs, a `VecEnv` wrapper over Phase 4's flat batch API,
  and a Gymnasium `Env`/`VectorEnv` adapter in `python/ml_planes/`.
- Settable curriculum control for `lstm_orbit`.
- Policy interchange (`.mpk` ↔ PyTorch), still explicitly undecided per CLAUDE.md §3.
- Feeding Phase 2's `extra` back into `eval_metrics`, which currently reconstructs errors by
  reading *scaled* observation elements (`MetricFamily`'s `Source::Scaled` /
  `Source::CircularError`). Raw signed errors in `info` are strictly better inputs, but that
  is a separate change to a working evaluation path.
