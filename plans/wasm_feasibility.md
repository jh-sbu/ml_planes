# WASM Target Feasibility Report

*Assessed against: Cargo.toml + Cargo.lock as of May 2026 (Bevy 0.18.1, bevy_rapier3d 0.33, burn 0.20.1 / burn-wgpu 0.20.1 (wgpu 26.0.1); Bevy visual stack (wgpu 27.0.1))*

---

## Summary

WASM support splits cleanly into two independent concerns:

| Concern | Verdict | Effort |
|---|---|---|
| **Visual sim (Bevy + Rapier + PID controllers)** | **Feasible** | Medium |
| **RL inference in the browser** | **Feasible with constraints** | Medium-High |
| **PPO training in the browser** | **Not feasible** | — |

The rendering stack and physics engine are either already WASM-ready or have
documented WASM targets. The training path is blocked by hard threading
requirements that WASM cannot satisfy without special browser headers.

---

## 1. Visual Simulation (Bevy + Rapier + PID)

### What's already WASM-ready

**Bevy 0.18** has an official `wasm32-unknown-unknown` target. The lock file
confirms WASM wiring is already present: `bevy_window` pulls in `wasm-bindgen`,
`wasm-bindgen-futures`, and `tracing-wasm` as conditional dependencies. Bevy
renders via WebGL2 by default on WASM (`bevy/default` enables `webgl2`);
WebGPU requires explicitly adding `bevy/webgpu`.

**Rapier / bevy_rapier3d 0.33** is pure Rust. Its dependency tree
(`rapier3d → parry3d → nalgebra`) carries no OS-specific code. The Rapier
project officially targets WASM and publishes `rapier3d-wasm` JavaScript
bindings separately. The Rust library itself compiles to WASM without patches.

**PID controllers and aerodynamic model** are pure floating-point math with
no threading or OS calls. They compile to WASM unchanged.

### Blockers for the visual build

1. **`rfd` (file dialogs)** — Currently in the `visual` feature. `rfd` 0.15
   does include `js-sys` in its dependency list, so it has *some* WASM support
   (browser file picker via `<input type="file">`). This should work but needs
   testing; native-style dialogs obviously don't exist in browsers.

2. **`naga`** — Listed as a direct `[dependencies]` entry (`Cargo.toml:19`),
   but has no direct usage in `src/`; it is a transitive dep pulled in by
   wgpu/bevy's shader compilation pipeline. WASM-compatible through those
   crates; not a standalone blocker.

3. **`std::fs` in `main.rs`** — `scan_models` (which calls
   `std::fs::read_dir("models/")`) is gated `#[cfg(feature = "training")]`
   (`src/main.rs:481`). For a Phase 1 training-free visual build this code is
   never compiled in; `std::fs` is not a Phase 1 blocker. It becomes relevant
   only when the `training` feature is included (Phase 2).

4. **`bevy_rapier3d` debug renderer** — `debug-render-3d` is **not** currently
   enabled: `Cargo.toml:13` has only `features = ["dim3"]` and the `visual`
   feature adds `["bevy/default", "bevy_egui", "rfd"]`. No WASM risk here.

5. **Audio** — Bevy's audio subsystem (if pulled in by `bevy/default`) has known
   WASM limitations; `rodio`'s WASM backend uses the Web Audio API. Not used in
   this project, but worth noting if audio is added later.

### What a WASM visual build would need

- Install the WASM toolchain: `rustup target add wasm32-unknown-unknown`.
- Optionally pin the default target in `.cargo/config.toml` (the file already
  exists with aliases and `[target.x86_64-unknown-linux-gnu]`; add a new
  `[target.wasm32-unknown-unknown]` section rather than replacing the file).
- Gate `std::fs` model scanning behind `#[cfg(not(target_arch = "wasm32"))]`.
- Embed a small set of model weights as `include_bytes!()` for the WASM build,
  or load via `fetch()` + `wasm-bindgen-futures`.
- Serve with correct MIME types (`application/wasm`); use `wasm-pack` or
  `trunk` as the bundler.
- **Recommendation:** Rather than using `visual = ["bevy/default", ...]` for WASM,
  introduce a dedicated `wasm` Cargo feature that enables only what the browser build
  needs (e.g., `bevy/bevy_render`, `bevy/bevy_core_pipeline`, `bevy/bevy_pbr`,
  `bevy/bevy_asset`, `bevy_rapier3d`) and explicitly excludes audio
  (`bevy/bevy_audio`) and gamepad (`bevy/bevy_gilrs`) subsystems. This reduces binary
  size, avoids runtime surprises from rodio/cpal's Web Audio path, and prevents gilrs
  from accessing platform HID APIs unavailable in browsers. The current `visual` feature
  *may* compile successfully for WASM (Bevy does test WASM CI and transitive deps have
  stubs), but the lean dedicated feature is more robust and is the recommended approach.

**Estimated effort: ~1–2 weeks** to get a playable browser demo without RL.

---

## 2. RL Inference in the Browser

### What works

`burn-wgpu` (via `cubecl-wgpu`) already depends on `wgpu 26`, which has explicit
WASM bindings (`js-sys`, `wasm-bindgen`, `web-sys` are all in the lock file for
`wgpu 26`). WebGPU is available in Chrome 113+ (default) and Firefox 141+
(default on Windows; available behind a flag from Firefox 121 — treat Firefox
141 as the practical floor for stable, no-opt-in WebGPU support). A trained
model running inference on `Wgpu` backend would leverage the browser's GPU.

`burn-ndarray` is a usable fallback *for inference only* — the rayon
parallelism is a training-time optimization, not required for single forward
passes. A single-threaded ndarray inference path technically compiles to WASM,
but rayon itself will panic at runtime if it tries to spawn threads. This must
be handled:

- Use the `NdArray` backend **without enabling rayon** — burn-ndarray's rayon
  usage is gated behind parallel iterators for batch operations; a batch-1
  forward pass may not trigger rayon's thread pool at all in practice, but this
  is not guaranteed by the API contract.
- Safer alternative: use the `Wgpu` backend for inference (WebGPU) — no rayon
  dependency, GPU-accelerated, and `wgpu 26` already has WASM wiring.

### The model loading problem

All RL controllers use `DefaultFileRecorder`, which in burn 0.20 is a type
alias for `NamedMpkFileRecorder` (confirmed: `burn-core-0.20.1/src/record/file.rs:18`).
It uses `std::fs::File` path I/O — **`std::fs` is not available in WASM**.
This is the primary inference blocker (not `memmap2`/`mmap`, which is not in
the recorder's dependency path).

Mitigation options (pick one):

**A. Embed weights at compile time**
```rust
const MODEL_BYTES: &[u8] = include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/models/orbit/ppo_orbit_1.mpk"));
```
Use `burn`'s `NamedMpkBytesRecorder` to deserialize from a byte slice —
it uses the same msgpack format as existing `.mpk` checkpoints
(`burn-core-0.20.1/src/record/memory.rs:73`). Do **not** use `BinBytesRecorder`
here; it uses bincode and would silently produce corrupt loads against `.mpk`
files.

**B. Fetch weights at runtime via HTTP**
Load weights from a URL using `wasm-bindgen-futures` + `web-sys` fetch API,
deserialize in-memory. More flexible but requires async entry points and a
server to host the weight files.

**C. Hard-code small networks** *(MLP controllers only)*
The MLP-based `ActorCritic` networks (`RlLevelHold` obs=10, `RlOrbit` obs=13,
`RlOrbitResidual`) are 2-layer MLPs (obs_dim→64→64→action_dim) at ~50K parameters;
they could be embedded as static float arrays evaluated without burn's serialization
infrastructure entirely. Extreme but eliminates all burn dependency for those three
controllers.

**This option does not cover `RlLstmOrbit`.** That controller uses a larger
FC-LSTM-FC architecture (`src/training/ppo/lstm_model.rs`): FC(obs→128)→FC(128→128)→
LSTM(128→128)→FC(128→128)→out, with `LSTM_HIDDEN = LSTM_FC = 128`. More importantly,
`RlLstmOrbit` is *stateful*: it carries a `LstmHiddenState` (h, c vectors of length 128)
between inference steps. Representing this without reimplementing LSTM cell math is
impractical. **Use Option A (`NamedMpkBytesRecorder` + `include_bytes!`) for any
deployment that includes `RlLstmOrbit`** — it covers all four controller kinds with
the same code path.

**RL controller inventory:**
| Kind | Architecture | Obs dim | Stateful? | Model dir |
|---|---|---|---|---|
| `RlLevelHold` | MLP 64×64 | 10 | No | `level_hold` |
| `RlOrbit` | MLP 64×64 | 13 | No | `orbit` |
| `RlOrbitResidual` | MLP 64×64 | 13 | No | `orbit_residual` |
| `RlLstmOrbit` | FC-LSTM-FC 128×128 | 13 | Yes (h+c, 128 each) | `lstm_orbit` |

### Threading constraint for inference

If the `NdArray` backend is used for WASM inference, rayon's global thread pool
must not be initialized. This requires `rayon` to be excluded from the WASM
build, which means either:

- Feature-gate the `NdArray` backend and use a no-rayon inference shim on
  `wasm32`, or
- Use the `Wgpu` backend exclusively for WASM (recommended — it's the cleaner
  path and WebGPU is broadly available).

**Estimated effort: 2–4 weeks** to port RL inference to WASM with embedded
weights and the Wgpu backend.

---

## 3. PPO Training in the Browser

### Verdict: Not feasible

PPO training is blocked by multiple hard constraints:

| Blocker | Severity | Notes |
|---|---|---|
| `rayon` in `burn-ndarray` | Fatal | Rayon requires OS threads; WASM has no `std::thread` |
| `burn-train` TUI (`ratatui`) | Fatal | Terminal rendering; no WASM target |
| `sysinfo` / `nvml-wrapper` in `burn-train` | Fatal | Hardware queries via OS syscalls |
| `systemstat` in `burn-train` | Fatal | OS-level metrics |
| `std::fs` throughout training binary | Fatal | No filesystem in WASM |
| `std::env::args()` | Fatal | No argv in WASM |
| Training throughput | Practical | WebGPU compute is ~5–10× slower than native Vulkan/CUDA for batch gradient updates; PPO here trains for 2M steps with parallel rollouts — not practical in a browser |

Even if every dependency were patched, training in a browser tab would be ~10×
slower than a native `--release` build and would provide no meaningful benefit
over running the native binary locally.

**Recommendation: do not pursue training in WASM.**

---

## 4. Dependency-Level Assessment

| Crate | WASM Status | Notes |
|---|---|---|
| `bevy 0.18` | ✅ Supported | Official WASM target; wasm-bindgen already wired |
| `bevy_rapier3d 0.33` | ✅ Supported | Pure Rust; Rapier project supports WASM |
| `rapier3d 0.31` | ✅ Supported | No OS dependencies in dep tree |
| `nalgebra` | ✅ Supported | Pure Rust math |
| `burn-wgpu 0.20` | ✅ With WebGPU | wgpu 26 has WASM bindings; WebGPU required |
| `burn-ndarray 0.20` | ⚠️ Partial | Rayon prevents multi-threaded use; single-thread inference may work but is not guaranteed |
| `burn-train 0.20` | ❌ Blocked | `ratatui`, `sysinfo`, `nvml-wrapper`, `systemstat` |
| `DefaultFileRecorder` (`NamedMpkFileRecorder`) | ❌ Blocked | `std::fs::File` path I/O; not available in WASM |
| `NamedMpkBytesRecorder` | ✅ Supported | In-memory msgpack deserialization; WASM-safe; compatible with existing `.mpk` checkpoints |
| `BinBytesRecorder` | ✅ Supported | In-memory bincode deserialization; WASM-safe; **not** compatible with `.mpk` files |
| `ron` | ✅ Supported | Pure Rust parsing |
| `serde` | ✅ Supported | Pure Rust |
| `rfd 0.15` | ⚠️ Partial | Has WASM branch (browser file picker); smoke test needed |
| `naga 26` | ✅ Supported | Transitive dep via bevy/wgpu; no direct usage in `src/`; WASM-compatible through those crates |
| `rayon` | ❌ Blocked | Requires OS threads |
| `memmap2` | ❌ Blocked | Requires `mmap(2)` syscall |
| `burn-train` (`ratatui` etc.) | ❌ Blocked | OS-level deps |

---

## 5. Recommended Approach

If browser deployment is a goal, pursue in this order:

### Phase 1 — WASM visual sim with PID only

1. Add `[target.wasm32-unknown-unknown]` section to `.cargo/config.toml`.
2. Gate `std::fs::read_dir` model scanning behind `#[cfg(not(target_arch = "wasm32"))]`.
3. Add a `wasm` Cargo feature (or a separate `[[bin]]`) that excludes the
   `training` feature entirely.
4. Create `index.html` (minimal Trunk entry point — without it `trunk serve`
   will not start) and `Trunk.toml` that explicitly copies the `assets/` directory:

   ```toml
   # Trunk.toml
   [[asset]]
   action = "copy"
   dir = "assets"
   ```

   Trunk does **not** serve `assets/` by default; only folders declared via
   `[[asset]]` entries or `data-trunk` attributes in `index.html` are copied to
   `dist/`. Neither file exists in the repo yet — both must be created.

5. Use `trunk` as the build/serve tool (`trunk serve`).
6. Verify the visual + Rapier sim runs in Chrome with WebGL2.
7. Confirm assets are served correctly: `planes/generic_jet.plane.ron` and
   `shaders/ground_grid.wgsl` must return HTTP 200. **If the plane config is
   not loaded, `apply_aerodynamic_forces` (`src/plane/systems.rs:47`) silently
   skips force application** — the plane spawns and the sim runs, but it flies
   ballistically with no aerodynamic response. Smoke-test by confirming the HUD
   shows non-zero airspeed and the plane maintains altitude under level-hold
   control within the first few seconds.

**Deliverable:** playable flight sim in the browser, PID controllers only.

### Phase 2 — RL inference in browser

**Prerequisite — feature split:** The current `training` feature enables
`burn/train`, `burn/tui`, and `burn/autodiff`, all of which are WASM-incompatible.
RL controllers are also gated `#[cfg(feature = "training")]`
(`src/controllers/mod.rs:9–16`). Simply adding `#[cfg(target_arch = "wasm32")]`
guards inside the existing `training` feature does not remove these heavyweight
crates from the compile graph. A separate Cargo feature is required:

```toml
# Cargo.toml — burn dependency must opt out of its default features so that the
# training-only crates (autodiff, train, tui) are not compiled into the inference build.
# The current declaration (features = ["wgpu","ndarray","autodiff","train","tui"]) fires
# all of those features whenever burn is enabled, regardless of which Cargo feature
# (training vs inference) activated it.  Change the dependency to:
[dependencies]
burn = { version = "0.20", optional = true, default-features = false }

[features]
# WASM / native inference only — no training stack.
# burn/webgpu (not burn/wgpu) is required for WASM: it additionally enables
# burn-wgpu/webgpu → cubecl-wgsl (WGSL shaders required by the browser WebGPU API).
# burn/webgpu is a strict superset of burn/wgpu so native inference also works.
inference = ["burn/std", "burn/ndarray", "burn/webgpu", "rfd"]
# rfd 0.15 is WASM-compatible (browser file-picker via <input type="file">):
# keep it in the inference feature so src/ui/file_load.rs compiles.
# src/ui/file_load.rs calls rfd::AsyncFileDialog::new() unconditionally;
# if rfd is absent the crate will fail to compile.
# Full training build — same features as the pre-split Cargo.toml:18 entry plus burn/std.
training  = ["burn/std", "burn/ndarray", "burn/wgpu", "burn/autodiff", "burn/train", "burn/tui"]
```

Verify with `cargo tree --no-default-features --features inference`: `burn-train`,
`ratatui`, `sysinfo`, and `nvml-wrapper` must not appear in the tree.

Also verify with `cargo tree --no-default-features --features training` that the
training build pulls in the same burn sub-crates as the pre-split declaration
(`wgpu`, `ndarray`, `autodiff`, `train`, `tui`). The current `Cargo.toml:18` has
no `default-features = false`, so burn's own defaults are currently included
implicitly; adding `default-features = false` may drop sub-features (e.g.
`burn/fusion`) that are not in the explicit list above. If `cargo tree` reveals
such omissions, add them to the `training` feature and document the change.

Gate RL controllers on `#[cfg(any(feature = "inference", feature = "training"))]`.
The WASM build enables `inference`; the native training build enables `training`.

1. Add the `inference` feature to `Cargo.toml` and re-gate RL controller modules
   on `any(feature = "inference", feature = "training")`.

   **All downstream wiring must also be re-gated** (currently `#[cfg(feature = "training")]`):
   - `src/main.rs:8–12` — RL controller import block → `any(feature = "inference", feature = "training")`
   - `src/main.rs:75` — `cycle_rl_model` system → `any(feature = "visual", feature = "inference")` + training
   - `src/main.rs:86–93` — `apply_rl_controller_switch` / `apply_model_switch` system registrations → same
   - `src/ui/hud.rs:345, 405, 443, 481, 519` — per-controller HUD blocks → `any(feature = "inference", feature = "training")`
   - `src/main.rs:257–294` and `src/main.rs:298–330` — startup RL model-load
     blocks. These are `#[cfg(feature = "training")]` today and use
     `std::path::Path::exists()` to probe for `.mpk` files on disk.
     `Path::exists()` compiles for WASM but always returns `false` at runtime
     (no filesystem). For WASM/inference builds, replace the `Path::exists()`
     probe with the same `include_bytes!` / `NamedMpkBytesRecorder` path used
     in step 2 below. The native path can keep the existing `Path::exists()`
     check unchanged behind `#[cfg(not(target_arch = "wasm32"))]`.

   **1a. Split `src/training/ppo/` into inference-safe and training-only sub-modules.**
   `src/controllers/rl_orbit.rs:20` imports `crate::training::ppo::model::ActorCritic`,
   but `ppo` is currently gated `#[cfg(feature = "training")]` in
   `src/training/mod.rs:6`. Re-gating RL controllers on `inference` without
   also re-gating `ppo::model` will produce `error[E0432]: unresolved import`.
   Move `model.rs` and `lstm_model.rs` (struct definitions + `forward()` only)
   to a sub-module gated on `any(feature = "inference", feature = "training")`.
   Keep `trainer.rs`, `buffer.rs`, and anything that imports `burn-train` or
   `burn-autodiff` gated on `feature = "training"` only. Update
   `src/training/mod.rs` accordingly.

2. Switch RL controller loading from `DefaultFileRecorder` to
   `NamedMpkBytesRecorder` with `include_bytes!()` embedded weights, gated on
   `wasm32` (same msgpack format; drop-in compatible with existing `.mpk` files).
3. Switch the inference backend from `NdArray` to `Wgpu` for the WASM build
   (feature-gate: `#[cfg(target_arch = "wasm32")]` selects `Wgpu`, native keeps
   `NdArray`).

   **Required: async WASM device initialization.** `Default::default()` for
   `WgpuDevice` panics on WASM — WebGPU adapter/device acquisition is inherently
   async in browsers (`cubecl-wgpu` exposes `init_setup_async` for this reason).
   The current synchronous pattern at `src/controllers/rl_orbit.rs:70`
   (`let device: <InfB as Backend>::Device = Default::default();`) and its
   analogues in `rl_level_hold.rs` and `rl_lstm_orbit.rs` must be replaced.
   Recommended pattern: pre-initialize the `WgpuDevice` at WASM app startup via
   `wasm-bindgen-futures::spawn_local`, store it in a shared handle, and pass it
   as a parameter into each controller's `load()` function instead of calling
   `Default::default()` inside `load()`.
4. Validate forward-pass correctness against the native NdArray baseline with
   known test observations.

**Deliverable:** RL policy running in the browser GPU-accelerated via WebGPU.

### Phase 3 (optional) — Remote training, local inference

Keep training 100% native (current setup unchanged). Add a `wasm-pack`-based
build step that bundles a pre-trained model as `include_bytes!`. CI trains
natively, exports weights, the WASM build embeds the latest checkpoint.

---

## 6. Effort and Risk Summary

| Phase | Est. Effort | Primary Risk |
|---|---|---|
| Phase 1 (visual + PID) | 1–2 weeks | `rfd` WASM behavior (smoke test needed) |
| Phase 2 (RL inference) | 2–3 weeks | `inference` feature split; `NamedMpkBytesRecorder` round-trip correctness; WebGPU availability across browsers |
| Training in WASM | Not recommended | Fundamental threading and OS constraints |

The project's clean separation between the training environments (headless,
no ECS) and the visual sim (Bevy + Rapier) is a significant advantage here.
The training code does not need to run in WASM at all; only the inference path
and the sim loop need porting.
