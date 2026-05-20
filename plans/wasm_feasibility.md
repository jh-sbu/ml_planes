# WASM Target Feasibility Report

*Assessed against: Cargo.toml + Cargo.lock as of May 2026 (Bevy 0.18.1, bevy_rapier3d 0.33, burn 0.20.1, wgpu 26)*

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
renders via WebGL2 or WebGPU depending on browser support.

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
- The `visual` feature as-is should otherwise compile; `rfd` needs smoke testing.

**Estimated effort: ~1–2 weeks** to get a playable browser demo without RL.

---

## 2. RL Inference in the Browser

### What works

`burn-wgpu` (via `cubecl-wgpu`) already depends on `wgpu 26`, which has explicit
WASM bindings (`js-sys`, `wasm-bindgen`, `web-sys` are all in the lock file for
`wgpu 26`). WebGPU is available in Chrome 113+/Firefox 121+ (behind a flag on
some versions). A trained model running inference on `Wgpu` backend would
leverage the browser's GPU.

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
const MODEL_BYTES: &[u8] = include_bytes!("../models/orbit/ppo_orbit_1.mpk");
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

**C. Hard-code small networks**
The ActorCritic networks are 2-layer MLPs (obs_dim→64→64→action_dim). At
~50K parameters, they could be embedded as static float arrays and evaluated
without burn's serialization infrastructure entirely. Extreme but eliminates
all burn dependency from the WASM binary.

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
4. Use `trunk` as the build/serve tool (`trunk serve`).
5. Verify the visual + Rapier sim runs in Chrome with WebGL2.

**Deliverable:** playable flight sim in the browser, PID controllers only.

### Phase 2 — RL inference in browser

**Prerequisite — feature split:** The current `training` feature enables
`burn/train`, `burn/tui`, and `burn/autodiff`, all of which are WASM-incompatible.
RL controllers are also gated `#[cfg(feature = "training")]`
(`src/controllers/mod.rs:9–16`). Simply adding `#[cfg(target_arch = "wasm32")]`
guards inside the existing `training` feature does not remove these heavyweight
crates from the compile graph. A separate Cargo feature is required:

```toml
inference = ["burn/ndarray", "burn/wgpu"]
```

Gate RL controllers on `#[cfg(any(feature = "inference", feature = "training"))]`.
The WASM build enables `inference`; the native training build enables `training`.

1. Add the `inference` feature to `Cargo.toml` and re-gate RL controller modules
   on `any(feature = "inference", feature = "training")`.
2. Switch RL controller loading from `DefaultFileRecorder` to
   `NamedMpkBytesRecorder` with `include_bytes!()` embedded weights, gated on
   `wasm32` (same msgpack format; drop-in compatible with existing `.mpk` files).
3. Switch the inference backend from `NdArray` to `Wgpu` for the WASM build
   (feature-gate: `#[cfg(target_arch = "wasm32")]` selects `Wgpu`, native keeps
   `NdArray`).
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
