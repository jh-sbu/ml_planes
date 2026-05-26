# ControllerContext Design

Replaces the per-controller ECS side-channel (feed_leader_state / LeaderRef / LeaderState)
with an explicit per-tick context argument threaded through FlightController::update.

---

## Motivation

`FlightController::update(&mut self, state: &FlightState, dt: f32)` only knows about one
plane. WingmanController works around this by having a dedicated ECS prepass
(`feed_leader_state`) downcast `ActiveController` to `WingmanController` and mutate a
`cached_leader` field before the main controller tick. Every future controller that needs
external state (aerial refueling tanker, multi-plane formation, RL policies with peer
observations) would need its own bespoke ECS system + `downcast_mut` injection.

The architectural rule going forward:
- **Per-tick mutable state feed through controller internals is prohibited.**
- **One-time configuration edits via `downcast_mut` are acceptable** (e.g. HUD changes
  orbit center, reconfigures leader, loads RL model).

---

## New Types

Add these in `src/plane/context.rs` and re-export them from `src/plane/mod.rs`. The
controller trait already depends on `crate::plane::{ControlInputs, FlightState}`, so keeping
`ControllerContext` beside `FlightState` avoids introducing a new dependency direction.

### PlaneId

```rust
#[derive(Component, Clone, Copy, PartialEq, Eq, Hash, Debug, Reflect)]
pub struct PlaneId(pub u32);

impl PlaneId {
    /// Canonical test-only ID. Runtime allocation starts at 1; these never collide.
    pub const TEST: PlaneId = PlaneId(0);
}

/// Allocator resource. Initialized so the first runtime ID is PlaneId(1).
#[derive(Resource)]
pub struct NextPlaneId(pub u32);

impl Default for NextPlaneId {
    fn default() -> Self { Self(1) }
}
```

`PlaneId` is the stable per-plane identity. It is separate from:
- `PlaneIndex` — display/camera ordering label (unchanged)
- `Entity` — Bevy ECS implementation detail, not usable in training code

Invariant: every entity that participates in `run_flight_controllers` has exactly one unique
`PlaneId`. `PlaneId::TEST` is for single-plane unit tests and `empty_for`; multi-plane tests
must use distinct IDs.

### SpawnedPlane

```rust
pub struct SpawnedPlane {
    pub entity: Entity,
    pub id:     PlaneId,
}
```

Return value of `spawn_plane`. Gives callers the stable ID immediately so relationship
controllers can be constructed at spawn time without a second query.

### PlaneSnapshot

```rust
pub struct PlaneSnapshot {
    pub id:    PlaneId,
    pub state: FlightState,
}
```

One plane's state for a single tick. Owns `FlightState` (small, Clone) to avoid Bevy borrow
conflicts. No role field — `PlaneRole` is deferred; add composable marker components
(`Leader`, `Tanker`, `FormationSlot { leader, offset }`) when mutual-exclusivity of a single
enum becomes a real constraint.

### ControllerContext

```rust
pub struct ControllerContext<'a> {
    pub own_id: PlaneId,
    pub planes: &'a [PlaneSnapshot],   // ALL planes (own + peers)
}

impl ControllerContext<'_> {
    /// For genuinely peerless callers: unit tests, single-plane training envs.
    /// NOT a shortcut for nested controllers that have real peer context — those
    /// must thread the same ctx through.
    pub fn empty_for(own_id: PlaneId) -> ControllerContext<'static> { ... }

    /// Iterator over peers only (excludes own).
    pub fn others(&self) -> impl Iterator<Item = &PlaneSnapshot> { ... }

    /// Look up any plane by ID, including own.
    pub fn find(&self, id: PlaneId) -> Option<&PlaneSnapshot> { ... }
}
```

### Updated FlightController trait

```rust
pub trait FlightController: Send + Sync + 'static {
    fn update(
        &mut self,
        own: &FlightState,
        ctx: &ControllerContext<'_>,
        dt:  f32,
    ) -> ControlInputs;

    fn name(&self) -> &'static str { "Unknown" }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;

    #[cfg(feature = "visual")]
    fn poll_input(&mut self, _keys: &ButtonInput<KeyCode>, _dt: f32) {}
}
```

Simple controllers (`LevelHold`, `Orbit`, `Waypoint`, `Ascent`, etc.) use `_ctx`. The ignored
parameter has no controller-specific cost; the shared snapshot vector is built once per fixed
tick.

---

## spawn_plane Signature

```rust
pub fn spawn_plane(
    commands: &mut Commands,
    ids:      &mut NextPlaneId,
    // ... existing params unchanged ...
) -> SpawnedPlane
```

Inside `spawn_plane`:
1. Take `current = ids.0`, emit `PlaneId(current)` as a component.
2. Increment `ids.0`.
3. Return `SpawnedPlane { entity, id: PlaneId(current) }`.

`PlanePlugin` initializes the allocator with `app.init_resource::<NextPlaneId>()` and registers
`PlaneId` for reflection.

Setup systems take `mut ids: ResMut<NextPlaneId>`, pass `&mut ids`, and immediately use
`SpawnedPlane.id` when constructing relationship controllers:

```rust
let leader  = spawn_plane(commands, &mut ids, ...);
let wingman = spawn_plane(
    commands,
    &mut ids,
    ...,
    Box::new(WingmanController::new(leader.id, &leader_initial, &own_initial, offset)),
    ControllerKind::Wingman,
    &cfg,
);
```

---

## run_flight_controllers System

Two-phase via `ParamSet` to avoid Bevy startup-panic from overlapping queries on the same
entity set:

```rust
fn run_flight_controllers(
    time: Res<Time<Fixed>>,
    mut params: ParamSet<(
        Query<(&PlaneId, &FlightState)>,
        Query<(&PlaneId, &FlightState, &mut ActiveController, &mut ControlInputs)>,
    )>,
) {
    let dt = time.delta_secs();

    // Phase 1: immutable pass — build snapshot vec.
    let snaps: Vec<PlaneSnapshot> = params.p0()
        .iter()
        .map(|(id, st)| PlaneSnapshot { id: *id, state: st.clone() })
        .collect();

    // Phase 2: mutable pass — tick each controller.
    for (id, state, mut ctrl, mut inputs) in params.p1().iter_mut() {
        let ctx = ControllerContext { own_id: *id, planes: &snaps };
        *inputs = ctrl.0.update(state, &ctx, dt);
        inputs.clamp();
    }
}
```

---

## WingmanController Changes

`cached_leader: Option<FlightState>` is deleted. Replace with:

```rust
pub struct WingmanController {
    pub leader_id:   PlaneId,       // config — set at construction
    pub offset:      FormationOffset,
    pub inner:       LevelHoldController,
    pub range_pid:   PidController,
    pub lateral_pid: PidController,
}
```

`leader_id` is `pub` because the HUD may reconfigure which plane to follow. That is an
occasional edit via `downcast_mut`, not per-tick injection. In `update`:

```rust
fn update(&mut self, own: &FlightState, ctx: &ControllerContext<'_>, dt: f32) -> ControlInputs {
    let Some(leader) = ctx.find(self.leader_id) else {
        return self.inner.update(own, ctx, dt);   // graceful fallback
    };
    // ... existing guidance law using leader.state ...
}
```

---

## Nested Controller Calling Convention

Nested calls must thread `ctx` through — they must NOT create a new `empty_for` context
unless that nested caller is itself genuinely peerless.

| Caller | Inner call |
|---|---|
| `WingmanController` → `self.inner` | `self.inner.update(state, ctx, dt)` |
| `OrbitController` → `self.inner` | `self.inner.update(state, ctx, dt)` |
| `WaypointController` → `self.inner` | `self.inner.update(state, ctx, dt)` |
| `AscentController` → `self.inner` | `self.inner.update(state, ctx, dt)` |
| `RlOrbitResidualController` (ECS) → `self.pid` | `self.pid.update(state, ctx, dt)` — same ctx |
| `ResidualOrbitEnv::step` (training, no peers) | `empty_for(own_id)` — correct |
| Unit tests | `empty_for(PlaneId::TEST)` |

---

## What Is Deleted

- `feed_leader_state` system
- `LeaderRef` component
- `LeaderState` component
- `WingmanController::cached_leader` field

---

## Migration Scope

| Location | Change |
|---|---|
| `plane/context.rs` | Add `PlaneId`, `NextPlaneId`, `PlaneSnapshot`, `ControllerContext`, `SpawnedPlane` |
| `plane/mod.rs` | Re-export new context/identity types |
| `environment/spawner.rs` | `spawn_plane` gains `ids: &mut NextPlaneId`, returns `SpawnedPlane` |
| `main.rs` setup system | `mut ids: ResMut<NextPlaneId>` param; use returned `SpawnedPlane.id` |
| `plane/plugin.rs` | Initialize/register `PlaneId`/`NextPlaneId`; remove `feed_leader_state` from system schedule |
| `controllers/wingman.rs` | Replace `cached_leader`/`LeaderRef`/`LeaderState` with `leader_id: PlaneId` |
| `controllers/traits.rs` | Add `ctx` param to `FlightController::update` |
| All `impl FlightController` (~10 files) | Add `_ctx: &ControllerContext<'_>` (ignored by simple controllers) |
| All inner controller calls | Thread `ctx` through (LevelHold, Orbit, Waypoint, Ascent, RlOrbitResidual) |
| `training/orbit_residual_env.rs` | Pass `ControllerContext::empty_for(own_id)` at the call sites on lines 290 and 448 |
| Integration test local spawn helpers | Insert `PlaneId` manually on any entity that enters a controller query |

---

## TDD Targets (write tests before implementation)

1. `ControllerContext::others()` excludes own plane; `find()` returns own plane by its ID.
2. `WingmanController::update` reads leader state from `ctx.find(self.leader_id)` — the
   struct has no `cached_leader` field.
3. Missing leader in context produces finite fallback outputs on all channels.
4. `ResidualOrbitEnv` and simple unit tests call controllers with
   `ControllerContext::empty_for(PlaneId::TEST)` without panicking.
5. **ECS behavioral test:** Two planes are spawned and tick together in a single
   `FixedUpdate`. The leader's `FlightState` is set to a known value. After one tick the
   wingman's `ControlInputs` reflect a response consistent with that leader state — proving
   per-tick context delivery through `ControllerContext`, not just the absence of the old
   components.
