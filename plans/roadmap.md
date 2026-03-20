 ml_planes — Environment Phase Implementation Roadmap

 Context

 Milestones 0–5 are fully implemented and passing tests. The core types (plane/, aerodynamics/, controllers/, training/) and the plane physics systems/plugin are all complete. The project is now in the environment phase: M6 (environment/ module), M7 (camera/ + ui/), M8 (main.rs app assembly), and M9 (integration tests) remain.

 ---
 Milestone 0 — Cargo.toml fixup [COMPLETE]

 All three build modes verified passing. No Cargo.toml changes were needed beyond
 the initial skeleton.

 Key insight: `bevy_asset` IS an optional feature of the `bevy` meta-crate.
 Always use `bevy = { version = "0.18", default-features = false, features = ["bevy_asset"] }`.
 Without `bevy_asset`, the `bevy::asset` module is absent and asset-related
 types (`Handle`, `Assets`, `AssetLoader`) are unavailable — even in headless
 builds that don't render.

 Other core sub-crates (`bevy_ecs`, `bevy_app`, `bevy_math`, `bevy_reflect`,
 `bevy_transform`, `bevy_time`, `bevy_hierarchy`) are non-optional and always
 present; do not list them as explicit features.

 Verified:
   cargo check --no-default-features                    ✓
   cargo check --no-default-features --features training ✓
   cargo check                                          ✓

 ---
 Milestone 1 — plane/ core types [COMPLETE]

 Files: src/plane/{mod,config,state,inputs}.rs

 Pure data — no Bevy systems, no Rapier. Establishes the body-frame convention used everywhere:
 - +X = forward (nose), +Y = right wing, +Z = up through cockpit

 PlaneConfig — Asset + TypePath + Reflect + Serialize + Deserialize. Fields match generic_jet.plane.ron exactly (snake_case). inertia: Vec3 (Ixx, Iyy, Izz diagonal).

 FlightState — Component + Default. Fields: position, velocity, attitude: Quat, angular_velocity (body frame, p/q/r), alpha, beta, airspeed, altitude. Method update_air_data(&mut self)
 rotates world-frame velocity into body frame to compute α, β, and airspeed.

 ControlInputs — Component + Default. Fields: aileron, elevator, rudder [-1,1]; throttle [0,1]. Method clamp(&mut self).

 Tests (inline #[cfg(test)] in state.rs):
 - update_air_data with pure forward velocity → α=0, β=0, airspeed=|v|
 - Velocity tilted 10° nose-up → α≈0.1745 rad
 - Near-zero airspeed → no NaN, α=β=0

 ---
 Milestone 2 — aerodynamics/ model [COMPLETE]

 Files: src/aerodynamics/{mod,model}.rs

 Pure function — no ECS. compute_aero_forces(state, inputs, cfg) -> AeroForces { force_body: Vec3, torque_body: Vec3 }. All output in body frame (caller rotates to world via attitude quat).

 Equations:
 q̄ = ½ · 1.225 · V
 δe = inputs.elevator · cfg.elevator_limit   (and similarly for a, r)

 CL = clamp(cl0 + cl_alpha·α + cl_delta_e·δe, -cl_max, cl_max)
 CD = cd0 + cd_induced·CL²
 lift  = q̄·S·CL                     → force_body.
 drag  = q̄·S·CD                     → force_body.x (negative, opposing motion
 thrust = inputs.throttle · thrust_max → force_body.x (positive)

 Cm = cm0 + cm_alpha·α + cm_q·(q·c̄/2V) + cm_delta_e·δ
 Cl = cl_beta·β + cl_p·(p·b/2V) + cl_r·(r·b/2V) + cl_delta_a·δa
 Cn = cn_beta·β + cn_r·(r·b/2V) + cn_delta_r·δr

 pitch_moment = q̄·S·c̄·Cm   → torque_body
 roll_moment  = q̄·S·b·Cl    → torque_body.
 yaw_moment   = q̄·S·b·Cn    → torque_body.

 Guard: return zero forces when V < 1e-3 to avoid division by zero.

 Note: PlaneConfig currently lacks cy_beta; side force is zero for now. Add later if needed.

 Tests (inline in model.rs — most critical tests in project):
 - zero_airspeed_returns_zero — no NaN/panic
 - level_flight_lift_vs_weight — at trim speed, lift ≈ 49050 N
 - drag_is_positive
 - aileron_positive_produces_positive_roll_moment
 - elevator_up_increases_lift_and_nose_up_pitch
 - cl_clamped_at_cl_max
 - cm_q_damping_sign — positive q → negative Cm contribution (cm_q = -8.0)

 ---
 Milestone 3 — controllers/ trait + PID + Manual [COMPLETE]

 Files: src/controllers/{mod,traits,pid,manual,component}.rs

 FlightController trait (traits.rs):
 pub trait FlightController: Send + Sync + 'static {
     fn update(&mut self, state: &FlightState, dt: f32) -> ControlInputs;
     fn name(&self) -> &'static str { "Unknown" }
 }

 PidController<T: f32> (pid.rs): Generic struct with kp, ki, kd, integral_clamp, output_min/max, accumulated integral: T, prev_error: Option<T>. Method update(error: T, dt: f32) -> f32 and
 reset(). For now instantiated only with T = f32; designed generically for future extensions.

 ManualController (manual.rs): Holds current: ControlInputs. Method read_keyboard(keys: &ButtonInput<KeyCode>, dt: f32) (W/S=elevator, A/D=aileron, Q/E=rudder, Shift/Ctrl=throttle) with
 input ramping. Implements FlightController by returning current.

 ActiveController (component.rs): Newtype Component wrapping Box<dyn FlightController>. Stored on plane entities.

 Input/physics decoupling: read_keyboard runs in Update, writes to current. run_flight_controllers runs in FixedUpdate, reads from current. This prevents input rate coupling to physics
 timestep.

 Tests (inline in pid.rs):
 - p_only — ki=kd=0, error=1.0 → output=kp
 - integral_accumulates — constant error, N steps
 - integral_windup_clamped
 - output_clamped_to_limits
 - derivative_zero_on_first_call
 - reset_clears_state

 ---
 Milestone 4 — training/ interface [COMPLETE]

 Files: src/training/{mod,env}.rs

 Interface only — no implementations yet.

 SpawnSpec: Optional fields for position, velocity, attitude: Quat, angular_velocity.

 TrainingEnv trait:
 pub trait TrainingEnv: Send + Sync + 'static {
     fn reset(&mut self) -> (Observation, SpawnSpec);
     fn step(&mut self, action: &[f32]) -> (Observation, f32, bool, StepInfo);
     fn observation_dim(&self) -> usize;
     fn action_dim(&self) -> usize;
 }
 pub type Observation = Vec<f32>;
 pub struct StepInfo { pub episode_step: u32, pub extra: HashMap<String, f32> }

 SpawnSpec is reused in M6's spawner, linking the reset interface to entity spawning without committing to an implementation.

 No tests this milestone.

 ---
 Milestone 5 — plane/ systems + plugin [COMPLETE]

 Files: src/plane/{systems,plugin}.rs

 Three FixedUpdate systems, chained in order:

 sync_flight_state: Reads Rapier's Transform + Velocity, writes FlightState. Critical: Velocity.angvel is world-frame in bevy_rapier3d — must rotate to body frame: state.angular_velocity =
 transform.rotation.conjugate().mul_vec3(rapier_vel.angvel). Then calls state.update_air_data().

 run_flight_controllers: Reads FlightState, calls controller.0.update(state, dt), writes ControlInputs, calls inputs.clamp(). Uses Time<Fixed> for dt.

 apply_aerodynamic_forces: Reads FlightState + ControlInputs + PlaneConfigHandle,
 looks up PlaneConfig via `plane_configs.get(&handle.0)`, calls compute_aero_forces,
 rotates body→world via state.attitude.mul_vec3(...), writes ExternalForce.
 Note: `Handle<T>` is NOT a Component in Bevy 0.18 — use the `PlaneConfigHandle`
 newtype wrapper instead. Note: ExternalForce is not auto-cleared by Rapier —
 overwriting it each frame is correct.

 PlanePlugin (plugin.rs):
 - app.init_asset::<PlaneConfig>()
 - app.register_type::<PlaneConfig>()
 - Custom AssetLoader for .plane.ron: implements bevy::asset::AssetLoader, deserializes with ron::de::from_bytes, registered with app.init_asset_loader::<PlaneConfigLoader>(). Extensions:
 ["plane.ron"].
 - Adds the three systems as .into_configs().chain() in FixedUpdate.
   Note: In Bevy 0.18, `.chain()` on a tuple of systems is ambiguous because
   `bevy::prelude::Curve` also defines a `chain` method. Use `.into_configs().chain()`
   — `into_configs()` is unambiguously from `IntoScheduleConfigs`.

 Scheduling note: Systems run in FixedUpdate before Rapier's PhysicsSet::StepSimulation. Forces applied via ExternalForce are consumed in that step. Verify ordering with bevy_rapier3d 0.28's
  PhysicsSet schedule documentation.

 ---
 Milestone 6 — environment/ module [COMPLETE]

 Files: src/environment/{mod,ground,spawner,plugin}.rs + visual.rs (cfg visual)

 ground.rs: Spawns Collider::halfspace(Vec3::Y).unwrap() with RigidBody::Fixed at Y=0 — the death
 plane. Note: Collider::halfspace returns Option<Self>, not Self — always call .unwrap() (Vec3::Y is
 always a valid halfspace normal).

 spawner.rs — spawn_plane(commands, params, cfg) -> Entity:
 Spawns entity with RigidBody::Dynamic, a Collider::cuboid (rough fuselage), Velocity { linvel, angvel } (angvel converted body→world), ExternalForce::default(),
 AdditionalMassProperties::MassProperties(MassProperties {
     local_center_of_mass: Vec3::ZERO,
     mass: cfg.mass,
     principal_inertia: cfg.inertia,
     principal_inertia_local_frame: Quat::IDENTITY,   // required — four fields, not three
 }),
 FlightState::default(), ControlInputs::default(), ActiveController(controller), PlaneConfigHandle(server.load("planes/generic_jet.plane.ron")),
 and Transform. Mesh/material components added only under #[cfg(feature = "visual")].

 Default spawn position: Vec3::new(0.0, 500.0, 0.0), default velocity: Vec3::new(100.0, 0.0, 0.0).

 Death detection (spawner.rs or ground.rs) — uses observer pattern (Bevy 0.18):
 fn detect_ground_contact(
     query: Query<Entity, With<FlightState>>,
     rapier_context: ReadRapierContext,              // NOT Res<RapierContext>
     ground_query: Query<Entity, (With<Collider>, Without<FlightState>)>,
     mut commands: Commands,
 ) {
     let Ok(ctx) = rapier_context.single() else { return };
     // ... check ctx.contact_pair(plane, ground) ...
     commands.trigger(PlaneGroundContactEvent(entity));
 }

 EnvironmentPlugin: Adds spawn_ground and (cfg visual) spawn_visual_ground in Startup;
 adds observer via app.add_observer(...) — no App::add_event / EventWriter needed (Bevy 0.18).

 visual.rs (cfg visual only): Spawns a large Plane3d mesh (5000m×5000m) at Y=-0.01 with a green
 StandardMaterial. Fog uses DistanceFog (not FogSettings):
   use bevy::pbr::{DistanceFog, FogFalloff};
   commands.spawn(DistanceFog { color: …, falloff: FogFalloff::Linear { start, end }, ..default() });
 Custom WGSL grid shader can be added later.

 ---
 Milestone 7 — camera/ + ui/ (visual only)

 Files: src/camera/{mod,mode,systems,plugin}.rs, src/ui/{mod,hud,plugin}.rs

 Both modules compile only under #[cfg(feature = "visual")].

 CameraMode resource: FreeLook | Follow(Entity).

 Camera systems (in Update):
 - update_free_look_camera: accumulate yaw/pitch from MouseMotion (while RMB held), clamp pitch ±89°, apply to camera Transform.
 - update_follow_camera: when Follow(e), compute world offset (30m behind, 10m above), lerp camera position, looking_at(target).

 spawn_camera (Startup): Spawns Camera3d::default() at (0, 10, 30) looking at origin.

 draw_flight_hud (Update, bevy_egui EguiContexts): egui window "Flight Data" showing airspeed (m/s + kts), altitude (m), α/β (deg), angular rates (deg/s), control input bars, controller
 name. Targets the Follow entity if set, otherwise first plane.

 No tests for camera/UI.

 ---
 Milestone 8 — main.rs app assembly

 Thin wiring. Pattern:
 #[cfg(feature = "visual")] app.add_plugins(DefaultPlugins);
 #[cfg(not(feature = "visual"))] app.add_plugins(MinimalPlugins);
 app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default());
 app.add_plugins(PlanePlugin).add_plugins(EnvironmentPlugin);
 #[cfg(feature = "visual")]
 app.add_plugins(EguiPlugin).add_plugins(CameraPlugin).add_plugins(UiPlugin);

 setup system: loads "planes/generic_jet.plane.ron" via AssetServer, spawns ground, spawns one plane with ManualController. Asset loading is async — apply_aerodynamic_forces silently skips
 until PlaneConfig is ready (guarded by plane_configs.get(handle)). Add a SimState (Loading/Running) state machine if needed.

 ManualController::read_keyboard runs in Update (before FixedUpdate systems), writing to controller.0's internal current: ControlInputs.

 ---
 Milestone 9 — Integration tests

 Location: tests/; all pass with cargo test --no-default-features.

 Helper (tests/common/mod.rs):
 fn build_headless_app() -> App {
     let mut app = App::new();
     app.add_plugins(MinimalPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(PlanePlugin);
     app
 }

 tests/aero_physics.rs: Spawn plane at 500m, trim velocity. Run app.update() N times. Assert altitude stays within ±50m (energy not exploding). Validates full aero→Rapier pipeline.

 tests/pid_convergence.rs: No Bevy needed. Simulate PidController<f32> against a first-order plant over N steps. Assert error < tolerance.

 tests/spawn_reset.rs: Call spawn_plane with a specific SpawnSpec. Run one sync_flight_state tick. Assert FlightState fields match spec values.

 ---
 Critical Files

 ┌─────────────────────────────────────┬─────────────────────────────────────────────────────────────────────────────────┐
 │                File                 │                                  Why Critical                                   │
 ├─────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────────────┤
 │ Cargo.toml                          │ Must compile correctly in all 3 modes (visual, training, --no-default-features) │
 ├─────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────────────┤
 │ src/plane/state.rs                  │ Establishes body-frame convention; sign errors here corrupt all physics         │
 ├─────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────────────┤
 │ src/aerodynamics/model.rs           │ Physical core; tested exhaustively before integration                           │
 ├─────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────────────┤
 │ src/plane/systems.rs                │ World↔body frame rotation for angvel and forces; scheduling order               │
 ├─────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────────────┤
 │ assets/planes/generic_jet.plane.ron │ Field names must match PlaneConfig exactly for RON deserialization              │
 └─────────────────────────────────────┴─────────────────────────────────────────────────────────────────────────────────┘

 Cross-Cutting Concerns

 - Body-frame convention (X fwd, Y right, Z up) declared in state.rs as a module-level comment; referenced in model.rs and systems.rs.
 - Velocity.angvel is world-frame in bevy_rapier3d — always rotate to body before storing in FlightState.
 - ExternalForce is not auto-cleared — overwriting it each FixedUpdate tick is correct and intentional.
 - Feature flag discipline — core types compile in all configurations; only rendering code is behind #[cfg(feature = "visual")].
 - RON asset loader — Bevy 0.18 does not provide a first-party RON loader for non-Scene assets; implement a custom AssetLoader (~20 lines using ron::de::from_bytes).
 - Bevy 0.18 event system is observer-based: use commands.trigger(MyEvent(…)) to fire and
   app.add_observer(|on: On<MyEvent>| …) to listen. EventWriter, EventReader, and App::add_event
   do NOT exist. No registration step needed.
 - Collider::halfspace returns Option<Self> — always .unwrap() (valid normals never return None).
 - RapierContext access: system param is ReadRapierContext; obtain context via rapier_context.single()
   which returns Result<_> — use `let Ok(ctx) = … else { return }`.
 - MassProperties (dim3) requires four fields: mass, local_center_of_mass, principal_inertia, AND
   principal_inertia_local_frame: Quat::IDENTITY.
 - Fog in Bevy 0.18 is DistanceFog (bevy::pbr::DistanceFog), not FogSettings.

 Verification

 # Headless compile + all tests
 cargo test --no-default-features

 # Visual build (no run needed; just verify it compiles)
 cargo build

 # Training build
 cargo build --no-default-features --features training

 # Interactive smoke test
 cargo run
 # → plane spawns at 500m, WASD controls work, HUD shows live flight data
 # → fly into ground → PlaneGroundContactEvent fires (log message)
