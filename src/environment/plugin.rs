use bevy::prelude::*;
// Physics-dependent pieces (ground collider, ground-contact detection, the
// physics-pose interpolation buffers) are compiled out of the pure networked
// client, which has no Rapier schedule.
#[cfg(any(not(feature = "net"), feature = "server"))]
use bevy_rapier3d::prelude::PhysicsSet;

#[cfg(any(not(feature = "net"), feature = "server"))]
use super::ground::spawn_ground;
#[cfg(any(not(feature = "net"), feature = "server"))]
use super::spawner::detect_ground_contact;

#[cfg(feature = "visual")]
use super::grid_material::{follow_camera, GridMaterial};
#[cfg(feature = "visual")]
use super::plane_model::attach_plane_models;
#[cfg(all(feature = "visual", any(not(feature = "net"), feature = "server")))]
use super::plane_model::sync_model_to_interpolated_pose;
#[cfg(feature = "visual")]
use super::visual::{
    draw_orbit_pin_gizmo, draw_plane_gizmos, spawn_scene_lighting, spawn_visual_ground,
};
#[cfg(all(feature = "visual", any(not(feature = "net"), feature = "server")))]
use super::visual::{save_curr_physics_pose, save_prev_physics_pose};
#[cfg(feature = "visual")]
use bevy::pbr::MaterialPlugin;

pub struct EnvironmentPlugin;

impl Plugin for EnvironmentPlugin {
    fn build(&self, app: &mut App) {
        // Physics ground collider (death plane) — server / local-sim builds only.
        #[cfg(any(not(feature = "net"), feature = "server"))]
        app.add_systems(Startup, spawn_ground);

        // `visual` is a *library* feature, so it can be unified into a binary that
        // renders nothing: `cargo run --features server --bin ml_planes_server` leaves
        // the default `client` feature on, and the server then links a visual-compiled
        // lib while running `MinimalPlugins`. Nothing below is meaningful there, and
        // every one of these systems takes a render-owned parameter (`GlobalAmbientLight`,
        // `Assets<Mesh>`, `Gizmos`) whose absence fails SystemParam validation and panics
        // on the first frame. Keying the whole block on the renderer actually being
        // present — rather than on the feature alone — is what keeps such a build
        // headless-safe; `MinimalPlugins` is added before this plugin in every headless
        // app, and `DefaultPlugins` before it in every rendering one, so the answer is
        // already settled by the time we ask.
        #[cfg(feature = "visual")]
        if app.is_plugin_added::<bevy::render::RenderPlugin>() {
            app.add_plugins(MaterialPlugin::<GridMaterial>::default());
            app.add_systems(Startup, (spawn_visual_ground, spawn_scene_lighting));
            // Gives every plane that ships a `PlaneVisual` its glTF scene. Runs for both
            // spawn paths: locally finalized planes and replicated ones.
            app.add_systems(Update, attach_plane_models);
            // On a local-sim build the plane's own `Transform` is the raw fixed-step pose,
            // so the model child is placed from the interpolated one instead. A `Read`:
            // it consumes the pose this frame establishes (see `PlaneRenderPose`).
            #[cfg(any(not(feature = "net"), feature = "server"))]
            app.add_systems(
                Update,
                sync_model_to_interpolated_pose
                    .after(attach_plane_models)
                    .in_set(crate::plane::PlaneRenderPose::Read),
            );
            // Draws the plane at its rendered pose, so it must run after whatever
            // establishes that pose this frame (see `PlaneRenderPose`).
            app.add_systems(
                Update,
                draw_plane_gizmos.in_set(crate::plane::PlaneRenderPose::Read),
            );
            app.add_systems(Update, draw_orbit_pin_gizmo);
            // Re-centre the infinite grid after camera movement.
            app.add_systems(
                Update,
                follow_camera
                    .after(crate::camera::systems::update_follow_camera)
                    .after(crate::camera::systems::update_free_look_camera),
            );

            // Physics-pose interpolation buffers feed the visual local sim; the client
            // interpolates from replicated state instead (see `crate::net::client`).
            #[cfg(any(not(feature = "net"), feature = "server"))]
            app.add_systems(
                FixedUpdate,
                save_prev_physics_pose.before(PhysicsSet::SyncBackend),
            );
            #[cfg(any(not(feature = "net"), feature = "server"))]
            app.add_systems(
                FixedUpdate,
                save_curr_physics_pose.after(PhysicsSet::Writeback),
            );
        }

        // Ground-contact detection reads the Rapier context — not present on the
        // client, which renders replicated planes and never collides locally.
        #[cfg(any(not(feature = "net"), feature = "server"))]
        app.add_systems(
            FixedUpdate,
            detect_ground_contact.after(PhysicsSet::StepSimulation),
        );
    }
}

#[cfg(all(test, feature = "visual"))]
mod tests {
    use super::*;
    use crate::plane::{PlanePlugin, PHYSICS_DT};
    use bevy_rapier3d::prelude::{NoUserData, RapierPhysicsPlugin, TimestepMode};

    /// `visual` is a *library* feature, and the documented server invocation
    /// (`cargo run --features server --bin ml_planes_server`) leaves the default
    /// `client` feature on — so the headless server links a `visual`-compiled lib.
    /// Its app is `MinimalPlugins`, which never builds the render stack, and every
    /// rendering system here takes a render-owned parameter (`GlobalAmbientLight`,
    /// `Assets<Mesh>`, `Gizmos`). Registering them unconditionally fails SystemParam
    /// validation and panics on the first frame.
    #[test]
    fn headless_app_boots_with_the_visual_feature_compiled_in() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .add_plugins(bevy::transform::TransformPlugin)
            .add_plugins(bevy::asset::AssetPlugin::default())
            .insert_resource(TimestepMode::Fixed {
                dt: PHYSICS_DT,
                substeps: 1,
            })
            .add_plugins(RapierPhysicsPlugin::<NoUserData>::default().in_fixed_schedule())
            .add_plugins(PlanePlugin)
            .add_plugins(EnvironmentPlugin);
        app.finish();

        app.update();
    }

    /// The other direction, and the one with teeth: the guard must not silently
    /// disable rendering in the real client. `MaterialPlugin::<GridMaterial>` is added
    /// only inside the guarded block, so its asset collection existing proves the block
    /// ran. `RenderPlugin` is added here for its registration alone — `build()` does not
    /// touch the GPU (device init is deferred to `finish()`, which this test never
    /// calls), so this stays a headless test. It also pins that
    /// `bevy::render::RenderPlugin` is the same type `DefaultPlugins` registers: if that
    /// path ever moves, this fails instead of the client quietly rendering nothing.
    #[test]
    fn an_app_with_a_renderer_registers_the_rendering_systems() {
        let mut app = App::new();
        app.add_plugins(MinimalPlugins)
            .add_plugins(bevy::transform::TransformPlugin)
            .add_plugins(bevy::asset::AssetPlugin::default())
            .add_plugins(bevy::render::RenderPlugin::default())
            .add_plugins(EnvironmentPlugin);

        assert!(
            app.world().get_resource::<Assets<GridMaterial>>().is_some(),
            "EnvironmentPlugin skipped its rendering block in an app that has a renderer"
        );
    }
}
