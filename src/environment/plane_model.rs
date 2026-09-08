//! Loading and placing a plane's visual model.
//!
//! An airframe that ships a [`PlaneVisual`] gets its glTF scene attached as a **child**
//! of the plane entity; one that does not keeps the gizmo wireframe
//! ([`super::visual::draw_plane_gizmos`]). Both spawn paths converge here: the local sim
//! gets `PlaneVisual` from `finalize_pending_spawns`, and the pure networked client —
//! which never loads a `.plane.ron` — gets it by replication.
//!
//! # Frames
//!
//! The plane entity's local frame **is** the simulator's body frame (+X nose, +Y right
//! wing, +Z up): `FlightState.attitude` is written straight into `Transform.rotation`,
//! and level flight is `Quat::from_rotation_x(-FRAC_PI_2)`. An exported model is
//! generally *not* in that frame — `ml_planes_assets` ships Blender's Y-up glTF
//! conversion — so [`ModelOrientation::fixup`](crate::plane::ModelOrientation::fixup)
//! rotates it there on the child.

use bevy::asset::AssetPath;
use bevy::gltf::GltfAssetLabel;
use bevy::prelude::*;
use bevy::scene::SceneRoot;

use crate::plane::PlaneVisual;

use super::spawner::sanitize_asset_path;

#[cfg(any(not(feature = "net"), feature = "server"))]
use super::visual::{rendered_pose, PhysicsInterp};

/// A plane's resolved visual model, inserted once [`attach_plane_models`] has decided
/// what to do with its [`PlaneVisual`]: `Some(child)` once the scene is attached,
/// `None` once the path has been rejected.
///
/// The component's mere presence means "already decided", which is what stops the
/// attach system from re-spawning a model (or re-logging a rejection) every frame.
#[derive(Component, Debug)]
pub struct PlaneModel(pub Option<Entity>);

/// The model child's local transform: the orientation fix-up that carries an exported
/// scene into the body frame, plus the airframe's scale and origin offset.
pub fn model_child_transform(visual: &PlaneVisual) -> Transform {
    Transform {
        translation: visual.offset,
        rotation: visual.orientation.fixup(),
        scale: Vec3::splat(visual.scale),
    }
}

/// Validate a [`PlaneVisual::scene`] and return the labelled glTF path to load, or
/// `None` if the path is not a clean assets-relative one.
///
/// **Security boundary (CLAUDE.md §7).** This is the `AssetServer::load` sink for
/// `scene`, and on a client that string arrives straight off the wire — a `PlaneVisual`
/// is a replicated component, so a peer's server chooses it. Validation therefore lives
/// here at the sink rather than where the component is inserted, so a future caller
/// cannot reintroduce the hole.
///
/// The glTF sub-asset label is appended **after** validation on purpose:
/// `GltfAssetLabel::Scene(0).from_asset(p)` yields `…glb#Scene0`, and
/// `sanitize_asset_path` rejects `#` outright (it is how Bevy's `AssetPath` selects a
/// sub-asset, a namespace escape the validator has no reason to allow through).
pub fn scene_asset_path(scene: &str) -> Option<AssetPath<'static>> {
    let safe = sanitize_asset_path(scene)?;
    Some(GltfAssetLabel::Scene(0).from_asset(safe))
}

/// Whether the fallback body wireframe should be drawn for a plane.
///
/// Only a plane with a real model attached loses it — an airframe with no
/// [`PlaneVisual`], one still waiting on the attach system, and one whose scene path was
/// rejected all keep the wireframe rather than rendering as nothing at all.
pub fn draws_body_wireframe(model: Option<&PlaneModel>) -> bool {
    !matches!(model, Some(PlaneModel(Some(_))))
}

/// Attach the glTF scene of every plane carrying a [`PlaneVisual`] and no
/// [`PlaneModel`] yet.
///
/// The scene is spawned as a child, so it inherits the plane's pose and is despawned
/// with it — no orphan bookkeeping. On the networked client the parent `Transform` is
/// already the interpolated render pose; on a local-sim build
/// [`sync_model_to_interpolated_pose`] corrects for the raw fixed-step one.
pub fn attach_plane_models(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    planes: Query<(Entity, &PlaneVisual), Without<PlaneModel>>,
) {
    for (entity, visual) in &planes {
        let Some(path) = scene_asset_path(&visual.scene) else {
            // The path alone, never a loader error: this string can arrive off the wire
            // and `ron`/`gltf` errors quote file contents (CLAUDE.md §7).
            eprintln!(
                "plane visual scene '{}' is not a valid asset path; drawing the \
                 wireframe instead",
                visual.scene
            );
            commands.entity(entity).insert(PlaneModel(None));
            continue;
        };

        let child = commands
            .spawn((
                SceneRoot(asset_server.load(path)),
                model_child_transform(visual),
            ))
            .id();
        commands.entity(entity).insert(PlaneModel(Some(child)));
        commands.entity(entity).add_child(child);
    }
}

/// Keep the model on the *rendered* pose on local-sim builds.
///
/// A plane's `Transform` there holds the raw 64 Hz fixed-step pose, which is why
/// [`super::visual::draw_plane_gizmos`] renders from [`PhysicsInterp`] instead; a child
/// mesh would inherit the un-interpolated one and judder. This rewrites the child's
/// *local* transform to cancel that, and degenerates to the plain fix-up whenever the
/// parent already holds the rendered pose (as it does on the networked client, where
/// this system is not compiled in at all).
#[cfg(any(not(feature = "net"), feature = "server"))]
pub fn sync_model_to_interpolated_pose(
    planes: Query<(&Transform, &PhysicsInterp, &PlaneVisual, &PlaneModel)>,
    mut models: Query<&mut Transform, Without<PlaneModel>>,
    time_fixed: Res<Time<Fixed>>,
) {
    let alpha = time_fixed.overstep_fraction();
    for (plane_transform, interp, visual, model) in &planes {
        let Some(child) = model.0 else { continue };
        let Ok(mut child_transform) = models.get_mut(child) else {
            continue;
        };
        let (pos, rot) = rendered_pose(Some(interp), plane_transform, alpha);
        *child_transform = interp_corrected_child_transform(plane_transform, pos, rot, visual);
    }
}

/// The child transform that puts the model at the *rendered* pose `(pos, rot)` while
/// still parented to a plane sitting at `plane_transform`.
///
/// The plane is never scaled, so the parent's inverse is just the conjugate quaternion
/// plus a translation. When the parent already holds the rendered pose this collapses
/// to [`model_child_transform`] exactly.
#[cfg(any(not(feature = "net"), feature = "server"))]
pub fn interp_corrected_child_transform(
    plane_transform: &Transform,
    pos: Vec3,
    rot: Quat,
    visual: &PlaneVisual,
) -> Transform {
    let base = model_child_transform(visual);
    let inv = plane_transform.rotation.inverse();
    Transform {
        translation: inv * ((pos + rot * base.translation) - plane_transform.translation),
        rotation: inv * rot * base.rotation,
        scale: base.scale,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plane::ModelOrientation;

    fn visual(scene: &str) -> PlaneVisual {
        PlaneVisual {
            scene: scene.to_string(),
            orientation: ModelOrientation::BlenderYUp,
            scale: 1.0,
            offset: Vec3::ZERO,
        }
    }

    #[test]
    fn scene_path_carries_the_gltf_scene_label() {
        // The label must survive onto the path, or the loader gets a `Gltf`, not a
        // `Scene`, and nothing renders.
        let path = scene_asset_path("models/generic_jet.glb").expect("a clean path");
        assert_eq!(path.to_string(), "models/generic_jet.glb#Scene0");
    }

    #[test]
    fn scene_path_rejects_traversal_and_absolute_paths() {
        // `PlaneVisual` is replicated, so `scene` is attacker-influenced on a client.
        for scene in [
            "../../etc/passwd",
            "/etc/passwd",
            "assets/models/generic_jet.glb",
            r"..\models\x.glb",
            "models/x.glb#Scene9",
            "",
        ] {
            assert!(
                scene_asset_path(scene).is_none(),
                "must not reach AssetServer::load: {scene}"
            );
        }
    }

    #[test]
    fn child_transform_applies_orientation_scale_and_offset() {
        let mut v = visual("models/generic_jet.glb");
        v.scale = 2.0;
        v.offset = Vec3::new(1.0, 0.0, 0.5);
        let t = model_child_transform(&v);

        assert_eq!(t.translation, Vec3::new(1.0, 0.0, 0.5));
        assert_eq!(t.scale, Vec3::splat(2.0));
        // Same contract as `ModelOrientation::fixup`: export up (+Y) becomes body up.
        assert!((t.rotation * Vec3::Y - Vec3::Z).length() < 1e-6);
    }

    #[test]
    fn a_body_frame_model_needs_no_rotation() {
        let mut v = visual("models/x.glb");
        v.orientation = ModelOrientation::BodyFrame;
        assert_eq!(model_child_transform(&v).rotation, Quat::IDENTITY);
    }

    /// On the networked client the parent already holds the rendered pose, so the
    /// correction must be a no-op — otherwise the model would be displaced on the one
    /// build where nothing is wrong.
    #[cfg(any(not(feature = "net"), feature = "server"))]
    #[test]
    fn correction_is_identity_when_the_parent_holds_the_rendered_pose() {
        let v = visual("models/generic_jet.glb");
        let plane = Transform::from_xyz(400.0, 500.0, -20.0)
            .with_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2));
        let corrected =
            interp_corrected_child_transform(&plane, plane.translation, plane.rotation, &v);
        let base = model_child_transform(&v);

        assert!(corrected.translation.abs_diff_eq(base.translation, 1e-4));
        assert!(corrected.rotation.abs_diff_eq(base.rotation, 1e-5));
        assert_eq!(corrected.scale, base.scale);
    }

    /// On a local-sim build the parent is a fixed step behind. Composed with that stale
    /// parent, the child must land on the rendered pose — which is where the gizmos are
    /// drawn, so the mesh and the nose arrow agree.
    #[cfg(any(not(feature = "net"), feature = "server"))]
    #[test]
    fn correction_puts_the_model_on_the_rendered_pose_under_a_stale_parent() {
        let mut v = visual("models/generic_jet.glb");
        v.offset = Vec3::new(0.5, 0.0, -0.25);
        // A step's worth of motion and rotation between the physics pose and the
        // rendered one.
        let plane = Transform::from_xyz(400.0, 500.0, 0.0)
            .with_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2));
        let pos = Vec3::new(401.6, 500.2, 0.3);
        let rot = plane.rotation * Quat::from_rotation_z(0.05);

        let child = interp_corrected_child_transform(&plane, pos, rot, &v);
        // Compose parent ∘ child, as Bevy's transform propagation does.
        let world = plane * child;
        let base = model_child_transform(&v);

        assert!(
            world
                .translation
                .abs_diff_eq(pos + rot * base.translation, 1e-3),
            "world translation {} should be the rendered pose",
            world.translation
        );
        assert!(
            world.rotation.abs_diff_eq(rot * base.rotation, 1e-5),
            "world rotation should be the rendered attitude times the fix-up"
        );
    }

    #[test]
    fn only_an_attached_model_suppresses_the_wireframe() {
        // No `PlaneVisual` at all, still-undecided, and a rejected path must all keep
        // the wireframe — otherwise the plane renders as nothing.
        assert!(draws_body_wireframe(None));
        assert!(draws_body_wireframe(Some(&PlaneModel(None))));
        assert!(!draws_body_wireframe(Some(&PlaneModel(Some(
            Entity::from_raw_u32(1).unwrap()
        )))));
    }
}
