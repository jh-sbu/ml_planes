//! The `PlaneRenderPose` ordering contract.
//!
//! Regression cover for the synchronized render "pulse". On the networked client
//! `render_net_interpolation` establishes each plane's rendered `Transform` during
//! `Update`, while `update_follow_camera` and `draw_plane_gizmos` read it in the *same*
//! schedule. They conflict on `Transform`, so Bevy serialised them — but with no ordering
//! edge the order was unspecified and flipped between frames, so the follow camera
//! intermittently smoothed toward the **previous** frame's pose. An exponential smoother
//! cannot oscillate on a monotonic input, but it does on an alternating one: the camera
//! oscillated at ~4-6 Hz, and since every plane shares one camera they all appeared to
//! pulse in perfect sync. World-space plane motion stayed perfectly smooth throughout,
//! which is exactly why world-space diagnostics reported `reversals=0` and the artifact
//! hid for so long.
//!
//! Bevy confirmed the defect directly via `ScheduleBuildSettings::ambiguity_detection`:
//!   `-- render_net_interpolation and update_follow_camera`
//!   `   conflict on: ["bevy_transform::components::transform::Transform"]`
//!
//! The real writer/readers are `net`/`visual`-gated and cannot be driven from a headless
//! core test, so this pins the contract `PlanePlugin` establishes for them: everything in
//! `PlaneRenderPose::Write` runs before everything in `PlaneRenderPose::Read`.

use bevy::prelude::*;
use ml_planes::plane::{PlanePlugin, PlaneRenderPose};

/// Stands in for the plane pose: `pose` is what a writer establishes this frame, and
/// `reader_saw` is what a reader observed. A reader that runs first sees the stale
/// pre-write value — the bug this contract prevents.
#[derive(Resource, Default)]
struct Probe {
    pose: u32,
    reader_saw: Option<u32>,
}

fn write_pose(mut probe: ResMut<Probe>) {
    probe.pose += 1;
}

fn read_pose(mut probe: ResMut<Probe>) {
    let pose = probe.pose;
    probe.reader_saw = Some(pose);
}

/// Both probes take `ResMut<Probe>`, so they conflict exactly as the real systems do on
/// `Transform` — Bevy must serialise them, and only the set ordering decides which wins.
fn probe_app() -> App {
    let mut app = App::new();
    app.add_plugins(MinimalPlugins);
    app.add_plugins(bevy::asset::AssetPlugin::default());
    app.add_plugins(PlanePlugin);
    app.init_resource::<Probe>();
    // Registered reader-first on purpose: without the set ordering, insertion order is
    // all that breaks the tie, so this would latch the stale value.
    app.add_systems(Update, read_pose.in_set(PlaneRenderPose::Read));
    app.add_systems(Update, write_pose.in_set(PlaneRenderPose::Write));
    app.finish();
    app
}

#[test]
fn readers_observe_the_pose_written_this_frame() {
    let mut app = probe_app();
    app.update();

    let probe = app.world().resource::<Probe>();
    assert_eq!(
        probe.reader_saw,
        Some(1),
        "a PlaneRenderPose::Read system must observe the pose written by \
         PlaneRenderPose::Write this frame (1), not the stale pre-write value (0)"
    );
}

/// The ordering must hold every frame, not just settle after one. A flipping order is
/// precisely what produced the pulse, so pin it across several updates.
#[test]
fn ordering_holds_across_frames() {
    let mut app = probe_app();
    for frame in 1..=8u32 {
        app.update();
        let probe = app.world().resource::<Probe>();
        assert_eq!(
            probe.reader_saw,
            Some(frame),
            "frame {frame}: reader must see this frame's pose, never a stale one"
        );
    }
}
