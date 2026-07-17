use std::sync::Arc;

use bevy::prelude::{Component, Entity, Reflect, Resource};

use crate::plane::FlightState;

#[derive(Component, Clone, Copy, PartialEq, Eq, Hash, Debug, Reflect)]
#[cfg_attr(feature = "net", derive(serde::Serialize, serde::Deserialize))]
pub struct PlaneId(pub u32);

impl PlaneId {
    /// Canonical id for genuinely peerless contexts: unit tests and single-plane
    /// training envs. Runtime allocation starts at 1 so these never collide.
    pub const TEST: PlaneId = PlaneId(0);
}

/// Allocator resource. The first id issued at runtime is `PlaneId(1)`.
#[derive(Resource)]
pub struct NextPlaneId(pub u32);

impl Default for NextPlaneId {
    fn default() -> Self {
        Self(1)
    }
}

/// Return value of `spawn_plane`. Gives callers the stable id immediately so
/// relationship controllers can be constructed at spawn time.
pub struct SpawnedPlane {
    pub entity: Entity,
    pub id: PlaneId,
}

/// One plane's state for a single tick. Owns `FlightState` (small, Clone) to
/// avoid Bevy borrow conflicts.
#[derive(Clone)]
pub struct PlaneSnapshot {
    pub id: PlaneId,
    pub state: FlightState,
}

/// Per-tick context threaded through `FlightController::update`.
///
/// `planes` contains ALL planes (own + peers) for the tick. Simple controllers
/// ignore it; `WingmanController` reads the leader via `find()`.
///
/// **Why a flat slice + linear-scan `find` (not a map).** The snapshot is
/// rebuilt from scratch every tick in `run_flight_controllers`, `N` is a handful
/// of aircraft, and the sole cross-plane lookup (`WingmanController`, once per
/// tick for its leader) is not a hot path. For small `N` a contiguous
/// `Arc<[_]>` scan beats a `HashMap` on cache locality, and a map would have to
/// be re-hashed every tick to save a few comparisons on one lookup — strictly
/// more build cost for no measurable gain. The `Arc<[_]>` is also trivially
/// cheap to share (one `Arc::clone` per controller in phase 2) and `Send +
/// Sync`. If massive multi-agent scenarios (hundreds/thousands of planes, many
/// doing per-tick peer lookups) ever land — deferred, not out of scope, see the
/// multi-agent scope decision in CLAUDE.md — build an `id → index` map once in
/// phase 1 and pass it alongside the slice; `find`/`others` already encapsulate
/// access, so that swap stays local.
#[derive(Clone)]
pub struct ControllerContext {
    pub own_id: PlaneId,
    pub planes: Arc<[PlaneSnapshot]>,
}

impl ControllerContext {
    /// For genuinely peerless callers: unit tests and single-plane training envs.
    ///
    /// Builds a context containing a single snapshot of `own_id` with a default
    /// `FlightState`. Do NOT use as a shortcut in nested controllers that have
    /// real peer context — those must thread the same `ctx` through.
    pub fn empty_for(own_id: PlaneId) -> Self {
        Self {
            own_id,
            planes: Arc::from(vec![PlaneSnapshot {
                id: own_id,
                state: FlightState::default(),
            }]),
        }
    }

    /// Iterate over peers only (excludes own plane).
    pub fn others(&self) -> impl Iterator<Item = &PlaneSnapshot> {
        self.planes.iter().filter(move |s| s.id != self.own_id)
    }

    /// Look up any plane by id, including own. Linear scan — intentional at the
    /// current plane count; see the type-level doc for the rationale and the
    /// map-based path to take if agent counts ever grow large.
    pub fn find(&self, id: PlaneId) -> Option<&PlaneSnapshot> {
        self.planes.iter().find(|s| s.id == id)
    }
}

// ---------------------------------------------------------------------------
// Tests

#[cfg(test)]
mod tests {
    use super::*;

    fn make_snap(id: u32) -> PlaneSnapshot {
        PlaneSnapshot {
            id: PlaneId(id),
            state: FlightState::default(),
        }
    }

    #[test]
    fn empty_for_contains_own_snapshot() {
        let ctx = ControllerContext::empty_for(PlaneId::TEST);
        assert_eq!(ctx.planes.len(), 1);
        assert_eq!(ctx.planes[0].id, PlaneId::TEST);
    }

    #[test]
    fn find_returns_own_plane() {
        let ctx = ControllerContext::empty_for(PlaneId(1));
        assert!(ctx.find(PlaneId(1)).is_some());
    }

    #[test]
    fn others_excludes_own() {
        let snaps: Arc<[PlaneSnapshot]> = Arc::from(vec![make_snap(1), make_snap(2), make_snap(3)]);
        let ctx = ControllerContext {
            own_id: PlaneId(1),
            planes: snaps,
        };
        let others: Vec<_> = ctx.others().collect();
        assert_eq!(others.len(), 2);
        assert!(others.iter().all(|s| s.id != PlaneId(1)));
    }

    #[test]
    fn find_returns_none_for_missing_id() {
        let ctx = ControllerContext::empty_for(PlaneId(1));
        assert!(ctx.find(PlaneId(99)).is_none());
    }

    #[test]
    fn empty_for_others_is_empty() {
        let ctx = ControllerContext::empty_for(PlaneId::TEST);
        assert_eq!(ctx.others().count(), 0);
    }
}
