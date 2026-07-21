//! gm-collision: capsule inventory, contact detection with speculative margins,
//! and tunneling (segment crossing) detection between poses.
//!
//! The contact model is a *ratchet*: existing penetration in a loaded pose is
//! tolerated (grappling positions are authored in tight contact), but a step may
//! never make any contact deeper, and separated capsules may never cross.

use gm_core::{
    capsule_radius, closest_segment_points, Joint, Limb, PlayerId, PlayerJoint, Pose, V3,
};
use serde::Serialize;

/// A collision capsule: a visible limb of one player.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct CapsuleDef {
    pub player: PlayerId,
    pub ends: [Joint; 2],
    pub radius: f64,
}

impl CapsuleDef {
    pub fn a(&self) -> PlayerJoint {
        PlayerJoint { player: self.player, joint: self.ends[0] }
    }

    pub fn b(&self) -> PlayerJoint {
        PlayerJoint { player: self.player, joint: self.ends[1] }
    }

    pub fn shares_joint_with(&self, o: &CapsuleDef) -> bool {
        self.player == o.player
            && (self.ends[0] == o.ends[0]
                || self.ends[0] == o.ends[1]
                || self.ends[1] == o.ends[0]
                || self.ends[1] == o.ends[1])
    }
}

/// Torso capsules: GrappleMap's *visible* limb list has no torso volume, which
/// would let arms pass through chests without contact. These invisible
/// structural limbs (plus cross-braces closing the chest plane so nothing can
/// thread between the outline capsules) are added to the collision inventory
/// with radii derived from their end joints.
const TORSO_CAPSULES: [([Joint; 2], f64); 10] = [
    ([Joint::LeftHip, Joint::Core], 0.09),
    ([Joint::RightHip, Joint::Core], 0.09),
    ([Joint::Core, Joint::LeftShoulder], 0.08),
    ([Joint::Core, Joint::RightShoulder], 0.08),
    ([Joint::LeftShoulder, Joint::Neck], 0.06),
    ([Joint::RightShoulder, Joint::Neck], 0.06),
    ([Joint::LeftHip, Joint::RightHip], 0.09),
    ([Joint::LeftShoulder, Joint::RightShoulder], 0.075),
    ([Joint::LeftHip, Joint::RightShoulder], 0.08),
    ([Joint::RightHip, Joint::LeftShoulder], 0.08),
];

/// Collision capsule inventory: every visible limb of both players, plus the
/// torso capsules.
pub fn capsules() -> Vec<CapsuleDef> {
    let mut out = Vec::with_capacity(2 * 22);
    for player in PlayerId::ALL {
        for limb in gm_core::all_limbs().filter(|l| l.visible) {
            out.push(CapsuleDef { player, ends: limb.ends, radius: capsule_radius(&limb) });
        }
        for (ends, radius) in TORSO_CAPSULES {
            out.push(CapsuleDef { player, ends, radius });
        }
    }
    out
}

/// Joints spanned by the torso's structural capsules. Same-player capsule
/// pairs entirely within this set (e.g. two chest braces) permanently overlap
/// by construction and must not collide with each other.
fn is_torso_joint(j: Joint) -> bool {
    matches!(
        j,
        Joint::LeftHip | Joint::RightHip | Joint::Core | Joint::LeftShoulder | Joint::RightShoulder | Joint::Neck
    )
}

/// Pairs of capsule indices that are allowed to collide (precomputed once).
/// Excluded: same-player capsules sharing a joint (always overlap) and
/// same-player pairs entirely inside the torso frame (structural overlap).
pub fn collidable_pairs(caps: &[CapsuleDef]) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    for i in 0..caps.len() {
        for j in (i + 1)..caps.len() {
            let (a, b) = (&caps[i], &caps[j]);
            if a.shares_joint_with(b) {
                continue;
            }
            if a.player == b.player
                && a.ends.iter().chain(b.ends.iter()).all(|&e| is_torso_joint(e))
            {
                continue;
            }
            pairs.push((i, j));
        }
    }
    pairs
}

/// A detected (or speculative) contact between two capsules.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Contact {
    pub cap_a: usize,
    pub cap_b: usize,
    /// Index into the collidable-pairs list (for persistent floor lookup).
    pub pair_idx: usize,
    /// Closest-point parameters along each capsule's segment.
    pub s: f64,
    pub t: f64,
    /// Unit normal pointing from capsule B's closest point toward capsule A's.
    pub normal: V3,
    /// Surface clearance (negative = penetrating).
    pub clearance: f64,
    /// Clearance floor this contact must respect: min(entry clearance, 0).
    /// This implements the ratchet: never deeper than at step entry.
    pub min_clearance: f64,
}

fn contact_normal(ca: V3, cb: V3, seg_b: (V3, V3)) -> V3 {
    if let Some(n) = (ca - cb).normalized(1e-9) {
        return n;
    }
    // Degenerate: closest points coincide. Fall back to something deterministic
    // perpendicular to capsule B's axis.
    let axis = (seg_b.1 - seg_b.0).normalized_or_zero();
    axis.cross(V3::Y)
        .normalized(1e-9)
        .unwrap_or_else(|| gm_core::v3(1.0, 0.0, 0.0))
}

/// Find all contacts with clearance below `margin` (speculative margin), against
/// the entry pose. `entry` provides the ratchet floor: penetration present at
/// entry is allowed but must not deepen.
pub fn find_contacts(
    entry: &Pose,
    caps: &[CapsuleDef],
    pairs: &[(usize, usize)],
    margin: f64,
) -> Vec<Contact> {
    let mut out = Vec::new();
    for (pair_idx, &(i, j)) in pairs.iter().enumerate() {
        let (a, b) = (&caps[i], &caps[j]);
        let (pa, qa) = (entry[a.a()], entry[a.b()]);
        let (pb, qb) = (entry[b.a()], entry[b.b()]);
        let (ca, cb, s, t) = closest_segment_points(pa, qa, pb, qb);
        let clearance = ca.distance(cb) - a.radius - b.radius;
        if clearance < margin {
            out.push(Contact {
                cap_a: i,
                cap_b: j,
                pair_idx,
                s,
                t,
                normal: contact_normal(ca, cb, (pb, qb)),
                clearance,
                min_clearance: clearance.min(0.0),
            });
        }
    }
    out
}

/// Re-evaluate a contact's clearance and closest-point geometry in a new pose.
pub fn eval_contact(pose: &Pose, caps: &[CapsuleDef], c: &Contact) -> (V3, V3, f64, f64, f64) {
    let a = &caps[c.cap_a];
    let b = &caps[c.cap_b];
    let (ca, cb, s, t) =
        closest_segment_points(pose[a.a()], pose[a.b()], pose[b.a()], pose[b.b()]);
    let clearance = ca.distance(cb) - a.radius - b.radius;
    (ca, cb, s, t, clearance)
}

/// Minimum surface clearance over all collidable pairs (negative = deepest penetration).
pub fn min_clearance(pose: &Pose, caps: &[CapsuleDef], pairs: &[(usize, usize)]) -> f64 {
    let mut min_seen = f64::INFINITY;
    for &(i, j) in pairs {
        let (a, b) = (&caps[i], &caps[j]);
        let (ca, cb, _, _) =
            closest_segment_points(pose[a.a()], pose[a.b()], pose[b.a()], pose[b.b()]);
        min_seen = min_seen.min(ca.distance(cb) - a.radius - b.radius);
    }
    if min_seen.is_finite() {
        min_seen
    } else {
        0.0
    }
}

/// Per-pair penetration report (used by the validator).
pub fn penetrations(pose: &Pose, caps: &[CapsuleDef], pairs: &[(usize, usize)]) -> Vec<(usize, usize, f64)> {
    let mut out = Vec::new();
    for &(i, j) in pairs {
        let (a, b) = (&caps[i], &caps[j]);
        let (ca, cb, _, _) =
            closest_segment_points(pose[a.a()], pose[a.b()], pose[b.a()], pose[b.b()]);
        let clearance = ca.distance(cb) - a.radius - b.radius;
        if clearance < 0.0 {
            out.push((i, j, -clearance));
        }
    }
    out
}

/// Detects whether the segment pair (i, j) crossed (passed through each other)
/// between `before` and `after`.
///
/// A sign flip of the scalar triple product (mutual orientation of the two
/// segment axes) is necessary but not sufficient: segments sliding in tight
/// contact flip orientation constantly while their surfaces stay apart. A true
/// pass-through additionally has the segment *cores* nearly coincident at the
/// crossing instant, so we interpolate the motion to the flip time and measure
/// the actual core distance there.
pub fn segments_crossed(
    before: &Pose,
    after: &Pose,
    caps: &[CapsuleDef],
    i: usize,
    j: usize,
    _margin: f64,
) -> bool {
    let a = &caps[i];
    let b = &caps[j];
    let orient = |pose: &Pose| -> f64 {
        let da = pose[a.b()] - pose[a.a()];
        let db = pose[b.b()] - pose[b.a()];
        let r = pose[b.a()] - pose[a.a()];
        da.cross(db).dot(r)
    };
    let o0 = orient(before);
    let o1 = orient(after);
    if o0 == 0.0 || o1 == 0.0 || (o0 > 0.0) == (o1 > 0.0) {
        return false;
    }

    // Estimate the flip instant and interpolate all four endpoints there.
    let tc = o0 / (o0 - o1);
    let lerp = |pj: PlayerJoint| before[pj].lerp(after[pj], tc);
    let (ca, cb, _, _) = closest_segment_points(lerp(a.a()), lerp(a.b()), lerp(b.a()), lerp(b.b()));
    let core_distance = ca.distance(cb);

    // Pass-through: cores meet well inside the combined capsule radii. Contact
    // sliding keeps cores near radius-sum separation even while axes flip.
    core_distance < 0.5 * (a.radius + b.radius)
}

/// Convenience: the limb definition backing a capsule (for radius lookups etc).
pub fn capsule_limb(cap: &CapsuleDef) -> Option<Limb> {
    gm_core::all_limbs().find(|l| l.ends == cap.ends)
}

#[cfg(test)]
mod tests {
    use super::*;
    use gm_core::{v3, Joint::*, P0, P1};

    fn caps_and_pairs() -> (Vec<CapsuleDef>, Vec<(usize, usize)>) {
        let caps = capsules();
        let pairs = collidable_pairs(&caps);
        (caps, pairs)
    }

    #[test]
    fn inventory_has_both_players_and_no_adjacent_pairs() {
        let (caps, pairs) = caps_and_pairs();
        assert_eq!(caps.len() % 2, 0);
        assert!(caps.len() >= 30, "expected >=15 visible limbs per player");
        for &(i, j) in &pairs {
            assert!(!caps[i].shares_joint_with(&caps[j]));
        }
    }

    #[test]
    fn overlapping_forearms_produce_contact() {
        let (caps, pairs) = caps_and_pairs();
        let mut pose = Pose::default();
        // Spread everything far apart per player to avoid noise, then overlap
        // p0 left forearm with p1 left forearm.
        for pj in PlayerJoint::all() {
            let spread = pj.flat() as f64;
            pose[pj] = v3(spread * 0.5, 0.5, if pj.player == P0 { -30.0 } else { 30.0 });
        }
        pose.set(P0, LeftElbow, v3(0.0, 1.0, 0.0));
        pose.set(P0, LeftWrist, v3(0.3, 1.0, 0.0));
        pose.set(P1, LeftElbow, v3(0.15, 1.02, -0.1));
        pose.set(P1, LeftWrist, v3(0.15, 1.02, 0.1));
        let contacts = find_contacts(&pose, &caps, &pairs, 0.0);
        assert!(
            contacts.iter().any(|c| {
                let (a, b) = (&caps[c.cap_a], &caps[c.cap_b]);
                a.player != b.player
                    && a.ends == [LeftElbow, LeftWrist]
                    && b.ends == [LeftElbow, LeftWrist]
            }),
            "expected forearm-forearm contact"
        );
    }

    #[test]
    fn crossing_detection_fires_on_pass_through() {
        let (caps, _) = caps_and_pairs();
        // p0 left forearm horizontal along x; p1 left forearm along z, above it,
        // then moved below it: they crossed.
        let mut before = Pose::default();
        for pj in PlayerJoint::all() {
            before[pj] = v3(pj.flat() as f64 * 2.0 + 10.0, 5.0, 40.0);
        }
        before.set(P0, LeftElbow, v3(-0.3, 1.0, 0.0));
        before.set(P0, LeftWrist, v3(0.3, 1.0, 0.0));
        before.set(P1, LeftElbow, v3(0.0, 1.05, -0.3));
        before.set(P1, LeftWrist, v3(0.0, 1.05, 0.3));
        let mut after = before;
        after.set(P1, LeftElbow, v3(0.0, 0.95, -0.3));
        after.set(P1, LeftWrist, v3(0.0, 0.95, 0.3));

        let i = caps
            .iter()
            .position(|c| c.player == P0 && c.ends == [LeftElbow, LeftWrist])
            .unwrap();
        let j = caps
            .iter()
            .position(|c| c.player == P1 && c.ends == [LeftElbow, LeftWrist])
            .unwrap();
        assert!(segments_crossed(&before, &after, &caps, i, j, 0.1));

        // Moving further above does not cross.
        let mut after_up = before;
        after_up.set(P1, LeftElbow, v3(0.0, 1.5, -0.3));
        after_up.set(P1, LeftWrist, v3(0.0, 1.5, 0.3));
        assert!(!segments_crossed(&before, &after_up, &caps, i, j, 0.1));
    }
}
