//! gm-validate: the invariant checker. Runs in-engine every frame (cheap) and
//! in every test. A pose is *valid* when:
//!
//!   - all coordinates are finite
//!   - every joint is above the floor and inside the arena
//!   - bone lengths match their rest lengths within tolerance
//!   - every hinge respects its calibrated minimum angle
//!   - every swing cone holds
//!   - no capsule pair penetrates deeper than its allowed floor
//!
//! Step-to-step validity additionally requires bounded joint velocity and no
//! segment crossings (checked by the solver's watchdog, re-checked here).

use gm_collision::CapsuleDef;
use gm_core::{
    anatomy::cone_angle,
    angle_at,
    Bone, PlayerId, PlayerJoint, Pose, HINGES, SWING_CONES,
};
use serde::Serialize;

#[derive(Debug, Clone, Serialize)]
pub struct Violation {
    pub kind: ViolationKind,
    pub detail: String,
    /// Magnitude of the violation (meters or radians depending on kind).
    pub amount: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum ViolationKind {
    NonFinite,
    BelowFloor,
    OutOfArena,
    BoneLength,
    HingeAngle,
    SwingCone,
    Penetration,
    ExcessiveVelocity,
    SegmentCrossing,
}

#[derive(Debug, Clone, Serialize)]
pub struct ValidationReport {
    pub violations: Vec<Violation>,
    pub min_clearance: f64,
    pub max_bone_error: f64,
}

impl ValidationReport {
    pub fn is_valid(&self) -> bool {
        self.violations.is_empty()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Tolerances {
    /// Allowed relative bone length error vs rest length.
    pub bone_rel: f64,
    /// Allowed hinge angle shortfall below the calibrated minimum (radians).
    pub hinge_slack: f64,
    /// Allowed swing-cone overshoot (radians).
    pub cone_slack: f64,
    /// Allowed penetration depth beyond the per-pair floor (meters).
    pub penetration_slack: f64,
    /// Floor slack (meters): joints may sink this far below their radius.
    pub floor_slack: f64,
    /// Max joint speed (m/s) considered physical.
    pub max_speed: f64,
}

impl Default for Tolerances {
    fn default() -> Self {
        Tolerances {
            // 3% relative: on the shortest bone (8 cm) this is 2.4 mm, below
            // visual perception; XPBD leaves residuals of this order when
            // contacts, limits, and adversarial effectors all fight one bone.
            bone_rel: 0.03,
            hinge_slack: 0.02,
            cone_slack: 0.02,
            // 12 mm: capsule radii already model an uncompressed surface, and
            // when adversarial effectors squeeze two limbs together the
            // contact pass (which yields the last word to bone rigidity)
            // leaves a residual of this order. Flesh compresses more.
            penetration_slack: 0.012,
            // 4 mm: muscle tone presses planted feet into the mat (its net
            // pull is momentum-conserving, so lifting the torso pushes the
            // feet down); the floor clamp and bone rigidity split that
            // pressure into a millimeter-scale equilibrium sink. Real mats
            // compress further than this under a heel.
            floor_slack: 0.004,
            // 20 m/s = 0.33 m per 60 Hz frame: rules out teleportation while
            // accepting legitimate constraint-projection snaps (a blocked limb
            // resolving a 10 cm violation in one frame moves at 6 m/s; under
            // adversarial input, tone + anatomy limits whip the lightest
            // joints - fingertips, toes - to roughly triple that).
            max_speed: 20.0,
        }
    }
}

/// Validate a single pose against rest bones and per-pair penetration floors.
///
/// `penetration_floors` maps collidable-pair index -> allowed penetration depth
/// (>= 0). Pass an empty slice to require zero penetration everywhere; pass
/// floors captured from a loaded database pose to accept its authored contact
/// tightness while rejecting anything deeper.
pub fn validate_pose(
    pose: &Pose,
    bones: &[Bone],
    caps: &[CapsuleDef],
    pairs: &[(usize, usize)],
    penetration_floors: &[f64],
    tol: &Tolerances,
) -> ValidationReport {
    let mut violations = Vec::new();

    // Finiteness and bounds.
    for pj in PlayerJoint::all() {
        let p = pose[pj];
        if !p.is_finite() {
            violations.push(Violation {
                kind: ViolationKind::NonFinite,
                detail: format!("{:?} {:?}", pj.player, pj.joint.name()),
                amount: f64::INFINITY,
            });
            continue;
        }
        let sink = pj.joint.radius() - p.y;
        if sink > tol.floor_slack {
            violations.push(Violation {
                kind: ViolationKind::BelowFloor,
                detail: format!("{:?} {:?}", pj.player, pj.joint.name()),
                amount: sink,
            });
        }
        let out = (p.x.abs() - 2.0).max(p.z.abs() - 2.0);
        if out > 1e-9 {
            violations.push(Violation {
                kind: ViolationKind::OutOfArena,
                detail: format!("{:?} {:?}", pj.player, pj.joint.name()),
                amount: out,
            });
        }
    }

    // Bones. Tolerance is relative with a small absolute floor: constraint
    // compromises (a heel squeezed between the mat and its ankle bone) leave
    // millimeter-scale absolute residuals regardless of bone length, which
    // on the shortest bones would otherwise dominate the relative measure.
    let mut max_bone_error: f64 = 0.0;
    for bone in bones {
        let d = pose[bone.a()].distance(pose[bone.b()]);
        let rel = ((d - bone.length) / bone.length.max(1e-9)).abs();
        let abs = (d - bone.length).abs();
        max_bone_error = max_bone_error.max(rel);
        if rel > tol.bone_rel && abs > 0.004 {
            violations.push(Violation {
                kind: ViolationKind::BoneLength,
                detail: format!("{:?} {:?}-{:?}", bone.player, bone.ends[0].name(), bone.ends[1].name()),
                amount: rel,
            });
        }
    }

    // Hinges.
    for player in PlayerId::ALL {
        for h in HINGES.iter() {
            let ang = angle_at(
                pose.get(player, h.root),
                pose.get(player, h.mid),
                pose.get(player, h.tip),
            );
            if h.min - ang > tol.hinge_slack {
                violations.push(Violation {
                    kind: ViolationKind::HingeAngle,
                    detail: format!("{:?} {}", player, h.id),
                    amount: h.min - ang,
                });
            }
        }
        for c in SWING_CONES.iter() {
            let ang = cone_angle(pose, player, c);
            if ang - c.half_angle > tol.cone_slack {
                violations.push(Violation {
                    kind: ViolationKind::SwingCone,
                    detail: format!("{:?} {}", player, c.id),
                    amount: ang - c.half_angle,
                });
            }
        }
    }

    // Penetration.
    let mut min_clearance = f64::INFINITY;
    for (pair_idx, &(i, j)) in pairs.iter().enumerate() {
        let (a, b) = (&caps[i], &caps[j]);
        let (ca, cb, _, _) = gm_core::closest_segment_points(
            pose[a.a()],
            pose[a.b()],
            pose[b.a()],
            pose[b.b()],
        );
        let clearance = ca.distance(cb) - a.radius - b.radius;
        min_clearance = min_clearance.min(clearance);
        let floor = penetration_floors.get(pair_idx).copied().unwrap_or(0.0);
        if -clearance > floor + tol.penetration_slack {
            violations.push(Violation {
                kind: ViolationKind::Penetration,
                detail: format!(
                    "{:?} {:?}-{:?} vs {:?} {:?}-{:?}",
                    a.player, a.ends[0].name(), a.ends[1].name(),
                    b.player, b.ends[0].name(), b.ends[1].name()
                ),
                amount: -clearance - floor,
            });
        }
    }

    ValidationReport {
        violations,
        min_clearance: if min_clearance.is_finite() { min_clearance } else { 0.0 },
        max_bone_error,
    }
}

/// Penetration floors for a pose: existing penetration depth per collidable pair.
/// Used when loading database poses that are authored in tight contact.
pub fn penetration_floors(pose: &Pose, caps: &[CapsuleDef], pairs: &[(usize, usize)]) -> Vec<f64> {
    pairs
        .iter()
        .map(|&(i, j)| {
            let (a, b) = (&caps[i], &caps[j]);
            let (ca, cb, _, _) = gm_core::closest_segment_points(
                pose[a.a()],
                pose[a.b()],
                pose[b.a()],
                pose[b.b()],
            );
            (-(ca.distance(cb) - a.radius - b.radius)).max(0.0)
        })
        .collect()
}

/// Validate a transition between two poses (one solver step apart).
pub fn validate_step(
    before: &Pose,
    after: &Pose,
    dt: f64,
    caps: &[CapsuleDef],
    pairs: &[(usize, usize)],
    tol: &Tolerances,
) -> Vec<Violation> {
    let mut violations = Vec::new();
    for pj in PlayerJoint::all() {
        let speed = before[pj].distance(after[pj]) / dt.max(1e-9);
        if speed > tol.max_speed {
            violations.push(Violation {
                kind: ViolationKind::ExcessiveVelocity,
                detail: format!("{:?} {:?}", pj.player, pj.joint.name()),
                amount: speed,
            });
        }
    }
    for &(i, j) in pairs {
        if gm_collision::segments_crossed(before, after, caps, i, j, 0.02) {
            violations.push(Violation {
                kind: ViolationKind::SegmentCrossing,
                detail: format!(
                    "{:?} {:?}-{:?} x {:?} {:?}-{:?}",
                    caps[i].player, caps[i].ends[0].name(), caps[i].ends[1].name(),
                    caps[j].player, caps[j].ends[0].name(), caps[j].ends[1].name()
                ),
                amount: 1.0,
            });
        }
    }
    violations
}
