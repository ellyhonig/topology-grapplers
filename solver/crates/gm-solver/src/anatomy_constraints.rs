//! Positional projections for anatomical limits.
//!
//! Hinge minimum angles are enforced as root-tip distance lower bounds (exact
//! given bone lengths, unconditionally stable). Hyperextension - bending past
//! straight into the wrong side - cannot be expressed as a 3-point angle limit
//! (particle angles never exceed pi), so it is enforced with *bend-direction
//! hysteresis*: while a hinge is clearly bent, its bend plane is remembered in
//! torso-local coordinates; when it approaches straight, the mid joint may not
//! cross to the opposite side of that remembered plane.

use gm_core::{
    angle_at, torso_frame, Hinge, PlayerId, PlayerJoint, Pose, V3, HINGES, SWING_CONES,
};

use crate::state::SolverState;

/// Angle below which (measured from straight) a hinge's bend plane is
/// considered well-defined and worth remembering.
const BEND_MEMORY_ANGLE: f64 = 0.20;

/// Bend component (mid relative to the root-tip axis), or None if degenerate.
fn bend_vector(pose: &Pose, player: PlayerId, h: &Hinge) -> Option<(V3, V3)> {
    let root = pose.get(player, h.root);
    let mid = pose.get(player, h.mid);
    let tip = pose.get(player, h.tip);
    let axis = (tip - root).normalized(1e-9)?;
    let bend = (mid - root).perp_to(axis);
    Some((bend, axis))
}

/// While nearly straight, the remembered direction drifts slowly toward the
/// current (tiny) bend at this rate per refresh. The bend plane is stored in
/// torso-local coordinates, but a real limb's flexion plane rotates with the
/// limb (shoulder rotation) - without this drift, an arm raised overhead
/// fights its own stale memory forever. The rate is slow enough (~a second
/// to reorient) that a fast snap through straight is still blocked.
const BEND_MEMORY_DRIFT: f64 = 0.10;

/// Update remembered bend directions: snap wherever the bend plane is
/// well-defined (clearly bent), drift slowly while nearly straight.
pub fn refresh_bend_memory(state: &mut SolverState) {
    for player in PlayerId::ALL {
        let frame = torso_frame(&state.pose, player);
        for (hi, h) in HINGES.iter().enumerate() {
            let ang = angle_at(
                state.pose.get(player, h.root),
                state.pose.get(player, h.mid),
                state.pose.get(player, h.tip),
            );
            let Some((bend, _)) = bend_vector(&state.pose, player, h) else {
                continue;
            };
            let Some(dir) = bend.normalized(1e-9) else {
                continue;
            };
            let slot = &mut state.bend_ref_local[player.index()][hi];
            if ang < std::f64::consts::PI - BEND_MEMORY_ANGLE {
                *slot = Some(frame.to_local(dir));
            } else if let Some(stored) = *slot {
                let blended = stored * (1.0 - BEND_MEMORY_DRIFT)
                    + frame.to_local(dir) * BEND_MEMORY_DRIFT;
                if let Some(n) = blended.normalized(1e-9) {
                    *slot = Some(n);
                }
            }
        }
    }
}

/// Project the hinge minimum-angle limits: enforce |root - tip| >= d_min where
/// d_min follows from the *current* bone lengths via the law of cosines.
/// Moves root and tip apart symmetrically (mass-weighted, both weight 1).
pub fn project_hinge_min_angles(pose: &mut Pose, inv_mass: &dyn Fn(PlayerJoint) -> f64) {
    for player in PlayerId::ALL {
        for h in HINGES.iter() {
            let root_pj = PlayerJoint { player, joint: h.root };
            let tip_pj = PlayerJoint { player, joint: h.tip };
            let root = pose[root_pj];
            let mid = pose.get(player, h.mid);
            let tip = pose[tip_pj];
            let l1 = root.distance(mid);
            let l2 = mid.distance(tip);
            if l1 < 1e-9 || l2 < 1e-9 {
                continue;
            }
            let d_min = (l1 * l1 + l2 * l2 - 2.0 * l1 * l2 * h.min.cos()).max(0.0).sqrt();
            let delta = tip - root;
            let d = delta.length();
            if d >= d_min || d < 1e-9 {
                continue;
            }
            let dir = delta / d;
            let corr = d_min - d;
            let w_root = inv_mass(root_pj);
            let w_tip = inv_mass(tip_pj);
            let w_sum = w_root + w_tip;
            if w_sum < 1e-12 {
                continue;
            }
            pose[root_pj] -= dir * (corr * w_root / w_sum);
            pose[tip_pj] += dir * (corr * w_tip / w_sum);
        }
    }
}

/// Angle from straight below which hyperextension protection engages.
const STRAIGHT_GUARD_ANGLE: f64 = 0.35;
/// Minimum bend the guard maintains on the remembered side while engaged.
const GUARD_MIN_BEND: f64 = 0.0;

/// Prevent bending past straight to the wrong side: when a hinge is nearly
/// straight and has remembered a bend direction, the mid joint's bend component
/// along that direction must stay >= 0. Projects only the mid joint - a small,
/// local, stable correction.
pub fn project_hyperextension_guards(state: &mut SolverState, inv_mass: &dyn Fn(PlayerJoint) -> f64) {
    for player in PlayerId::ALL {
        let frame = torso_frame(&state.pose, player);
        for (hi, h) in HINGES.iter().enumerate() {
            let Some(ref_local) = state.bend_ref_local[player.index()][hi] else {
                continue;
            };
            let mid_pj = PlayerJoint { player, joint: h.mid };
            if inv_mass(mid_pj) < 1e-12 {
                continue;
            }
            let ang = angle_at(
                state.pose.get(player, h.root),
                state.pose.get(player, h.mid),
                state.pose.get(player, h.tip),
            );
            if ang < std::f64::consts::PI - STRAIGHT_GUARD_ANGLE {
                continue; // clearly bent: no hyperextension risk
            }
            let Some((bend, axis)) = bend_vector(&state.pose, player, h) else {
                continue;
            };
            let ref_world = frame.to_world(ref_local).perp_to(axis);
            let Some(ref_dir) = ref_world.normalized(1e-9) else {
                continue;
            };
            let side = bend.dot(ref_dir);
            if side >= GUARD_MIN_BEND {
                continue;
            }
            // Mid joint slid to the wrong side of straight: pull it back to the
            // remembered side (exactly onto the straight axis plane boundary).
            state.pose[mid_pj] += ref_dir * (GUARD_MIN_BEND - side);
        }
    }
}

/// Project swing-cone limits (neck orientation, spine coherence). Moves the limb
/// end back onto the cone surface; downstream joints are carried rigidly.
pub fn project_swing_cones(pose: &mut Pose, inv_mass: &dyn Fn(PlayerJoint) -> f64) {
    for player in PlayerId::ALL {
        for cone in SWING_CONES.iter() {
            let limb_pj = PlayerJoint { player, joint: cone.limb };
            if inv_mass(limb_pj) < 1e-12 {
                continue;
            }
            let apex = gm_core::anatomy::cone_apex(pose, player, cone);
            let axis = gm_core::anatomy::cone_world_ref(pose, player, cone);
            let dir = pose[limb_pj] - apex;
            let len = dir.length();
            if len < 1e-9 || axis.length() < 0.5 {
                continue;
            }
            let d = dir / len;
            let cos_limit = cone.half_angle.cos();
            let along = d.dot(axis);
            if along >= cos_limit {
                continue;
            }
            // Rotate direction back to the cone boundary within the (axis, d) plane.
            let side = d.perp_to(axis).normalized(1e-9).unwrap_or_else(|| {
                axis.cross(V3::Y).normalized(1e-9).unwrap_or(gm_core::v3(1.0, 0.0, 0.0))
            });
            let clamped = (axis * cos_limit + side * cone.half_angle.sin()).normalized_or_zero();
            let new_pos = apex + clamped * len;
            let carry = new_pos - pose[limb_pj];
            pose[limb_pj] = new_pos;
            for &j in cone.downstream {
                let pj = PlayerJoint { player, joint: j };
                if inv_mass(pj) > 1e-12 {
                    pose[pj] += carry;
                }
            }
        }
    }
}

/// Worst hinge violation in radians below min (for diagnostics/validation).
pub fn max_hinge_violation(pose: &Pose) -> f64 {
    let mut worst: f64 = 0.0;
    for player in PlayerId::ALL {
        for h in HINGES.iter() {
            let ang = angle_at(
                pose.get(player, h.root),
                pose.get(player, h.mid),
                pose.get(player, h.tip),
            );
            worst = worst.max(h.min - ang);
        }
    }
    worst
}
