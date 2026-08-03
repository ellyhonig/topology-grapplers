//! The XPBD solve loop.
//!
//! Every rule is a positional constraint projected in a substepped Gauss-Seidel
//! loop, in a fixed deterministic order:
//!
//!   1. effectors (compliant - input can only *suggest* motion)
//!   2. muscle tone (compliant shape matching - holds the pose against gravity)
//!   3. bone distance constraints (hard)
//!   4. hinge minimum angles (hard)
//!   5. hyperextension guards (hard, hysteresis-based)
//!   6. rest-relative ankle orientation (soft) + swing/twist envelope (hard)
//!   7. swing cones (hard)
//!   8. capsule contacts with ratchet floors + friction (hard)
//!   9. floor and arena bounds (hard)
//!
//! After the whole step, a safety watchdog checks positional and anatomical
//! bounds, transition speed, segment crossings, and writhe jumps. A dirty step
//! is re-solved from the entry state with doubled substeps; if it remains dirty
//! at the retry limit, the step is rejected (state unchanged).

use gm_collision::{
    capsules, collidable_pairs, find_contacts, segments_crossed, CapsuleDef, Contact,
};
use gm_core::{capture_bones, v3, Bone, Joint, PlayerId, PlayerJoint, Pose, SWING_CONES, V3};
use serde::Serialize;

use crate::anatomy_constraints::{
    max_hinge_violation, project_hinge_min_angles, project_hyperextension_guards,
    project_swing_cones, refresh_bend_memory,
};
use crate::ankle_constraints::{
    ankle_angles, capture_ankle_references, project_hard_ankle_envelopes,
    project_soft_ankle_orientations, AnkleReferences, LegSide,
};
use crate::config::SolverConfig;
use crate::effector::Effector;
use crate::state::SolverState;

/// Joints that move together when a limb is driven. These groups define grip
/// ownership for every limb, and the binary relaxation policy retained for
/// arms, neck, and core. Legs use the graded policy in `tone_scales`: a foot
/// drive must not turn the hip-knee-ankle chain into a passive rope.
const LIMB_GROUPS: [(&[Joint], Option<Joint>); 6] = [
    (
        &[
            Joint::LeftElbow,
            Joint::LeftWrist,
            Joint::LeftHand,
            Joint::LeftFingers,
        ],
        Some(Joint::LeftShoulder),
    ),
    (
        &[
            Joint::RightElbow,
            Joint::RightWrist,
            Joint::RightHand,
            Joint::RightFingers,
        ],
        Some(Joint::RightShoulder),
    ),
    (
        &[
            Joint::LeftKnee,
            Joint::LeftAnkle,
            Joint::LeftHeel,
            Joint::LeftToe,
        ],
        Some(Joint::LeftHip),
    ),
    (
        &[
            Joint::RightKnee,
            Joint::RightAnkle,
            Joint::RightHeel,
            Joint::RightToe,
        ],
        Some(Joint::RightHip),
    ),
    (&[Joint::Neck, Joint::Head], None),
    (&[Joint::Core], None),
];

/// How relaxed a driven arm/neck/core group's tone is (fraction of normal tone
/// kept). Lower limbs deliberately use the non-binary scales below.
const DRIVEN_TONE_SCALE: f64 = 0.0;

/// Residual tone on a driven arm's root joint (shoulder).
const DRIVEN_ROOT_TONE_SCALE: f64 = 0.3;

/// Tone retained by the untargeted points of a driven foot frame. The exact
/// effector joint is always relaxed to zero below, so input still wins locally.
const DRIVEN_FOOT_TONE_SCALE: f64 = 0.1;

/// Tone retained by the knee while any point of its lower limb is driven.
const DRIVEN_KNEE_TONE_SCALE: f64 = 0.4;

/// Tone retained by the hip while its lower limb is driven. The increasing
/// foot -> knee -> hip gradient keeps proximal pose retention from dropping to
/// zero merely because a distal tracker engaged.
const DRIVEN_HIP_TONE_SCALE: f64 = 0.7;

/// Physical admissibility widths shared with validation.  These are not
/// numerical fudge factors: the collision model treats the joint/capsule radii
/// as uncompressed surfaces, while the rendered mat and flesh are compliant.
const MAT_COMPRESSION_ALLOWANCE: f64 = 0.004;
const FLESH_COMPRESSION_ALLOWANCE: f64 = 0.012;
const BONE_RELATIVE_ALLOWANCE: f64 = 0.03;
const BONE_ABSOLUTE_ALLOWANCE: f64 = 0.004;
const HINGE_ANGLE_ALLOWANCE: f64 = 0.02;
const SWING_CONE_ALLOWANCE: f64 = 0.02;
const ANKLE_ANGLE_ALLOWANCE: f64 = 0.002;
const MAX_ACCEPTED_STEP_SPEED: f64 = 20.0;

/// hip, knee, ankle, heel, toe for each lower limb.
const LEG_GROUPS: [[Joint; 5]; 2] = [
    [
        Joint::LeftHip,
        Joint::LeftKnee,
        Joint::LeftAnkle,
        Joint::LeftHeel,
        Joint::LeftToe,
    ],
    [
        Joint::RightHip,
        Joint::RightKnee,
        Joint::RightAnkle,
        Joint::RightHeel,
        Joint::RightToe,
    ],
];

/// A grip: a hand or hooking foot tethered to another capsule, captured from
/// the loaded pose. Grappling positions are authored with grips (collar ties,
/// clasped hands, butterfly hooks) that contacts alone cannot express -
/// contacts only push apart, while a grip also refuses to let go.
/// Modeled as a *tether to the capsule's segment*: the gripping joint may
/// slide along the limb and approach it freely (contacts handle compression)
/// but may not move farther from the segment than at load. Tethering to the
/// segment rather than a fixed anchor point matters: a fixed anchor fights
/// tangential sliding and over-constrains entangled poses until they shake
/// apart.
#[derive(Debug, Clone, Copy)]
struct Grip {
    joint: PlayerJoint,
    /// Index of the gripped capsule.
    cap: usize,
    /// Captured joint-to-segment distance (the tether length).
    len: f64,
    /// Runtime grips are explicitly held by controller input. Unlike authored
    /// pose grips, an effector on the gripping hand must not suspend them.
    runtime: bool,
    wrist: Option<PlayerJoint>,
    /// The finger centroid is a second contact point for live hand grips. It
    /// lets the solver form an actual hook around a limb instead of reducing a
    /// hand to one tethered particle.
    finger: Option<PlayerJoint>,
    finger_len: f64,
    /// Solver-selected maximum-writhe continuation around the target capsule.
    hand_wrap: f64,
    target_wrap: f64,
    selected_writhe: f64,
    alternate_writhe: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct GripCandidate {
    pub capsule: usize,
    pub player: PlayerId,
    pub ends: [Joint; 2],
    pub closest: V3,
    pub surface_gap: f64,
    pub palm_alignment: f64,
    pub wrap_alignment: f64,
    pub score: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct RuntimeGripState {
    pub active: bool,
    pub broken: bool,
    pub capsule: usize,
    pub player: PlayerId,
    pub ends: [Joint; 2],
    pub strain: f64,
    pub release: f64,
    pub wrap: f64,
    pub contact: f64,
    pub coverage: f64,
    pub strength: f64,
    pub direction: f64,
    pub selected_writhe: f64,
    pub alternate_writhe: f64,
}

#[derive(Debug, Clone, Copy)]
struct RuntimeWrapPlan {
    hand_wrap: f64,
    finger_wrap: f64,
    selected_writhe: f64,
    alternate_writhe: f64,
}

/// Joints that can grip: hands grasp, feet hook.
const GRIP_JOINTS: [Joint; 8] = [
    Joint::LeftHand,
    Joint::RightHand,
    Joint::LeftFingers,
    Joint::RightFingers,
    Joint::LeftToe,
    Joint::RightToe,
    Joint::LeftHeel,
    Joint::RightHeel,
];

/// A gripping joint in contact this close to a capsule surface (meters) at
/// load is holding it on purpose. Generous on purpose: authored grips are in
/// tight contact, but hooks (a foot behind a knee, an instep in a hip crease)
/// are drawn with a few centimeters of slack, and a false-positive grip is
/// harmless (a tether at the captured distance changes nothing until the
/// joints try to separate, which gripped joints would not).
const GRIP_CONTACT: f64 = 0.04;

/// Slack added to the captured tether length before a grip engages. In the
/// authored pose the gripping joint typically also sits exactly at a contact
/// ratchet floor; with a zero-slack tether both hard constraints are active
/// at once and Gauss-Seidel ping-pongs between them, pumping energy into
/// tiny-mass hand/foot joints. The slack keeps the tether dormant until the
/// joint has genuinely tried to let go.
const GRIP_SLACK: f64 = 0.03;

/// Controller-held runtime grips should engage almost immediately. Authored
/// pose grips keep the larger slack above to avoid competing contact/tether
/// constraints, but a live grab needs to feel attached to the palm.
const RUNTIME_GRIP_SLACK: f64 = 0.005;

/// Live acquisition is a short constraint-guided curl, not an instantaneous
/// teleport. The hand/finger pair closes toward the capsule surface while
/// contact, bones, anatomy, and the topology watchdog remain authoritative.
const RUNTIME_WRAP_RATE: f64 = 8.0;
const RUNTIME_WRAP_SPEED: f64 = 1.25;
/// A wrap point becomes a permanent suction-cup-style anchor once it is this
/// close to the target surface. The wrap target itself carries 5 mm of slack,
/// so 8 mm reliably captures contact without latching a merely nearby point.
const RUNTIME_STICKY_CAPTURE_GAP: f64 = 0.008;
const RUNTIME_WRAP_ABORT_GAP: f64 = 0.10;
const RUNTIME_WRAP_MIN_STEP_ANGLE: f64 = 28.0 * std::f64::consts::PI / 180.0;
const RUNTIME_WRAP_MAX_STEP_ANGLE: f64 = 72.0 * std::f64::consts::PI / 180.0;

/// Grips have finite strength against *deliberate* pulls. While the gripped
/// limb is user-driven, each substep's attempted stretch beyond the tether
/// length (meters) is added to a leaky per-grip strain accumulator; past this
/// threshold the grip starts to fail (pay out). A sustained effector yank
/// plateaus above 0.4 and crosses this in well under a second; incidental
/// brushes of the effector against the tether stay below it.
const GRIP_STRENGTH: f64 = 0.3;

/// Live controller grips are binary: zero before a contact point is captured,
/// maximum strength afterward.
pub const RUNTIME_GRIP_STRENGTH: f64 = 1.0;

/// Decay rate (1/s) of the grip strain accumulator: brief tugs are forgiven,
/// sustained ones are not.
const GRIP_STRAIN_DECAY: f64 = 2.0;

/// How fast a failing grip pays out tether (m/s). A hard constraint deleted
/// mid-frame snaps the stretched limb back at teleport speed; paying the
/// tether out instead turns breaking into a slip over a few hundred ms.
const GRIP_RELEASE_RATE: f64 = 1.0;

/// A grip that has paid out this much tether (meters) is no longer holding
/// anything and is removed outright.
const GRIP_RELEASE_LIMIT: f64 = 0.25;

/// Payout past this point latches the failure: the grip keeps paying out even
/// if the strain relaxes. Without the latch, payout relieves exactly enough
/// stretch to hold strain at the threshold and the grip "slips" indefinitely
/// instead of breaking.
const GRIP_FAIL_LATCH: f64 = 0.02;

/// Can `joint` grip capsule `cap`? Opponent capsules always (that's what
/// grappling is); own capsules only for a hand clasping the *other* arm
/// (gable grips and the like). Everything else - a hand hanging by the own
/// thigh, a foot next to its own calf - is incidental contact, not a grip.
fn can_grip(joint: PlayerJoint, cap: &CapsuleDef) -> bool {
    if cap.player != joint.player {
        return true;
    }
    let other_arm: &[Joint] = match joint.joint {
        Joint::LeftHand | Joint::LeftFingers => &[
            Joint::RightElbow,
            Joint::RightWrist,
            Joint::RightHand,
            Joint::RightFingers,
        ],
        Joint::RightHand | Joint::RightFingers => &[
            Joint::LeftElbow,
            Joint::LeftWrist,
            Joint::LeftHand,
            Joint::LeftFingers,
        ],
        _ => return false,
    };
    cap.ends.iter().all(|e| other_arm.contains(e))
}

/// Detect grips in a loaded pose: each grip-capable joint tethers to the
/// closest capsule it is in contact with (at most one grip per joint).
fn capture_grips(pose: &Pose, caps: &[CapsuleDef]) -> Vec<Grip> {
    let mut grips = Vec::new();
    for player in PlayerId::ALL {
        for joint in GRIP_JOINTS {
            let pj = PlayerJoint { player, joint };
            let p = pose[pj];
            let mut best: Option<Grip> = None;
            let mut best_clearance = GRIP_CONTACT;
            for (ci, cap) in caps.iter().enumerate() {
                if !can_grip(pj, cap) {
                    continue;
                }
                let (a, b) = (pose[cap.a()], pose[cap.b()]);
                let (anchor, _) = gm_core::closest_point_on_segment(p, a, b);
                let clearance = p.distance(anchor) - cap.radius - joint.radius();
                if clearance < best_clearance {
                    best_clearance = clearance;
                    best = Some(Grip {
                        joint: pj,
                        cap: ci,
                        len: p.distance(anchor) + GRIP_SLACK,
                        runtime: false,
                        wrist: None,
                        finger: None,
                        finger_len: 0.0,
                        hand_wrap: 0.0,
                        target_wrap: 0.0,
                        selected_writhe: 0.0,
                        alternate_writhe: 0.0,
                    });
                }
            }
            grips.extend(best);
        }
    }
    grips
}

fn is_foot_joint(j: Joint) -> bool {
    matches!(
        j,
        Joint::LeftToe
            | Joint::RightToe
            | Joint::LeftHeel
            | Joint::RightHeel
            | Joint::LeftAnkle
            | Joint::RightAnkle
    )
}

const FEET: [[Joint; 3]; 2] = [
    [Joint::LeftToe, Joint::LeftHeel, Joint::LeftAnkle],
    [Joint::RightToe, Joint::RightHeel, Joint::RightAnkle],
];

/// Standing support of one player: the mean of its planted feet's toe-heel
/// midpoints, plus the lowest grounded-joint height. A foot is planted when
/// any of its joints touches the floor (judged in the `contact` pose); the
/// anchor uses whole-foot geometry from `pose`, so it does not jump when
/// individual toe/heel contacts flicker between substeps.
///
/// Returns None unless the player is *standing*: some foot planted and
/// nothing else grounded. Balance is a standing skill - shoving the COM
/// around while kneeling, lying, or posting a hand would wreck authored
/// ground-grappling positions.
fn standing_support(pose: &Pose, contact: &Pose, player: PlayerId) -> Option<(V3, f64)> {
    let mut support_y = f64::INFINITY;
    for j in Joint::ALL {
        let pj = PlayerJoint { player, joint: j };
        if contact[pj].y <= j.radius() + 0.01 {
            if !is_foot_joint(j) {
                return None;
            }
            support_y = support_y.min(pose[pj].y);
        }
    }
    let mut center = V3::ZERO;
    let mut planted = 0.0;
    for foot in FEET {
        let grounded = foot.iter().any(|&j| {
            let pj = PlayerJoint { player, joint: j };
            contact[pj].y <= j.radius() + 0.01
        });
        if grounded {
            let toe = pose[PlayerJoint {
                player,
                joint: foot[0],
            }];
            let heel = pose[PlayerJoint {
                player,
                joint: foot[1],
            }];
            center += v3(0.5 * (toe.x + heel.x), 0.0, 0.5 * (toe.z + heel.z));
            planted += 1.0;
        }
    }
    if planted < 1.0 {
        return None;
    }
    Some((center / planted, support_y))
}

/// Which grips project this step. A grip belongs to the *gripping* player's
/// limb; while an effector is driving any joint of that limb the user is
/// commanding it (reach, pull away, let go), so its own grip is suspended -
/// it stops resisting immediately, and pays out tether to match wherever the
/// user takes the limb (see the grip-wear pass in `solve_pass`). Grips held
/// *by the opponent* on the driven limb are unaffected - being held is not
/// something you can override by moving your own arm; those must be broken by
/// sustained pulling (grip strain).
fn grip_mask(grips: &[Grip], effectors: &[Effector], broken: u32) -> Vec<bool> {
    grips
        .iter()
        .enumerate()
        .map(|(i, g)| {
            if broken & (1 << i) != 0 {
                return false;
            }
            g.runtime || !effectors.iter().any(|e| {
                e.joint.player == g.joint.player
                    && LIMB_GROUPS.iter().any(|(group, _)| {
                        group.contains(&g.joint.joint) && group.contains(&e.joint.joint)
                    })
            })
        })
        .collect()
}

/// Which grips can accumulate breaking strain this step: only those whose
/// *gripped* limb is currently user-driven. Yanking your arm out of the
/// opponent's hold is deliberate and should eventually succeed; every other
/// load (gravity, tone disagreeing with an entangled pose by a few mm each
/// substep) is one the grip must carry indefinitely - authored holds like a
/// suspended butterfly hang on their grips forever, and constraint chatter
/// must never quietly wear one down.
fn grip_strainable(grips: &[Grip], caps: &[CapsuleDef], effectors: &[Effector]) -> Vec<bool> {
    grips
        .iter()
        .map(|g| {
            let cap = &caps[g.cap];
            let target_driven = effectors.iter().any(|e| {
                e.joint.player == cap.player
                    && LIMB_GROUPS.iter().any(|(group, root)| {
                        let in_group = |j: Joint| group.contains(&j) || *root == Some(j);
                        in_group(e.joint.joint) && cap.ends.iter().any(|&end| in_group(end))
                    })
            });
            let source_driven = g.runtime && effectors.iter().any(|e| {
                e.joint.player == g.joint.player
                    && LIMB_GROUPS.iter().any(|(group, _)| {
                        group.contains(&g.joint.joint) && group.contains(&e.joint.joint)
                    })
            });
            target_driven || source_driven
        })
        .collect()
}

fn tone_scales(effectors: &[Effector]) -> [[f64; gm_core::JOINT_COUNT]; gm_core::PLAYER_COUNT] {
    let mut scales = [[1.0f64; gm_core::JOINT_COUNT]; gm_core::PLAYER_COUNT];
    for e in effectors {
        if let Some(leg) = LEG_GROUPS
            .iter()
            .find(|leg| leg[1..].contains(&e.joint.joint))
        {
            let retained = [
                DRIVEN_HIP_TONE_SCALE,
                DRIVEN_KNEE_TONE_SCALE,
                DRIVEN_FOOT_TONE_SCALE,
                DRIVEN_FOOT_TONE_SCALE,
                DRIVEN_FOOT_TONE_SCALE,
            ];
            for (&joint, scale) in leg.iter().zip(retained) {
                let s = &mut scales[e.joint.player.index()][joint.index()];
                *s = (*s).min(scale);
            }
        } else {
            for (group, root) in LIMB_GROUPS {
                if group.contains(&e.joint.joint) {
                    for &j in group {
                        let s = &mut scales[e.joint.player.index()][j.index()];
                        *s = (*s).min(DRIVEN_TONE_SCALE);
                    }
                    if let Some(root) = root {
                        let s = &mut scales[e.joint.player.index()][root.index()];
                        *s = (*s).min(DRIVEN_ROOT_TONE_SCALE);
                    }
                }
            }
        }
        // The directly commanded coordinate gets no direct tone component.
        scales[e.joint.player.index()][e.joint.joint.index()] = 0.0;
    }
    scales
}

/// Keep release of a displaced foot gradual. Once orientation effectors are
/// removed, restoring full global foot tone in one frame would convert their
/// accumulated residual into a snap. While either local ankle coordinate is
/// outside the orientation dead zone, retain the driven knee/hip gradient and
/// keep the three orientation coordinates free of direct global tone; the
/// dedicated soft angular rule then unloads the foot progressively. Normal
/// foot tone resumes only inside the neutral zone.
fn retain_displaced_ankle_tone(
    pose: &Pose,
    references: &AnkleReferences,
    release_active: &[[bool; 2]; gm_core::PLAYER_COUNT],
    dead_zone: f64,
    scales: &mut [[f64; gm_core::JOINT_COUNT]; gm_core::PLAYER_COUNT],
) {
    for player in PlayerId::ALL {
        for (leg_index, side) in LegSide::ALL.into_iter().enumerate() {
            if !release_active[player.index()][leg_index] {
                continue;
            }
            let Some(angles) =
                ankle_angles(pose, player, side, &references[player.index()][leg_index])
            else {
                continue;
            };
            let displacement = angles.swing.max(angles.twist.abs());
            if displacement <= f64::EPSILON {
                continue;
            }
            // At and outside the angular dead-zone boundary the distal frame
            // has zero direct global tone. Inside it, tone ramps continuously
            // back to one as the foot reaches neutral; this avoids a second
            // snap precisely when the soft angular rule becomes inactive.
            let release = gm_core::clamp(displacement / dead_zone.max(1e-9), 0.0, 1.0);
            let retained = [
                1.0 - release * (1.0 - DRIVEN_HIP_TONE_SCALE),
                1.0 - release * (1.0 - DRIVEN_KNEE_TONE_SCALE),
                1.0 - release,
                1.0 - release,
                1.0 - release,
            ];
            for (&joint, scale) in LEG_GROUPS[leg_index].iter().zip(retained) {
                let value = &mut scales[player.index()][joint.index()];
                *value = (*value).min(scale);
            }
        }
    }
}

fn driven_ankles(effectors: &[Effector]) -> [[bool; 2]; gm_core::PLAYER_COUNT] {
    let mut driven = [[false; 2]; gm_core::PLAYER_COUNT];
    for effector in effectors {
        for (leg_index, leg) in LEG_GROUPS.iter().enumerate() {
            if leg[2..].contains(&effector.joint.joint) {
                driven[effector.joint.player.index()][leg_index] = true;
            }
        }
    }
    driven
}

#[cfg(test)]
mod tone_scale_tests {
    use super::*;
    use gm_core::{P0, P1};

    fn effector(player: PlayerId, joint: Joint) -> Effector {
        Effector {
            joint: PlayerJoint { player, joint },
            target: V3::ZERO,
            stiffness: 1.0,
        }
    }

    #[test]
    fn foot_drive_retains_a_monotone_tone_gradient_toward_the_hip() {
        let scales = tone_scales(&[effector(P0, Joint::LeftToe)]);
        let p0 = &scales[P0.index()];

        assert_eq!(p0[Joint::LeftToe.index()], 0.0);
        assert_eq!(p0[Joint::LeftHeel.index()], DRIVEN_FOOT_TONE_SCALE);
        assert_eq!(p0[Joint::LeftAnkle.index()], DRIVEN_FOOT_TONE_SCALE);
        assert_eq!(p0[Joint::LeftKnee.index()], DRIVEN_KNEE_TONE_SCALE);
        assert_eq!(p0[Joint::LeftHip.index()], DRIVEN_HIP_TONE_SCALE);
        assert!(
            p0[Joint::LeftToe.index()] < p0[Joint::LeftAnkle.index()]
                && p0[Joint::LeftAnkle.index()] < p0[Joint::LeftKnee.index()]
                && p0[Joint::LeftKnee.index()] < p0[Joint::LeftHip.index()]
        );

        // The policy is local to the driven leg and player.
        assert_eq!(p0[Joint::RightKnee.index()], 1.0);
        assert_eq!(p0[Joint::Core.index()], 1.0);
        assert_eq!(scales[P1.index()][Joint::LeftKnee.index()], 1.0);
    }

    #[test]
    fn foot_orientation_effectors_do_not_re_relax_the_knee_or_hip() {
        let scales = tone_scales(&[
            effector(P0, Joint::LeftAnkle),
            effector(P0, Joint::LeftHeel),
            effector(P0, Joint::LeftToe),
        ]);
        let p0 = &scales[P0.index()];

        assert_eq!(p0[Joint::LeftAnkle.index()], 0.0);
        assert_eq!(p0[Joint::LeftHeel.index()], 0.0);
        assert_eq!(p0[Joint::LeftToe.index()], 0.0);
        assert_eq!(p0[Joint::LeftKnee.index()], DRIVEN_KNEE_TONE_SCALE);
        assert_eq!(p0[Joint::LeftHip.index()], DRIVEN_HIP_TONE_SCALE);
    }

    #[test]
    fn arm_relaxation_policy_is_unchanged() {
        let scales = tone_scales(&[effector(P0, Joint::RightHand)]);
        let p0 = &scales[P0.index()];

        for joint in [
            Joint::RightElbow,
            Joint::RightWrist,
            Joint::RightHand,
            Joint::RightFingers,
        ] {
            assert_eq!(p0[joint.index()], DRIVEN_TONE_SCALE);
        }
        assert_eq!(p0[Joint::RightShoulder.index()], DRIVEN_ROOT_TONE_SCALE);
    }
}

#[cfg(test)]
mod runtime_wrap_tests {
    use super::*;
    use gm_core::{P0, P1};

    fn fixture() -> (Solver, SolverState, PlayerJoint, usize) {
        let mut pose = Pose::default();
        for pj in PlayerJoint::all() {
            let n = pj.flat() as f64;
            pose[pj] = v3(20.0 + n * 2.0, 5.0 + n * 0.1, 30.0);
        }
        pose.set(P1, Joint::LeftShoulder, v3(0.0, 0.0, 0.0));
        pose.set(P1, Joint::LeftElbow, v3(0.0, 1.0, 0.0));
        let solver = Solver::new(&pose, SolverConfig::default());
        let mut state = SolverState::from_pose(pose);
        let hand = PlayerJoint { player: P0, joint: Joint::LeftHand };
        let wrist = PlayerJoint { player: P0, joint: Joint::LeftWrist };
        let finger = PlayerJoint { player: P0, joint: Joint::LeftFingers };
        let radius = 0.070;
        let angle = 25.0f64.to_radians();
        state.pose[wrist] = v3(0.09, 0.5, 0.02);
        state.pose[hand] = v3(radius, 0.5, 0.0);
        state.pose[finger] = v3(radius * angle.cos(), 0.575, -radius * angle.sin());
        let capsule = solver
            .capsules()
            .iter()
            .position(|cap| {
                cap.player == P1
                    && cap.ends == [Joint::LeftShoulder, Joint::LeftElbow]
            })
            .unwrap();
        (solver, state, hand, capsule)
    }

    #[test]
    fn wrist_contact_authorizes_a_floppy_hand_without_palm_aim() {
        let (solver, mut state, hand, capsule) = fixture();
        let (_, finger) = Solver::hand_chain(hand).unwrap();
        state.pose[hand] += v3(0.18, 0.0, 0.0);
        state.pose[finger] += v3(0.24, 0.0, 0.0);
        let candidate = solver
            .query_grip_candidate(
                &state,
                hand,
                state.pose[hand],
                v3(1.0, 0.0, 0.0),
                0.04,
                0.2,
            )
            .expect("wrist contact should authorize a solver wrap");
        assert_eq!(candidate.capsule, capsule);
        assert!(candidate.surface_gap < 0.04);
    }

    #[test]
    fn maximum_writhe_wrap_is_dramatic_local_and_captures_full_strength_anchors() {
        let (mut solver, mut state, hand, capsule) = fixture();
        assert!(solver.begin_runtime_grip(&mut state, hand, capsule, 0.04));
        let index = solver.grip_count() - 1;
        let wrist = solver.grips[index].wrist.unwrap();
        let finger = solver.grips[index].finger.unwrap();
        assert!(
            solver.grips[index].selected_writhe + 1e-9
                >= solver.grips[index].alternate_writhe,
            "solver did not choose the maximum-writhe direction",
        );
        let initial = Solver::signed_wrap_angle(
            &state.pose,
            &solver.caps[capsule],
            state.pose[wrist],
            state.pose[finger],
        )
        .abs();
        let wrist_before = state.pose[wrist];
        let cap_a = solver.caps[capsule].a();
        let cap_b = solver.caps[capsule].b();
        let cap_a_before = state.pose[cap_a];
        let cap_b_before = state.pose[cap_b];
        let hand_before = state.pose[hand];
        let finger_before = state.pose[finger];
        let active = vec![true; solver.grip_count()];
        for step in 1..=40 {
            state.grip_wrap[index] = step as f64 / 40.0;
            solver.project_runtime_wraps(
                &mut state.pose,
                &active,
                &state.grip_wrap,
                &state.grip_contact_bits,
                1.0 / 120.0,
            );
        }
        let curled = Solver::signed_wrap_angle(
            &state.pose,
            &solver.caps[capsule],
            state.pose[wrist],
            state.pose[finger],
        )
        .abs();
        assert!(
            curled > initial + 45.0f64.to_radians(),
            "wrap was not visually dramatic: {} -> {} degrees",
            initial.to_degrees(),
            curled.to_degrees(),
        );
        assert_eq!(state.pose[wrist], wrist_before, "acquisition moved the tracked wrist");
        assert_eq!(state.pose[cap_a], cap_a_before, "acquisition moved the target capsule");
        assert_eq!(state.pose[cap_b], cap_b_before, "acquisition moved the target capsule");
        assert!(state.pose[hand].distance(hand_before) > 0.01);
        assert!(state.pose[finger].distance(finger_before) > 0.05);

        let mut active = active;
        solver.update_runtime_grip_quality(&mut state, 0.02, &mut active);
        assert!(state.grip_coverage[index] > 0.65);
        assert_eq!(state.grip_contact_bits[index], 0b11);
        assert_eq!(state.grip_contact[index], 1.0);
        assert_eq!(state.grip_strength[index], RUNTIME_GRIP_STRENGTH);
        assert!(solver.runtime_grip_state(&state, hand).unwrap().active);

        // Proof that these are fixed material contact points rather than the
        // previous sliding capsule tethers: pull both captured points far
        // away, project once, and verify they return to their stored axial and
        // radial anchors at full stiffness.
        state.pose[hand] += v3(0.20, 0.12, 0.08);
        state.pose[finger] += v3(-0.16, 0.15, 0.10);
        for _ in 0..200 {
            solver.project_grips(
                &mut state.pose,
                &active,
                &state.grip_release,
                &state.grip_contact_bits,
                &state.grip_hand_anchor_s,
                &state.grip_hand_anchor_radial,
                &state.grip_finger_anchor_s,
                &state.grip_finger_anchor_radial,
                &|_| 1.0,
            );
        }
        let hand_anchor = state.pose[cap_a] * (1.0 - state.grip_hand_anchor_s[index])
            + state.pose[cap_b] * state.grip_hand_anchor_s[index]
            + state.grip_hand_anchor_radial[index];
        let finger_anchor = state.pose[cap_a] * (1.0 - state.grip_finger_anchor_s[index])
            + state.pose[cap_b] * state.grip_finger_anchor_s[index]
            + state.grip_finger_anchor_radial[index];
        let hand_error = state.pose[hand].distance(hand_anchor);
        let finger_error = state.pose[finger].distance(finger_anchor);
        assert!(hand_error < 0.002, "sticky hand anchor error {hand_error}");
        assert!(finger_error < 0.002, "sticky finger anchor error {finger_error}");

        // A shallow wrap receives the exact same full holding strength as a
        // dramatic one as soon as it makes contact.
        let (mut shallow_solver, mut shallow, shallow_hand, shallow_capsule) = fixture();
        assert!(shallow_solver.begin_runtime_grip(
            &mut shallow, shallow_hand, shallow_capsule, 0.04));
        let shallow_index = shallow_solver.grip_count() - 1;
        let mut shallow_active = vec![true; shallow_solver.grip_count()];
        shallow_solver.update_runtime_grip_quality(
            &mut shallow, 0.02, &mut shallow_active);
        assert!(shallow.grip_coverage[shallow_index] < state.grip_coverage[index]);
        assert_ne!(shallow.grip_contact_bits[shallow_index], 0);
        assert_eq!(
            shallow.grip_strength[shallow_index],
            state.grip_strength[index],
            "wrap amount incorrectly scaled sticky grip strength",
        );
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct StepDiagnostics {
    /// Number of active contacts in the last substep.
    pub contact_count: usize,
    /// Minimum surface clearance across all capsule pairs after the step.
    pub min_clearance: f64,
    /// Worst bone length error after the step (relative to rest length).
    pub max_bone_error: f64,
    /// Worst hinge min-angle violation (radians).
    pub max_hinge_violation: f64,
    /// Largest single-step writhe jump seen by the watchdog.
    pub max_writhe_jump: f64,
    /// Watchdog re-solves performed (0 = clean first pass).
    pub retries: usize,
    /// True if the step was rejected outright (crossing persisted at max substeps).
    pub rejected: bool,
    /// Per-effector residual distance to target after the step (meters).
    pub effector_residuals: Vec<f64>,
}

pub struct Solver {
    pub config: SolverConfig,
    bones: Vec<Bone>,
    caps: Vec<CapsuleDef>,
    pairs: Vec<(usize, usize)>,
    /// Persistent clearance floor per collidable pair: min(clearance at load, 0).
    /// Database poses are authored in tight contact; that depth is accepted but
    /// may never deepen, and pairs that were separated at load may never penetrate.
    clearance_floors: Vec<f64>,
    /// Hard-pinned joints (infinite mass). Distinct from effector pins:
    /// these never move at all.
    pins: Vec<PlayerJoint>,
    /// Authored COM offset from the foot-support center at load, per player
    /// (None when the player was not standing on their feet in the loaded
    /// pose). The balance servo holds this offset rather than centering the
    /// COM, so authored leans are preserved exactly.
    balance_ref: [Option<(f64, f64)>; 2],
    /// Grips captured from the loaded pose (hands/feet in tight contact hold
    /// on). Tether constraints: they resist separation, never compression.
    grips: Vec<Grip>,
    /// Authored foot orientation in each shin-local frame, plus deterministic
    /// near-straight frame fallbacks.
    ankle_references: AnkleReferences,
}

impl Solver {
    /// Create a solver anchored to a loaded pose. Bone rest lengths and contact
    /// floors are captured from the pose so database positions load without
    /// distortion.
    pub fn new(pose: &Pose, config: SolverConfig) -> Solver {
        let caps = capsules();
        let pairs = collidable_pairs(&caps);
        let clearance_floors = pairs
            .iter()
            .map(|&(i, j)| {
                let (a, b) = (&caps[i], &caps[j]);
                let (ca, cb, _, _) = gm_core::closest_segment_points(
                    pose[a.a()],
                    pose[a.b()],
                    pose[b.a()],
                    pose[b.b()],
                );
                (ca.distance(cb) - a.radius - b.radius).min(0.0)
            })
            .collect();
        // Authored poses are trusted as-is: whatever COM offset a standing
        // player was loaded with (including heavy leans held up by the
        // opponent) is the offset the servo maintains.
        let balance_ref = [PlayerId(0), PlayerId(1)].map(|player| {
            standing_support(pose, pose, player).map(|(center, _)| {
                let com = crate::tone::centroid(pose, player);
                (com.x - center.x, com.z - center.z)
            })
        });
        let grips = capture_grips(pose, &caps);
        let ankle_references = capture_ankle_references(pose);
        Solver {
            config,
            bones: capture_bones(pose),
            caps,
            pairs,
            clearance_floors,
            pins: Vec::new(),
            balance_ref,
            grips,
            ankle_references,
        }
    }

    /// Release all grips captured at load (e.g. when the user wants the
    /// players to disengage).
    pub fn release_grips(&mut self) {
        self.grips.clear();
    }

    fn hand_chain(hand: PlayerJoint) -> Option<(PlayerJoint, PlayerJoint)> {
        let (wrist, finger) = match hand.joint {
            Joint::LeftHand => (Joint::LeftWrist, Joint::LeftFingers),
            Joint::RightHand => (Joint::RightWrist, Joint::RightFingers),
            _ => return None,
        };
        Some((
            PlayerJoint { player: hand.player, joint: wrist },
            PlayerJoint { player: hand.player, joint: finger },
        ))
    }

    fn radial_about_capsule(pose: &Pose, cap: &CapsuleDef, point: V3) -> Option<(V3, V3, f64)> {
        let a = pose[cap.a()];
        let b = pose[cap.b()];
        let axis = (b - a).normalized(1e-9)?;
        let (anchor, s) = gm_core::closest_point_on_segment(point, a, b);
        Some(((point - anchor).normalized(1e-9)?, axis, s))
    }

    fn signed_wrap_angle(pose: &Pose, cap: &CapsuleDef, hand: V3, finger: V3) -> f64 {
        let Some((hand_radial, axis, _)) = Self::radial_about_capsule(pose, cap, hand) else {
            return 0.0;
        };
        let Some((finger_radial, _, _)) = Self::radial_about_capsule(pose, cap, finger) else {
            return 0.0;
        };
        axis.dot(hand_radial.cross(finger_radial))
            .atan2(hand_radial.dot(finger_radial))
    }

    fn rotate_radial(radial: V3, axis: V3, angle: f64) -> V3 {
        let (sin, cos) = angle.sin_cos();
        radial * cos + axis.cross(radial) * sin
    }

    fn wrap_step_angle(span: f64, surface_radius: f64) -> f64 {
        (span / (2.0 * surface_radius).max(1e-9))
            .clamp(0.0, 0.96)
            .asin()
            .mul_add(2.0 * 0.92, 0.0)
            .clamp(RUNTIME_WRAP_MIN_STEP_ANGLE, RUNTIME_WRAP_MAX_STEP_ANGLE)
    }

    fn wrap_direction_score(
        pose: &Pose,
        cap: &CapsuleDef,
        wrist: PlayerJoint,
        hand: PlayerJoint,
        finger: PlayerJoint,
        hand_angle: f64,
        finger_angle: f64,
    ) -> Option<f64> {
        let a = pose[cap.a()];
        let b = pose[cap.b()];
        let axis = (b - a).normalized(1e-9)?;
        let (wrist_axis, _) = gm_core::closest_point_on_segment(pose[wrist], a, b);
        let wrist_radial = (pose[wrist] - wrist_axis).normalized(1e-9)?;
        let (hand_axis, _) = gm_core::closest_point_on_segment(pose[hand], a, b);
        let (finger_axis, _) = gm_core::closest_point_on_segment(pose[finger], a, b);
        let hand_target = hand_axis
            + Self::rotate_radial(wrist_radial, axis, hand_angle)
                * (cap.radius + hand.joint.radius() + RUNTIME_GRIP_SLACK);
        let finger_target = finger_axis
            + Self::rotate_radial(wrist_radial, axis, finger_angle)
                * (cap.radius + finger.joint.radius() + RUNTIME_GRIP_SLACK);
        let writhe =
            gm_topology::segment_writhe(pose[wrist], hand_target, a, b)
                + gm_topology::segment_writhe(hand_target, finger_target, a, b);
        Some(writhe.abs())
    }

    fn plan_runtime_wrap(
        pose: &Pose,
        cap: &CapsuleDef,
        wrist: PlayerJoint,
        hand: PlayerJoint,
        finger: PlayerJoint,
    ) -> Option<RuntimeWrapPlan> {
        let hand_span = pose[wrist].distance(pose[hand]);
        let finger_span = pose[hand].distance(pose[finger]);
        let hand_angle = Self::wrap_step_angle(
            hand_span,
            cap.radius + 0.5 * (wrist.joint.radius() + hand.joint.radius()),
        );
        let second_angle = Self::wrap_step_angle(
            finger_span,
            cap.radius + 0.5 * (hand.joint.radius() + finger.joint.radius()),
        );
        let finger_angle = (hand_angle + second_angle).min(144.0f64.to_radians());
        let positive = Self::wrap_direction_score(
            pose, cap, wrist, hand, finger, hand_angle, finger_angle)?;
        let negative = Self::wrap_direction_score(
            pose, cap, wrist, hand, finger, -hand_angle, -finger_angle)?;
        let existing =
            Self::signed_wrap_angle(pose, cap, pose[wrist], pose[finger]);
        let direction = if positive > negative + 1e-9 {
            1.0
        } else if negative > positive + 1e-9 {
            -1.0
        } else if existing.abs() > 1e-6 {
            existing.signum()
        } else {
            1.0
        };
        let (selected_writhe, alternate_writhe) = if direction > 0.0 {
            (positive, negative)
        } else {
            (negative, positive)
        };
        Some(RuntimeWrapPlan {
            hand_wrap: direction * hand_angle,
            finger_wrap: direction * finger_angle,
            selected_writhe,
            alternate_writhe,
        })
    }

    pub fn query_grip_candidate(
        &self,
        state: &SolverState,
        hand: PlayerJoint,
        palm: V3,
        palm_aim: V3,
        max_gap: f64,
        _min_alignment: f64,
    ) -> Option<GripCandidate> {
        if !matches!(hand.joint, Joint::LeftHand | Joint::RightHand)
            || !palm.is_finite()
            || !palm_aim.is_finite()
            || !max_gap.is_finite()
            || max_gap <= 0.0
        {
            return None;
        }
        let aim = palm_aim.normalized(1e-9)?;
        let (wrist, finger) = Self::hand_chain(hand)?;
        let mut best: Option<GripCandidate> = None;
        for (capsule, cap) in self.caps.iter().enumerate() {
            if !can_grip(hand, cap) {
                continue;
            }
            let (wrist_axis, _) = gm_core::closest_point_on_segment(
                state.pose[wrist],
                state.pose[cap.a()],
                state.pose[cap.b()],
            );
            let surface_gap =
                (state.pose[wrist].distance(wrist_axis) - cap.radius - wrist.joint.radius()).max(0.0);
            if surface_gap > max_gap {
                continue;
            }
            let Some(plan) =
                Self::plan_runtime_wrap(&state.pose, cap, wrist, hand, finger)
            else {
                continue;
            };
            let outward = (state.pose[wrist] - wrist_axis)
                .normalized(1e-9)
                .unwrap_or(-aim);
            let closest = wrist_axis + outward * cap.radius;
            let palm_alignment = (closest - palm).normalized(1e-9).unwrap_or(aim).dot(aim);
            let wrap_alignment = (plan.selected_writhe / 0.35).clamp(0.0, 1.0);
            let proximity = 1.0 - surface_gap / max_gap;
            let score = 0.82 * proximity + 0.18 * wrap_alignment;
            let candidate = GripCandidate {
                capsule,
                player: cap.player,
                ends: cap.ends,
                closest,
                surface_gap,
                palm_alignment,
                wrap_alignment,
                score,
            };
            if best.map_or(true, |current| {
                score > current.score + 1e-12
                    || ((score - current.score).abs() <= 1e-12 && capsule < current.capsule)
            }) {
                best = Some(candidate);
            }
        }
        best
    }

    pub fn begin_runtime_grip(
        &mut self,
        state: &mut SolverState,
        hand: PlayerJoint,
        capsule: usize,
        max_gap: f64,
    ) -> bool {
        if !matches!(hand.joint, Joint::LeftHand | Joint::RightHand) {
            return false;
        }
        let Some(cap) = self.caps.get(capsule) else { return false; };
        if !can_grip(hand, cap) {
            return false;
        }
        let Some((wrist, finger)) = Self::hand_chain(hand) else { return false; };
        let (wrist_anchor, _) = gm_core::closest_point_on_segment(
            state.pose[wrist], state.pose[cap.a()], state.pose[cap.b()]);
        let wrist_gap =
            (state.pose[wrist].distance(wrist_anchor) - cap.radius - wrist.joint.radius()).max(0.0);
        if !wrist_gap.is_finite() || wrist_gap > max_gap {
            return false;
        }
        let Some(plan) = Self::plan_runtime_wrap(&state.pose, cap, wrist, hand, finger) else {
            return false;
        };

        self.end_runtime_grip(state, hand);
        let replacement = self.grips.iter().enumerate().find_map(|(index, grip)| {
            (grip.runtime && state.broken_grips & (1 << index) != 0).then_some(index)
        });
        let surface_radius = cap.radius + hand.joint.radius() + RUNTIME_GRIP_SLACK;
        let grip = Grip {
            joint: hand,
            cap: capsule,
            len: surface_radius,
            runtime: true,
            wrist: Some(wrist),
            finger: Some(finger),
            finger_len: cap.radius + finger.joint.radius() + RUNTIME_GRIP_SLACK,
            hand_wrap: plan.hand_wrap,
            target_wrap: plan.finger_wrap,
            selected_writhe: plan.selected_writhe,
            alternate_writhe: plan.alternate_writhe,
        };
        let index = if let Some(index) = replacement {
            self.grips[index] = grip;
            index
        } else {
            if self.grips.len() >= 32 {
                return false;
            }
            self.grips.push(grip);
            self.grips.len() - 1
        };
        state.broken_grips &= !(1 << index);
        state.grip_strain[index] = 0.0;
        state.grip_release[index] = 0.0;
        state.grip_wrap[index] = 0.0;
        state.grip_contact[index] = 0.0;
        state.grip_coverage[index] = 0.0;
        state.grip_strength[index] = 0.0;
        state.grip_contact_bits[index] = 0;
        state.grip_anchor_axis[index] = V3::ZERO;
        state.grip_hand_anchor_s[index] = 0.0;
        state.grip_hand_anchor_radial[index] = V3::ZERO;
        state.grip_finger_anchor_s[index] = 0.0;
        state.grip_finger_anchor_radial[index] = V3::ZERO;
        true
    }

    pub fn end_runtime_grip(&self, state: &mut SolverState, hand: PlayerJoint) -> bool {
        let mut ended = false;
        for (index, grip) in self.grips.iter().enumerate() {
            if grip.runtime && grip.joint == hand && state.broken_grips & (1 << index) == 0 {
                state.broken_grips |= 1 << index;
                state.grip_strain[index] = 0.0;
                state.grip_wrap[index] = 0.0;
                state.grip_contact[index] = 0.0;
                state.grip_coverage[index] = 0.0;
                state.grip_strength[index] = 0.0;
                state.grip_contact_bits[index] = 0;
                ended = true;
            }
        }
        ended
    }

    pub fn release_runtime_grips(&self, state: &mut SolverState) {
        for (index, grip) in self.grips.iter().enumerate() {
            if grip.runtime {
                state.broken_grips |= 1 << index;
                state.grip_strain[index] = 0.0;
                state.grip_wrap[index] = 0.0;
                state.grip_contact[index] = 0.0;
                state.grip_coverage[index] = 0.0;
                state.grip_strength[index] = 0.0;
                state.grip_contact_bits[index] = 0;
            }
        }
    }

    pub fn runtime_grip_state(
        &self,
        state: &SolverState,
        hand: PlayerJoint,
    ) -> Option<RuntimeGripState> {
        self.grips.iter().enumerate().rev().find_map(|(index, grip)| {
            if !grip.runtime || grip.joint != hand {
                return None;
            }
            let cap = &self.caps[grip.cap];
            let broken = state.broken_grips & (1 << index) != 0;
            Some(RuntimeGripState {
                active: !broken,
                broken,
                capsule: grip.cap,
                player: cap.player,
                ends: cap.ends,
                strain: state.grip_strain[index],
                release: state.grip_release[index],
                wrap: state.grip_wrap[index],
                contact: state.grip_contact[index],
                coverage: state.grip_coverage[index],
                strength: state.grip_strength[index],
                direction: grip.target_wrap.signum(),
                selected_writhe: grip.selected_writhe,
                alternate_writhe: grip.alternate_writhe,
            })
        })
    }

    pub fn grip_count(&self) -> usize {
        self.grips.len()
    }

    /// Grip descriptions for debugging/UI: (gripping joint, gripped player,
    /// gripped capsule ends).
    pub fn grip_summaries(&self) -> Vec<(PlayerJoint, PlayerId, [Joint; 2])> {
        self.grips
            .iter()
            .map(|g| {
                let cap = &self.caps[g.cap];
                (g.joint, cap.player, cap.ends)
            })
            .collect()
    }

    pub fn set_pins(&mut self, pins: Vec<PlayerJoint>) {
        self.pins = pins;
    }

    pub fn bones(&self) -> &[Bone] {
        &self.bones
    }

    pub fn capsules(&self) -> &[CapsuleDef] {
        &self.caps
    }

    pub fn pairs(&self) -> &[(usize, usize)] {
        &self.pairs
    }

    fn inv_mass(&self, pj: PlayerJoint) -> f64 {
        if self.pins.contains(&pj) {
            0.0
        } else {
            1.0 / pj.joint.mass()
        }
    }

    /// Advance the state by `dt` seconds under the given effector inputs.
    /// Pure with respect to `state`: returns the new state and diagnostics.
    pub fn step(
        &self,
        state: &SolverState,
        effectors: &[Effector],
        dt: f64,
    ) -> (SolverState, StepDiagnostics) {
        let dt = dt.clamp(1e-4, 0.1);
        let writhes_before = gm_topology::all_pair_writhes(&state.pose);

        let mut retries = 0;
        let mut substeps = self.config.substeps.max(1);
        loop {
            let (next, mut diag) = self.solve_pass(state, effectors, dt, substeps);

            let crossed = self.any_crossing(&state.pose, &next.pose);
            let writhes_after = gm_topology::all_pair_writhes(&next.pose);
            let linking = gm_topology::linking_report(&writhes_before, &writhes_after);
            diag.max_writhe_jump = linking.max_writhe_jump;
            diag.retries = retries;

            let dirty = self.violates_universal_positional_bounds(&next.pose)
                || self.violates_anatomical_bounds(&next.pose)
                || next.pose.max_displacement(&state.pose) > MAX_ACCEPTED_STEP_SPEED * dt
                || crossed
                || linking.max_writhe_jump > gm_topology::WRITHE_JUMP_THRESHOLD;
            if !dirty {
                return (next, diag);
            }
            if retries >= self.config.max_retries {
                // Reject the step entirely: freeze at the entry state (zeroing
                // velocity) rather than ever exhibiting a topology glitch.
                let mut frozen = *state;
                frozen.velocity = Default::default();
                diag.rejected = true;
                diag.effector_residuals = effectors
                    .iter()
                    .map(|e| frozen.pose[e.joint].distance(e.target))
                    .collect();
                return (frozen, diag);
            }
            retries += 1;
            substeps *= 2;
        }
    }

    fn any_crossing(&self, before: &Pose, after: &Pose) -> bool {
        self.pairs.iter().any(|&(i, j)| {
            segments_crossed(before, after, &self.caps, i, j, self.config.contact_margin)
        })
    }

    /// Exact postcondition used by the step watchdog.  Starting from an
    /// admissible state, returning the entry state on failure proves that no
    /// accepted step can cross a floor, arena, or capsule-compression bound,
    /// even when the requested constraints are mutually infeasible.
    fn violates_universal_positional_bounds(&self, pose: &Pose) -> bool {
        const EPSILON: f64 = 1e-9;
        let extent = self.config.arena_half_extent;
        if PlayerJoint::all().any(|pj| {
            let point = pose[pj];
            !point.is_finite()
                || point.y < pj.joint.radius() - MAT_COMPRESSION_ALLOWANCE
                || point.x.abs() > extent + EPSILON
                || point.z.abs() > extent + EPSILON
        }) {
            return true;
        }

        self.pairs.iter().enumerate().any(|(pair_index, &(i, j))| {
            let a = &self.caps[i];
            let b = &self.caps[j];
            let (ca, cb, _, _) =
                gm_core::closest_segment_points(pose[a.a()], pose[a.b()], pose[b.a()], pose[b.b()]);
            let clearance = ca.distance(cb) - a.radius - b.radius;
            clearance + FLESH_COMPRESSION_ALLOWANCE < self.clearance_floors[pair_index]
        })
    }

    /// Anatomical postconditions paired with the universal watchdog above.
    /// These allowances are the explicit material/numerical widths used by the
    /// solver contract.  A failed candidate is never partially committed: the
    /// retry loop either finds an admissible projection or returns the prior
    /// admissible state, making the invariant inductive over accepted steps.
    fn violates_anatomical_bounds(&self, pose: &Pose) -> bool {
        if self.bones.iter().any(|bone| {
            let error = pose[bone.a()].distance(pose[bone.b()]) - bone.length;
            error.abs() > BONE_ABSOLUTE_ALLOWANCE
                && error.abs() / bone.length.max(1e-9) > BONE_RELATIVE_ALLOWANCE
        }) || max_hinge_violation(pose) > HINGE_ANGLE_ALLOWANCE
        {
            return true;
        }

        if PlayerId::ALL.into_iter().any(|player| {
            SWING_CONES.iter().any(|cone| {
                gm_core::anatomy::cone_angle(pose, player, cone)
                    > cone.half_angle + SWING_CONE_ALLOWANCE
            })
        }) {
            return true;
        }

        let max_limit = std::f64::consts::PI - 1e-6;
        let swing_limit = self.config.ankle_swing_limit.clamp(0.0, max_limit);
        let twist_limit = self.config.ankle_twist_limit.clamp(0.0, max_limit);
        PlayerId::ALL.into_iter().any(|player| {
            LegSide::ALL
                .into_iter()
                .enumerate()
                .any(|(side_index, side)| {
                    ankle_angles(
                        pose,
                        player,
                        side,
                        &self.ankle_references[player.index()][side_index],
                    )
                    .map(|angles| {
                        angles.swing > swing_limit + ANKLE_ANGLE_ALLOWANCE
                            || angles.twist.abs() > twist_limit + ANKLE_ANGLE_ALLOWANCE
                    })
                    .unwrap_or(true)
                })
        })
    }

    /// A hard ankle correction is admissible only when it preserves every
    /// higher-priority positional invariant.  For an already-valid floor,
    /// arena, or capsule constraint this means remaining valid; for an
    /// inherited violation it means never making the violation deeper.
    ///
    /// Written as a scalar inequality, each post-projection margin `m1` must
    /// satisfy `m1 >= min(m0, 0)`, where `m0` is its pre-projection margin.
    /// Thus accepting a correction is monotone in every universal validity
    /// margin.  Rejecting restores the bit-identical input pose.
    fn ankle_projection_preserves_validity(&self, before: &Pose, after: &Pose) -> bool {
        const EPSILON: f64 = 1e-9;
        let extent = self.config.arena_half_extent;

        for pj in PlayerJoint::all() {
            let floor_before = before[pj].y - pj.joint.radius() + MAT_COMPRESSION_ALLOWANCE;
            let floor_after = after[pj].y - pj.joint.radius() + MAT_COMPRESSION_ALLOWANCE;
            if floor_after + EPSILON < floor_before.min(0.0) {
                return false;
            }

            for (before_axis, after_axis) in
                [(before[pj].x, after[pj].x), (before[pj].z, after[pj].z)]
            {
                let arena_before = extent - before_axis.abs();
                let arena_after = extent - after_axis.abs();
                if arena_after + EPSILON < arena_before.min(0.0) {
                    return false;
                }
            }
        }

        for (pair_index, &(i, j)) in self.pairs.iter().enumerate() {
            let a = &self.caps[i];
            let b = &self.caps[j];
            let clearance = |pose: &Pose| {
                let (ca, cb, _, _) = gm_core::closest_segment_points(
                    pose[a.a()],
                    pose[a.b()],
                    pose[b.a()],
                    pose[b.b()],
                );
                ca.distance(cb) - a.radius - b.radius
            };
            let before_margin =
                clearance(before) - self.clearance_floors[pair_index] + FLESH_COMPRESSION_ALLOWANCE;
            let after_margin =
                clearance(after) - self.clearance_floors[pair_index] + FLESH_COMPRESSION_ALLOWANCE;
            if after_margin + EPSILON < before_margin.min(0.0) {
                return false;
            }
        }
        true
    }

    fn solve_pass(
        &self,
        state: &SolverState,
        effectors: &[Effector],
        dt: f64,
        substeps: usize,
    ) -> (SolverState, StepDiagnostics) {
        let mut st = *state;
        let dt_s = dt / substeps as f64;
        let inv_mass = |pj: PlayerJoint| self.inv_mass(pj);
        let tone_scale = tone_scales(effectors);
        let ankle_driven = driven_ankles(effectors);
        let mut grip_active = grip_mask(&self.grips, effectors, st.broken_grips);
        let grip_strainable = grip_strainable(&self.grips, &self.caps, effectors);

        let mut last_contacts: Vec<Contact> = Vec::new();

        for _ in 0..substeps {
            let entry_pose = st.pose;

            for (i, grip) in self.grips.iter().enumerate() {
                if grip.runtime
                    && st.broken_grips & (1 << i) == 0
                    && st.grip_wrap[i] < 1.0
                {
                    st.grip_wrap[i] =
                        (st.grip_wrap[i] + RUNTIME_WRAP_RATE * dt_s).min(1.0);
                }
            }

            // -- Integrate: velocity (damped) predicts positions.
            let damp = (-self.config.damping * dt_s).exp();
            for pj in PlayerJoint::all() {
                let (pi, ji) = (pj.player.index(), pj.joint.index());
                let mut v = st.velocity[pi][ji] * damp;
                v.y += self.config.gravity * dt_s;
                let v = v.clamped_len(self.config.max_joint_speed);
                st.velocity[pi][ji] = v;
                if self.inv_mass(pj) > 0.0 {
                    st.pose[pj] += v * dt_s;
                }
            }

            // -- Effectors: compliant pull toward targets, speed-limited.
            for e in effectors {
                if self.inv_mass(e.joint) <= 0.0 {
                    continue;
                }
                let stiffness = gm_core::clamp(e.stiffness, 0.0, 1.0);
                let pull = ((e.target - st.pose[e.joint]) * stiffness)
                    .clamped_len(self.config.max_effector_speed * dt_s);
                st.pose[e.joint] += pull;
            }

            // -- Muscle tone: compliant shape matching holds the pose against
            //    gravity and keeps drags local. Weaker than effectors, so input
            //    wins where it acts; plastic adaptation (below) makes sustained
            //    input become the new held pose.
            for player in PlayerId::ALL {
                for (leg_index, side) in LegSide::ALL.into_iter().enumerate() {
                    if ankle_driven[player.index()][leg_index] {
                        st.ankle_release_active[player.index()][leg_index] = true;
                    } else if st.ankle_release_active[player.index()][leg_index] {
                        let settled = ankle_angles(
                            &st.pose,
                            player,
                            side,
                            &self.ankle_references[player.index()][leg_index],
                        )
                        .map(|angles| angles.swing.max(angles.twist.abs()) <= 1.0f64.to_radians())
                        .unwrap_or(false);
                        if settled {
                            st.ankle_release_active[player.index()][leg_index] = false;
                        }
                    }
                }
            }
            let mut substep_tone_scale = tone_scale;
            retain_displaced_ankle_tone(
                &st.pose,
                &self.ankle_references,
                &st.ankle_release_active,
                self.config.ankle_orientation_deadzone,
                &mut substep_tone_scale,
            );
            crate::tone::project_muscle_tone(
                &mut st.pose,
                &st.tone_rest,
                self.config.tone_stiffness,
                &substep_tone_scale,
                &inv_mass,
            );

            // -- Rest-relative ankle tone is orientation-only: it rotates heel
            //    and toe rigidly about the ankle, leaving foot position input
            //    independent. A separate hard envelope below always wins.
            project_soft_ankle_orientations(
                &mut st.pose,
                &self.ankle_references,
                self.config.ankle_orientation_deadzone,
                self.config.ankle_orientation_stiffness,
                &inv_mass,
            );

            // -- Balance servo: keep a standing body's COM at its authored
            //    offset over the feet while the drift is recoverable; give up
            //    beyond the margin (the body falls).
            self.project_balance(&mut st.pose, &entry_pose, &inv_mass);

            // -- Grip wear. For each surviving grip, look at the stretch
            //    demanded of it right after inputs and tone (before projection
            //    reverts it) - that is the load the grip carries this substep.
            //
            //    Engaged grips whose *gripped* limb is user-driven (see
            //    grip_strainable) accumulate that load into a leaky strain
            //    integrator; past GRIP_STRENGTH the grip starts *paying out
            //    tether* (grip_release) - failing as a quick slip rather than
            //    a deleted constraint snapping the stretched limb back. Loads
            //    with no effector behind them (gravity, authored holds) never
            //    strain a grip: they must hold forever.
            //
            //    Suspended grips (own limb user-driven, see grip_mask) don't
            //    resist, but their payout ratchets up to the actual
            //    separation, so if the user stops driving before the release
            //    limit the grip re-holds at the current distance - no snap.
            //
            //    Either way, a grip paid out past GRIP_RELEASE_LIMIT is gone.
            for (i, (g, on)) in self.grips.iter().zip(grip_active.iter_mut()).enumerate() {
                if st.broken_grips & (1 << i) != 0 {
                    continue;
                }
                // A held live grip is an explicit controller command, not a
                // finite authored-pose tether. Once any wrap point captures
                // contact it stays full-strength until button release; load,
                // coverage, and coordinated peeling cannot pay it out.
                if g.runtime {
                    st.grip_strain[i] = 0.0;
                    st.grip_release[i] = 0.0;
                    continue;
                }
                let cap = &self.caps[g.cap];
                let (anchor, _) = gm_core::closest_point_on_segment(
                    st.pose[g.joint],
                    st.pose[cap.a()],
                    st.pose[cap.b()],
                );
                let dist = st.pose[g.joint].distance(anchor);
                let release = &mut st.grip_release[i];
                if *on {
                    let stretch = if grip_strainable[i] {
                        (dist - (g.len + *release)).max(0.0)
                    } else {
                        0.0
                    };
                    let strain = &mut st.grip_strain[i];
                    *strain = (*strain + stretch) * (-GRIP_STRAIN_DECAY * dt_s).exp();
                    let strength = GRIP_STRENGTH;
                    if *strain > strength || *release > GRIP_FAIL_LATCH {
                        *release += GRIP_RELEASE_RATE * dt_s;
                    }
                } else {
                    *release = release.max(dist - g.len);
                }
                if *release > GRIP_RELEASE_LIMIT {
                    st.broken_grips |= 1 << i;
                    *on = false;
                }
            }

            // -- Contacts are gathered once per substep (speculative margin),
            //    with ratchet floors taken from the substep entry pose.
            let contacts = find_contacts(
                &entry_pose,
                &self.caps,
                &self.pairs,
                self.config.contact_margin,
            );

            // -- Constraint iterations.
            for _ in 0..self.config.iterations {
                self.project_bones(&mut st.pose, &inv_mass);
                self.project_runtime_wraps(
                    &mut st.pose,
                    &grip_active,
                    &st.grip_wrap,
                    &st.grip_contact_bits,
                    dt_s / self.config.iterations.max(1) as f64,
                );
                self.project_grips(
                    &mut st.pose,
                    &grip_active,
                    &st.grip_release,
                    &st.grip_contact_bits,
                    &st.grip_hand_anchor_s,
                    &st.grip_hand_anchor_radial,
                    &st.grip_finger_anchor_s,
                    &st.grip_finger_anchor_radial,
                    &inv_mass,
                );
                project_hinge_min_angles(&mut st.pose, &inv_mass);
                project_hyperextension_guards(&mut st, &inv_mass);
                let before_ankle_projection = st.pose;
                project_hard_ankle_envelopes(
                    &mut st.pose,
                    &self.ankle_references,
                    self.config.ankle_swing_limit,
                    self.config.ankle_twist_limit,
                    &inv_mass,
                );
                if st.pose != before_ankle_projection
                    && !self.ankle_projection_preserves_validity(&before_ankle_projection, &st.pose)
                {
                    st.pose = before_ankle_projection;
                }
                project_swing_cones(&mut st.pose, &inv_mass);
                self.project_contacts(&mut st.pose, &contacts, &inv_mass);
                self.project_floor_and_bounds(&mut st.pose, &entry_pose, &inv_mass);
            }
            // -- Friction once per substep, against pressing contacts only.
            self.apply_contact_friction(&mut st.pose, &entry_pose, &contacts, &inv_mass);

            // -- Polish: rigidity is the most visually load-bearing invariant,
            //    so bones get the last word each substep - but alternated with
            //    contacts, or the bone pass would quietly re-penetrate pairs
            //    the iteration loop just separated (tone re-presses them every
            //    substep, so even a millimeter of re-penetration accumulates
            //    into a steady-state violation).
            // Floor/arena are clamped between contacts and bones: the bone
            // pass keeps the last word on rigidity (the visually load-bearing
            // invariant) and re-introduces only a millimeter-scale floor sink
            // at planted feet, which the validator's floor slack absorbs.
            for _ in 0..3 {
                self.project_contacts(&mut st.pose, &contacts, &inv_mass);
                self.clamp_floor_and_arena(&mut st.pose, &inv_mass);
                self.project_bones(&mut st.pose, &inv_mass);
            }
            // Universal positional validity keeps the last word. The ankle
            // envelope was projected in every main iteration above; this
            // existing generic polish only resolves millimeter-scale conflicts
            // with floor, contact, and bone constraints.
            self.clamp_arena(&mut st.pose, &inv_mass);
            self.lift_unpinned_pose_to_mat(&mut st.pose);
            self.update_runtime_grip_quality(&mut st, dt_s, &mut grip_active);

            // -- Velocity update from actual displacement.
            for pj in PlayerJoint::all() {
                let (pi, ji) = (pj.player.index(), pj.joint.index());
                let v = ((st.pose[pj] - entry_pose[pj]) / dt_s)
                    .clamped_len(self.config.max_joint_speed);
                st.velocity[pi][ji] = v;
            }

            // -- Taut grips are inelastic: kill separating relative velocity
            //    along the tether, or every positional snap-back converts into
            //    velocity that the next substep amplifies (an energy pump that
            //    slowly shakes tightly-entangled poses apart).
            self.damp_grip_velocities(
                &st.pose,
                &grip_active,
                &st.grip_release,
                &st.grip_wrap,
                &mut st.velocity,
            );

            // The soft release correction is a controlled angular servo, not
            // momentum to integrate again next substep. Critically damp its
            // heel/toe coordinates after velocity reconstruction so the
            // configured angular rate cap also bounds frame-to-frame release.
            for player in PlayerId::ALL {
                for leg_index in 0..LEG_GROUPS.len() {
                    if st.ankle_release_active[player.index()][leg_index]
                        && !ankle_driven[player.index()][leg_index]
                    {
                        for joint in &LEG_GROUPS[leg_index][3..] {
                            st.velocity[player.index()][joint.index()] = V3::ZERO;
                        }
                    }
                }
            }

            crate::tone::adapt_rest_shape(
                &st.pose,
                &mut st.tone_rest,
                self.config.tone_plasticity,
                self.config.tone_deadzone,
                dt_s,
            );
            refresh_bend_memory(&mut st);
            last_contacts = contacts;
        }

        let diag = StepDiagnostics {
            contact_count: last_contacts.iter().filter(|c| c.clearance < 0.005).count(),
            min_clearance: gm_collision::min_clearance(&st.pose, &self.caps, &self.pairs),
            max_bone_error: self.max_bone_error(&st.pose),
            max_hinge_violation: max_hinge_violation(&st.pose),
            max_writhe_jump: 0.0,
            retries: 0,
            rejected: false,
            effector_residuals: effectors
                .iter()
                .map(|e| st.pose[e.joint].distance(e.target))
                .collect(),
        };
        (st, diag)
    }

    fn project_bones(&self, pose: &mut Pose, inv_mass: &dyn Fn(PlayerJoint) -> f64) {
        for bone in &self.bones {
            let (a, b) = (bone.a(), bone.b());
            let delta = pose[b] - pose[a];
            let len = delta.length();
            if len < 1e-9 {
                continue;
            }
            let w_a = inv_mass(a);
            let w_b = inv_mass(b);
            let w_sum = w_a + w_b;
            if w_sum < 1e-12 {
                continue;
            }
            let corr = delta * ((len - bone.length) / len);
            pose[a] += corr * (w_a / w_sum);
            pose[b] -= corr * (w_b / w_sum);
        }
    }

    fn project_runtime_wraps(
        &self,
        pose: &mut Pose,
        active: &[bool],
        wrap: &[f64; 32],
        contact_bits: &[u8; 32],
        dt: f64,
    ) {
        let max_move = RUNTIME_WRAP_SPEED * dt;
        for (i, (grip, &on)) in self.grips.iter().zip(active).enumerate() {
            if !on || !grip.runtime || contact_bits[i] & 0b11 == 0b11 {
                continue;
            }
            let Some(wrist) = grip.wrist else { continue; };
            let Some(finger) = grip.finger else { continue; };
            let cap = &self.caps[grip.cap];
            let a = pose[cap.a()];
            let b = pose[cap.b()];
            let Some(axis) = (b - a).normalized(1e-9) else { continue; };
            let (wrist_axis, _) = gm_core::closest_point_on_segment(pose[wrist], a, b);
            let wrist_radial = (pose[wrist] - wrist_axis)
                .normalized(1e-9)
                .unwrap_or_else(|| axis.cross(V3::Y).normalized_or_zero());
            if wrist_radial.length_squared() < 1e-12 {
                continue;
            }
            let (hand_axis, _) = gm_core::closest_point_on_segment(pose[grip.joint], a, b);
            let eased = wrap[i] * wrap[i] * (3.0 - 2.0 * wrap[i]);
            let hand_radial =
                Self::rotate_radial(wrist_radial, axis, grip.hand_wrap * eased);
            let finger_radial =
                Self::rotate_radial(wrist_radial, axis, grip.target_wrap * eased);
            let (finger_axis, _) = gm_core::closest_point_on_segment(pose[finger], a, b);
            let hand_target = hand_axis + hand_radial * grip.len;
            let finger_target = finger_axis + finger_radial * grip.finger_len;
            let hand_point = pose[grip.joint];
            let finger_point = pose[finger];

            if contact_bits[i] & 0b01 == 0 {
                pose[grip.joint] += (hand_target - hand_point).clamped_len(max_move);
            }
            if contact_bits[i] & 0b10 == 0 {
                pose[finger] += (finger_target - finger_point).clamped_len(max_move);
            }
        }
    }

    fn rotate_between_axes(value: V3, from: V3, to: V3) -> V3 {
        let cosine = from.dot(to).clamp(-1.0, 1.0);
        let cross = from.cross(to);
        let sine = cross.length();
        if sine < 1e-9 {
            return if cosine >= 0.0 { value } else { -value };
        }
        let axis = cross / sine;
        // Rodrigues rotation by the shortest arc from the previous capsule
        // axis to its current axis.
        value * cosine + axis.cross(value) * sine
            + axis * (axis.dot(value) * (1.0 - cosine))
    }

    fn update_runtime_grip_quality(
        &self,
        state: &mut SolverState,
        _dt: f64,
        active: &mut [bool],
    ) {
        for (i, grip) in self.grips.iter().enumerate() {
            if !grip.runtime || state.broken_grips & (1 << i) != 0 {
                continue;
            }
            let Some(wrist) = grip.wrist else { continue; };
            let Some(finger) = grip.finger else { continue; };
            let cap = &self.caps[grip.cap];
            let gap = |joint: PlayerJoint| {
                let (axis_point, _) = gm_core::closest_point_on_segment(
                    state.pose[joint], state.pose[cap.a()], state.pose[cap.b()]);
                (state.pose[joint].distance(axis_point) - cap.radius - joint.joint.radius()).max(0.0)
            };
            let wrist_gap = gap(wrist);
            let hand_gap = gap(grip.joint);
            let finger_gap = gap(finger);
            if state.grip_contact_bits[i] == 0 && wrist_gap > RUNTIME_WRAP_ABORT_GAP {
                state.broken_grips |= 1 << i;
                active[i] = false;
                continue;
            }

            let cap_a = state.pose[cap.a()];
            let cap_b = state.pose[cap.b()];
            let Some(axis) = (cap_b - cap_a).normalized(1e-9) else { continue; };
            let previous_axis = state.grip_anchor_axis[i];
            if state.grip_contact_bits[i] != 0
                && previous_axis.length_squared() > 1e-12
            {
                state.grip_hand_anchor_radial[i] = Self::rotate_between_axes(
                    state.grip_hand_anchor_radial[i], previous_axis, axis);
                state.grip_finger_anchor_radial[i] = Self::rotate_between_axes(
                    state.grip_finger_anchor_radial[i], previous_axis, axis);
            }
            state.grip_anchor_axis[i] = axis;

            if state.grip_contact_bits[i] & 0b01 == 0
                && hand_gap <= RUNTIME_STICKY_CAPTURE_GAP
            {
                let (anchor, s) = gm_core::closest_point_on_segment(
                    state.pose[grip.joint], cap_a, cap_b);
                state.grip_hand_anchor_s[i] = s;
                state.grip_hand_anchor_radial[i] = state.pose[grip.joint] - anchor;
                state.grip_contact_bits[i] |= 0b01;
            }
            if state.grip_contact_bits[i] & 0b10 == 0
                && finger_gap <= RUNTIME_STICKY_CAPTURE_GAP
            {
                let (anchor, s) = gm_core::closest_point_on_segment(
                    state.pose[finger], cap_a, cap_b);
                state.grip_finger_anchor_s[i] = s;
                state.grip_finger_anchor_radial[i] = state.pose[finger] - anchor;
                state.grip_contact_bits[i] |= 0b10;
            }

            let achieved =
                Self::signed_wrap_angle(&state.pose, cap, state.pose[wrist], state.pose[finger]);
            let raw_coverage =
                (achieved * grip.target_wrap.signum() / grip.target_wrap.abs().max(1e-9))
                    .clamp(0.0, 1.0);
            state.grip_coverage[i] = raw_coverage;
            state.grip_contact[i] =
                state.grip_contact_bits[i].count_ones() as f64 * 0.5;
            state.grip_strength[i] = if state.grip_contact_bits[i] == 0 {
                0.0
            } else {
                RUNTIME_GRIP_STRENGTH
            };
        }
    }

    fn project_grip_point(
        pose: &mut Pose,
        joint: PlayerJoint,
        cap: &CapsuleDef,
        len: f64,
        stiffness: f64,
        inv_mass: &dyn Fn(PlayerJoint) -> f64,
    ) {
        let (ea, eb) = (cap.a(), cap.b());
        let (anchor, s) = gm_core::closest_point_on_segment(pose[joint], pose[ea], pose[eb]);
        let delta = pose[joint] - anchor;
        let dist = delta.length();
        if dist <= len + 1e-9 || dist < 1e-9 {
            return;
        }
        let n = delta / dist;
        let (wj, wa, wb) = (inv_mass(joint), inv_mass(ea), inv_mass(eb));
        let denom = wj + wa * (1.0 - s) * (1.0 - s) + wb * s * s;
        if denom < 1e-12 {
            return;
        }
        let lambda = (dist - len) / denom * stiffness.clamp(0.0, 1.0);
        pose[joint] -= n * (lambda * wj);
        pose[ea] += n * (lambda * wa * (1.0 - s));
        pose[eb] += n * (lambda * wb * s);
    }

    /// Grips as unilateral tethers: a gripping joint may not move farther
    /// from the gripped capsule's *segment* than it was at load (the closest
    /// point is re-evaluated every iteration, so the grip slides freely along
    /// the limb). The correction is shared between the joint and the capsule's
    /// endpoint joints (weighted by closest-point barycentrics and inverse
    /// masses), so a grip genuinely transmits load - a hooked leg lifts the
    /// hooker.
    fn project_grips(
        &self,
        pose: &mut Pose,
        active: &[bool],
        release: &[f64; 32],
        contact_bits: &[u8; 32],
        hand_anchor_s: &[f64; 32],
        hand_anchor_radial: &[V3; 32],
        finger_anchor_s: &[f64; 32],
        finger_anchor_radial: &[V3; 32],
        inv_mass: &dyn Fn(PlayerJoint) -> f64,
    ) {
        for (i, (g, &on)) in self.grips.iter().zip(active).enumerate() {
            if !on {
                continue;
            }
            let cap = &self.caps[g.cap];
            if g.runtime {
                if contact_bits[i] & 0b01 != 0 {
                    Self::project_sticky_grip_point(
                        pose, g.joint, cap, hand_anchor_s[i],
                        hand_anchor_radial[i], inv_mass);
                }
                if contact_bits[i] & 0b10 != 0 {
                    if let Some(finger) = g.finger {
                        Self::project_sticky_grip_point(
                            pose, finger, cap, finger_anchor_s[i],
                            finger_anchor_radial[i], inv_mass);
                    }
                }
                continue;
            }
            Self::project_grip_point(
                pose, g.joint, cap, g.len + release[i], 1.0, inv_mass);
            if let Some(finger) = g.finger {
                Self::project_grip_point(
                    pose,
                    finger,
                    cap,
                    g.finger_len + release[i],
                    1.0,
                    inv_mass,
                );
            }
        }
    }

    fn project_sticky_grip_point(
        pose: &mut Pose,
        joint: PlayerJoint,
        cap: &CapsuleDef,
        s: f64,
        radial: V3,
        inv_mass: &dyn Fn(PlayerJoint) -> f64,
    ) {
        let (ea, eb) = (cap.a(), cap.b());
        let s = s.clamp(0.0, 1.0);
        let target = pose[ea] * (1.0 - s) + pose[eb] * s + radial;
        let error = pose[joint] - target;
        if error.length_squared() < 1e-18 {
            return;
        }
        let (wj, wa, wb) = (inv_mass(joint), inv_mass(ea), inv_mass(eb));
        let denom = wj + wa * (1.0 - s) * (1.0 - s) + wb * s * s;
        if denom < 1e-12 {
            return;
        }
        pose[joint] -= error * (wj / denom);
        pose[ea] += error * (wa * (1.0 - s) / denom);
        pose[eb] += error * (wb * s / denom);
    }

    /// Remove the separating component of relative velocity across each taut
    /// grip (see the call site). Approaching velocity is untouched - grips
    /// only ever resist separation.
    fn damp_grip_point(
        pose: &Pose,
        joint: PlayerJoint,
        cap: &CapsuleDef,
        len: f64,
        velocity: &mut [[V3; gm_core::JOINT_COUNT]; gm_core::PLAYER_COUNT],
    ) {
        let (ea, eb) = (cap.a(), cap.b());
        let (anchor, s) = gm_core::closest_point_on_segment(pose[joint], pose[ea], pose[eb]);
        let delta = pose[joint] - anchor;
        let dist = delta.length();
        if dist < len - 0.005 || dist < 1e-9 {
            return;
        }
        let n = delta / dist;
        let vj = velocity[joint.player.index()][joint.joint.index()];
        let va = velocity[ea.player.index()][ea.joint.index()];
        let vb = velocity[eb.player.index()][eb.joint.index()];
        let v_anchor = va * (1.0 - s) + vb * s;
        let sep = (vj - v_anchor).dot(n);
        if sep <= 0.0 {
            return;
        }
        let mj = joint.joint.mass();
        let ma = ea.joint.mass() * (1.0 - s) + eb.joint.mass() * s;
        let total = mj + ma;
        velocity[joint.player.index()][joint.joint.index()] -= n * (sep * ma / total);
        let up = n * (sep * mj / total);
        velocity[ea.player.index()][ea.joint.index()] += up * (1.0 - s);
        velocity[eb.player.index()][eb.joint.index()] += up * s;
    }

    fn damp_grip_velocities(
        &self,
        pose: &Pose,
        active: &[bool],
        release: &[f64; 32],
        wrap: &[f64; 32],
        velocity: &mut [[V3; gm_core::JOINT_COUNT]; gm_core::PLAYER_COUNT],
    ) {
        for (i, (g, &on)) in self.grips.iter().zip(active).enumerate() {
            if !on || (g.runtime && wrap[i] < 1.0) {
                continue;
            }
            let cap = &self.caps[g.cap];
            Self::damp_grip_point(pose, g.joint, cap, g.len + release[i], velocity);
            if let Some(finger) = g.finger {
                Self::damp_grip_point(
                    pose,
                    finger,
                    cap,
                    g.finger_len + release[i],
                    velocity,
                );
            }
        }
    }

    fn project_contacts(
        &self,
        pose: &mut Pose,
        contacts: &[Contact],
        inv_mass: &dyn Fn(PlayerJoint) -> f64,
    ) {
        for c in contacts {
            let a = &self.caps[c.cap_a];
            let b = &self.caps[c.cap_b];
            let (ca, cb, s, t, clearance) = gm_collision::eval_contact(pose, &self.caps, c);
            let floor = self.clearance_floors[c.pair_idx];
            if clearance >= floor {
                continue;
            }
            let normal = (ca - cb).normalized(1e-9).unwrap_or(c.normal);
            let needed = floor - clearance;

            // Distribute over the four segment endpoints, weighted by the
            // closest-point barycentrics and inverse masses.
            let ends = [
                (a.a(), (1.0 - s), 1.0),
                (a.b(), s, 1.0),
                (b.a(), (1.0 - t), -1.0),
                (b.b(), t, -1.0),
            ];
            let denom: f64 = ends.iter().map(|(pj, w, _)| inv_mass(*pj) * w * w).sum();
            if denom < 1e-12 {
                continue;
            }
            let lambda = needed / denom;
            for (pj, w, side) in ends {
                pose[pj] += normal * (lambda * inv_mass(pj) * w * side);
            }
        }
    }

    /// Position-level contact friction, applied *once per substep* after the
    /// constraint iterations, and only at contacts that are actually pressing
    /// (at/below their ratchet floor). Friction inside the iteration loop, or
    /// at merely-speculative contacts, compounds into a nonphysical glue that
    /// pumps energy into entangled poses.
    fn apply_contact_friction(
        &self,
        pose: &mut Pose,
        entry: &Pose,
        contacts: &[Contact],
        inv_mass: &dyn Fn(PlayerJoint) -> f64,
    ) {
        if self.config.friction <= 0.0 {
            return;
        }
        for c in contacts {
            let a = &self.caps[c.cap_a];
            let b = &self.caps[c.cap_b];
            let (ca, cb, s, t, clearance) = gm_collision::eval_contact(pose, &self.caps, c);
            let floor = self.clearance_floors[c.pair_idx];
            if clearance > floor + 0.002 {
                continue;
            }
            let normal = (ca - cb).normalized(1e-9).unwrap_or(c.normal);
            let disp_a =
                (pose[a.a()] - entry[a.a()]) * (1.0 - s) + (pose[a.b()] - entry[a.b()]) * s;
            let disp_b =
                (pose[b.a()] - entry[b.a()]) * (1.0 - t) + (pose[b.b()] - entry[b.b()]) * t;
            let tangential = (disp_a - disp_b).perp_to(normal);
            let fix = tangential * (self.config.friction * 0.5);
            for (pj, w, side) in [
                (a.a(), 1.0 - s, -1.0),
                (a.b(), s, -1.0),
                (b.a(), 1.0 - t, 1.0),
                (b.b(), t, 1.0),
            ] {
                if inv_mass(pj) > 0.0 {
                    pose[pj] += fix * (w * side);
                }
            }
        }
    }

    /// Active balance (ankle strategy): while a player is standing on its
    /// feet, hold the COM at its *authored* offset from the support center
    /// (captured at load), so authored stances - including deliberate leans
    /// onto the opponent - are preserved while numerical drift is corrected.
    /// Beyond `balance_margin` of drift the servo surrenders and the body
    /// falls. Not standing (kneeling, lying, posted hand, airborne) = no
    /// balance authority at all.
    fn project_balance(
        &self,
        pose: &mut Pose,
        entry: &Pose,
        inv_mass: &dyn Fn(PlayerJoint) -> f64,
    ) {
        if self.config.balance_stiffness <= 0.0 {
            return;
        }
        for player in PlayerId::ALL {
            let Some((center, support_y)) = standing_support(pose, entry, player) else {
                continue;
            };
            let (ref_x, ref_z) = self.balance_ref[player.index()].unwrap_or((0.0, 0.0));
            let com = crate::tone::centroid(pose, player);
            let err = v3(com.x - center.x - ref_x, 0.0, com.z - center.z - ref_z);
            let err_len = err.length();
            let deadzone = 0.02;
            let excess = err_len - deadzone;
            if excess <= 0.0 || excess > self.config.balance_margin {
                continue;
            }
            let correction = err * (excess / err_len);
            let lever_height = (com.y - support_y).max(0.3);
            for j in Joint::ALL {
                let pj = PlayerJoint { player, joint: j };
                if inv_mass(pj) <= 0.0 || entry[pj].y <= j.radius() + 0.01 {
                    continue;
                }
                let lever = gm_core::clamp((pose[pj].y - support_y) / lever_height, 0.0, 1.5);
                pose[pj] -= correction * (self.config.balance_stiffness * lever);
            }
        }
    }

    fn project_floor_and_bounds(
        &self,
        pose: &mut Pose,
        entry: &Pose,
        inv_mass: &dyn Fn(PlayerJoint) -> f64,
    ) {
        let ext = self.config.arena_half_extent;
        for pj in PlayerJoint::all() {
            if inv_mass(pj) <= 0.0 {
                continue;
            }
            let radius = pj.joint.radius();
            let e = entry[pj];
            // A joint that entered the substep resting on the floor stays in
            // frictional contact even while its y sits exactly at the radius;
            // without this, feet skate horizontally and bodies can never stand.
            let in_contact = e.y <= radius + 0.005;
            let p = &mut pose[pj];
            let clamped = p.y < radius;
            if clamped {
                p.y = radius;
            }
            if (in_contact || clamped) && self.config.floor_friction > 0.0 {
                p.x = e.x + (p.x - e.x) * (1.0 - self.config.floor_friction);
                p.z = e.z + (p.z - e.z) * (1.0 - self.config.floor_friction);
            }
            p.x = gm_core::clamp(p.x, -ext, ext);
            p.z = gm_core::clamp(p.z, -ext, ext);
        }
    }

    /// Hard arena x/z bounds only: the absolute invariant pass that runs
    /// after everything else each substep.
    fn clamp_arena(&self, pose: &mut Pose, inv_mass: &dyn Fn(PlayerJoint) -> f64) {
        let ext = self.config.arena_half_extent;
        for pj in PlayerJoint::all() {
            if inv_mass(pj) <= 0.0 {
                continue;
            }
            let p = &mut pose[pj];
            p.x = gm_core::clamp(p.x, -ext, ext);
            p.z = gm_core::clamp(p.z, -ext, ext);
        }
    }

    /// Restore the physical mat-compression bound without perturbing any
    /// relative geometry.  With no application pins, adding one common
    /// vertical displacement to every joint is a rigid translation: all bone
    /// lengths, angles, capsule clearances, and topology observables are
    /// exactly invariant.  If a pin exists, this pass yields to it.
    fn lift_unpinned_pose_to_mat(&self, pose: &mut Pose) {
        if !self.pins.is_empty() {
            return;
        }
        let lift = PlayerJoint::all().fold(0.0f64, |required, pj| {
            required.max(pj.joint.radius() - MAT_COMPRESSION_ALLOWANCE - pose[pj].y)
        });
        if lift <= 0.0 {
            return;
        }
        for pj in PlayerJoint::all() {
            pose[pj].y += lift;
        }
    }

    /// Floor + arena hard clamp without friction (used inside the polish).
    fn clamp_floor_and_arena(&self, pose: &mut Pose, inv_mass: &dyn Fn(PlayerJoint) -> f64) {
        let ext = self.config.arena_half_extent;
        for pj in PlayerJoint::all() {
            if inv_mass(pj) <= 0.0 {
                continue;
            }
            let p = &mut pose[pj];
            p.y = p.y.max(pj.joint.radius());
            p.x = gm_core::clamp(p.x, -ext, ext);
            p.z = gm_core::clamp(p.z, -ext, ext);
        }
    }

    fn max_bone_error(&self, pose: &Pose) -> f64 {
        self.bones
            .iter()
            .map(|b| {
                let d = pose[b.a()].distance(pose[b.b()]);
                ((d - b.length) / b.length.max(1e-9)).abs()
            })
            .fold(0.0, f64::max)
    }
}
