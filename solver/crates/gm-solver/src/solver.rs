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
//!   6. swing cones (hard)
//!   7. capsule contacts with ratchet floors + friction (hard)
//!   8. floor and arena bounds (hard)
//!
//! After the whole step, a topology watchdog checks for segment crossings and
//! writhe jumps; a dirty step is re-solved from the entry state with doubled
//! substeps, and if it still crosses, the step is rejected (state unchanged).

use gm_collision::{capsules, collidable_pairs, find_contacts, segments_crossed, CapsuleDef, Contact};
use gm_core::{capture_bones, v3, Bone, Joint, PlayerId, PlayerJoint, Pose, V3};
use serde::Serialize;

use crate::anatomy_constraints::{
    max_hinge_violation, project_hinge_min_angles, project_hyperextension_guards,
    project_swing_cones, refresh_bend_memory,
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
        &[Joint::LeftElbow, Joint::LeftWrist, Joint::LeftHand, Joint::LeftFingers],
        Some(Joint::LeftShoulder),
    ),
    (
        &[Joint::RightElbow, Joint::RightWrist, Joint::RightHand, Joint::RightFingers],
        Some(Joint::RightShoulder),
    ),
    (
        &[Joint::LeftKnee, Joint::LeftAnkle, Joint::LeftHeel, Joint::LeftToe],
        Some(Joint::LeftHip),
    ),
    (
        &[Joint::RightKnee, Joint::RightAnkle, Joint::RightHeel, Joint::RightToe],
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

/// Grips have finite strength against *deliberate* pulls. While the gripped
/// limb is user-driven, each substep's attempted stretch beyond the tether
/// length (meters) is added to a leaky per-grip strain accumulator; past this
/// threshold the grip starts to fail (pay out). A sustained effector yank
/// plateaus above 0.4 and crosses this in well under a second; incidental
/// brushes of the effector against the tether stay below it.
const GRIP_STRENGTH: f64 = 0.3;

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
        Joint::LeftHand | Joint::LeftFingers => {
            &[Joint::RightElbow, Joint::RightWrist, Joint::RightHand, Joint::RightFingers]
        }
        Joint::RightHand | Joint::RightFingers => {
            &[Joint::LeftElbow, Joint::LeftWrist, Joint::LeftHand, Joint::LeftFingers]
        }
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
            let toe = pose[PlayerJoint { player, joint: foot[0] }];
            let heel = pose[PlayerJoint { player, joint: foot[1] }];
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
            !effectors.iter().any(|e| {
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
            effectors.iter().any(|e| {
                e.joint.player == cap.player
                    && LIMB_GROUPS.iter().any(|(group, root)| {
                        let in_group = |j: Joint| group.contains(&j) || *root == Some(j);
                        in_group(e.joint.joint) && cap.ends.iter().any(|&end| in_group(end))
                    })
            })
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
        Solver {
            config,
            bones: capture_bones(pose),
            caps,
            pairs,
            clearance_floors,
            pins: Vec::new(),
            balance_ref,
            grips,
        }
    }

    /// Release all grips captured at load (e.g. when the user wants the
    /// players to disengage).
    pub fn release_grips(&mut self) {
        self.grips.clear();
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

            let dirty = crossed || linking.max_writhe_jump > gm_topology::WRITHE_JUMP_THRESHOLD;
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
        self.pairs.iter().enumerate().any(|(pair_idx, &(i, j))| {
            // Pairs authored in deep contact are legitimately interpenetrated;
            // orientation flips there are sliding, not tunneling.
            let (ra, rb) = (self.caps[i].radius, self.caps[j].radius);
            if self.clearance_floors[pair_idx] < -0.25 * (ra + rb) {
                return false;
            }
            segments_crossed(before, after, &self.caps, i, j, self.config.contact_margin)
        })
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
        let mut grip_active = grip_mask(&self.grips, effectors, st.broken_grips);
        let grip_strainable = grip_strainable(&self.grips, &self.caps, effectors);

        let mut last_contacts: Vec<Contact> = Vec::new();

        for _ in 0..substeps {
            let entry_pose = st.pose;

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
            crate::tone::project_muscle_tone(
                &mut st.pose,
                &st.tone_rest,
                self.config.tone_stiffness,
                &tone_scale,
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
                    if *strain > GRIP_STRENGTH || *release > GRIP_FAIL_LATCH {
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
            let contacts =
                find_contacts(&entry_pose, &self.caps, &self.pairs, self.config.contact_margin);

            // -- Constraint iterations.
            for _ in 0..self.config.iterations {
                self.project_bones(&mut st.pose, &inv_mass);
                self.project_grips(&mut st.pose, &grip_active, &st.grip_release, &inv_mass);
                project_hinge_min_angles(&mut st.pose, &inv_mass);
                project_hyperextension_guards(&mut st, &inv_mass);
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
            self.clamp_arena(&mut st.pose, &inv_mass);

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
            self.damp_grip_velocities(&st.pose, &grip_active, &st.grip_release, &mut st.velocity);

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
        inv_mass: &dyn Fn(PlayerJoint) -> f64,
    ) {
        for (i, (g, &on)) in self.grips.iter().zip(active).enumerate() {
            if !on {
                continue;
            }
            let len = g.len + release[i];
            let cap = &self.caps[g.cap];
            let (ea, eb) = (cap.a(), cap.b());
            let (anchor, s) =
                gm_core::closest_point_on_segment(pose[g.joint], pose[ea], pose[eb]);
            let delta = pose[g.joint] - anchor;
            let dist = delta.length();
            if dist <= len + 1e-9 || dist < 1e-9 {
                continue;
            }
            let n = delta / dist;
            let (wj, wa, wb) = (inv_mass(g.joint), inv_mass(ea), inv_mass(eb));
            let denom = wj + wa * (1.0 - s) * (1.0 - s) + wb * s * s;
            if denom < 1e-12 {
                continue;
            }
            let lambda = (dist - len) / denom;
            pose[g.joint] -= n * (lambda * wj);
            pose[ea] += n * (lambda * wa * (1.0 - s));
            pose[eb] += n * (lambda * wb * s);
        }
    }

    /// Remove the separating component of relative velocity across each taut
    /// grip (see the call site). Approaching velocity is untouched - grips
    /// only ever resist separation.
    fn damp_grip_velocities(
        &self,
        pose: &Pose,
        active: &[bool],
        release: &[f64; 32],
        velocity: &mut [[V3; gm_core::JOINT_COUNT]; gm_core::PLAYER_COUNT],
    ) {
        for (i, (g, &on)) in self.grips.iter().zip(active).enumerate() {
            if !on {
                continue;
            }
            let cap = &self.caps[g.cap];
            let (ea, eb) = (cap.a(), cap.b());
            let (anchor, s) =
                gm_core::closest_point_on_segment(pose[g.joint], pose[ea], pose[eb]);
            let delta = pose[g.joint] - anchor;
            let dist = delta.length();
            if dist < g.len + release[i] - 0.005 || dist < 1e-9 {
                continue;
            }
            let n = delta / dist;
            let vj = velocity[g.joint.player.index()][g.joint.joint.index()];
            let va = velocity[ea.player.index()][ea.joint.index()];
            let vb = velocity[eb.player.index()][eb.joint.index()];
            let v_anchor = va * (1.0 - s) + vb * s;
            let sep = (vj - v_anchor).dot(n);
            if sep <= 0.0 {
                continue;
            }
            // Split the impulse between the joint and the anchor by mass.
            let mj = g.joint.joint.mass();
            let ma = ea.joint.mass() * (1.0 - s) + eb.joint.mass() * s;
            let total = mj + ma;
            velocity[g.joint.player.index()][g.joint.joint.index()] -=
                n * (sep * ma / total);
            let up = n * (sep * mj / total);
            velocity[ea.player.index()][ea.joint.index()] += up * (1.0 - s);
            velocity[eb.player.index()][eb.joint.index()] += up * s;
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
