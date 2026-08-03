//! Rest-relative ankle orientation constraints.
//!
//! A point-only shin has no axial orientation, so the shin frame combines the
//! ankle-to-knee axis with the hip-knee-ankle bend-plane normal.  The foot
//! frame combines the heel-to-toe direction with the foot-triangle normal.
//! Both frames are local to the leg; a shared rigid world motion therefore
//! cancels from their relative orientation.
//!
//! When the leg is nearly straight its bend-plane normal is ill-conditioned.
//! In that case the solver uses the bend-plane direction captured at load and
//! transported by the current pelvis frame.  This is an explicit deterministic
//! fallback: a degenerate knee never disables the hard ankle envelope and is
//! never reported as a plausible zero angle.

use gm_core::{clamp, pelvis_frame, v3, Joint, PlayerId, PlayerJoint, Pose, PLAYER_COUNT, V3};

const GEOMETRY_EPSILON: f64 = 1e-9;
/// Rate cap for each soft angular coordinate per substep. Together with rigid
/// foot rotation this bounds release motion even after a target saturates.
const MAX_SOFT_ANGLE_STEP: f64 = 0.02;
/// Below three degrees of knee bend, prefer the captured frame direction to a
/// cross product whose normalization would amplify solver noise.
const MIN_BEND_SINE: f64 = 0.052_335_956_242_943_835;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LegSide {
    Left,
    Right,
}

impl LegSide {
    pub const ALL: [LegSide; 2] = [LegSide::Left, LegSide::Right];

    fn joints(self) -> LegJoints {
        match self {
            LegSide::Left => LegJoints {
                hip: Joint::LeftHip,
                knee: Joint::LeftKnee,
                ankle: Joint::LeftAnkle,
                heel: Joint::LeftHeel,
                toe: Joint::LeftToe,
            },
            LegSide::Right => LegJoints {
                hip: Joint::RightHip,
                knee: Joint::RightKnee,
                ankle: Joint::RightAnkle,
                heel: Joint::RightHeel,
                toe: Joint::RightToe,
            },
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct LegJoints {
    hip: Joint,
    knee: Joint,
    ankle: Joint,
    heel: Joint,
    toe: Joint,
}

#[derive(Debug, Clone, Copy)]
struct Frame {
    /// Hip-knee-ankle plane normal.
    side: V3,
    /// Ankle-to-knee direction.
    up: V3,
    /// Completes the right-handed frame: forward = side x up.
    forward: V3,
}

impl Frame {
    fn to_local(self, world: V3) -> V3 {
        v3(
            world.dot(self.side),
            world.dot(self.up),
            world.dot(self.forward),
        )
    }

    fn to_world(self, local: V3) -> V3 {
        self.side * local.x + self.up * local.y + self.forward * local.z
    }
}

/// Rest orientation and straight-leg fallback for one ankle.
#[derive(Debug, Clone, Copy)]
pub struct AnkleReference {
    foot_forward_local: V3,
    foot_normal_local: V3,
    bend_normal_pelvis_local: V3,
}

/// Observable rest-relative swing/twist coordinates, in radians.
#[derive(Debug, Clone, Copy)]
pub struct AnkleAngles {
    /// Geodesic angle between rest and current heel-to-toe directions.
    pub swing: f64,
    /// Signed foot-plane rotation after the minimum swing has been removed.
    pub twist: f64,
    /// True when the current knee was too straight to define a reliable plane.
    pub used_straight_leg_fallback: bool,
}

pub type AnkleReferences = [[AnkleReference; 2]; PLAYER_COUNT];

fn unit_or(v: V3, fallback: V3) -> V3 {
    v.normalized(GEOMETRY_EPSILON).unwrap_or(fallback)
}

fn initial_shin_frame(pose: &Pose, player: PlayerId, side: LegSide) -> Frame {
    let j = side.joints();
    let pelvis = pelvis_frame(pose, player);
    let up = unit_or(
        pose.get(player, j.knee) - pose.get(player, j.ankle),
        pelvis.up,
    );
    let upper = unit_or(pose.get(player, j.knee) - pose.get(player, j.hip), -up);
    let lower = unit_or(pose.get(player, j.ankle) - pose.get(player, j.knee), -up);
    let bend_normal = upper.cross(lower);
    let fallback = pelvis
        .right
        .perp_to(up)
        .normalized(GEOMETRY_EPSILON)
        .or_else(|| pelvis.forward.perp_to(up).normalized(GEOMETRY_EPSILON))
        .unwrap_or_else(|| v3(1.0, 0.0, 0.0));
    let side_axis = bend_normal.normalized(GEOMETRY_EPSILON).unwrap_or(fallback);
    let forward = unit_or(side_axis.cross(up), pelvis.forward);
    let side_axis = unit_or(up.cross(forward), side_axis);
    Frame {
        side: side_axis,
        up,
        forward,
    }
}

fn shin_frame(
    pose: &Pose,
    player: PlayerId,
    side: LegSide,
    reference: &AnkleReference,
) -> (Frame, bool) {
    let j = side.joints();
    let pelvis = pelvis_frame(pose, player);
    let up = unit_or(
        pose.get(player, j.knee) - pose.get(player, j.ankle),
        pelvis.up,
    );
    let upper = unit_or(pose.get(player, j.knee) - pose.get(player, j.hip), -up);
    let lower = unit_or(pose.get(player, j.ankle) - pose.get(player, j.knee), -up);
    let bend_normal = upper.cross(lower);
    let use_fallback = bend_normal.length() < MIN_BEND_SINE;
    let transported_fallback = pelvis
        .to_world(reference.bend_normal_pelvis_local)
        .perp_to(up)
        .normalized(GEOMETRY_EPSILON)
        .or_else(|| pelvis.right.perp_to(up).normalized(GEOMETRY_EPSILON))
        .or_else(|| pelvis.forward.perp_to(up).normalized(GEOMETRY_EPSILON))
        .unwrap_or_else(|| v3(1.0, 0.0, 0.0));
    let side_axis = if use_fallback {
        transported_fallback
    } else {
        bend_normal / bend_normal.length()
    };
    let forward = unit_or(side_axis.cross(up), pelvis.forward);
    let side_axis = unit_or(up.cross(forward), side_axis);
    (
        Frame {
            side: side_axis,
            up,
            forward,
        },
        use_fallback,
    )
}

fn foot_axes(pose: &Pose, player: PlayerId, side: LegSide) -> Option<(V3, V3)> {
    let j = side.joints();
    let ankle = pose.get(player, j.ankle);
    let heel = pose.get(player, j.heel);
    let toe = pose.get(player, j.toe);
    let forward = (toe - heel).normalized(GEOMETRY_EPSILON)?;
    let normal = forward.cross(ankle - heel).normalized(GEOMETRY_EPSILON)?;
    Some((forward, normal))
}

pub fn capture_ankle_reference(pose: &Pose, player: PlayerId, side: LegSide) -> AnkleReference {
    let frame = initial_shin_frame(pose, player, side);
    let pelvis = pelvis_frame(pose, player);
    let (foot_forward, foot_normal) =
        foot_axes(pose, player, side).unwrap_or((frame.forward, frame.side));
    AnkleReference {
        foot_forward_local: unit_or(frame.to_local(foot_forward), v3(0.0, 0.0, 1.0)),
        foot_normal_local: unit_or(frame.to_local(foot_normal), v3(1.0, 0.0, 0.0)),
        bend_normal_pelvis_local: unit_or(pelvis.to_local(frame.side), v3(1.0, 0.0, 0.0)),
    }
}

pub fn capture_ankle_references(pose: &Pose) -> AnkleReferences {
    PlayerId::ALL.map(|player| LegSide::ALL.map(|side| capture_ankle_reference(pose, player, side)))
}

fn rotate(v: V3, unit_axis: V3, radians: f64) -> V3 {
    let (sin, cos) = radians.sin_cos();
    v * cos + unit_axis.cross(v) * sin + unit_axis * (unit_axis.dot(v) * (1.0 - cos))
}

/// Rotate `v` by the minimum rotation taking unit `from` to unit `to`.
/// At the antipode the minimum path is non-unique; `preferred_axis` selects a
/// deterministic one perpendicular to `from`.
fn transport(v: V3, from: V3, to: V3, preferred_axis: V3) -> V3 {
    let cosine = clamp(from.dot(to), -1.0, 1.0);
    let cross = from.cross(to);
    if let Some(axis) = cross.normalized(GEOMETRY_EPSILON) {
        rotate(v, axis, cosine.acos())
    } else if cosine >= 0.0 {
        v
    } else {
        let axis = preferred_axis
            .perp_to(from)
            .normalized(GEOMETRY_EPSILON)
            .or_else(|| v3(1.0, 0.0, 0.0).perp_to(from).normalized(GEOMETRY_EPSILON))
            .or_else(|| v3(0.0, 1.0, 0.0).perp_to(from).normalized(GEOMETRY_EPSILON))
            .unwrap_or(v3(0.0, 0.0, 1.0));
        rotate(v, axis, std::f64::consts::PI)
    }
}

fn signed_angle(from: V3, to: V3, unit_axis: V3) -> f64 {
    let from = unit_or(from.perp_to(unit_axis), from);
    let to = unit_or(to.perp_to(unit_axis), to);
    unit_axis.dot(from.cross(to)).atan2(from.dot(to))
}

/// Signed heel-to-toe swing about the shin's proximal axis. This is the
/// one-dimensional coordinate used to unwrap a commanded planted-foot orbit;
/// unlike total swing it is undefined when either foot direction is parallel
/// to the shin, and reports `None` rather than a fabricated zero.
pub fn ankle_axial_swing(
    pose: &Pose,
    player: PlayerId,
    side: LegSide,
    reference: &AnkleReference,
) -> Option<f64> {
    let (frame, _) = shin_frame(pose, player, side, reference);
    let (foot_forward, _) = foot_axes(pose, player, side)?;
    let current = frame.to_local(foot_forward);
    let axis = v3(0.0, 1.0, 0.0);
    let from = reference
        .foot_forward_local
        .perp_to(axis)
        .normalized(GEOMETRY_EPSILON)?;
    let to = current.perp_to(axis).normalized(GEOMETRY_EPSILON)?;
    Some(axis.dot(from.cross(to)).atan2(from.dot(to)))
}

pub fn ankle_angles(
    pose: &Pose,
    player: PlayerId,
    side: LegSide,
    reference: &AnkleReference,
) -> Option<AnkleAngles> {
    let (frame, used_straight_leg_fallback) = shin_frame(pose, player, side, reference);
    let (foot_forward, foot_normal) = foot_axes(pose, player, side)?;
    let current_forward = unit_or(frame.to_local(foot_forward), reference.foot_forward_local);
    let current_normal = unit_or(frame.to_local(foot_normal), reference.foot_normal_local);
    let swing = clamp(reference.foot_forward_local.dot(current_forward), -1.0, 1.0).acos();
    let transported_normal = transport(
        reference.foot_normal_local,
        reference.foot_forward_local,
        current_forward,
        reference.foot_normal_local,
    );
    let twist = signed_angle(transported_normal, current_normal, current_forward);
    Some(AnkleAngles {
        swing,
        twist,
        used_straight_leg_fallback,
    })
}

fn swing_direction(reference: &AnkleReference, current_forward: V3, radians: f64) -> V3 {
    if radians <= GEOMETRY_EPSILON {
        return reference.foot_forward_local;
    }
    let cross = reference.foot_forward_local.cross(current_forward);
    let axis = cross
        .normalized(GEOMETRY_EPSILON)
        .unwrap_or(reference.foot_normal_local);
    unit_or(
        rotate(reference.foot_forward_local, axis, radians),
        reference.foot_forward_local,
    )
}

fn project_one(
    pose: &mut Pose,
    player: PlayerId,
    side: LegSide,
    reference: &AnkleReference,
    target_swing: f64,
    target_twist: f64,
    inv_mass: &dyn Fn(PlayerJoint) -> f64,
) {
    let j = side.joints();
    let heel_pj = PlayerJoint {
        player,
        joint: j.heel,
    };
    let toe_pj = PlayerJoint {
        player,
        joint: j.toe,
    };
    // A hard application pin is stronger than anatomy.  Requiring both foot
    // orientation points to be movable avoids violating either pin or foot
    // rigidity.  Pinning only the ankle/shin (the normal test setup) is fine.
    if inv_mass(heel_pj) <= 0.0 || inv_mass(toe_pj) <= 0.0 {
        return;
    }
    let (frame, _) = shin_frame(pose, player, side, reference);
    let Some((current_forward_world, current_normal_world)) = foot_axes(pose, player, side) else {
        return;
    };
    let current_forward_local = unit_or(
        frame.to_local(current_forward_world),
        reference.foot_forward_local,
    );
    let desired_forward_local = swing_direction(reference, current_forward_local, target_swing);
    let transported_normal = transport(
        reference.foot_normal_local,
        reference.foot_forward_local,
        desired_forward_local,
        reference.foot_normal_local,
    );
    let desired_normal_local = unit_or(
        rotate(transported_normal, desired_forward_local, target_twist),
        transported_normal,
    );
    let desired_forward_world =
        unit_or(frame.to_world(desired_forward_local), current_forward_world);
    let desired_normal_world = unit_or(
        frame
            .to_world(desired_normal_local)
            .perp_to(desired_forward_world),
        current_normal_world,
    );

    let current_binormal = unit_or(current_forward_world.cross(current_normal_world), frame.up);
    let desired_binormal = unit_or(desired_forward_world.cross(desired_normal_world), frame.up);
    let map = |v: V3| {
        desired_forward_world * v.dot(current_forward_world)
            + desired_normal_world * v.dot(current_normal_world)
            + desired_binormal * v.dot(current_binormal)
    };
    let ankle = pose.get(player, j.ankle);
    pose[heel_pj] = ankle + map(pose[heel_pj] - ankle);
    pose[toe_pj] = ankle + map(pose[toe_pj] - ankle);
}

fn softened(value: f64, dead_zone: f64, stiffness: f64) -> f64 {
    let magnitude = value.abs();
    if magnitude <= dead_zone {
        value
    } else {
        value.signum() * (magnitude - stiffness * (magnitude - dead_zone))
    }
}

/// Apply one compliant rest-relative correction per substep.  Correction grows
/// linearly outside the neutral zone, while effectors remain free to overcome
/// it until the separate hard envelope is reached.
pub fn project_soft_ankle_orientations(
    pose: &mut Pose,
    references: &AnkleReferences,
    dead_zone: f64,
    stiffness: f64,
    inv_mass: &dyn Fn(PlayerJoint) -> f64,
) {
    let dead_zone = clamp(dead_zone, 0.0, std::f64::consts::PI);
    let stiffness = clamp(stiffness, 0.0, 1.0);
    if stiffness <= 0.0 {
        return;
    }
    for player in PlayerId::ALL {
        for (side_index, side) in LegSide::ALL.into_iter().enumerate() {
            let reference = &references[player.index()][side_index];
            let Some(angles) = ankle_angles(pose, player, side, reference) else {
                continue;
            };
            let requested_swing = softened(angles.swing, dead_zone, stiffness);
            let requested_twist = softened(angles.twist, dead_zone, stiffness);
            let target_swing = angles.swing
                + clamp(
                    requested_swing - angles.swing,
                    -MAX_SOFT_ANGLE_STEP,
                    MAX_SOFT_ANGLE_STEP,
                );
            let target_twist = angles.twist
                + clamp(
                    requested_twist - angles.twist,
                    -MAX_SOFT_ANGLE_STEP,
                    MAX_SOFT_ANGLE_STEP,
                );
            if (target_swing - angles.swing).abs() <= f64::EPSILON
                && (target_twist - angles.twist).abs() <= f64::EPSILON
            {
                continue;
            }
            project_one(
                pose,
                player,
                side,
                reference,
                target_swing,
                target_twist,
                inv_mass,
            );
        }
    }
}

/// Project every foot rigidly onto its hard rest-relative swing/twist envelope.
/// Limits are clamped below pi so a full half-turn is never admissible.
pub fn project_hard_ankle_envelopes(
    pose: &mut Pose,
    references: &AnkleReferences,
    swing_limit: f64,
    twist_limit: f64,
    inv_mass: &dyn Fn(PlayerJoint) -> f64,
) {
    let max_limit = std::f64::consts::PI - 1e-6;
    let swing_limit = clamp(swing_limit, 0.0, max_limit);
    let twist_limit = clamp(twist_limit, 0.0, max_limit);
    for player in PlayerId::ALL {
        for (side_index, side) in LegSide::ALL.into_iter().enumerate() {
            let reference = &references[player.index()][side_index];
            let Some(angles) = ankle_angles(pose, player, side, reference) else {
                continue;
            };
            let target_swing = angles.swing.min(swing_limit);
            let target_twist = clamp(angles.twist, -twist_limit, twist_limit);
            if (target_swing - angles.swing).abs() <= f64::EPSILON
                && (target_twist - angles.twist).abs() <= f64::EPSILON
            {
                continue;
            }
            project_one(
                pose,
                player,
                side,
                reference,
                target_swing,
                target_twist,
                inv_mass,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gm_core::{v3, Joint::*, P0};

    const TOLERANCE: f64 = 1e-9;

    fn standing_pose() -> Pose {
        let mut pose = Pose::default();
        for player in PlayerId::ALL {
            let z = if player == P0 { -0.5 } else { 0.5 };
            pose.set(player, LeftHip, v3(-0.115, 0.95, z));
            pose.set(player, RightHip, v3(0.115, 0.95, z));
            pose.set(player, LeftShoulder, v3(-0.17, 1.48, z));
            pose.set(player, RightShoulder, v3(0.17, 1.48, z));
            pose.set(player, LeftKnee, v3(-0.13, 0.53, z + 0.05));
            pose.set(player, RightKnee, v3(0.13, 0.53, z + 0.05));
            pose.set(player, LeftAnkle, v3(-0.13, 0.11, z));
            pose.set(player, RightAnkle, v3(0.13, 0.11, z));
            pose.set(player, LeftHeel, v3(-0.13, 0.03, z - 0.04));
            pose.set(player, RightHeel, v3(0.13, 0.03, z - 0.04));
            pose.set(player, LeftToe, v3(-0.13, 0.025, z + 0.18));
            pose.set(player, RightToe, v3(0.13, 0.025, z + 0.18));
        }
        pose
    }

    fn rotate_foot(pose: &mut Pose, player: PlayerId, side: LegSide, axis: V3, angle: f64) {
        let j = side.joints();
        let ankle = pose.get(player, j.ankle);
        let axis = axis.normalized(GEOMETRY_EPSILON).unwrap();
        for joint in [j.heel, j.toe] {
            let p = pose.get(player, joint);
            pose.set(player, joint, ankle + rotate(p - ankle, axis, angle));
        }
    }

    fn all_movable(_: PlayerJoint) -> f64 {
        1.0
    }

    #[test]
    fn angles_are_invariant_under_shared_rigid_world_motion() {
        let reference_pose = standing_pose();
        let original_reference = capture_ankle_reference(&reference_pose, P0, LegSide::Left);
        let mut current = reference_pose;
        let axis = (current.get(P0, LeftToe) - current.get(P0, LeftHeel)).normalized_or_zero();
        rotate_foot(&mut current, P0, LegSide::Left, axis, 0.37);
        let expected = ankle_angles(&current, P0, LegSide::Left, &original_reference).unwrap();

        let transform = |p: V3| v3(p.z, p.y, -p.x) + v3(4.0, -2.0, 7.0);
        let mut transformed_reference = reference_pose;
        let mut transformed_current = current;
        for joint in Joint::ALL {
            transformed_reference.set(P0, joint, transform(reference_pose.get(P0, joint)));
            transformed_current.set(P0, joint, transform(current.get(P0, joint)));
        }
        let transformed_ref = capture_ankle_reference(&transformed_reference, P0, LegSide::Left);
        let actual =
            ankle_angles(&transformed_current, P0, LegSide::Left, &transformed_ref).unwrap();
        assert!((actual.swing - expected.swing).abs() < TOLERANCE);
        assert!((actual.twist - expected.twist).abs() < TOLERANCE);
    }

    #[test]
    fn hard_projection_has_identity_boundary_idempotence_and_rigidity() {
        let reference_pose = standing_pose();
        let references = capture_ankle_references(&reference_pose);
        let reference = &references[P0.index()][0];
        let foot_axis = (reference_pose.get(P0, LeftToe) - reference_pose.get(P0, LeftHeel))
            .normalized_or_zero();

        let mut interior = reference_pose;
        rotate_foot(&mut interior, P0, LegSide::Left, foot_axis, 0.2);
        let unchanged = interior;
        project_hard_ankle_envelopes(&mut interior, &references, 1.0, 0.5, &all_movable);
        assert_eq!(
            interior, unchanged,
            "an interior pose must be bit-identical"
        );

        let mut outside = reference_pose;
        rotate_foot(&mut outside, P0, LegSide::Left, foot_axis, 1.2);
        let j = LegSide::Left.joints();
        let before_edges = [
            outside.get(P0, j.heel).distance(outside.get(P0, j.ankle)),
            outside.get(P0, j.toe).distance(outside.get(P0, j.ankle)),
            outside.get(P0, j.toe).distance(outside.get(P0, j.heel)),
        ];
        let ankle_before = outside.get(P0, j.ankle);
        project_hard_ankle_envelopes(&mut outside, &references, 1.0, 0.5, &all_movable);
        let angles = ankle_angles(&outside, P0, LegSide::Left, reference).unwrap();
        assert!(angles.swing <= 1.0 + TOLERANCE);
        assert!((angles.twist.abs() - 0.5).abs() < TOLERANCE);
        let after_edges = [
            outside.get(P0, j.heel).distance(outside.get(P0, j.ankle)),
            outside.get(P0, j.toe).distance(outside.get(P0, j.ankle)),
            outside.get(P0, j.toe).distance(outside.get(P0, j.heel)),
        ];
        for (before, after) in before_edges.into_iter().zip(after_edges) {
            assert!((after - before).abs() < TOLERANCE);
        }
        assert_eq!(outside.get(P0, j.ankle), ankle_before);
        let once = outside;
        project_hard_ankle_envelopes(&mut outside, &references, 1.0, 0.5, &all_movable);
        assert!(outside.max_displacement(&once) < TOLERANCE);
    }

    #[test]
    fn soft_projection_has_a_dead_zone_and_error_proportional_response() {
        let reference_pose = standing_pose();
        let references = capture_ankle_references(&reference_pose);
        let reference = &references[P0.index()][0];
        let foot_axis = (reference_pose.get(P0, LeftToe) - reference_pose.get(P0, LeftHeel))
            .normalized_or_zero();
        let dead_zone = 8.0f64.to_radians();

        let mut interior = reference_pose;
        rotate_foot(
            &mut interior,
            P0,
            LegSide::Left,
            foot_axis,
            5.0f64.to_radians(),
        );
        let unchanged = interior;
        project_soft_ankle_orientations(&mut interior, &references, dead_zone, 0.25, &all_movable);
        assert_eq!(interior, unchanged);

        let mut outside = reference_pose;
        rotate_foot(
            &mut outside,
            P0,
            LegSide::Left,
            foot_axis,
            20.0f64.to_radians(),
        );
        project_soft_ankle_orientations(&mut outside, &references, dead_zone, 0.08, &all_movable);
        let angles = ankle_angles(&outside, P0, LegSide::Left, reference).unwrap();
        // 20 - 0.08 * (20 - 8) = 19.04 degrees, below the rate cap.
        assert!((angles.twist.abs() - 19.04f64.to_radians()).abs() < TOLERANCE);
    }

    #[test]
    fn straight_leg_fallback_is_finite_active_and_symmetric() {
        let reference_pose = standing_pose();
        let references = capture_ankle_references(&reference_pose);
        let mut current = reference_pose;
        for player in PlayerId::ALL {
            for side in LegSide::ALL {
                let j = side.joints();
                let hip = current.get(player, j.hip);
                let ankle = current.get(player, j.ankle);
                current.set(player, j.knee, hip.lerp(ankle, 0.5));
                let foot_axis =
                    (current.get(player, j.toe) - current.get(player, j.heel)).normalized_or_zero();
                rotate_foot(&mut current, player, side, foot_axis, 1.2);
            }
        }
        project_hard_ankle_envelopes(&mut current, &references, 0.9, 0.4, &all_movable);
        assert!(current.is_finite());
        for player in PlayerId::ALL {
            for (side_index, side) in LegSide::ALL.into_iter().enumerate() {
                let angles = ankle_angles(
                    &current,
                    player,
                    side,
                    &references[player.index()][side_index],
                )
                .unwrap();
                assert!(angles.used_straight_leg_fallback);
                assert!(angles.swing <= 0.9 + TOLERANCE);
                assert!(angles.twist.abs() <= 0.4 + TOLERANCE);
            }
        }
    }
}
