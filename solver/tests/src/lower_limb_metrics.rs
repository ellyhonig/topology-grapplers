//! Geometric measurements shared by the lower-limb diagnostic probes.
//!
//! Angles are signed and returned in radians. Callers get `None` when an axis
//! or plane is degenerate; diagnostics must not turn undefined anatomy into a
//! plausible-looking zero.

use gm_core::{pelvis_frame, Joint::*, PlayerId, Pose, V3};

const GEOMETRY_EPSILON: f64 = 1e-9;

/// Signed angle from `from` to `to` about `axis`, in `(-pi, pi]`.
///
/// Both directions are projected onto the plane normal to the axis first, so
/// components along the axis cannot bias the measurement.
pub fn signed_angle_about_axis(from: V3, to: V3, axis: V3) -> Option<f64> {
    let unit_axis = axis.normalized(GEOMETRY_EPSILON)?;
    let from = from.perp_to(unit_axis).normalized(GEOMETRY_EPSILON)?;
    let to = to.perp_to(unit_axis).normalized(GEOMETRY_EPSILON)?;
    Some(unit_axis.dot(from.cross(to)).atan2(from.dot(to)))
}

/// Shortest signed difference `angle - reference`, in `(-pi, pi]`.
pub fn signed_angle_difference(angle: f64, reference: f64) -> f64 {
    let delta = angle - reference;
    delta.sin().atan2(delta.cos())
}

/// Rotate a point around a line using Rodrigues' formula.
pub fn rotate_about_axis(point: V3, origin: V3, axis: V3, radians: f64) -> Option<V3> {
    let axis = axis.normalized(GEOMETRY_EPSILON)?;
    let relative = point - origin;
    let (sin, cos) = radians.sin_cos();
    Some(
        origin
            + relative * cos
            + axis.cross(relative) * sin
            + axis * (axis.dot(relative) * (1.0 - cos)),
    )
}

/// Longitudinal heel-to-toe direction of the foot.
pub fn foot_forward(pose: &Pose, player: PlayerId) -> Option<V3> {
    (pose.get(player, LeftToe) - pose.get(player, LeftHeel)).normalized(GEOMETRY_EPSILON)
}

fn foot_plane_normal(pose: &Pose, player: PlayerId) -> Option<V3> {
    let heel = pose.get(player, LeftHeel);
    let ankle = pose.get(player, LeftAnkle);
    let toe = pose.get(player, LeftToe);
    (ankle - heel)
        .cross(toe - heel)
        .normalized(GEOMETRY_EPSILON)
}

fn leg_plane_normal(pose: &Pose, player: PlayerId) -> Option<V3> {
    let hip = pose.get(player, LeftHip);
    let knee = pose.get(player, LeftKnee);
    let ankle = pose.get(player, LeftAnkle);
    (knee - hip)
        .cross(ankle - knee)
        .normalized(GEOMETRY_EPSILON)
}

/// Signed azimuth of the left knee's bend direction relative to pelvis
/// forward, measured about the hip-to-ankle axis.
///
/// Zero means the knee bends toward pelvis-forward. Positive angles follow the
/// right-hand rule about the hip-to-ankle axis.
pub fn left_knee_plane_angle(pose: &Pose, player: PlayerId) -> Option<f64> {
    let hip = pose.get(player, LeftHip);
    let knee = pose.get(player, LeftKnee);
    let ankle = pose.get(player, LeftAnkle);
    let leg_axis = (ankle - hip).normalized(GEOMETRY_EPSILON)?;
    let bend = (knee - hip).perp_to(leg_axis);
    signed_angle_about_axis(pelvis_frame(pose, player).forward, bend, leg_axis)
}

/// Signed roll of the foot triangle relative to the hip-knee-ankle plane,
/// measured about the foot's heel-to-toe direction.
///
/// The leg-plane normal supplies the rotational reference that a point-only
/// shin segment cannot supply on its own.
pub fn left_foot_to_shin_roll(pose: &Pose, player: PlayerId) -> Option<f64> {
    signed_angle_about_axis(
        leg_plane_normal(pose, player)?,
        foot_plane_normal(pose, player)?,
        foot_forward(pose, player)?,
    )
}

/// World-space roll of the current foot triangle from a reference pose about
/// the reference foot's forward axis.
pub fn left_foot_roll_from(reference: &Pose, current: &Pose, player: PlayerId) -> Option<f64> {
    signed_angle_about_axis(
        foot_plane_normal(reference, player)?,
        foot_plane_normal(current, player)?,
        foot_forward(reference, player)?,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use gm_core::{v3, PlayerJoint, P0};

    const TOLERANCE: f64 = 1e-10;

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < TOLERANCE,
            "expected {expected}, got {actual}"
        );
    }

    #[test]
    fn signed_angle_recovers_direction_and_wraps_differences() {
        let x = v3(1.0, 0.0, 0.0);
        let y = v3(0.0, 1.0, 0.0);
        let z = v3(0.0, 0.0, 1.0);
        assert_close(
            signed_angle_about_axis(x, y, z).unwrap(),
            std::f64::consts::FRAC_PI_2,
        );
        assert_close(
            signed_angle_about_axis(y, x, z).unwrap(),
            -std::f64::consts::FRAC_PI_2,
        );
        assert_close(
            signed_angle_difference((-179.0f64).to_radians(), 179.0f64.to_radians()),
            2.0f64.to_radians(),
        );
    }

    #[test]
    fn rodrigues_rotation_is_rigid_and_keeps_axis_points_fixed() {
        let pose = crate::standing_pose();
        let ankle = pose.get(P0, LeftAnkle);
        let heel = pose.get(P0, LeftHeel);
        let toe = pose.get(P0, LeftToe);
        let axis = foot_forward(&pose, P0).unwrap();
        let angle = 31.0f64.to_radians();
        let rotated_ankle = rotate_about_axis(ankle, ankle, axis, angle).unwrap();
        let rotated_heel = rotate_about_axis(heel, ankle, axis, angle).unwrap();
        let rotated_toe = rotate_about_axis(toe, ankle, axis, angle).unwrap();

        assert!(rotated_ankle.distance(ankle) < TOLERANCE);
        assert_close(rotated_heel.distance(rotated_toe), heel.distance(toe));
        assert_close(rotated_heel.distance(rotated_ankle), heel.distance(ankle));
        assert_close(rotated_toe.distance(rotated_ankle), toe.distance(ankle));
    }

    #[test]
    fn knee_plane_angle_is_invariant_under_rigid_world_motion() {
        let pose = crate::standing_pose();
        let expected = left_knee_plane_angle(&pose, P0).unwrap();
        let translation = v3(4.0, -2.0, 7.0);
        let mut transformed = pose;
        // A proper 90-degree rotation about world Y, followed by translation.
        for joint in gm_core::Joint::ALL {
            let p = pose.get(P0, joint);
            transformed[PlayerJoint { player: P0, joint }] = v3(p.z, p.y, -p.x) + translation;
        }
        assert_close(left_knee_plane_angle(&transformed, P0).unwrap(), expected);
    }

    #[test]
    fn foot_to_shin_metric_recovers_a_known_pure_roll() {
        let reference = crate::standing_pose();
        let baseline = left_foot_to_shin_roll(&reference, P0).unwrap();
        let ankle = reference.get(P0, LeftAnkle);
        let axis = foot_forward(&reference, P0).unwrap();
        let commanded = 23.0f64.to_radians();
        let mut rolled = reference;
        for joint in [LeftHeel, LeftToe] {
            rolled.set(
                P0,
                joint,
                rotate_about_axis(reference.get(P0, joint), ankle, axis, commanded).unwrap(),
            );
        }

        assert_close(
            left_foot_roll_from(&reference, &rolled, P0).unwrap(),
            commanded,
        );
        assert_close(
            signed_angle_difference(left_foot_to_shin_roll(&rolled, P0).unwrap(), baseline),
            commanded,
        );
    }

    #[test]
    fn degenerate_geometry_is_reported_as_undefined() {
        let pose = Pose::default();
        assert!(left_knee_plane_angle(&pose, P0).is_none());
        assert!(left_foot_to_shin_roll(&pose, P0).is_none());
        assert!(rotate_about_axis(V3::ZERO, V3::ZERO, V3::ZERO, 1.0).is_none());
    }
}
