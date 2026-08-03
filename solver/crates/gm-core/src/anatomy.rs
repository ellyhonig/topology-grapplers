//! Anatomical limit schema: hinge angle ranges and swing cones.
//!
//! Limits are expressed positionally (angles between joint triples, and limb
//! directions relative to torso/pelvis frames) because the pose model is pure
//! particles. Ranges were seeded from biomechanics norms and then widened just
//! enough that every pose in GrappleMap.txt validates (see the calibration
//! test in gm-tests); real grappling reaches near-extreme ranges, so database
//! calibration is the authoritative source of truth.

use crate::body::Joint::{self, *};
use serde::Serialize;

/// Angle range at `mid`, measured between `root` and `tip`, in radians in [0, pi].
/// pi = fully straight. `max` below pi prevents hyperextension.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Hinge {
    pub id: &'static str,
    pub root: Joint,
    pub mid: Joint,
    pub tip: Joint,
    pub min: f64,
    pub max: f64,
}

/// Swing cone: the direction `root -> limb` must stay within `half_angle` of a
/// reference direction expressed in the player's torso (or pelvis) frame.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct SwingCone {
    pub id: &'static str,
    pub root: Joint,
    pub limb: Joint,
    /// Reference direction in torso-local coordinates (x=right, y=up, z=forward).
    pub local_ref: [f64; 3],
    pub half_angle: f64,
    /// Use the pelvis frame instead of the full torso frame.
    pub pelvis: bool,
    /// Joints carried along when the limb direction is clamped.
    pub downstream: &'static [Joint],
}

/// Hinge limits, calibrated against all 8,323 database frames (observed minima
/// padded down slightly; see tests/examples/calibrate.rs for the mining tool).
///
/// `max` is exactly pi for elbows/knees: the database contains fully straight
/// limbs, and in a particle model an angle can never exceed pi. Hyperextension
/// (folding past straight in the wrong direction) is therefore prevented by the
/// solver's bend-direction hysteresis constraint, not by the max angle.
pub const HINGES: [Hinge; 10] = [
    Hinge { id: "left-elbow", root: LeftShoulder, mid: LeftElbow, tip: LeftWrist, min: 0.15, max: std::f64::consts::PI },
    Hinge { id: "right-elbow", root: RightShoulder, mid: RightElbow, tip: RightWrist, min: 0.15, max: std::f64::consts::PI },
    Hinge { id: "left-knee", root: LeftHip, mid: LeftKnee, tip: LeftAnkle, min: 0.19, max: std::f64::consts::PI },
    Hinge { id: "right-knee", root: RightHip, mid: RightKnee, tip: RightAnkle, min: 0.19, max: std::f64::consts::PI },
    Hinge { id: "left-wrist", root: LeftElbow, mid: LeftWrist, tip: LeftHand, min: 0.12, max: std::f64::consts::PI },
    Hinge { id: "right-wrist", root: RightElbow, mid: RightWrist, tip: RightHand, min: 0.12, max: std::f64::consts::PI },
    Hinge { id: "left-hand", root: LeftWrist, mid: LeftHand, tip: LeftFingers, min: 1.25, max: std::f64::consts::PI },
    Hinge { id: "right-hand", root: RightWrist, mid: RightHand, tip: RightFingers, min: 1.25, max: std::f64::consts::PI },
    Hinge { id: "neck", root: Core, mid: Neck, tip: Head, min: 1.35, max: std::f64::consts::PI },
    Hinge { id: "spine", root: LeftHip, mid: Core, tip: Neck, min: 1.30, max: std::f64::consts::PI },
];

/// Swing cones, calibrated against the database (see gm-tests).
///
/// Shoulder and hip cones were deliberately dropped: database mining shows real
/// grappling reaches the full sphere at those joints (observed max swing >3.0 rad
/// against any fixed torso-local reference), so a cone there would reject genuine
/// positions. Limb anatomy at shoulders/hips is instead carried by hinge minima,
/// bone lengths, bend-direction hysteresis, and collision.
pub const SWING_CONES: [SwingCone; 2] = [
    // Head direction relative to torso up (observed max 2.21).
    SwingCone {
        id: "neck",
        root: Neck,
        limb: Head,
        local_ref: [0.0, 1.0, 0.0],
        half_angle: 2.30,
        pelvis: false,
        downstream: &[],
    },
    // Spine coherence: hips-center -> neck stays close to the torso up axis
    // (observed max 0.23; this is the tightest anatomical invariant in the DB).
    SwingCone {
        id: "core",
        root: Core,
        limb: Neck,
        local_ref: [0.0, 1.0, 0.0],
        half_angle: 0.32,
        pelvis: true,
        downstream: &[],
    },
];

use crate::frame::{pelvis_frame, torso_frame};
use crate::math::{v3, V3};
use crate::pose::{PlayerId, Pose};

/// The cone's apex point. The "core" spine cone measures from the hips center
/// (that is the invariant mined from the database); all others from the root joint.
pub fn cone_apex(pose: &Pose, player: PlayerId, cone: &SwingCone) -> V3 {
    if cone.id == "core" {
        (pose.get(player, LeftHip) + pose.get(player, RightHip)) * 0.5
    } else {
        pose.get(player, cone.root)
    }
}

/// World-space reference axis of the cone for this pose.
pub fn cone_world_ref(pose: &Pose, player: PlayerId, cone: &SwingCone) -> V3 {
    let frame = if cone.pelvis { pelvis_frame(pose, player) } else { torso_frame(pose, player) };
    frame
        .to_world(v3(cone.local_ref[0], cone.local_ref[1], cone.local_ref[2]))
        .normalized_or_zero()
}

/// Angle between the limb direction and the cone axis, in radians.
/// Returns 0 for degenerate (zero-length) limb directions.
pub fn cone_angle(pose: &Pose, player: PlayerId, cone: &SwingCone) -> f64 {
    let apex = cone_apex(pose, player, cone);
    let dir = pose.get(player, cone.limb) - apex;
    let axis = cone_world_ref(pose, player, cone);
    match dir.normalized(1e-9) {
        Some(d) => crate::math::clamp(d.dot(axis), -1.0, 1.0).acos(),
        None => 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hinge_ranges_are_sane() {
        for h in HINGES {
            assert!(h.min >= 0.0 && h.min < h.max && h.max <= std::f64::consts::PI + 1e-9, "{}", h.id);
        }
    }

    #[test]
    fn cone_refs_are_nonzero() {
        for c in SWING_CONES {
            let [x, y, z] = c.local_ref;
            assert!((x * x + y * y + z * z).sqrt() > 0.5, "{}", c.id);
            assert!(c.half_angle > 0.0 && c.half_angle < std::f64::consts::PI, "{}", c.id);
        }
    }
}
