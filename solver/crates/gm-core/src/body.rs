//! Body schema: joints, limbs (bones + collision capsules), and limb chains.
//! Ported from `topology-grapplers/src/players.hpp` and `positions.cpp` so the
//! Rust solver agrees exactly with the GrappleMap database conventions.

use serde::{Deserialize, Serialize};

pub const JOINT_COUNT: usize = 23;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum Joint {
    LeftToe = 0,
    RightToe = 1,
    LeftHeel = 2,
    RightHeel = 3,
    LeftAnkle = 4,
    RightAnkle = 5,
    LeftKnee = 6,
    RightKnee = 7,
    LeftHip = 8,
    RightHip = 9,
    LeftShoulder = 10,
    RightShoulder = 11,
    LeftElbow = 12,
    RightElbow = 13,
    LeftWrist = 14,
    RightWrist = 15,
    LeftHand = 16,
    RightHand = 17,
    LeftFingers = 18,
    RightFingers = 19,
    Core = 20,
    Neck = 21,
    Head = 22,
}

pub use Joint::*;

impl Joint {
    pub const ALL: [Joint; JOINT_COUNT] = [
        LeftToe, RightToe, LeftHeel, RightHeel, LeftAnkle, RightAnkle, LeftKnee, RightKnee,
        LeftHip, RightHip, LeftShoulder, RightShoulder, LeftElbow, RightElbow, LeftWrist,
        RightWrist, LeftHand, RightHand, LeftFingers, RightFingers, Core, Neck, Head,
    ];

    pub fn index(self) -> usize {
        self as usize
    }

    pub fn from_index(i: usize) -> Option<Joint> {
        Joint::ALL.get(i).copied()
    }

    pub fn name(self) -> &'static str {
        match self {
            LeftToe => "left toe",
            RightToe => "right toe",
            LeftHeel => "left heel",
            RightHeel => "right heel",
            LeftAnkle => "left ankle",
            RightAnkle => "right ankle",
            LeftKnee => "left knee",
            RightKnee => "right knee",
            LeftHip => "left hip",
            RightHip => "right hip",
            LeftShoulder => "left shoulder",
            RightShoulder => "right shoulder",
            LeftElbow => "left elbow",
            RightElbow => "right elbow",
            LeftWrist => "left wrist",
            RightWrist => "right wrist",
            LeftHand => "left hand",
            RightHand => "right hand",
            LeftFingers => "left fingers",
            RightFingers => "right fingers",
            Core => "core",
            Neck => "neck",
            Head => "head",
        }
    }

    /// Joint display/contact radius in meters (from `positions.cpp` jointDefs).
    pub fn radius(self) -> f64 {
        match self {
            LeftToe | RightToe => 0.025,
            LeftHeel | RightHeel => 0.03,
            LeftAnkle | RightAnkle => 0.03,
            LeftKnee | RightKnee => 0.05,
            LeftHip | RightHip => 0.09,
            LeftShoulder | RightShoulder => 0.08,
            LeftElbow | RightElbow => 0.045,
            LeftWrist | RightWrist => 0.02,
            LeftHand | RightHand => 0.02,
            LeftFingers | RightFingers => 0.02,
            Core => 0.1,
            Neck => 0.05,
            Head => 0.11,
        }
    }

    /// Joint mass in kg (~75 kg adult): each joint carries a share of its
    /// adjacent body segments. The torso is far heavier than the extremities,
    /// so pulling a hand extends the arm instead of towing the whole body.
    pub fn mass(self) -> f64 {
        match self {
            LeftToe | RightToe => 0.3,
            LeftHeel | RightHeel => 0.4,
            LeftAnkle | RightAnkle => 1.0,
            LeftKnee | RightKnee => 3.0,
            LeftHip | RightHip => 7.0,
            LeftShoulder | RightShoulder => 4.0,
            LeftElbow | RightElbow => 1.5,
            LeftWrist | RightWrist => 0.5,
            LeftHand | RightHand => 0.4,
            LeftFingers | RightFingers => 0.2,
            Core => 12.0,
            Neck => 2.0,
            Head => 5.0,
        }
    }

    pub fn mirror(self) -> Joint {
        match self {
            LeftToe => RightToe,
            RightToe => LeftToe,
            LeftHeel => RightHeel,
            RightHeel => LeftHeel,
            LeftAnkle => RightAnkle,
            RightAnkle => LeftAnkle,
            LeftKnee => RightKnee,
            RightKnee => LeftKnee,
            LeftHip => RightHip,
            RightHip => LeftHip,
            LeftShoulder => RightShoulder,
            RightShoulder => LeftShoulder,
            LeftElbow => RightElbow,
            RightElbow => LeftElbow,
            LeftWrist => RightWrist,
            RightWrist => LeftWrist,
            LeftHand => RightHand,
            RightHand => LeftHand,
            LeftFingers => RightFingers,
            RightFingers => LeftFingers,
            j => j,
        }
    }
}

/// A rigid segment between two joints of one player.
/// `length` is the nominal (anatomical) length in meters.
/// `midpoint_radius` is the collision capsule radius where defined by GrappleMap;
/// segments without one still collide using joint radii (see `capsule_radius`).
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Limb {
    pub ends: [Joint; 2],
    pub length: f64,
    pub midpoint_radius: Option<f64>,
    pub visible: bool,
}

/// Full structural limb table from `players.hpp` (order preserved).
pub const LIMBS: [Limb; 27] = [
    Limb { ends: [LeftToe, LeftHeel], length: 0.23, midpoint_radius: None, visible: true },
    Limb { ends: [LeftToe, LeftAnkle], length: 0.18, midpoint_radius: None, visible: false },
    Limb { ends: [LeftHeel, LeftAnkle], length: 0.09, midpoint_radius: None, visible: false },
    Limb { ends: [LeftAnkle, LeftKnee], length: 0.43, midpoint_radius: Some(0.055), visible: true },
    Limb { ends: [LeftKnee, LeftHip], length: 0.43, midpoint_radius: Some(0.085), visible: true },
    Limb { ends: [LeftHip, Core], length: 0.27, midpoint_radius: None, visible: false },
    Limb { ends: [Core, LeftShoulder], length: 0.37, midpoint_radius: None, visible: false },
    Limb { ends: [LeftShoulder, LeftElbow], length: 0.29, midpoint_radius: None, visible: true },
    Limb { ends: [LeftElbow, LeftWrist], length: 0.26, midpoint_radius: Some(0.03), visible: true },
    Limb { ends: [LeftWrist, LeftHand], length: 0.08, midpoint_radius: None, visible: true },
    Limb { ends: [LeftHand, LeftFingers], length: 0.08, midpoint_radius: None, visible: true },
    Limb { ends: [LeftWrist, LeftFingers], length: 0.14, midpoint_radius: None, visible: false },
    Limb { ends: [RightToe, RightHeel], length: 0.23, midpoint_radius: None, visible: true },
    Limb { ends: [RightToe, RightAnkle], length: 0.18, midpoint_radius: None, visible: false },
    Limb { ends: [RightHeel, RightAnkle], length: 0.09, midpoint_radius: None, visible: false },
    Limb { ends: [RightAnkle, RightKnee], length: 0.43, midpoint_radius: Some(0.055), visible: true },
    Limb { ends: [RightKnee, RightHip], length: 0.43, midpoint_radius: Some(0.085), visible: true },
    Limb { ends: [RightHip, Core], length: 0.27, midpoint_radius: None, visible: false },
    Limb { ends: [Core, RightShoulder], length: 0.37, midpoint_radius: None, visible: false },
    Limb { ends: [RightShoulder, RightElbow], length: 0.29, midpoint_radius: None, visible: true },
    Limb { ends: [RightElbow, RightWrist], length: 0.27, midpoint_radius: Some(0.03), visible: true },
    Limb { ends: [RightWrist, RightHand], length: 0.08, midpoint_radius: None, visible: true },
    Limb { ends: [RightHand, RightFingers], length: 0.08, midpoint_radius: None, visible: true },
    Limb { ends: [RightWrist, RightFingers], length: 0.14, midpoint_radius: None, visible: false },
    Limb { ends: [LeftHip, RightHip], length: 0.23, midpoint_radius: None, visible: false },
    Limb { ends: [LeftShoulder, Neck], length: 0.175, midpoint_radius: None, visible: false },
    Limb { ends: [RightShoulder, Neck], length: 0.175, midpoint_radius: None, visible: false },
];

/// Neck-to-head segment, kept separate because players.hpp lists it last.
pub const NECK_HEAD: Limb =
    Limb { ends: [Neck, Head], length: 0.165, midpoint_radius: Some(0.05), visible: true };

/// All structural limbs including neck-head (28 total). Every entry acts as a
/// hard distance constraint; visible ones with radii also act as collision capsules.
pub fn all_limbs() -> impl Iterator<Item = Limb> {
    LIMBS.iter().copied().chain(std::iter::once(NECK_HEAD))
}

/// Collision capsule radius for a limb: explicit midpoint radius when GrappleMap
/// defines one, otherwise the smaller of the two end-joint radii (conservative).
pub fn capsule_radius(limb: &Limb) -> f64 {
    limb.midpoint_radius
        .unwrap_or_else(|| limb.ends[0].radius().min(limb.ends[1].radius()))
}

/// Named limb chain of one player, used for topology (writhe) computation
/// and input-driver semantics. Mirrors `chainDefs` in the JS demo.
#[derive(Debug, Clone, Serialize)]
pub struct Chain {
    pub id: &'static str,
    pub joints: &'static [Joint],
}

pub const CHAINS: [Chain; 5] = [
    Chain { id: "left-arm", joints: &[LeftShoulder, LeftElbow, LeftWrist, LeftHand, LeftFingers] },
    Chain { id: "right-arm", joints: &[RightShoulder, RightElbow, RightWrist, RightHand, RightFingers] },
    Chain { id: "left-leg", joints: &[LeftHip, LeftKnee, LeftAnkle, LeftToe] },
    Chain { id: "right-leg", joints: &[RightHip, RightKnee, RightAnkle, RightToe] },
    Chain { id: "spine", joints: &[Core, Neck, Head] },
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn limb_table_covers_every_joint() {
        let mut seen = [false; JOINT_COUNT];
        for limb in all_limbs() {
            seen[limb.ends[0].index()] = true;
            seen[limb.ends[1].index()] = true;
        }
        assert!(seen.iter().all(|&s| s), "every joint must appear in some limb");
    }

    #[test]
    fn mirror_is_involution() {
        for j in Joint::ALL {
            assert_eq!(j.mirror().mirror(), j);
        }
    }
}
