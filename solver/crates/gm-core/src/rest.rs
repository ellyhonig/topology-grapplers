//! Rest lengths: the per-pose bone lengths the solver preserves as hard constraints.
//!
//! Database poses were authored with a soft spring (see positions.cpp), so their
//! actual segment lengths deviate slightly from the nominal limb table. To stay
//! faithful to loaded poses we capture rest lengths from the pose itself and treat
//! *those* as the hard constraint targets, while the validator separately checks
//! that rest lengths stay within a calibrated tolerance of the nominal lengths.

use crate::body::{all_limbs, Joint};
use crate::pose::{PlayerId, PlayerJoint, Pose};
use serde::Serialize;

#[derive(Debug, Clone, Copy, Serialize)]
pub struct Bone {
    pub player: PlayerId,
    pub ends: [Joint; 2],
    /// Rest length captured from the source pose (meters).
    pub length: f64,
    /// Nominal anatomical length from the limb table (meters).
    pub nominal: f64,
}

impl Bone {
    pub fn a(&self) -> PlayerJoint {
        PlayerJoint { player: self.player, joint: self.ends[0] }
    }

    pub fn b(&self) -> PlayerJoint {
        PlayerJoint { player: self.player, joint: self.ends[1] }
    }
}

/// Capture the full bone list (both players, all structural limbs) from a pose.
pub fn capture_bones(pose: &Pose) -> Vec<Bone> {
    let mut bones = Vec::with_capacity(2 * 28);
    for player in PlayerId::ALL {
        for limb in all_limbs() {
            let a = pose.get(player, limb.ends[0]);
            let b = pose.get(player, limb.ends[1]);
            bones.push(Bone {
                player,
                ends: limb.ends,
                length: a.distance(b),
                nominal: limb.length,
            });
        }
    }
    bones
}

/// Worst relative deviation of captured rest lengths from nominal lengths.
pub fn max_nominal_deviation(bones: &[Bone]) -> f64 {
    bones
        .iter()
        .map(|b| ((b.length - b.nominal) / b.nominal).abs())
        .fold(0.0, f64::max)
}
