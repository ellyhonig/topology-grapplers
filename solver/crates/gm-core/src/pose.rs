//! Two-player pose: 23 joints per player as absolute world-space points,
//! exactly matching the GrappleMap database representation.

use crate::body::{Joint, JOINT_COUNT};
use crate::math::V3;
use serde::{Deserialize, Serialize};
use std::ops::{Index, IndexMut};

pub const PLAYER_COUNT: usize = 2;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PlayerId(pub u8);

pub const P0: PlayerId = PlayerId(0);
pub const P1: PlayerId = PlayerId(1);

impl PlayerId {
    pub const ALL: [PlayerId; PLAYER_COUNT] = [P0, P1];

    pub fn index(self) -> usize {
        self.0 as usize
    }

    pub fn opponent(self) -> PlayerId {
        PlayerId(1 - self.0)
    }
}

/// A joint of a specific player; the fundamental particle index of the solver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PlayerJoint {
    pub player: PlayerId,
    pub joint: Joint,
}

pub const PARTICLE_COUNT: usize = PLAYER_COUNT * JOINT_COUNT;

impl PlayerJoint {
    /// Flat particle index in [0, 46).
    pub fn flat(self) -> usize {
        self.player.index() * JOINT_COUNT + self.joint.index()
    }

    pub fn from_flat(i: usize) -> Option<PlayerJoint> {
        if i >= PARTICLE_COUNT {
            return None;
        }
        Some(PlayerJoint {
            player: PlayerId((i / JOINT_COUNT) as u8),
            joint: Joint::from_index(i % JOINT_COUNT)?,
        })
    }

    pub fn all() -> impl Iterator<Item = PlayerJoint> {
        (0..PARTICLE_COUNT).map(|i| PlayerJoint::from_flat(i).unwrap())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub joints: [[V3; JOINT_COUNT]; PLAYER_COUNT],
}

impl Default for Pose {
    fn default() -> Self {
        Pose { joints: [[V3::ZERO; JOINT_COUNT]; PLAYER_COUNT] }
    }
}

impl Index<PlayerJoint> for Pose {
    type Output = V3;
    fn index(&self, pj: PlayerJoint) -> &V3 {
        &self.joints[pj.player.index()][pj.joint.index()]
    }
}

impl IndexMut<PlayerJoint> for Pose {
    fn index_mut(&mut self, pj: PlayerJoint) -> &mut V3 {
        &mut self.joints[pj.player.index()][pj.joint.index()]
    }
}

impl Pose {
    pub fn get(&self, player: PlayerId, joint: Joint) -> V3 {
        self.joints[player.index()][joint.index()]
    }

    pub fn set(&mut self, player: PlayerId, joint: Joint, v: V3) {
        self.joints[player.index()][joint.index()] = v;
    }

    pub fn is_finite(&self) -> bool {
        self.joints.iter().flatten().all(|v| v.is_finite())
    }

    /// Largest joint displacement between two poses (meters).
    pub fn max_displacement(&self, other: &Pose) -> f64 {
        PlayerJoint::all()
            .map(|pj| self[pj].distance(other[pj]))
            .fold(0.0, f64::max)
    }

    /// Flatten to [p0j0x, p0j0y, p0j0z, p0j1x, ...] (player 0 then player 1).
    pub fn to_flat(&self) -> Vec<f64> {
        let mut out = Vec::with_capacity(PARTICLE_COUNT * 3);
        for player in &self.joints {
            for v in player {
                out.extend_from_slice(&[v.x, v.y, v.z]);
            }
        }
        out
    }

    pub fn from_flat(data: &[f64]) -> Option<Pose> {
        if data.len() != PARTICLE_COUNT * 3 {
            return None;
        }
        let mut pose = Pose::default();
        for (i, chunk) in data.chunks_exact(3).enumerate() {
            let pj = PlayerJoint::from_flat(i)?;
            pose[pj] = V3 { x: chunk[0], y: chunk[1], z: chunk[2] };
        }
        Some(pose)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::v3;

    #[test]
    fn flat_round_trip() {
        let mut pose = Pose::default();
        for (i, pj) in PlayerJoint::all().enumerate() {
            pose[pj] = v3(i as f64, i as f64 * 0.5, -(i as f64));
        }
        let flat = pose.to_flat();
        assert_eq!(flat.len(), PARTICLE_COUNT * 3);
        assert_eq!(Pose::from_flat(&flat).unwrap(), pose);
    }

    #[test]
    fn flat_indices_are_bijective() {
        for i in 0..PARTICLE_COUNT {
            assert_eq!(PlayerJoint::from_flat(i).unwrap().flat(), i);
        }
    }
}
