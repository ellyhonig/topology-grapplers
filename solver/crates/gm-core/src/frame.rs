//! Torso reference frames derived from the hip/shoulder quad.
//! Joint-limit cones and bend-plane references are expressed in these frames so
//! they follow the body through arbitrary world orientations (inverted, twisted, etc).

use crate::body::Joint::*;
use crate::math::{v3, V3};
use crate::pose::{PlayerId, Pose};

/// Right-handed orthonormal frame attached to a player's torso.
#[derive(Debug, Clone, Copy)]
pub struct TorsoFrame {
    pub origin: V3,
    /// Left-to-right axis (toward the player's right side).
    pub right: V3,
    /// Hips-to-shoulders axis.
    pub up: V3,
    /// Chest-facing direction (right-handed: forward = right x up).
    pub forward: V3,
}

impl TorsoFrame {
    /// Express a world direction in torso coordinates.
    pub fn to_local(&self, world: V3) -> V3 {
        v3(world.dot(self.right), world.dot(self.up), world.dot(self.forward))
    }

    /// Express a torso-local direction in world coordinates.
    pub fn to_world(&self, local: V3) -> V3 {
        self.right * local.x + self.up * local.y + self.forward * local.z
    }
}

/// Compute the torso frame from hips and shoulders. Total function: degenerate
/// configurations (which bone-length constraints prevent anyway) fall back to
/// world axes rather than returning NaN.
pub fn torso_frame(pose: &Pose, player: PlayerId) -> TorsoFrame {
    let lh = pose.get(player, LeftHip);
    let rh = pose.get(player, RightHip);
    let ls = pose.get(player, LeftShoulder);
    let rs = pose.get(player, RightShoulder);

    let hips = (lh + rh) * 0.5;
    let shoulders = (ls + rs) * 0.5;
    let origin = (hips + shoulders) * 0.5;

    let up = (shoulders - hips).normalized(1e-9).unwrap_or(V3::Y);
    let right_raw = (rh - lh) + (rs - ls);
    // Orthonormalize right against up.
    let right = right_raw
        .perp_to(up)
        .normalized(1e-9)
        .or_else(|| v3(1.0, 0.0, 0.0).perp_to(up).normalized(1e-9))
        .unwrap_or(v3(1.0, 0.0, 0.0));
    let forward = right.cross(up);

    TorsoFrame { origin, right, up, forward }
}

/// Pelvis frame: like the torso frame but anchored at the hips, used for leg cones.
pub fn pelvis_frame(pose: &Pose, player: PlayerId) -> TorsoFrame {
    let mut f = torso_frame(pose, player);
    let lh = pose.get(player, LeftHip);
    let rh = pose.get(player, RightHip);
    f.origin = (lh + rh) * 0.5;
    f
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pose::P0;

    #[test]
    fn upright_pose_gives_world_aligned_frame() {
        let mut pose = Pose::default();
        pose.set(P0, LeftHip, v3(-0.1, 1.0, 0.0));
        pose.set(P0, RightHip, v3(0.1, 1.0, 0.0));
        pose.set(P0, LeftShoulder, v3(-0.15, 1.5, 0.0));
        pose.set(P0, RightShoulder, v3(0.15, 1.5, 0.0));
        let f = torso_frame(&pose, P0);
        assert!((f.up - V3::Y).length() < 1e-9);
        assert!((f.right - v3(1.0, 0.0, 0.0)).length() < 1e-9);
        assert!((f.forward - v3(0.0, 0.0, 1.0)).length() < 1e-9);
        // Round trip local<->world.
        let d = v3(0.3, -0.4, 0.5);
        assert!((f.to_world(f.to_local(d)) - d).length() < 1e-9);
    }

    #[test]
    fn degenerate_pose_still_orthonormal() {
        let pose = Pose::default(); // all joints at origin
        let f = torso_frame(&pose, P0);
        assert!(f.right.is_finite() && f.up.is_finite() && f.forward.is_finite());
        assert!((f.right.length() - 1.0).abs() < 1e-9);
        assert!((f.up.length() - 1.0).abs() < 1e-9);
        assert!(f.right.dot(f.up).abs() < 1e-9);
    }
}
