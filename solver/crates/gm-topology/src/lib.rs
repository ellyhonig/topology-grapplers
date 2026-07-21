//! gm-topology: writhe matrices and topology coordinates (Ho & Komura,
//! "Character Motion Synthesis by Topology Coordinates", Eurographics 2009),
//! plus the linking watchdog that detects illegitimate entanglement changes
//! between solver steps.
//!
//! In the redesigned architecture this crate is *analysis*, not the motion
//! driver: the XPBD solver owns motion, and topology quantities are used to
//! (a) report entanglement state to the game layer and (b) veto steps whose
//! writhe jumps discontinuously (strands passing through each other).

use gm_core::{Chain, PlayerId, PlayerJoint, Pose, V3, CHAINS};
use serde::Serialize;

/// A chain instance: a chain definition bound to a player.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct PlayerChain {
    pub player: PlayerId,
    pub chain: &'static Chain,
}

/// All 10 player-chains (5 per player), in deterministic order.
pub fn player_chains() -> Vec<PlayerChain> {
    let mut out = Vec::with_capacity(10);
    for player in PlayerId::ALL {
        for chain in CHAINS.iter() {
            out.push(PlayerChain { player, chain });
        }
    }
    out
}

fn chain_points(pose: &Pose, pc: &PlayerChain) -> Vec<V3> {
    pc.chain
        .joints
        .iter()
        .map(|&j| pose[PlayerJoint { player: pc.player, joint: j }])
        .collect()
}

fn normal_or_zero(a: V3, b: V3) -> V3 {
    a.cross(b).normalized(1e-9).unwrap_or(V3::ZERO)
}

fn safe_asin(x: f64) -> f64 {
    gm_core::clamp(x, -1.0, 1.0).asin()
}

/// Gauss-linking-integral contribution of one segment pair (writhe).
pub fn segment_writhe(a0: V3, a1: V3, b0: V3, b1: V3) -> f64 {
    let rac = b0 - a0;
    let rad = b1 - a0;
    let rbd = b1 - a1;
    let rbc = b0 - a1;
    let na = normal_or_zero(rac, rad);
    let nb = normal_or_zero(rad, rbd);
    let nc = normal_or_zero(rbd, rbc);
    let nd = normal_or_zero(rbc, rac);
    let omega =
        safe_asin(na.dot(nb)) + safe_asin(nb.dot(nc)) + safe_asin(nc.dot(nd)) + safe_asin(nd.dot(na));
    omega / (4.0 * std::f64::consts::PI)
}

/// Writhe matrix between two chains: entry (i, j) is the writhe contribution of
/// segment i of chain A against segment j of chain B.
pub fn writhe_matrix(pose: &Pose, a: &PlayerChain, b: &PlayerChain) -> Vec<Vec<f64>> {
    let pa = chain_points(pose, a);
    let pb = chain_points(pose, b);
    let mut m = vec![vec![0.0; pb.len() - 1]; pa.len() - 1];
    for i in 0..pa.len() - 1 {
        for j in 0..pb.len() - 1 {
            m[i][j] = segment_writhe(pa[i], pa[i + 1], pb[j], pb[j + 1]);
        }
    }
    m
}

pub fn matrix_sum(m: &[Vec<f64>]) -> f64 {
    m.iter().flatten().sum()
}

/// Topology coordinates of a writhe matrix: total writhe, weighted centers along
/// each chain in [-1, 1], and density (principal axis angle of the writhe mass).
#[derive(Debug, Clone, Copy, Serialize)]
pub struct TopologyCoordinates {
    pub writhe: f64,
    pub center_a: f64,
    pub center_b: f64,
    pub density: f64,
}

pub fn topology_coordinates(m: &[Vec<f64>]) -> TopologyCoordinates {
    let rows = m.len();
    let cols = if rows > 0 { m[0].len() } else { 0 };
    let w = matrix_sum(m);
    let mut weighted = 0.0;
    let mut x = 0.0;
    let mut y = 0.0;
    let mut points = Vec::with_capacity(rows * cols);

    for (i, row) in m.iter().enumerate() {
        for (j, &v) in row.iter().enumerate() {
            let weight = v.abs();
            let nx = if rows == 1 { 0.0 } else { (i as f64 / (rows as f64 - 1.0)) * 2.0 - 1.0 };
            let ny = if cols == 1 { 0.0 } else { (j as f64 / (cols as f64 - 1.0)) * 2.0 - 1.0 };
            weighted += weight;
            x += nx * weight;
            y += ny * weight;
            points.push((nx, ny, weight));
        }
    }

    if weighted > 0.0 {
        x /= weighted;
        y /= weighted;
    }

    let (mut xx, mut xy, mut yy) = (0.0, 0.0, 0.0);
    for (px, py, pw) in points {
        let dx = px - x;
        let dy = py - y;
        xx += pw * dx * dx;
        xy += pw * dx * dy;
        yy += pw * dy * dy;
    }

    let principal = 0.5 * (2.0 * xy).atan2(xx - yy);
    let density = gm_core::clamp(
        principal - std::f64::consts::FRAC_PI_4,
        -std::f64::consts::FRAC_PI_4,
        std::f64::consts::FRAC_PI_4,
    );

    TopologyCoordinates { writhe: w, center_a: x, center_b: y, density }
}

/// Total writhe for every ordered chain pair (i < j) across both players.
/// Deterministic ordering: pairs of `player_chains()` indices.
pub fn all_pair_writhes(pose: &Pose) -> Vec<f64> {
    let chains = player_chains();
    let mut out = Vec::with_capacity(chains.len() * (chains.len() - 1) / 2);
    for i in 0..chains.len() {
        for j in (i + 1)..chains.len() {
            out.push(matrix_sum(&writhe_matrix(pose, &chains[i], &chains[j])));
        }
    }
    out
}

/// Writhe-jump watchdog report.
#[derive(Debug, Clone, Serialize)]
pub struct LinkingReport {
    /// Largest |delta total writhe| over all chain pairs.
    pub max_writhe_jump: f64,
    /// Chain-pair index (into the `all_pair_writhes` ordering) of the largest jump.
    pub worst_pair: usize,
}

/// A strand passing through another changes a pair's total writhe by ~1 near the
/// crossing point; legitimate motion changes it smoothly. Any single-step jump
/// above this threshold is topologically suspicious and should be re-solved
/// with smaller substeps.
pub const WRITHE_JUMP_THRESHOLD: f64 = 0.35;

/// Compare pair writhes before/after a step.
pub fn linking_report(before: &[f64], after: &[f64]) -> LinkingReport {
    debug_assert_eq!(before.len(), after.len());
    let mut max_jump = 0.0;
    let mut worst = 0;
    for (idx, (b, a)) in before.iter().zip(after.iter()).enumerate() {
        let jump = (a - b).abs();
        if jump > max_jump {
            max_jump = jump;
            worst = idx;
        }
    }
    LinkingReport { max_writhe_jump: max_jump, worst_pair: worst }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gm_core::{v3, Joint::*, P0, P1};

    /// Build a pose where p0's left arm is straight along +x and p1's left arm
    /// wraps around it (half-turn helix) vs. lying far away.
    fn arm_pose(wrapped: bool) -> Pose {
        let mut pose = Pose::default();
        for pj in PlayerJoint::all() {
            pose[pj] = v3(pj.flat() as f64 * 3.0 + 20.0, 10.0, 50.0);
        }
        // p0 left arm along x at height 1.
        let arm0 = [LeftShoulder, LeftElbow, LeftWrist, LeftHand, LeftFingers];
        for (k, j) in arm0.iter().enumerate() {
            pose.set(P0, *j, v3(k as f64 * 0.2, 1.0, 0.0));
        }
        // p1 left arm: helix around p0's arm axis (wrapped) or offset straight line.
        let arm1 = [LeftShoulder, LeftElbow, LeftWrist, LeftHand, LeftFingers];
        for (k, j) in arm1.iter().enumerate() {
            let t = k as f64 / 4.0;
            if wrapped {
                let angle = t * std::f64::consts::PI * 2.0;
                pose.set(
                    P1,
                    *j,
                    v3(0.1 + t * 0.6, 1.0 + 0.15 * angle.cos(), 0.15 * angle.sin()),
                );
            } else {
                pose.set(P1, *j, v3(0.1 + t * 0.6, 2.0, 0.5));
            }
        }
        pose
    }

    #[test]
    fn wrapped_arms_have_higher_writhe_than_separated() {
        let chains = player_chains();
        let a = chains.iter().find(|c| c.player == P0 && c.chain.id == "left-arm").unwrap();
        let b = chains.iter().find(|c| c.player == P1 && c.chain.id == "left-arm").unwrap();
        let w_wrapped = matrix_sum(&writhe_matrix(&arm_pose(true), a, b)).abs();
        let w_apart = matrix_sum(&writhe_matrix(&arm_pose(false), a, b)).abs();
        assert!(w_wrapped > 0.5, "wrapped writhe {}", w_wrapped);
        assert!(w_apart < 0.1, "separated writhe {}", w_apart);
    }

    #[test]
    fn linking_report_flags_jump() {
        let before = all_pair_writhes(&arm_pose(false));
        let after = all_pair_writhes(&arm_pose(true));
        let report = linking_report(&before, &after);
        assert!(report.max_writhe_jump > WRITHE_JUMP_THRESHOLD);
    }

    #[test]
    fn topology_coordinates_of_zero_matrix_are_zero() {
        let m = vec![vec![0.0; 4]; 4];
        let tc = topology_coordinates(&m);
        assert_eq!(tc.writhe, 0.0);
        assert_eq!(tc.center_a, 0.0);
        assert_eq!(tc.center_b, 0.0);
    }
}
