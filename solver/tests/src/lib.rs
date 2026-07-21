//! Integration test crate. Shared helpers for loading the real database and
//! building test poses live here; the tests themselves are in `tests/`.

use gm_core::{v3, Joint::*, PlayerJoint, Pose, P0};
use std::path::PathBuf;

pub mod lower_limb_metrics;

/// Path to the canonical GrappleMap.txt at the repository root.
pub fn database_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../GrappleMap.txt")
}

pub fn load_database() -> Vec<gm_core::DbEntry> {
    let text = std::fs::read_to_string(database_path()).expect("GrappleMap.txt must exist");
    gm_core::parse_database(&text).expect("database must parse")
}

/// Two humans standing ~upright facing each other, one meter apart, with
/// near-nominal limb lengths. Used as a neutral start for synthetic tests.
pub fn standing_pose() -> Pose {
    let mut pose = Pose::default();
    for player in gm_core::PlayerId::ALL {
        let zoff = if player == P0 { -0.5 } else { 0.5 };
        let flip = if player == P0 { 1.0 } else { -1.0 };
        let mut set = |j, x: f64, y: f64, z: f64| pose.set(player, j, v3(x, y, z * flip + zoff));
        set(LeftHip, -0.115, 0.95, 0.0);
        set(RightHip, 0.115, 0.95, 0.0);
        set(Core, 0.0, 1.15, 0.0);
        set(LeftShoulder, -0.17, 1.48, 0.0);
        set(RightShoulder, 0.17, 1.48, 0.0);
        set(Neck, 0.0, 1.55, 0.0);
        set(Head, 0.0, 1.71, 0.0);
        set(LeftKnee, -0.13, 0.53, 0.05);
        set(RightKnee, 0.13, 0.53, 0.05);
        set(LeftAnkle, -0.13, 0.11, 0.0);
        set(RightAnkle, 0.13, 0.11, 0.0);
        set(LeftHeel, -0.13, 0.03, -0.04);
        set(RightHeel, 0.13, 0.03, -0.04);
        set(LeftToe, -0.13, 0.025, 0.18);
        set(RightToe, 0.13, 0.025, 0.18);
        set(LeftElbow, -0.22, 1.20, 0.05);
        set(RightElbow, 0.22, 1.20, 0.05);
        set(LeftWrist, -0.25, 0.95, 0.10);
        set(RightWrist, 0.25, 0.95, 0.10);
        set(LeftHand, -0.26, 0.88, 0.12);
        set(RightHand, 0.26, 0.88, 0.12);
        set(LeftFingers, -0.27, 0.81, 0.14);
        set(RightFingers, 0.27, 0.81, 0.14);
    }
    pose
}

/// Assert a full validation pass for a state produced by the solver, using
/// penetration floors captured from the *source* pose the solver was loaded with.
pub fn assert_valid(
    pose: &Pose,
    solver: &gm_solver::Solver,
    floors: &[f64],
    context: &str,
) {
    let report = gm_validate::validate_pose(
        pose,
        solver.bones(),
        solver.capsules(),
        solver.pairs(),
        floors,
        &gm_validate::Tolerances::default(),
    );
    assert!(
        report.is_valid(),
        "{}: {} violations, first: {:?}",
        context,
        report.violations.len(),
        report.violations.first()
    );
}

/// Deterministic xorshift PRNG so fuzz streams are reproducible across runs
/// and platforms without pulling RNG crates into non-test code.
pub struct XorShift(pub u64);

impl XorShift {
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    /// Uniform in [lo, hi).
    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        let u = (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64;
        lo + u * (hi - lo)
    }

    pub fn joint(&mut self) -> PlayerJoint {
        PlayerJoint::from_flat((self.next_u64() % 46) as usize).unwrap()
    }
}
