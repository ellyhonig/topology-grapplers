//! gm-solver: the XPBD constraint solver that owns all motion.
//!
//! Input (VR 6DOF, GUI drags, tests) enters exclusively as compliant
//! [`Effector`]s; hard constraints (bones, anatomical limits, contacts, floor)
//! are projected afterward each substep, so no input can produce an invalid
//! pose. A topology watchdog re-solves or rejects steps that would change limb
//! entanglement by tunneling.

pub mod anatomy_constraints;
pub mod config;
pub mod effector;
pub mod solver;
pub mod state;
pub mod tone;

pub use config::SolverConfig;
pub use effector::Effector;
pub use solver::{Solver, StepDiagnostics};
pub use state::SolverState;

#[cfg(test)]
mod tests {
    use super::*;
    use gm_core::{v3, Joint::*, PlayerJoint, Pose, P0, P1};

    /// Two humans standing ~upright, one meter apart, with nominal limb lengths.
    fn standing_pose() -> Pose {
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

    #[test]
    fn supported_idle_pose_stands_under_gravity() {
        let pose = standing_pose();
        let solver = Solver::new(&pose, SolverConfig::default());
        let mut state = SolverState::from_pose(pose);
        for _ in 0..240 {
            let (next, diag) = solver.step(&state, &[], 1.0 / 60.0);
            assert!(!diag.rejected);
            state = next;
        }
        // Gravity sags the pose and the balance servo settles the COM over the
        // support, but muscle tone must keep both bodies standing: no
        // crumpling, heads stay up.
        assert!(
            state.pose.max_displacement(&pose) < 0.25,
            "idle drift {} m",
            state.pose.max_displacement(&pose)
        );
        for player in gm_core::PlayerId::ALL {
            let head_y = state.pose.get(player, Head).y;
            assert!(head_y > 1.5, "player {:?} slumped: head at {}", player, head_y);
        }
    }

    #[test]
    fn unsupported_leaning_body_topples() {
        // The solver is anchored to the upright pose (that is what was
        // "authored"); the body is then tilted during play.
        let solver = Solver::new(&standing_pose(), SolverConfig::default());
        let mut pose = standing_pose();
        // Tilt p0 backward (away from the opponent) 25 degrees about the line
        // through its ankles: center of mass well outside the foot support.
        let pivot = (pose.get(P0, LeftAnkle) + pose.get(P0, RightAnkle)) * 0.5;
        let ang = -0.45f64;
        let (sin, cos) = ang.sin_cos();
        for j in gm_core::Joint::ALL {
            let p = pose.get(P0, j) - pivot;
            pose.set(P0, j, pivot + v3(p.x, p.y * cos - p.z * sin, p.y * sin + p.z * cos));
        }
        let mut state = SolverState::from_pose(pose);
        for _ in 0..300 {
            let (next, _) = solver.step(&state, &[], 1.0 / 60.0);
            state = next;
        }
        // The body must have fallen over, not held an impossible lean.
        let head_y = state.pose.get(P0, Head).y;
        assert!(head_y < 1.0, "leaning body should topple: head still at {}", head_y);
        // The supported opponent must still be standing.
        assert!(state.pose.get(P1, Head).y > 1.5);
    }

    #[test]
    fn arm_drag_stays_local() {
        let pose = standing_pose();
        let solver = Solver::new(&pose, SolverConfig::default());
        let mut state = SolverState::from_pose(pose);
        // Drag the right hand well out to the side, within reach.
        let shoulder = pose.get(P0, RightShoulder);
        let target = shoulder + v3(0.55, -0.15, 0.0);
        let e = Effector {
            joint: PlayerJoint { player: P0, joint: RightHand },
            target,
            stiffness: 0.9,
        };
        for _ in 0..180 {
            let (next, _) = solver.step(&state, &[e], 1.0 / 60.0);
            state = next;
        }
        let hand_moved = state.pose.get(P0, RightHand).distance(pose.get(P0, RightHand));
        let core_moved = state.pose.get(P0, Core).distance(pose.get(P0, Core));
        assert!(hand_moved > 0.3, "hand only moved {} m", hand_moved);
        assert!(
            core_moved < 0.12,
            "dragging an arm dragged the body: core moved {} m",
            core_moved
        );
    }

    #[test]
    fn effector_moves_hand_without_breaking_bones() {
        let pose = standing_pose();
        let solver = Solver::new(&pose, SolverConfig::default());
        let mut state = SolverState::from_pose(pose);
        // Clearly reachable: up-forward of the right shoulder, away from the chest.
        let target = v3(0.5, 1.65, -0.15);
        let e = Effector {
            joint: PlayerJoint { player: P0, joint: RightHand },
            target,
            stiffness: 0.8,
        };
        // Reaching against muscle tone converges via plastic adaptation, which
        // takes a couple of seconds of sustained input.
        for _ in 0..240 {
            let (next, diag) = solver.step(&state, &[e], 1.0 / 60.0);
            assert!(diag.max_bone_error < 0.05, "bone error {}", diag.max_bone_error);
            state = next;
        }
        let residual = state.pose.get(P0, RightHand).distance(target);
        assert!(residual < 0.1, "hand residual {} m", residual);
    }

    #[test]
    fn teleporting_effector_cannot_explode_pose() {
        let pose = standing_pose();
        let solver = Solver::new(&pose, SolverConfig::default());
        let mut state = SolverState::from_pose(pose);
        // Adversarial: target teleports wildly every frame.
        for i in 0..120 {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            let e = Effector {
                joint: PlayerJoint { player: P1, joint: LeftHand },
                target: v3(50.0 * sign, -30.0, 40.0 * sign),
                stiffness: 1.0,
            };
            let (next, diag) = solver.step(&state, &[e], 1.0 / 60.0);
            assert!(next.pose.is_finite());
            assert!(diag.max_bone_error < 0.05);
            state = next;
        }
        // Everything still on/above the floor and inside the arena.
        for pj in PlayerJoint::all() {
            let p = state.pose[pj];
            assert!(p.y >= 0.0 && p.x.abs() <= 2.0 + 1e-9 && p.z.abs() <= 2.0 + 1e-9);
        }
    }
}
