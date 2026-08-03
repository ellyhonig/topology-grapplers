//! Translation probe for the first lower-limb retention intervention.
//!
//! Holds ankle/heel/toe targets successively farther toward the head and prints
//! the resulting target residual and proximal motion. Keep this deterministic
//! so before/after solver revisions can be compared by diffing the CSV output.

use gm_core::{Joint::*, PlayerJoint, P0};
use gm_solver::{Effector, Solver, SolverConfig, SolverState};

fn main() {
    let pose = gm_tests::standing_pose();
    let config = SolverConfig::default();
    let mut solver = Solver::new(&pose, config);
    // Anchor the pelvis so this measures leg deformation, not whole-body
    // translation or the standing-balance controller's recovery strategy.
    solver.set_pins(vec![
        PlayerJoint {
            player: P0,
            joint: LeftHip,
        },
        PlayerJoint {
            player: P0,
            joint: RightHip,
        },
        PlayerJoint {
            player: P0,
            joint: Core,
        },
    ]);
    let mut state = SolverState::from_pose(pose);

    // Establish a zero-input reference before engaging the tracker.
    for _ in 0..60 {
        state = solver.step(&state, &[], 1.0 / 60.0).0;
    }
    let reference_state = state;
    let reference = state.pose;
    let ankle = PlayerJoint {
        player: P0,
        joint: LeftAnkle,
    };
    let foot = [LeftAnkle, LeftHeel, LeftToe];
    let toward_head = (reference.get(P0, Head) - reference[ankle]).normalized_or_zero();

    println!("target_cm,residual_cm,ankle_cm,knee_cm,hip_cm,core_cm,max_bone_error,rejected");
    for target_cm in (0..=20).step_by(2) {
        // Each row starts from the same settled pose, so it measures target
        // distance rather than the history accumulated by previous rows.
        state = reference_state;
        let translation = toward_head * (target_cm as f64 / 100.0);
        let effectors = foot.map(|joint| Effector {
            joint: PlayerJoint { player: P0, joint },
            target: reference.get(P0, joint) + translation,
            stiffness: 0.9,
        });
        let mut last_diag = None;
        for _ in 0..60 {
            let (next, diag) = solver.step(&state, &effectors, 1.0 / 60.0);
            state = next;
            last_diag = Some(diag);
        }
        let diag = last_diag.expect("the probe always advances at least one frame");
        let displacement_cm =
            |joint| 100.0 * state.pose.get(P0, joint).distance(reference.get(P0, joint));
        println!(
            "{target_cm},{:.4},{:.4},{:.4},{:.4},{:.4},{:.8},{}",
            100.0 * diag.effector_residuals[0],
            displacement_cm(LeftAnkle),
            displacement_cm(LeftKnee),
            displacement_cm(LeftHip),
            displacement_cm(Core),
            diag.max_bone_error,
            diag.rejected,
        );
    }
}
