//! Medial/lateral foot-translation probe for knee-plane stability.
//!
//! Positive target values move the left foot medially (toward pelvis-right);
//! negative values move it laterally. Every row starts from the same settled
//! pose so the CSV records displacement, not sweep history.

use gm_core::{angle_at, pelvis_frame, Joint::*, PlayerJoint, P0};
use gm_solver::{Effector, Solver, SolverConfig, SolverState};
use gm_tests::lower_limb_metrics::{left_knee_plane_angle, signed_angle_difference};

fn degrees(radians: f64) -> f64 {
    radians.to_degrees()
}

fn main() {
    let pose = gm_tests::standing_pose();
    let mut solver = Solver::new(&pose, SolverConfig::default());
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
    for _ in 0..60 {
        state = solver.step(&state, &[], 1.0 / 60.0).0;
    }
    let reference_state = state;
    let reference = state.pose;
    let reference_angle = left_knee_plane_angle(&reference, P0)
        .expect("the standing pose has a well-defined left knee plane");
    let medial = pelvis_frame(&reference, P0).right;

    println!(
        "medial_target_cm,knee_plane_deg,knee_plane_delta_deg,knee_flexion_deg,ankle_cm,knee_cm,hip_cm,core_cm,max_residual_cm,max_bone_error,rejected_frames"
    );
    for medial_cm in (-20..=20).step_by(2) {
        state = reference_state;
        let translation = medial * (medial_cm as f64 / 100.0);
        let effectors = [LeftAnkle, LeftHeel, LeftToe].map(|joint| Effector {
            joint: PlayerJoint { player: P0, joint },
            target: reference.get(P0, joint) + translation,
            stiffness: 0.9,
        });
        let mut last_diag = None;
        let mut rejected_frames = 0;
        for _ in 0..60 {
            let (next, diag) = solver.step(&state, &effectors, 1.0 / 60.0);
            rejected_frames += usize::from(diag.rejected);
            state = next;
            last_diag = Some(diag);
        }
        let diag = last_diag.expect("the probe always advances at least one frame");
        let angle = left_knee_plane_angle(&state.pose, P0)
            .expect("the constrained leg must retain a well-defined knee plane");
        let displacement_cm =
            |joint| 100.0 * state.pose.get(P0, joint).distance(reference.get(P0, joint));
        let knee_flexion = std::f64::consts::PI
            - angle_at(
                state.pose.get(P0, LeftHip),
                state.pose.get(P0, LeftKnee),
                state.pose.get(P0, LeftAnkle),
            );
        let max_residual_cm = 100.0 * diag.effector_residuals.iter().copied().fold(0.0, f64::max);
        println!(
            "{medial_cm},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{max_residual_cm:.4},{:.8},{rejected_frames}",
            degrees(angle),
            degrees(signed_angle_difference(angle, reference_angle)),
            degrees(knee_flexion),
            displacement_cm(LeftAnkle),
            displacement_cm(LeftKnee),
            displacement_cm(LeftHip),
            displacement_cm(Core),
            diag.max_bone_error,
        );
    }
}
