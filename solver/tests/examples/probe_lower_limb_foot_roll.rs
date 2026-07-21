//! Fixed-ankle foot-roll probe for the underconstrained ankle-to-shin joint.
//!
//! The ankle target stays at its settled reference position. Heel and toe
//! targets undergo the same rigid rotation about the heel-to-toe direction
//! through that ankle, so the command changes orientation without changing any
//! target edge length in the foot triangle.

use gm_core::{Joint::*, PlayerJoint, P0};
use gm_solver::{Effector, Solver, SolverConfig, SolverState};
use gm_solver::ankle_constraints::{
    ankle_angles, ankle_axial_swing, capture_ankle_reference, LegSide,
};
use gm_tests::lower_limb_metrics::{
    foot_forward, left_foot_roll_from, left_foot_to_shin_roll, rotate_about_axis,
    signed_angle_difference,
};

fn main() {
    let pose = gm_tests::standing_pose();
    let orbit_mode = std::env::args().any(|arg| arg == "--orbit");
    let config = if orbit_mode {
        SolverConfig {
            gravity: 0.0,
            balance_stiffness: 0.0,
            friction: 0.0,
            floor_friction: 0.0,
            ankle_orientation_stiffness: 0.0,
            ..SolverConfig::default()
        }
    } else {
        SolverConfig::default()
    };
    let ankle_reference = capture_ankle_reference(&pose, P0, LegSide::Left);
    let mut solver = Solver::new(&pose, config);
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
    let ankle = reference.get(P0, LeftAnkle);
    let roll_axis = foot_forward(&reference, P0)
        .expect("the standing pose has a well-defined heel-to-toe direction");
    let reference_foot_shin_roll = left_foot_to_shin_roll(&reference, P0)
        .expect("the standing pose has well-defined foot and leg planes");

    if orbit_mode {
        println!(
            "command_deg,principal_achieved_deg,unwrapped_achieved_deg,ankle_swing_deg,ankle_twist_deg,max_residual_cm,max_bone_error,rejected_frames"
        );
        let orbit_axis = gm_core::V3::Y;
        let mut previous_principal = 0.0f64;
        let mut unwrapped = 0.0f64;
        for command_deg in (0..=360).step_by(5) {
            let radians = (command_deg as f64).to_radians();
            let target = |joint| {
                rotate_about_axis(reference.get(P0, joint), ankle, orbit_axis, radians)
                    .expect("world Y is a valid orbit axis")
            };
            let effectors = [LeftAnkle, LeftHeel, LeftToe].map(|joint| Effector {
                joint: PlayerJoint { player: P0, joint },
                target: target(joint),
                stiffness: 0.9,
            });
            let mut rejected_frames = 0usize;
            let mut last_diag = None;
            for _ in 0..3 {
                let (next, diag) = solver.step(&state, &effectors, 1.0 / 60.0);
                rejected_frames += usize::from(diag.rejected);
                state = next;
                last_diag = Some(diag);
            }
            let angles = ankle_angles(&state.pose, P0, LegSide::Left, &ankle_reference)
                .expect("the constrained foot frame must remain defined");
            let axial = ankle_axial_swing(&state.pose, P0, LegSide::Left, &ankle_reference)
                .expect("the planted foot direction must not align with the shin");
            let principal = angles.swing.copysign(axial);
            unwrapped += signed_angle_difference(principal, previous_principal);
            previous_principal = principal;
            let diag = last_diag.unwrap();
            let max_residual_cm =
                100.0 * diag.effector_residuals.iter().copied().fold(0.0, f64::max);
            println!(
                "{command_deg},{:.4},{:.4},{:.4},{:.4},{max_residual_cm:.4},{:.8},{rejected_frames}",
                principal.to_degrees(),
                unwrapped.to_degrees(),
                angles.swing.to_degrees(),
                angles.twist.to_degrees(),
                diag.max_bone_error,
            );
        }
        eprintln!(
            "configured_swing_limit_deg={:.4},configured_twist_limit_deg={:.4}",
            config.ankle_swing_limit.to_degrees(),
            config.ankle_twist_limit.to_degrees(),
        );
        return;
    }

    println!(
        "command_deg,achieved_roll_deg,foot_shin_roll_deg,foot_shin_delta_deg,heel_orbit_cm,knee_cm,hip_cm,core_cm,max_residual_cm,max_bone_error,rejected_frames"
    );
    for command_deg in (-30..=30).step_by(5) {
        state = reference_state;
        let radians = (command_deg as f64).to_radians();
        let target = |joint| {
            rotate_about_axis(reference.get(P0, joint), ankle, roll_axis, radians)
                .expect("the roll axis was validated above")
        };
        let effectors = [
            Effector {
                joint: PlayerJoint {
                    player: P0,
                    joint: LeftAnkle,
                },
                target: ankle,
                stiffness: 0.9,
            },
            Effector {
                joint: PlayerJoint {
                    player: P0,
                    joint: LeftHeel,
                },
                target: target(LeftHeel),
                stiffness: 0.9,
            },
            Effector {
                joint: PlayerJoint {
                    player: P0,
                    joint: LeftToe,
                },
                target: target(LeftToe),
                stiffness: 0.9,
            },
        ];
        let mut last_diag = None;
        let mut rejected_frames = 0;
        for _ in 0..60 {
            let (next, diag) = solver.step(&state, &effectors, 1.0 / 60.0);
            rejected_frames += usize::from(diag.rejected);
            state = next;
            last_diag = Some(diag);
        }
        let diag = last_diag.expect("the probe always advances at least one frame");
        let achieved_roll = left_foot_roll_from(&reference, &state.pose, P0)
            .expect("the constrained foot must retain a well-defined plane");
        let foot_shin_roll = left_foot_to_shin_roll(&state.pose, P0)
            .expect("the constrained foot and leg must retain well-defined planes");
        let displacement_cm =
            |joint| 100.0 * state.pose.get(P0, joint).distance(reference.get(P0, joint));
        let max_residual_cm = 100.0 * diag.effector_residuals.iter().copied().fold(0.0, f64::max);
        println!(
            "{command_deg},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{max_residual_cm:.4},{:.8},{rejected_frames}",
            achieved_roll.to_degrees(),
            foot_shin_roll.to_degrees(),
            signed_angle_difference(foot_shin_roll, reference_foot_shin_roll).to_degrees(),
            displacement_cm(LeftHeel),
            displacement_cm(LeftKnee),
            displacement_cm(LeftHip),
            displacement_cm(Core),
            diag.max_bone_error,
        );
    }
}
