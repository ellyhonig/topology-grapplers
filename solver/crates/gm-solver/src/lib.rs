//! gm-solver: the XPBD constraint solver that owns all motion.
//!
//! Input (VR 6DOF, GUI drags, tests) enters exclusively as compliant
//! [`Effector`]s; hard constraints (bones, anatomical limits, contacts, floor)
//! are projected afterward each substep, so no input can produce an invalid
//! pose. A topology watchdog re-solves or rejects steps that would change limb
//! entanglement by tunneling.

pub mod anatomy_constraints;
pub mod ankle_constraints;
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
    use crate::ankle_constraints::{
        ankle_angles, ankle_axial_swing, capture_ankle_reference, LegSide,
    };
    use gm_core::{v3, Joint::*, PlayerJoint, Pose, V3, P0, P1};

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
    fn stationary_foot_tracker_does_not_collapse_the_driven_leg() {
        let pose = standing_pose();
        let mut solver = Solver::new(&pose, SolverConfig::default());
        // Isolate lower-limb deformation from whole-body translation and the
        // standing balance controller's recovery strategy.
        solver.set_pins(vec![
            PlayerJoint { player: P0, joint: LeftHip },
            PlayerJoint { player: P0, joint: RightHip },
            PlayerJoint { player: P0, joint: Core },
        ]);
        let mut state = SolverState::from_pose(pose);
        for _ in 0..60 {
            state = solver.step(&state, &[], 1.0 / 60.0).0;
        }
        let reference = state.pose;
        let effectors = [LeftAnkle, LeftHeel, LeftToe].map(|joint| Effector {
            joint: PlayerJoint { player: P0, joint },
            target: reference.get(P0, joint),
            stiffness: 0.9,
        });

        let mut max_residual = 0.0f64;
        for _ in 0..60 {
            let (next, diag) = solver.step(&state, &effectors, 1.0 / 60.0);
            assert!(!diag.rejected);
            max_residual = diag
                .effector_residuals
                .iter()
                .copied()
                .fold(max_residual, f64::max);
            state = next;
        }

        let ankle_drift = state.pose.get(P0, LeftAnkle).distance(reference.get(P0, LeftAnkle));
        let knee_drift = state.pose.get(P0, LeftKnee).distance(reference.get(P0, LeftKnee));
        assert!(
            ankle_drift < 0.02,
            "stationary foot tracker let ankle drift {ankle_drift} m"
        );
        assert!(
            knee_drift < 0.02,
            "stationary foot tracker let knee drift {knee_drift} m"
        );
        assert!(
            max_residual < 0.02,
            "stationary foot tracker accumulated {max_residual} m residual"
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

    fn rotate_about_axis(point: V3, origin: V3, axis: V3, radians: f64) -> V3 {
        let axis = axis.normalized_or_zero();
        let relative = point - origin;
        let (sin, cos) = radians.sin_cos();
        origin
            + relative * cos
            + axis.cross(relative) * sin
            + axis * (axis.dot(relative) * (1.0 - cos))
    }

    fn run_continuous_planted_foot_orbit(pin_knee: bool) {
        let pose = standing_pose();
        let reference = capture_ankle_reference(&pose, P0, LegSide::Left);
        let config = SolverConfig {
            gravity: 0.0,
            balance_stiffness: 0.0,
            friction: 0.0,
            floor_friction: 0.0,
            ..SolverConfig::default()
        };
        let limit = config.ankle_swing_limit;
        assert!(limit < std::f64::consts::PI - 0.5);
        let mut solver = Solver::new(&pose, config);
        let mut pins = vec![
            PlayerJoint { player: P0, joint: LeftHip },
            PlayerJoint { player: P0, joint: RightHip },
            PlayerJoint { player: P0, joint: Core },
            PlayerJoint { player: P0, joint: LeftAnkle },
        ];
        if pin_knee {
            pins.push(PlayerJoint { player: P0, joint: LeftKnee });
        }
        solver.set_pins(pins.clone());
        let mut state = SolverState::from_pose(pose);
        let ankle = pose.get(P0, LeftAnkle);
        let axis = V3::Y;
        let mut previous_principal = 0.0f64;
        let mut unwrapped = 0.0f64;
        let mut interior_residual = 0.0f64;
        let mut saturated_residual = 0.0f64;
        let mut saturated_state = None;

        // Successive targets matter: a static 360-degree target is identical
        // to neutral and cannot prove that a solver blocked the intervening orbit.
        for command_deg in (0..=360).step_by(5) {
            let command = (command_deg as f64).to_radians();
            let target = |joint| rotate_about_axis(pose.get(P0, joint), ankle, axis, command);
            let effectors = [
                Effector {
                    joint: PlayerJoint { player: P0, joint: LeftAnkle },
                    target: ankle,
                    stiffness: 1.0,
                },
                Effector {
                    joint: PlayerJoint { player: P0, joint: LeftHeel },
                    target: target(LeftHeel),
                    stiffness: 1.0,
                },
                Effector {
                    joint: PlayerJoint { player: P0, joint: LeftToe },
                    target: target(LeftToe),
                    stiffness: 1.0,
                },
            ];
            let mut last_diag = None;
            for _ in 0..3 {
                let (next, diag) = solver.step(&state, &effectors, 1.0 / 60.0);
                assert!(!diag.rejected);
                assert!(
                    diag.max_bone_error < 0.01,
                    "command {command_deg}: bone error {}",
                    diag.max_bone_error,
                );
                state = next;
                last_diag = Some(diag);
            }
            assert!(state.pose.is_finite());
            for pin in &pins {
                assert!(state.pose[*pin].distance(pose[*pin]) < 1e-12);
            }

            let angles = ankle_angles(&state.pose, P0, LegSide::Left, &reference).unwrap();
            assert!(
                angles.swing <= limit + 2e-3,
                "command {command_deg}: swing {} exceeded {}",
                angles.swing.to_degrees(),
                limit.to_degrees(),
            );
            assert!(angles.twist.abs() <= config.ankle_twist_limit + 2e-3);

            let axial = ankle_axial_swing(&state.pose, P0, LegSide::Left, &reference)
                .expect("the planted foot direction must not align with the shin");
            let principal = angles.swing.copysign(axial);
            let delta = (principal - previous_principal)
                .sin()
                .atan2((principal - previous_principal).cos());
            unwrapped += delta;
            previous_principal = principal;
            assert!(
                unwrapped.abs() <= limit + 0.03,
                "continuous orbit crossed the hard boundary at command {command_deg}: {} deg",
                unwrapped.to_degrees(),
            );

            let max_residual = last_diag
                .unwrap()
                .effector_residuals
                .into_iter()
                .fold(0.0, f64::max);
            if command_deg == 20 {
                interior_residual = max_residual;
            }
            if command_deg == 160 {
                // Hold the saturated demand for one second before release, as
                // a controller would be held against the anatomical stop.
                let mut held_diag = None;
                for _ in 0..60 {
                    let (next, diag) = solver.step(&state, &effectors, 1.0 / 60.0);
                    assert!(!diag.rejected);
                    state = next;
                    held_diag = Some(diag);
                }
                saturated_residual = held_diag
                    .unwrap()
                    .effector_residuals
                    .into_iter()
                    .fold(0.0, f64::max);
                saturated_state = Some(state);
            }
        }
        assert!(
            saturated_residual > interior_residual + 0.05,
            "load did not build after saturation: interior={interior_residual}, saturated={saturated_residual}",
        );

        // Releasing a saturated target must relax through bounded finite
        // motion, not convert accumulated residual into a positional snap.
        let mut released = saturated_state.expect("the sweep includes 160 degrees");
        for release_frame in 0..120 {
            let before = released.pose;
            let before_angles = ankle_angles(&before, P0, LegSide::Left, &reference).unwrap();
            let (fast_joint, fast_speed) = PlayerJoint::all()
                .map(|joint| (joint, released.velocity[joint.player.index()][joint.joint.index()].length()))
                .max_by(|a, b| a.1.total_cmp(&b.1))
                .unwrap();
            let (next, diag) = solver.step(&released, &[], 1.0 / 60.0);
            assert!(!diag.rejected);
            assert!(next.pose.is_finite());
            let frame_motion = next.pose.max_displacement(&before);
            let moved_joint = PlayerJoint::all()
                .max_by(|a, b| {
                    next.pose[*a]
                        .distance(before[*a])
                        .total_cmp(&next.pose[*b].distance(before[*b]))
                })
                .unwrap();
            assert!(
                frame_motion < 0.05,
                "release frame {release_frame} moved {frame_motion} m at {moved_joint:?} from swing={} twist={}, fastest={fast_joint:?} at {fast_speed} m/s",
                before_angles.swing.to_degrees(),
                before_angles.twist.to_degrees(),
            );
            assert!(next
                .velocity
                .iter()
                .flatten()
                .all(|velocity| velocity.is_finite()
                    && velocity.length() <= config.max_joint_speed + 1e-9));
            let angles = ankle_angles(&next.pose, P0, LegSide::Left, &reference).unwrap();
            assert!(angles.swing <= limit + 2e-3);
            assert!(angles.twist.abs() <= config.ankle_twist_limit + 2e-3);
            released = next;
        }
    }

    #[test]
    fn continuous_foot_orbit_is_blocked_with_fixed_shin_reference() {
        run_continuous_planted_foot_orbit(true);
    }

    #[test]
    fn continuous_foot_orbit_is_blocked_with_solver_controlled_knee() {
        run_continuous_planted_foot_orbit(false);
    }
}
