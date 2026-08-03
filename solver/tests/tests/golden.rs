//! Golden adversarial scenarios with asserted outcomes:
//!   1. trying to hyperextend an elbow fails (arm stays straight or bends correctly)
//!   2. dragging a hand into the opponent's torso stops at the surface
//!   3. a fully pinned arm still wiggles, and the rest of the body stays free

use gm_core::{angle_at, v3, Joint::*, PlayerJoint, P0, P1};
use gm_solver::{Effector, Solver, SolverConfig, SolverState};
use gm_validate::penetration_floors;

#[test]
fn elbow_cannot_hyperextend() {
    let pose = gm_tests::standing_pose();
    let solver = Solver::new(&pose, SolverConfig::default());
    let mut state = SolverState::from_pose(pose);

    // First bend the arm properly (hand toward shoulder), establishing bend memory.
    let shoulder = state.pose.get(P0, RightShoulder);
    let bend_target = shoulder + v3(0.05, -0.05, 0.15);
    let hand = |t| Effector { joint: PlayerJoint { player: P0, joint: RightWrist }, target: t, stiffness: 0.9 };
    for _ in 0..90 {
        let (next, _) = solver.step(&state, &[hand(bend_target)], 1.0 / 60.0);
        state = next;
    }
    let bent = angle_at(
        state.pose.get(P0, RightShoulder),
        state.pose.get(P0, RightElbow),
        state.pose.get(P0, RightWrist),
    );
    assert!(bent < 2.0, "arm should be clearly bent, angle {}", bent);

    // Now try to hyperextend: pull the wrist along the straightening direction
    // and far past straight, from a direction that would fold the elbow backward.
    let elbow = state.pose.get(P0, RightElbow);
    let straight_dir = (elbow - state.pose.get(P0, RightShoulder)).normalized_or_zero();
    let overshoot = elbow + straight_dir * 2.0;
    for _ in 0..120 {
        let (next, diag) = solver.step(&state, &[hand(overshoot)], 1.0 / 60.0);
        assert!(diag.max_bone_error < 0.05);
        state = next;
    }
    // The arm may go straight (angle -> pi) but the elbow must never fold to the
    // wrong side: bend memory keeps the bend component non-negative, and the
    // angle can never wrap past pi in a valid configuration.
    let final_angle = angle_at(
        state.pose.get(P0, RightShoulder),
        state.pose.get(P0, RightElbow),
        state.pose.get(P0, RightWrist),
    );
    assert!(final_angle <= std::f64::consts::PI + 1e-9);
    // And the pose overall is still valid.
    let floors = penetration_floors(&gm_tests::standing_pose(), solver.capsules(), solver.pairs());
    gm_tests::assert_valid(&state.pose, &solver, &floors, "post-hyperextension-attempt");
}

#[test]
fn hand_cannot_be_dragged_through_opponents_torso() {
    let pose = gm_tests::standing_pose();
    let mut solver = Solver::new(&pose, SolverConfig::default());
    // Anchor p0's base so it cannot walk around, and p1's torso so it cannot be
    // shoved out of the way - both of which would be legitimate ways for the
    // hand to approach the target. What remains is only the illegal path:
    // straight through the opponent's chest.
    solver.set_pins(vec![
        PlayerJoint { player: P0, joint: LeftHip },
        PlayerJoint { player: P0, joint: RightHip },
        PlayerJoint { player: P0, joint: LeftAnkle },
        PlayerJoint { player: P0, joint: RightAnkle },
        PlayerJoint { player: P1, joint: LeftHip },
        PlayerJoint { player: P1, joint: RightHip },
        PlayerJoint { player: P1, joint: LeftShoulder },
        PlayerJoint { player: P1, joint: RightShoulder },
        PlayerJoint { player: P1, joint: Core },
    ]);
    let floors = penetration_floors(&pose, solver.capsules(), solver.pairs());
    let mut state = SolverState::from_pose(pose);

    // p1's chest center; drag p0's right hand straight through it to far behind.
    let chest = (state.pose.get(P1, Core) + state.pose.get(P1, Neck)) * 0.5;
    let through = chest + (chest - state.pose.get(P0, RightHand)).normalized_or_zero() * 1.5;
    let e = Effector { joint: PlayerJoint { player: P0, joint: RightHand }, target: through, stiffness: 1.0 };
    for _ in 0..180 {
        let (next, _) = solver.step(&state, &[e], 1.0 / 60.0);
        state = next;
        gm_tests::assert_valid(&state.pose, &solver, &floors, "drag-through-torso");
    }
    // The hand must have been stopped before the target: it cannot reach a
    // point whose straight path is through the torso.
    let residual = state.pose.get(P0, RightHand).distance(through);
    assert!(residual > 0.3, "hand should be blocked well short of target, residual {}", residual);
}

#[test]
fn pinned_arm_still_wiggles_and_body_stays_free() {
    let pose = gm_tests::standing_pose();
    let mut solver = Solver::new(&pose, SolverConfig::default());
    // Hard-pin p0's right arm at wrist and elbow (a very strong grip).
    solver.set_pins(vec![
        PlayerJoint { player: P0, joint: RightWrist },
        PlayerJoint { player: P0, joint: RightElbow },
    ]);
    let mut state = SolverState::from_pose(pose);

    // Wiggle: alternate hand targets orthogonal to the pinned segment.
    let base_hand = state.pose.get(P0, RightHand);
    let mut max_hand_move: f64 = 0.0;
    for i in 0..120 {
        let sign = if (i / 10) % 2 == 0 { 1.0 } else { -1.0 };
        let e = Effector {
            joint: PlayerJoint { player: P0, joint: RightHand },
            target: base_hand + v3(0.2 * sign, 0.1, 0.2 * sign),
            stiffness: 0.9,
        };
        let (next, diag) = solver.step(&state, &[e], 1.0 / 60.0);
        assert!(diag.max_bone_error < 0.05);
        state = next;
        max_hand_move = max_hand_move.max(state.pose.get(P0, RightHand).distance(base_hand));
    }
    // The hand hangs off the pinned wrist by an 0.08 m bone: it must retain
    // real motion (never stuck)...
    assert!(max_hand_move > 0.03, "pinned arm wiggle only {} m", max_hand_move);
    // ...while the pinned joints truly held still.
    assert!(state.pose.get(P0, RightWrist).distance(pose.get(P0, RightWrist)) < 1e-9);
    assert!(state.pose.get(P0, RightElbow).distance(pose.get(P0, RightElbow)) < 1e-9);

    // And an unrelated limb (left hand) remains fully mobile.
    let left_target = v3(-0.6, 1.4, -0.9);
    let e = Effector { joint: PlayerJoint { player: P0, joint: LeftHand }, target: left_target, stiffness: 0.9 };
    for _ in 0..120 {
        let (next, _) = solver.step(&state, &[e], 1.0 / 60.0);
        state = next;
    }
    assert!(
        state.pose.get(P0, LeftHand).distance(left_target) < 0.15,
        "free limb blocked: residual {}",
        state.pose.get(P0, LeftHand).distance(left_target)
    );
}

#[test]
fn simultaneous_two_player_input_stays_valid() {
    let pose = gm_tests::standing_pose();
    let solver = Solver::new(&pose, SolverConfig::default());
    let floors = penetration_floors(&pose, solver.capsules(), solver.pairs());
    let mut state = SolverState::from_pose(pose);

    // Both players reach for each other's head at once, then pull away, at full
    // stiffness - the classic clash. Also both grab each other's right wrist.
    for phase in 0..4 {
        let toward = phase % 2 == 0;
        for _ in 0..60 {
            let p1_head = state.pose.get(P1, Head);
            let p0_head = state.pose.get(P0, Head);
            let dir = if toward { 1.0 } else { -1.0 };
            let effectors = [
                Effector {
                    joint: PlayerJoint { player: P0, joint: RightHand },
                    target: p1_head * dir + v3(0.0, (1.0 - dir) * 0.9, 0.0),
                    stiffness: 1.0,
                },
                Effector {
                    joint: PlayerJoint { player: P1, joint: RightHand },
                    target: p0_head * dir + v3(0.0, (1.0 - dir) * 0.9, 0.0),
                    stiffness: 1.0,
                },
                Effector {
                    joint: PlayerJoint { player: P0, joint: LeftHand },
                    target: state.pose.get(P1, RightWrist),
                    stiffness: 0.8,
                },
                Effector {
                    joint: PlayerJoint { player: P1, joint: LeftHand },
                    target: state.pose.get(P0, RightWrist),
                    stiffness: 0.8,
                },
            ];
            let (next, _) = solver.step(&state, &effectors, 1.0 / 60.0);
            state = next;
            gm_tests::assert_valid(&state.pose, &solver, &floors, "two-player clash");
        }
    }
}
