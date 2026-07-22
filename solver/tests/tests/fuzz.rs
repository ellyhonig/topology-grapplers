//! Property-based robustness: whatever the input stream - teleporting targets,
//! opposing pulls on both players, targets inside the opponent's body - every
//! intermediate state must satisfy every invariant, forever.

use gm_core::{v3, PlayerJoint};
use gm_solver::{Effector, Solver, SolverConfig, SolverState};
use gm_validate::{penetration_floors, validate_pose, validate_step, Tolerances};
use proptest::prelude::*;

fn effector_strategy() -> impl Strategy<Value = Effector> {
    (
        0usize..46,
        // Includes far-outside-arena and below-floor targets on purpose.
        (-60.0f64..60.0, -30.0f64..30.0, -60.0f64..60.0),
        0.0f64..1.0,
    )
        .prop_map(|(flat, (x, y, z), stiffness)| Effector {
            joint: PlayerJoint::from_flat(flat).unwrap(),
            target: v3(x, y, z),
            stiffness,
        })
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 24, ..ProptestConfig::default() })]

    /// Random 40-step adversarial dual-player input streams keep every invariant.
    #[test]
    fn arbitrary_input_streams_never_break_invariants(
        streams in proptest::collection::vec(
            proptest::collection::vec(effector_strategy(), 0..5),
            40
        )
    ) {
        let pose = gm_tests::standing_pose();
        let solver = Solver::new(&pose, SolverConfig::default());
        let floors = penetration_floors(&pose, solver.capsules(), solver.pairs());
        let tol = Tolerances::default();
        let mut state = SolverState::from_pose(pose);

        for (i, effectors) in streams.iter().enumerate() {
            let before = state.pose;
            let (next, _) = solver.step(&state, effectors, 1.0 / 60.0);

            let report = validate_pose(
                &next.pose,
                solver.bones(),
                solver.capsules(),
                solver.pairs(),
                &floors,
                &tol,
            );
            prop_assert!(
                report.is_valid(),
                "step {}: {:?}",
                i,
                report.violations.first()
            );

            let step_violations = validate_step(
                &before,
                &next.pose,
                1.0 / 60.0,
                solver.capsules(),
                solver.pairs(),
                &tol,
            );
            prop_assert!(
                step_violations.is_empty(),
                "step {}: {:?}",
                i,
                step_violations.first()
            );

            state = next;
        }
    }
}
