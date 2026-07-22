//! Database regression: every position in GrappleMap.txt must validate under
//! the calibrated invariants, and must stay put when the solver settles it.

use gm_solver::{Solver, SolverConfig, SolverState};
use gm_validate::{penetration_floors, validate_pose, Tolerances};

/// Every frame of every entry (positions and transition keyframes) validates.
#[test]
fn every_database_frame_validates() {
    let entries = gm_tests::load_database();
    let mut checked = 0usize;
    for entry in &entries {
        for (fi, pose) in entry.frames.iter().enumerate() {
            let solver = Solver::new(pose, SolverConfig::default());
            let floors = penetration_floors(pose, solver.capsules(), solver.pairs());
            let report = validate_pose(
                pose,
                solver.bones(),
                solver.capsules(),
                solver.pairs(),
                &floors,
                &Tolerances::default(),
            );
            assert!(
                report.is_valid(),
                "{:?} frame {}: {:?}",
                entry.name(),
                fi,
                report.violations.first()
            );
            checked += 1;
        }
    }
    assert!(checked > 8000, "checked {} frames", checked);
}

/// Named positions behave physically when the solver runs with no input.
///
/// Two tiers, because the database mixes static holds with mid-action
/// snapshots (throws in flight, jumps, elevations):
/// - *Everything* must converge to a still, valid state - the solver may
///   never vibrate, walk, or explode.
/// - The overwhelming majority must also *stay put* (small settling under
///   gravity is fine). Mid-action poses legitimately fall to the mat -
///   an airborne body with no base is supposed to come down - so a small
///   fraction of large-but-converged settles is expected.
#[test]
fn database_positions_are_solver_stable() {
    let entries = gm_tests::load_database();
    let positions: Vec<_> = entries.iter().filter(|e| e.is_position()).collect();
    let mut checked = 0usize;
    let mut moved_far: Vec<String> = Vec::new();
    // Every 7th position (~86 poses) keeps this test fast while covering the
    // full variety of guards, pins, and entanglements.
    for entry in positions.iter().step_by(7) {
        let pose = entry.frames[0];
        let solver = Solver::new(&pose, SolverConfig::default());
        let floors = penetration_floors(&pose, solver.capsules(), solver.pairs());
        let mut state = SolverState::from_pose(pose);
        // Ten simulated seconds: static holds settle in well under one, and
        // even a full fall from a throw snapshot comes to rest within ten
        // (a fallen body slumps, rolls, and needs a few seconds to be still).
        for _ in 0..570 {
            let (next, _) = solver.step(&state, &[], 1.0 / 60.0);
            state = next;
        }
        let settled = state.pose;
        for _ in 0..30 {
            let (next, _) = solver.step(&state, &[], 1.0 / 60.0);
            state = next;
        }
        let drift = state.pose.max_displacement(&pose);
        let late_motion = state.pose.max_displacement(&settled);
        assert!(
            late_motion < 0.05,
            "{:?}: still moving {} m after 10 s of settling (not converging)",
            entry.name(),
            late_motion
        );
        gm_tests::assert_valid(&state.pose, &solver, &floors, entry.name());
        if drift > 0.30 {
            moved_far.push(format!("{:?} ({:.2} m)", entry.name(), drift));
        }
        checked += 1;
    }
    // Regression guard on overall stability: only genuinely unsupported
    // (mid-action) poses may travel; if this fraction creeps up, the solver
    // has started destroying authored positions.
    assert!(
        moved_far.len() * 10 <= checked,
        "{} of {} sampled poses moved far from their authored position: {}",
        moved_far.len(),
        checked,
        moved_far.join(", ")
    );
}
