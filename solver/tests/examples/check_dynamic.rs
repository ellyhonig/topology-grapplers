//! Debug: verify that mid-action poses settle (converge) within 10 s even
//! though they legitimately move a lot.

use gm_solver::{Solver, SolverConfig, SolverState};

const DYNAMIC: [&str; 7] = [
    "suspended",
    "low flying",
    "kimura\\nthrow/sweep",
    "standing\\nback mount",
    "butterfly\\nelevation",
    "backward roll",
    "o-soto-gari",
];

fn main() {
    let entries = gm_tests::load_database();
    for needle in DYNAMIC {
        let Some(entry) = entries
            .iter()
            .filter(|e| e.is_position())
            .find(|e| e.name().contains(needle))
        else {
            println!("{:28} NOT FOUND", needle);
            continue;
        };
        let pose = entry.frames[0];
        let solver = Solver::new(&pose, SolverConfig::default());
        let mut state = SolverState::from_pose(pose);
        for _ in 0..570 {
            state = solver.step(&state, &[], 1.0 / 60.0).0;
        }
        let settled = state.pose;
        let mut rejected = 0;
        for _ in 0..30 {
            let (next, diag) = solver.step(&state, &[], 1.0 / 60.0);
            if diag.rejected {
                rejected += 1;
            }
            state = next;
        }
        println!(
            "{:28} drift {:.3}  late {:.3}  rejected {}",
            needle,
            state.pose.max_displacement(&pose),
            state.pose.max_displacement(&settled),
            rejected
        );
    }
}
