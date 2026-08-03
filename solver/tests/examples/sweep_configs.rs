//! Debug: run the db_regression pose sample under several solver configs and
//! report, per config, how many poses violate the drift/late-motion bounds.

use gm_solver::{Solver, SolverConfig, SolverState};

fn main() {
    let entries = gm_tests::load_database();
    let positions: Vec<_> = entries.iter().filter(|e| e.is_position()).collect();
    let d = SolverConfig::default;
    let configs: Vec<(&str, SolverConfig)> = vec![
        ("default", d()),
        ("no balance", SolverConfig { balance_stiffness: 0.0, ..d() }),
    ];
    for (label, cfg) in configs {
        let mut fails = 0;
        println!("== {}", label);
        for entry in positions.iter().step_by(7) {
            if entry.name().contains("suspended") {
                continue;
            }
            let pose = entry.frames[0];
            let solver = Solver::new(&pose, cfg);
            let mut state = SolverState::from_pose(pose);
            for _ in 0..180 {
                state = solver.step(&state, &[], 1.0 / 60.0).0;
            }
            let settled = state.pose;
            for _ in 0..30 {
                state = solver.step(&state, &[], 1.0 / 60.0).0;
            }
            let drift = state.pose.max_displacement(&pose);
            let late = state.pose.max_displacement(&settled);
            if drift > 0.20 || late > 0.05 {
                fails += 1;
                println!("  drift {:.3}  late {:.3}  {}", drift, late, entry.name());
            }
        }
        println!("  total fails: {}", fails);
    }
}
