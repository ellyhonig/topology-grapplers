//! Debug: sweep config variants on a named pose, report drift over 300 steps.

use gm_solver::{Solver, SolverConfig, SolverState};

fn main() {
    let entries = gm_tests::load_database();
    let needle = std::env::args().nth(1).unwrap_or_else(|| "bottom armbar".into());
    let entry = entries
        .iter()
        .filter(|e| e.is_position())
        .find(|e| e.name().contains(&needle))
        .expect("pose present");
    println!("pose: {:?}", entry.name());
    let pose = entry.frames[0];
    let d = SolverConfig::default;
    let configs: Vec<(&str, SolverConfig)> = vec![
        ("default", d()),
        ("no grips", d()),
        ("no balance", SolverConfig { balance_stiffness: 0.0, ..d() }),
        ("no tone", SolverConfig { tone_stiffness: 0.0, ..d() }),
        ("no fric", SolverConfig { friction: 0.0, ..d() }),
        ("damping 20", SolverConfig { damping: 20.0, ..d() }),
        ("substeps 8", SolverConfig { substeps: 8, ..d() }),
        ("iters 24", SolverConfig { iterations: 24, ..d() }),
    ];
    for (label, cfg) in configs {
        let mut solver = Solver::new(&pose, cfg);
        if label == "no grips" {
            solver.release_grips();
        }
        let mut state = SolverState::from_pose(pose);
        let mut worst_step = 0.0f64;
        for _ in 0..180 {
            let prev = state.pose;
            let (next, _) = solver.step(&state, &[], 1.0 / 60.0);
            state = next;
            worst_step = worst_step.max(state.pose.max_displacement(&prev));
        }
        let settled = state.pose;
        for _ in 0..30 {
            state = solver.step(&state, &[], 1.0 / 60.0).0;
        }
        println!(
            "{:12} drift {:.3}  late {:.3}  worst step {:.3}",
            label,
            state.pose.max_displacement(&pose),
            state.pose.max_displacement(&settled),
            worst_step
        );
    }
}
