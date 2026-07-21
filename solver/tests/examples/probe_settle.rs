//! Debug: settle a named pose for 12 s, log motion + grip strain each second.

use gm_solver::{Solver, SolverConfig, SolverState};

fn main() {
    let entries = gm_tests::load_database();
    let needle = std::env::args().nth(1).unwrap_or_else(|| "o-soto-gari".into());
    let entry = entries
        .iter()
        .filter(|e| e.is_position())
        .find(|e| e.name().contains(&needle))
        .expect("pose present");
    println!("pose: {:?}", entry.name());
    let pose = entry.frames[0];
    let solver = Solver::new(&pose, SolverConfig::default());
    println!("grips: {}", solver.grip_count());
    for (j, p, ends) in solver.grip_summaries() {
        println!("  p{} {:?} -> p{} {:?}", j.player.index(), j.joint, p.index(), ends);
    }
    let mut state = SolverState::from_pose(pose);
    for i in 0..720 {
        let prev = state.pose;
        state = solver.step(&state, &[], 1.0 / 60.0).0;
        if i % 60 == 59 {
            println!(
                "t={:>4.1}s step motion {:.4} broken {:08b} strain {:?}",
                (i + 1) as f64 / 60.0,
                state.pose.max_displacement(&prev),
                state.broken_grips,
                &state.grip_strain[..solver.grip_count().min(8)]
                    .iter()
                    .map(|s| (s * 100.0).round() / 100.0)
                    .collect::<Vec<_>>(),
            );
        }
    }
}
