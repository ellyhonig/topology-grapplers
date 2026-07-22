//! Debug: step a named pose with no input and print per-step motion spikes.
//!
//! Usage: cargo run -p gm-tests --example trace_pose -- <name substring> [steps]

use gm_core::PlayerJoint;
use gm_solver::{Solver, SolverConfig, SolverState};

fn main() {
    let entries = gm_tests::load_database();
    let needle = std::env::args().nth(1).unwrap_or_else(|| "side ctrl w/".into());
    let steps: usize =
        std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(240);
    let entry = entries
        .iter()
        .filter(|e| e.is_position())
        .find(|e| e.name().contains(&needle))
        .expect("pose present");
    println!("pose: {:?}", entry.name());
    let pose = entry.frames[0];
    let solver = Solver::new(&pose, SolverConfig::default());
    for (joint, player, ends) in solver.grip_summaries() {
        println!(
            "grip: p{}:{:?} -> p{}:{:?}-{:?}",
            joint.player.index(),
            joint.joint,
            player.index(),
            ends[0],
            ends[1]
        );
    }
    let mut state = SolverState::from_pose(pose);
    for step in 0..steps {
        let prev = state.pose;
        let (next, diag) = solver.step(&state, &[], 1.0 / 60.0);
        state = next;
        let (worst, moved) = PlayerJoint::all()
            .map(|pj| (pj, state.pose[pj].distance(prev[pj])))
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        if moved > 0.02 || diag.retries > 0 || diag.rejected {
            println!(
                "step {:3}  p{}:{:?} moved {:.3}  retries {}  rejected {}  drift {:.3}",
                step + 1,
                worst.player.index(),
                worst.joint,
                moved,
                diag.retries,
                diag.rejected,
                state.pose.max_displacement(&pose),
            );
        }
    }
    println!("final drift {:.3}", state.pose.max_displacement(&pose));
}
