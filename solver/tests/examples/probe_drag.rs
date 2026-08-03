//! Debug: drag a hand effector on a named pose and report how far it gets.

use gm_core::{Joint, PlayerJoint, P0};
use gm_solver::{Effector, Solver, SolverConfig, SolverState};

fn main() {
    let entries = gm_tests::load_database();
    let needle = std::env::args().nth(1).unwrap_or_else(|| "distant standing collar-tie".into());
    let entry = entries
        .iter()
        .filter(|e| e.is_position())
        .find(|e| e.name().contains(&needle))
        .expect("pose present");
    println!("pose: {:?}", entry.name());
    let pose = entry.frames[0];
    let joint = match std::env::args().nth(2).as_deref() {
        Some("left") => Joint::LeftHand,
        _ => Joint::RightHand,
    };
    let hand = PlayerJoint { player: P0, joint };
    let core = PlayerJoint { player: P0, joint: Joint::Core };

    for (label, release) in [("with grips", false), ("grips released", true)] {
        let mut solver = Solver::new(&pose, SolverConfig::default());
        println!("{label}: grip count {}", solver.grip_count());
        for (j, p, ends) in solver.grip_summaries() {
            println!("  grip: p{} {:?} -> p{} {:?}", j.player.index(), j.joint, p.index(), ends);
        }
        if release {
            solver.release_grips();
        }
        let mut state = SolverState::from_pose(pose);
        // Settle first.
        for _ in 0..120 {
            state = solver.step(&state, &[], 1.0 / 60.0).0;
        }
        let opp = PlayerJoint { player: gm_core::P1, joint: Joint::LeftHand };
        let start_hand = state.pose[hand];
        let start_core = state.pose[core];
        let start_opp = state.pose[opp];
        let target = start_hand + gm_core::v3(0.3, 0.35, 0.1);
        let eff = [Effector { joint: hand, target, stiffness: 0.8 }];
        for i in 0..240 {
            let (next, diag) = solver.step(&state, &eff, 1.0 / 60.0);
            state = next;
            if i % 60 == 59 {
                println!(
                    "  t={:.1}s residual {:.3} moved {:.3} core {:.3} oppHand {:.3} rejected {} retries {} broken {:06b} strain {:?}",
                    (i + 1) as f64 / 60.0,
                    diag.effector_residuals[0],
                    state.pose[hand].distance(start_hand),
                    state.pose[core].distance(start_core),
                    state.pose[opp].distance(start_opp),
                    diag.rejected,
                    diag.retries,
                    state.broken_grips,
                    &state.grip_strain[..6].iter().map(|s| (s * 1000.0).round() / 1000.0).collect::<Vec<_>>(),
                );
            }
        }
    }
}
