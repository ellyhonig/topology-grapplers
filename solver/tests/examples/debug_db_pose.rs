//! Debug: trace the database pose that destabilizes under gravity+tone.

use gm_core::PlayerJoint;
use gm_solver::{Solver, SolverConfig, SolverState};

fn main() {
    let entries = gm_tests::load_database();
    let needle = std::env::args().nth(1).unwrap_or_else(|| "side ctrl w/".into());
    let entry = entries
        .iter()
        .filter(|e| e.is_position())
        .find(|e| e.name().contains(&needle))
        .expect("pose present");
    println!("pose: {:?}", entry.name());
    let pose = entry.frames[0];
    let configs: [(&str, SolverConfig); 5] = [
        ("default", SolverConfig::default()),
        ("no balance", SolverConfig { balance_stiffness: 0.0, ..SolverConfig::default() }),
        ("no plasticity", SolverConfig { tone_plasticity: 0.0, ..SolverConfig::default() }),
        ("no tone", SolverConfig { tone_stiffness: 0.0, tone_plasticity: 0.0, ..SolverConfig::default() }),
        (
            "no tone no balance",
            SolverConfig {
                tone_stiffness: 0.0,
                tone_plasticity: 0.0,
                balance_stiffness: 0.0,
                ..SolverConfig::default()
            },
        ),
    ];
    for (label, cfg) in configs {
        let solver = Solver::new(&pose, cfg);
        let mut state = SolverState::from_pose(pose);
        let mut worst_step = 0.0f64;
        let mut drift_at = [0.0f64; 4]; // after 30, 60, 120, 240 steps
        for step in 0..240 {
            let prev = state.pose;
            let (next, _) = solver.step(&state, &[], 1.0 / 60.0);
            state = next;
            for pj in PlayerJoint::all() {
                worst_step = worst_step.max(state.pose[pj].distance(prev[pj]));
            }
            match step + 1 {
                30 => drift_at[0] = state.pose.max_displacement(&pose),
                60 => drift_at[1] = state.pose.max_displacement(&pose),
                120 => drift_at[2] = state.pose.max_displacement(&pose),
                240 => drift_at[3] = state.pose.max_displacement(&pose),
                _ => {}
            }
        }
        let worst_joint = PlayerJoint::all()
            .max_by(|a, b| {
                let da = state.pose[*a].distance(pose[*a]);
                let db = state.pose[*b].distance(pose[*b]);
                da.partial_cmp(&db).unwrap()
            })
            .unwrap();
        println!(
            "{:20} drift@30/60/120/240 {:.3}/{:.3}/{:.3}/{:.3}  worst step {:.3}  worst joint p{}:{:?} ({:.3})",
            label,
            drift_at[0],
            drift_at[1],
            drift_at[2],
            drift_at[3],
            worst_step,
            worst_joint.player.index(),
            worst_joint.joint,
            state.pose[worst_joint].distance(pose[worst_joint]),
        );
    }
}
