//! Debug: trace the standing pose under gravity+tone.

use gm_core::{v3, Joint::*, P0};
use gm_solver::{Solver, SolverConfig, SolverState};

fn trace(label: &str, pose: gm_core::Pose, steps: usize) {
    println!("== {}", label);
    let solver = Solver::new(&pose, SolverConfig::default());
    let mut state = SolverState::from_pose(pose);
    let mut rejections = 0;
    for i in 0..steps {
        let (next, diag) = solver.step(&state, &[], 1.0 / 60.0);
        state = next;
        if diag.rejected {
            rejections += 1;
        }
        if i % 30 == 0 || i == steps - 1 {
            let h = state.pose.get(P0, Head);
            let c = state.pose.get(P0, Core);
            let k = state.pose.get(P0, LeftKnee);
            let toe = state.pose.get(P0, LeftToe);
            let heel = state.pose.get(P0, LeftHeel);
            let hand = state.pose.get(P0, LeftHand);
            println!(
                "step {:3}  head y{:.2} z{:+.2}  core y{:.2} z{:+.2}  knee y{:.2} z{:+.2}  toe y{:.2} z{:+.2}  heel y{:.2} z{:+.2}  hand y{:.2}  rej {}",
                i, h.y, h.z, c.y, c.z, k.y, k.z, toe.y, toe.z, heel.y, heel.z, hand.y, diag.rejected
            );
        }
    }
    println!("max disp {:.3}  rejections {}", state.pose.max_displacement(&pose), rejections);
}

fn leaning_pose(ang: f64) -> gm_core::Pose {
    let mut pose = gm_tests::standing_pose();
    let pivot = (pose.get(P0, LeftAnkle) + pose.get(P0, RightAnkle)) * 0.5;
    let (sin, cos) = ang.sin_cos();
    for j in gm_core::Joint::ALL {
        let p = pose.get(P0, j) - pivot;
        pose.set(P0, j, pivot + v3(p.x, p.y * cos - p.z * sin, p.y * sin + p.z * cos));
    }
    pose
}

fn arm_drag_trace(label: &str, cfg: SolverConfig) {
    use gm_core::PlayerJoint;
    use gm_solver::Effector;
    println!("== arm drag {}", label);
    let pose = gm_tests::standing_pose();
    let solver = Solver::new(&pose, cfg);
    let mut state = SolverState::from_pose(pose);
    let shoulder = pose.get(P0, RightShoulder);
    let target = shoulder + v3(0.55, -0.15, 0.0);
    let e = Effector {
        joint: PlayerJoint { player: P0, joint: RightHand },
        target,
        stiffness: 0.9,
    };
    for i in 0..180 {
        let (next, _) = solver.step(&state, &[e], 1.0 / 60.0);
        state = next;
        if i % 20 == 0 || i == 179 {
            let h = state.pose.get(P0, RightHand);
            let c = state.pose.get(P0, Core);
            let s = state.pose.get(P0, RightShoulder);
            println!(
                "step {:3}  hand ({:+.2},{:.2},{:+.2}) resid {:.3}  core ({:+.2},{:.2},{:+.2})  shoulder ({:+.2},{:.2},{:+.2})",
                i, h.x, h.y, h.z, h.distance(target), c.x, c.y, c.z, s.x, s.y, s.z
            );
        }
    }
}

fn main() {
    trace("standing", gm_tests::standing_pose(), 240);
    trace("leaning 25deg forward (toward opponent)", leaning_pose(0.45), 300);
    trace("leaning 25deg backward", leaning_pose(-0.45), 300);
    arm_drag_trace("default", SolverConfig::default());
    arm_drag_trace("no balance", SolverConfig { balance_stiffness: 0.0, ..SolverConfig::default() });
    arm_drag_trace(
        "tone only (no gravity, no balance)",
        SolverConfig { gravity: 0.0, balance_stiffness: 0.0, ..SolverConfig::default() },
    );
    arm_drag_trace(
        "no tone (gravity + balance only)",
        SolverConfig { tone_stiffness: 0.0, tone_plasticity: 0.0, ..SolverConfig::default() },
    );
    arm_drag_trace(
        "nothing (no tone, no gravity, no balance)",
        SolverConfig {
            tone_stiffness: 0.0,
            tone_plasticity: 0.0,
            gravity: 0.0,
            balance_stiffness: 0.0,
            ..SolverConfig::default()
        },
    );
}
