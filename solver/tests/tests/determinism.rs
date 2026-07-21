//! Determinism: identical input streams must produce bit-identical trajectories.

use gm_core::PlayerJoint;
use gm_solver::{Effector, Solver, SolverConfig, SolverState};
use gm_tests::XorShift;

fn run_stream(seed: u64, steps: usize) -> SolverState {
    let pose = gm_tests::standing_pose();
    let solver = Solver::new(&pose, SolverConfig::default());
    let mut state = SolverState::from_pose(pose);
    let mut rng = XorShift(seed);
    for _ in 0..steps {
        let effectors = [
            Effector {
                joint: rng.joint(),
                target: gm_core::v3(rng.range(-1.5, 1.5), rng.range(0.0, 2.0), rng.range(-1.5, 1.5)),
                stiffness: rng.range(0.1, 1.0),
            },
            Effector {
                joint: rng.joint(),
                target: gm_core::v3(rng.range(-1.5, 1.5), rng.range(0.0, 2.0), rng.range(-1.5, 1.5)),
                stiffness: rng.range(0.1, 1.0),
            },
        ];
        let (next, _) = solver.step(&state, &effectors, 1.0 / 60.0);
        state = next;
    }
    state
}

#[test]
fn identical_streams_are_bit_identical() {
    let a = run_stream(0x5eed_1234, 240);
    let b = run_stream(0x5eed_1234, 240);
    for pj in PlayerJoint::all() {
        let (pa, pb) = (a.pose[pj], b.pose[pj]);
        assert!(
            pa.x.to_bits() == pb.x.to_bits()
                && pa.y.to_bits() == pb.y.to_bits()
                && pa.z.to_bits() == pb.z.to_bits(),
            "trajectory diverged at {:?}",
            pj
        );
    }
}

#[test]
fn different_seeds_diverge() {
    let a = run_stream(1, 60);
    let b = run_stream(2, 60);
    assert!(a.pose.max_displacement(&b.pose) > 1e-6);
}
