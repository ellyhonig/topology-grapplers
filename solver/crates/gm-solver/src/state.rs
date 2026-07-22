//! Solver state: pose + velocities + per-hinge bend-direction memory.
//! Plain data, cheap to clone; `step` is state-in state-out.

use gm_core::{PlayerId, Pose, V3, HINGES, JOINT_COUNT, PLAYER_COUNT};
use serde::{Deserialize, Serialize};

pub const HINGE_COUNT: usize = HINGES.len();

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SolverState {
    pub pose: Pose,
    pub velocity: [[V3; JOINT_COUNT]; PLAYER_COUNT],
    /// Remembered bend direction per hinge, in torso-local coordinates.
    /// Updated only while the hinge is clearly bent (well-defined bend plane);
    /// consulted when near straight to block hyperextension (bending past
    /// straight to the opposite side). `None` until first observed bent.
    pub bend_ref_local: [[Option<V3>; HINGE_COUNT]; PLAYER_COUNT],
    /// Muscle-tone rest shape per player: joint offsets from the mass-weighted
    /// centroid. Plastically adapted each step (see `tone`).
    pub tone_rest: crate::tone::RestShape,
    /// Bitmask of grips (by index into the solver's grip list) that are gone
    /// for good: the user drove the gripping limb away (deliberately let go)
    /// or the grip was strained past its strength and fully paid out. A
    /// broken grip never re-engages.
    pub broken_grips: u32,
    /// Accumulated separation demand per grip (meters of attempted stretch,
    /// leaky). Gravity-scale loads decay away; a sustained deliberate yank
    /// accumulates until the grip breaks (grips have finite strength).
    pub grip_strain: [f64; 32],
    /// Extra tether length per grip; nonzero marks a grip that has failed and
    /// is paying out. Grows at a fixed rate until past usefulness, then the
    /// broken bit is set - a smooth "letting go" instead of a hard constraint
    /// vanishing mid-frame (which snaps the stretched limb back in one frame).
    pub grip_release: [f64; 32],
}

impl SolverState {
    /// Create a state at rest in the given pose, with bend memory initialized
    /// from the pose itself.
    pub fn from_pose(pose: Pose) -> SolverState {
        let mut state = SolverState {
            pose,
            velocity: [[V3::ZERO; JOINT_COUNT]; PLAYER_COUNT],
            bend_ref_local: [[None; HINGE_COUNT]; PLAYER_COUNT],
            tone_rest: crate::tone::capture_rest_shape(&pose),
            broken_grips: 0,
            grip_strain: [0.0; 32],
            grip_release: [0.0; 32],
        };
        crate::anatomy_constraints::refresh_bend_memory(&mut state);
        state
    }

    pub fn vel(&self, player: PlayerId, joint: gm_core::Joint) -> V3 {
        self.velocity[player.index()][joint.index()]
    }
}
