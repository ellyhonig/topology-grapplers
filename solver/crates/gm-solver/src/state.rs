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
    /// A foot-orientation drive has displaced this ankle and its release is
    /// still being unloaded by the soft local constraint. This gates gradual
    /// tone recovery so unrelated idle ankle motion never relaxes a leg.
    pub ankle_release_active: [[bool; 2]; PLAYER_COUNT],
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
    /// Solver-owned acquisition progress for live hand grips. Zero is the
    /// captured hand shape; one is the full contact-following curl.
    pub grip_wrap: [f64; 32],
    /// Fraction of the two live hand points that have made sticky contact.
    /// Unlike the old dwell value this is monotonic for the life of a grip:
    /// once a hand/finger point touches, it stays latched until explicit
    /// release.
    pub grip_contact: [f64; 32],
    /// Fraction of the solver-selected wrist-to-finger curl that the hand
    /// actually achieved around the target capsule.
    pub grip_coverage: [f64; 32],
    /// Binary live grip strength. Any captured contact point makes the runtime
    /// grip full strength; wrap coverage never scales holding force.
    pub grip_strength: [f64; 32],
    /// Bit 0 is the hand contact and bit 1 is the finger contact.
    pub grip_contact_bits: [u8; 32],
    /// Material anchors for sticky runtime contacts. `s` locates the point
    /// along the target capsule's segment; the radial offset follows changes
    /// in the segment axis so the contact point moves with the grabbed limb.
    pub grip_anchor_axis: [V3; 32],
    pub grip_hand_anchor_s: [f64; 32],
    pub grip_hand_anchor_radial: [V3; 32],
    pub grip_finger_anchor_s: [f64; 32],
    pub grip_finger_anchor_radial: [V3; 32],
}

impl SolverState {
    /// Create a state at rest in the given pose, with bend memory initialized
    /// from the pose itself.
    pub fn from_pose(pose: Pose) -> SolverState {
        let mut state = SolverState {
            pose,
            velocity: [[V3::ZERO; JOINT_COUNT]; PLAYER_COUNT],
            bend_ref_local: [[None; HINGE_COUNT]; PLAYER_COUNT],
            ankle_release_active: [[false; 2]; PLAYER_COUNT],
            tone_rest: crate::tone::capture_rest_shape(&pose),
            broken_grips: 0,
            grip_strain: [0.0; 32],
            grip_release: [0.0; 32],
            grip_wrap: [0.0; 32],
            grip_contact: [0.0; 32],
            grip_coverage: [0.0; 32],
            grip_strength: [0.0; 32],
            grip_contact_bits: [0; 32],
            grip_anchor_axis: [V3::ZERO; 32],
            grip_hand_anchor_s: [0.0; 32],
            grip_hand_anchor_radial: [V3::ZERO; 32],
            grip_finger_anchor_s: [0.0; 32],
            grip_finger_anchor_radial: [V3::ZERO; 32],
        };
        crate::anatomy_constraints::refresh_bend_memory(&mut state);
        state
    }

    pub fn vel(&self, player: PlayerId, joint: gm_core::Joint) -> V3 {
        self.velocity[player.index()][joint.index()]
    }
}
