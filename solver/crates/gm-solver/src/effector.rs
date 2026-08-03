//! Effectors: the only way input enters the solver.
//!
//! A VR controller pose, a GUI gizmo drag, or a scripted test all reduce to the
//! same thing: "pull this joint of this player toward this world point with this
//! stiffness". Effectors are *compliant* - they are projected before the hard
//! constraints (bones, joint limits, contacts), so no input, however adversarial,
//! can force an anatomically invalid or penetrating pose. A pinned limb under an
//! effector simply expresses whatever residual motion the constraints allow.

use gm_core::{PlayerJoint, V3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Effector {
    pub joint: PlayerJoint,
    pub target: V3,
    /// Fraction of the remaining distance closed per frame at stiffness 1.0
    /// (before hard constraints and the per-substep speed clamp apply).
    /// Typical GUI drag: 0.6-1.0. Soft pull: 0.1-0.3.
    pub stiffness: f64,
}
