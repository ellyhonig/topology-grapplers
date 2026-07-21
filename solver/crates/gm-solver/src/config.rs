//! Solver configuration. Defaults are tuned for interactive manipulation at
//! 60-90 Hz frames with real-time input from both players.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SolverConfig {
    /// Substeps per step call. More substeps = better contact/tunneling behavior.
    pub substeps: usize,
    /// Constraint (Gauss-Seidel) iterations per substep.
    pub iterations: usize,
    /// Gravity (m/s^2, negative = down). On by default: bodies must be
    /// supported (floor, base, or the opponent) or they fall.
    pub gravity: f64,
    /// Exponential velocity damping rate (1/s). High damping makes motion
    /// deliberate and prevents oscillation under adversarial input.
    pub damping: f64,
    /// Muscle tone: fraction of the distance to the shape-matching goal closed
    /// per substep. Tone is what lets a body hold its pose against gravity and
    /// keeps a drag localized (the pose does not fall apart when one limb moves).
    pub tone_stiffness: f64,
    /// Plastic adaptation rate (1/s): how fast the held shape absorbs
    /// deviations beyond the dead zone. This makes sustained input (a dragged
    /// arm, a fall) become the new held pose instead of springing back.
    pub tone_plasticity: f64,
    /// Deviation dead zone (m) below which the held shape does not adapt, so
    /// gravity sag and constraint noise never melt the pose.
    pub tone_deadzone: f64,
    /// Balance servo: fraction of the COM-over-support drift corrected per
    /// substep while the body is standing on its feet. Models active human
    /// balance (ankle strategy); without it a standing body is an unstable
    /// inverted pendulum that slowly tips over from numerical asymmetry.
    pub balance_stiffness: f64,
    /// COM drift (m, beyond the built-in dead zone) at which the balance servo
    /// gives up. Small on purpose: it must catch numerical drift of a standing
    /// body but must not fight authored leans (e.g. bases leaning on the
    /// opponent) or rescue a genuine push - those fall.
    pub balance_margin: f64,
    /// Speculative contact margin (m): contacts activate before touching.
    pub contact_margin: f64,
    /// Maximum speed at which an effector can pull its joint (m/s).
    pub max_effector_speed: f64,
    /// Maximum joint speed (m/s); clamps runaway velocities from any source.
    pub max_joint_speed: f64,
    /// Contact friction coefficient in [0, 1] (fraction of tangential motion
    /// removed while a contact is active).
    pub friction: f64,
    /// Floor friction coefficient in [0, 1].
    pub floor_friction: f64,
    /// Watchdog: max whole-step re-solves (each doubling substeps) when a
    /// segment crossing (topology change) is detected.
    pub max_retries: usize,
    /// Arena half-extent in x and z (GrappleMap database encoding requires
    /// coordinates within [-2, 2]).
    pub arena_half_extent: f64,
}

impl Default for SolverConfig {
    fn default() -> Self {
        SolverConfig {
            substeps: 4,
            iterations: 12,
            gravity: -9.8,
            damping: 12.0,
            tone_stiffness: 0.4,
            tone_plasticity: 0.0,
            tone_deadzone: 0.05,
            balance_stiffness: 0.05,
            balance_margin: 0.15,
            contact_margin: 0.02,
            max_effector_speed: 2.0,
            max_joint_speed: 5.0,
            friction: 0.4,
            floor_friction: 1.0,
            max_retries: 2,
            arena_half_extent: 2.0,
        }
    }
}
