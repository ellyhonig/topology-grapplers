//! gm-core: math primitives, body schema, pose representation, and the
//! GrappleMap.txt codec. Everything here is pure and deterministic.

pub mod anatomy;
pub mod body;
pub mod codec;
pub mod frame;
pub mod math;
pub mod pose;
pub mod rest;

pub use anatomy::{Hinge, SwingCone, HINGES, SWING_CONES};
pub use body::{all_limbs, capsule_radius, Chain, Joint, Limb, CHAINS, JOINT_COUNT, LIMBS, NECK_HEAD};
pub use codec::{decode_pose, encode_pose, parse_database, CodecError, DbEntry};
pub use frame::{pelvis_frame, torso_frame, TorsoFrame};
pub use math::{angle_at, clamp, closest_point_on_segment, closest_segment_points, v3, V3};
pub use pose::{PlayerId, PlayerJoint, Pose, P0, P1, PARTICLE_COUNT, PLAYER_COUNT};
pub use rest::{capture_bones, max_nominal_deviation, Bone};
