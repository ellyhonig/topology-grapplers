//! Mine GrappleMap.txt for observed anatomical ranges: hinge angles, swing-cone
//! angles, and bone-length deviation from nominal. Used to calibrate the limits
//! in gm-core::anatomy so no real database pose is ever rejected.

use gm_core::anatomy::cone_angle;
use gm_core::{angle_at, capture_bones, PlayerId, HINGES, SWING_CONES};

fn main() {
    let text = std::fs::read_to_string(
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../GrappleMap.txt"),
    )
    .unwrap();
    let entries = gm_core::parse_database(&text).unwrap();

    let mut hinge_min = vec![f64::MAX; HINGES.len()];
    let mut hinge_max = vec![f64::MIN; HINGES.len()];
    let mut cone_max = vec![f64::MIN; SWING_CONES.len()];
    let mut bone_dev: f64 = 0.0;
    let mut frames = 0usize;

    for entry in &entries {
        for pose in &entry.frames {
            frames += 1;
            for player in PlayerId::ALL {
                for (i, h) in HINGES.iter().enumerate() {
                    let ang = angle_at(
                        pose.get(player, h.root),
                        pose.get(player, h.mid),
                        pose.get(player, h.tip),
                    );
                    hinge_min[i] = hinge_min[i].min(ang);
                    hinge_max[i] = hinge_max[i].max(ang);
                }
                for (i, c) in SWING_CONES.iter().enumerate() {
                    cone_max[i] = cone_max[i].max(cone_angle(pose, player, c));
                }
            }
            bone_dev = bone_dev.max(gm_core::max_nominal_deviation(&capture_bones(pose)));
        }
    }

    println!("frames analyzed: {}", frames);
    println!("\nhinges (observed min..max rad, current limits in brackets):");
    for (i, h) in HINGES.iter().enumerate() {
        println!(
            "  {:12} {:.3}..{:.3}  [{:.3}..{:.3}]{}",
            h.id,
            hinge_min[i],
            hinge_max[i],
            h.min,
            h.max,
            if hinge_min[i] < h.min || hinge_max[i] > h.max { "  VIOLATED" } else { "" }
        );
    }
    println!("\nswing cones (observed max rad, current half-angle in brackets):");
    for (i, c) in SWING_CONES.iter().enumerate() {
        println!(
            "  {:15} {:.3}  [{:.3}]{}",
            c.id,
            cone_max[i],
            c.half_angle,
            if cone_max[i] > c.half_angle { "  VIOLATED" } else { "" }
        );
    }
    println!("\nmax bone deviation from nominal: {:.4} (relative)", bone_dev);

    let mut floor_sink: f64 = 0.0;
    for entry in &entries {
        for pose in &entry.frames {
            for pj in gm_core::PlayerJoint::all() {
                floor_sink = floor_sink.max(pj.joint.radius() - pose[pj].y);
            }
        }
    }
    println!("max floor sink below joint radius: {:.4} m", floor_sink);
}
