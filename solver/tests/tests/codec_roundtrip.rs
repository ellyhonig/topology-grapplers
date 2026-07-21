//! Full-database codec regression: every pose block in GrappleMap.txt must
//! decode and re-encode to byte-identical text.

use gm_core::{encode_pose, parse_database};

#[test]
fn full_database_parses() {
    let entries = gm_tests::load_database();
    let positions = entries.iter().filter(|e| e.is_position()).count();
    let transitions = entries.len() - positions;
    assert!(positions >= 600, "expected >=600 positions, got {}", positions);
    assert!(transitions >= 1400, "expected >=1400 transitions, got {}", transitions);
    let total_frames: usize = entries.iter().map(|e| e.frames.len()).sum();
    assert!(total_frames >= 8000, "expected >=8000 frames, got {}", total_frames);
}

#[test]
fn full_database_round_trips_byte_exact() {
    let text = std::fs::read_to_string(gm_tests::database_path()).unwrap();
    let entries = parse_database(&text).unwrap();

    // Re-serialize: description lines followed by encoded frames, in order.
    let mut out = String::with_capacity(text.len());
    for entry in &entries {
        for line in &entry.description {
            out.push_str(line);
            out.push('\n');
        }
        for frame in &entry.frames {
            out.push_str(&encode_pose(frame).unwrap());
        }
    }

    assert_eq!(out.len(), text.len(), "re-encoded database differs in size");
    assert!(out == text, "re-encoded database differs from original");
}

#[test]
fn all_poses_are_finite_and_in_bounds() {
    for entry in gm_tests::load_database() {
        for (i, frame) in entry.frames.iter().enumerate() {
            assert!(frame.is_finite(), "{} frame {}", entry.name(), i);
            for pj in gm_core::PlayerJoint::all() {
                let p = frame[pj];
                assert!(
                    (-2.0..=2.0).contains(&p.x)
                        && (0.0..=2.0).contains(&p.y)
                        && (-2.0..=2.0).contains(&p.z),
                    "{} frame {} joint {:?} out of bounds: {:?}",
                    entry.name(),
                    i,
                    pj,
                    p
                );
            }
        }
    }
}
