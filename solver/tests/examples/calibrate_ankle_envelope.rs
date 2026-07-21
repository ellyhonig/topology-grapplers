//! Mine shin-local ankle-orientation changes from every adjacent database frame.
//!
//! Each first frame supplies the same rest-relative reference used by Solver;
//! the following frame is measured in its own current shin frame.  Position
//! entries and every individual frame are also checked as their own authored
//! neutral, which must measure exactly zero and remain finite.

use gm_solver::ankle_constraints::{ankle_angles, capture_ankle_reference, LegSide};

fn percentile(sorted: &[f64], fraction: f64) -> f64 {
    let index = ((sorted.len() - 1) as f64 * fraction).round() as usize;
    sorted[index]
}

fn main() {
    let entries = gm_tests::load_database();
    let mut swings = Vec::new();
    let mut twists = Vec::new();
    let mut neutral_checked = 0usize;
    let mut adjacent_checked = 0usize;
    let mut fallback_measurements = 0usize;

    for entry in &entries {
        for pose in &entry.frames {
            for player in gm_core::PlayerId::ALL {
                for side in LegSide::ALL {
                    let reference = capture_ankle_reference(pose, player, side);
                    let neutral = ankle_angles(pose, player, side, &reference)
                        .expect("database foot triangles must be nondegenerate");
                    assert!(neutral.swing.abs() < 1e-7);
                    assert!(neutral.twist.abs() < 1e-7);
                    neutral_checked += 1;
                }
            }
        }
        for pair in entry.frames.windows(2) {
            for player in gm_core::PlayerId::ALL {
                for side in LegSide::ALL {
                    let reference = capture_ankle_reference(&pair[0], player, side);
                    let angles = ankle_angles(&pair[1], player, side, &reference)
                        .expect("database foot triangles must be nondegenerate");
                    swings.push(angles.swing.to_degrees());
                    twists.push(angles.twist.abs().to_degrees());
                    fallback_measurements += usize::from(angles.used_straight_leg_fallback);
                    adjacent_checked += 1;
                }
            }
        }
    }

    swings.sort_by(f64::total_cmp);
    twists.sort_by(f64::total_cmp);
    println!("neutral_frames_checked={neutral_checked}");
    println!("adjacent_ankle_pairs_checked={adjacent_checked}");
    println!("straight_leg_fallback_measurements={fallback_measurements}");
    for (label, fraction) in [("p50", 0.50), ("p90", 0.90), ("p95", 0.95), ("p99", 0.99)] {
        println!(
            "{label}_swing_deg={:.4},{label}_abs_twist_deg={:.4}",
            percentile(&swings, fraction),
            percentile(&twists, fraction),
        );
    }
    println!(
        "max_swing_deg={:.4},max_abs_twist_deg={:.4}",
        swings.last().unwrap(),
        twists.last().unwrap(),
    );
}
