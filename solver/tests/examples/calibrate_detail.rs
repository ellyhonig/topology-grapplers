//! Percentile-level calibration detail: distributions for hinge angles, swing
//! cone angles, and per-limb bone deviations, plus worst-offender entry names.

use gm_core::anatomy::cone_angle;
use gm_core::{angle_at, all_limbs, PlayerId, HINGES, SWING_CONES};

fn pct(sorted: &[f64], p: f64) -> f64 {
    let idx = ((sorted.len() as f64 - 1.0) * p).round() as usize;
    sorted[idx]
}

fn main() {
    let text = std::fs::read_to_string(
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../GrappleMap.txt"),
    )
    .unwrap();
    let entries = gm_core::parse_database(&text).unwrap();

    let mut hinge_samples: Vec<Vec<(f64, String)>> = vec![Vec::new(); HINGES.len()];
    let mut cone_samples: Vec<Vec<(f64, String)>> = vec![Vec::new(); SWING_CONES.len()];
    let limbs: Vec<_> = all_limbs().collect();
    let mut limb_dev: Vec<Vec<f64>> = vec![Vec::new(); limbs.len()];

    for entry in &entries {
        for pose in &entry.frames {
            for player in PlayerId::ALL {
                for (i, h) in HINGES.iter().enumerate() {
                    let ang = angle_at(
                        pose.get(player, h.root),
                        pose.get(player, h.mid),
                        pose.get(player, h.tip),
                    );
                    hinge_samples[i].push((ang, entry.name().to_string()));
                }
                for (i, c) in SWING_CONES.iter().enumerate() {
                    cone_samples[i].push((cone_angle(pose, player, c), entry.name().to_string()));
                }
                for (i, limb) in limbs.iter().enumerate() {
                    let len = pose.get(player, limb.ends[0]).distance(pose.get(player, limb.ends[1]));
                    limb_dev[i].push((len - limb.length) / limb.length);
                }
            }
        }
    }

    println!("hinges: p0.1% / p99.9% / min / max (worst-min entry):");
    for (i, h) in HINGES.iter().enumerate() {
        let mut vals: Vec<f64> = hinge_samples[i].iter().map(|s| s.0).collect();
        vals.sort_by(f64::total_cmp);
        let worst = hinge_samples[i].iter().min_by(|a, b| a.0.total_cmp(&b.0)).unwrap();
        println!(
            "  {:12} {:.3} / {:.3} / {:.3} / {:.3}   ({})",
            h.id,
            pct(&vals, 0.001),
            pct(&vals, 0.999),
            vals[0],
            vals[vals.len() - 1],
            worst.1
        );
    }

    println!("\nswing cones: p99% / p99.9% / max (worst entry):");
    for (i, c) in SWING_CONES.iter().enumerate() {
        let mut vals: Vec<f64> = cone_samples[i].iter().map(|s| s.0).collect();
        vals.sort_by(f64::total_cmp);
        let worst = cone_samples[i].iter().max_by(|a, b| a.0.total_cmp(&b.0)).unwrap();
        println!(
            "  {:15} {:.3} / {:.3} / {:.3}   ({})",
            c.id,
            pct(&vals, 0.99),
            pct(&vals, 0.999),
            vals[vals.len() - 1],
            worst.1
        );
    }

    println!("\nbone deviation from nominal (relative): p99.9% / min / max");
    for (i, limb) in limbs.iter().enumerate() {
        let mut vals = limb_dev[i].clone();
        vals.sort_by(f64::total_cmp);
        let hi = vals[vals.len() - 1];
        let lo = vals[0];
        if hi > 0.12 || lo < -0.12 {
            println!(
                "  {:?}-{:?}: {:.3} / {:.3} / {:.3}",
                limb.ends[0],
                limb.ends[1],
                pct(&vals, 0.999),
                lo,
                hi
            );
        }
    }
}
