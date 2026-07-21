# grapple-solver

Rust rewrite of the GrappleMap topology solver: an XPBD (extended position-based
dynamics) constraint solver for real-time two-person grappling, driven by
arbitrary 6-DOF input (VR controllers, GUI gizmo drags, scripts) with hard
guarantees that the result always looks like two anatomically valid humans.

## Design

All motion is produced by one constraint solver. Input never moves joints
directly - it enters as *compliant effectors* ("pull this joint toward this
point with this stiffness") that are projected before the hard constraints:

1. bone distance constraints (rest lengths captured from the loaded pose)
2. hinge minimum angles (calibrated against all 8,323 database frames)
3. hyperextension guards (bend-direction hysteresis; a hinge near straight may
   not fold past straight to the wrong side)
4. swing cones (neck orientation, spine coherence)
5. capsule contacts with per-pair ratchet floors (authored penetration in
   database poses is accepted but may never deepen) and position-level friction
6. floor and arena bounds

After each step a topology watchdog checks for capsule crossings and writhe
jumps (Ho & Komura topology coordinates); a dirty step is re-solved with doubled
substeps and, failing that, rejected outright. Entanglement therefore cannot
change by tunneling.

The whole pipeline is `f64`, allocation-light, iteration-order deterministic:
identical input streams produce bit-identical trajectories (tested).

## Crates

| crate | contents |
|---|---|
| `gm-core` | vector math, joint/limb/chain schema, torso frames, anatomy limits, GrappleMap.txt base62 codec |
| `gm-collision` | capsule inventory (visible limbs + torso volume), contact detection with speculative margins, segment-crossing (tunneling) detection |
| `gm-solver` | the XPBD loop: effectors, constraint projections, watchdog, diagnostics |
| `gm-topology` | writhe matrices, topology coordinates, linking reports |
| `gm-validate` | invariant checker used both in-engine and by every test |
| `gm-wasm` | wasm-bindgen `Engine` API for the browser demo |
| `tests` (`gm-tests`) | database regression, determinism, proptest fuzzing, golden adversarial scenarios, calibration mining tools |

## Build and test

```sh
cargo test --release --workspace          # full suite incl. database regression
cargo run --release -p gm-tests --example calibrate   # re-mine anatomy ranges
```

WASM package for the browser demo (output consumed by
`../src/solver-demo.html`):

```sh
wasm-pack build crates/gm-wasm --target web \
  --out-dir ../../../src/solver-demo/pkg
```

Serve the repo and open the demo:

```sh
cd .. && python3 -m http.server 8765
# http://localhost:8765/src/solver-demo.html
```

Drag joints (multi-pointer: one finger per grappler works), shift-click to pin,
and use the "Two-player input test" button for a scripted simultaneous-input
stress run. The HUD shows live validation, contact clearance, writhe-jump
watchdog status, and solve time (~1.5 ms/step in WASM, well inside a 90 Hz
budget).

## Guarantees and how they are enforced

| guarantee | mechanism | test |
|---|---|---|
| joints never hyperextend | hinge minima + bend-direction hysteresis | `golden.rs::elbow_cannot_hyperextend` |
| bodies never interpenetrate deeper than authored | contact ratchet floors | `fuzz.rs`, `golden.rs::hand_cannot_be_dragged_through_opponents_torso` |
| entanglement never glitches | crossing detection + writhe watchdog, reject-and-resolve | solver watchdog + `validate_step` |
| never stuck, pinned limbs still wiggle | compliant effectors, hard constraints win | `golden.rs::pinned_arm_still_wiggles_and_body_stays_free` |
| robust to any input from both players | all effectors in one constraint set | `fuzz.rs` proptest, `golden.rs::simultaneous_two_player_input_stays_valid` |
| database compatibility | rest lengths + floors captured per pose; limits calibrated on all frames | `db_regression.rs`, `codec_roundtrip.rs` (byte-exact re-encode) |
| determinism | fixed iteration order, pure f64 | `determinism.rs` (bit-exact) |
