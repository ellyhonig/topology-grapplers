# Lower-limb pose retention in the WASM grappler solver

## Scope

This note began as an observation pass. It identifies why foot dragging felt
more like pulling a rope than manipulating a leg and outlines the longer-term
constraint work. It now also records the first bounded implementation task,
its proof obligation, measured result, and reproduction steps.

The browser demo was observed with the current WASM build, including a standing
pose and an engaged foot tracker. The behavior agrees with the Rust solver's
constraint and input paths. Nothing in the WASM wrapper appears to be dropping
an anatomical constraint: the wrapper passes effectors directly into the Rust
solver and returns the resulting particle positions.

## First implementation task: retain tone when a foot tracker engages

**Status: completed.** The first task was deliberately narrower than solving
the complete lower-limb anatomy model:

> Engaging a stationary ankle/heel/toe tracker must not turn the driven leg
> into a passive chain or cause it to collapse under gravity.

This is a useful first theorem because a stationary target supplies no intended
motion. Any large movement after engagement is therefore solver-induced drift,
not an ambiguous question of how strongly an athlete should resist a drag.

### Change

`solver/crates/gm-solver/src/solver.rs` now uses a lower-limb-specific
tone-retention gradient. For a driven leg, the retained fractions of normal
tone are:

| Coordinate | Retained tone |
|---|---:|
| directly targeted foot coordinate | 0.00 |
| untargeted ankle/heel/toe coordinate | 0.10 |
| knee | 0.40 |
| hip | 0.70 |

The directly commanded coordinate remains free of a direct restoring pull, so
the effector still wins locally. Retention increases toward the pelvis, where
unintended motion is more costly. Arm, neck, core, grip-ownership, and hard
constraint behavior are unchanged.

Desktop foot orientation normally supplies ankle, heel, and toe effectors at
once. Each target coordinate is therefore at zero direct tone, while repeated
orientation effectors leave the knee at `0.40` and the hip at `0.70`; applying
the same attenuation more than once is idempotent.

### Mathematical proof obligation

For joint `j`, the tone projection first computes the restoring component

`r_j = k s_j (g_j - p_j)`,

where `k` is configured tone stiffness, `s_j` is the retained-tone scale,
`g_j` is the shape-matching goal, and `p_j` is the current point. The solver
then subtracts a shared mass-weighted mean correction so tone cannot translate
the center of mass. That shared correction may move a directly targeted point,
but it does not change the ordering of the direct restoring coefficients.

For the new leg policy,

`0 = s_target < s_foot = 0.10 < s_knee = 0.40 < s_hip = 0.70 <= 1`.

Consequently, for equal goal error, the magnitudes of the direct restoring
terms have the ratio `0 : 0.10 : 0.40 : 0.70`. With the default `k = 0.40`,
the corresponding per-substep coefficients are `0`, `0.04`, `0.16`, and
`0.28`. Thus the exact target is not directly opposed, while the knee and hip
provably cannot have the zero restoring coefficient that produced the passive
chain. Unit tests exhaust the other discrete cases relevant to this change:

- the scale is strictly increasing from target to hip;
- the opposite leg and other player remain at full scale;
- ankle/heel/toe orientation effectors cannot cumulatively re-relax the knee
  or hip; and
- the existing arm policy is unchanged.

This proves the attenuation policy, not anatomical realism. The latter still
requires local joint frames and the constraints listed later in this note.

### Behavioral regression result

The deterministic regression pose pins the pelvis (`LeftHip`, `RightHip`, and
`Core`) to remove whole-body translation, settles for 60 frames, and then holds
stationary left ankle/heel/toe targets for 60 frames at 60 Hz. The acceptance
bound is less than 2 cm of ankle drift, knee drift, and tracker residual.

| Release build | Ankle drift after 1 s | Knee drift after 1 s | Result |
|---|---:|---:|---|
| old binary leg relaxation | 28.0003 cm | 43.0233 cm | fail |
| graded `0/0.10/0.40/0.70` policy | 0.3500 cm | 0.4546 cm | pass |

The automated test is
`stationary_foot_tracker_does_not_collapse_the_driven_leg`. A reusable
translation sweep is in
`solver/tests/examples/probe_lower_limb_tone.rs`; it emits CSV for targets from
0 through 20 cm so later constraint work can be compared against the same
setup.

### How to verify this yourself

From `topology-grapplers/solver`, run the focused behavioral regression:

```sh
cargo test --release -p gm-solver \
  stationary_foot_tracker_does_not_collapse_the_driven_leg
```

Then verify the exact scale invariants and print the diagnostic sweep:

```sh
cargo test --release -p gm-solver tone_scale_tests
cargo run --release -p gm-tests --example probe_lower_limb_tone
```

In the sweep, the `target_cm=0` row should report about `0.35` cm ankle
residual/drift and `0.45` cm knee drift, with zero pinned hip/core movement and
`rejected=false`. Small floating-point variation is acceptable; the automated
bound is 2 cm. Finally, run every solver, database, fuzz, determinism, and WASM
test:

```sh
cargo test --release --workspace
```

For a visual check, rebuild the browser package and serve the repository:

```sh
wasm-pack build crates/gm-wasm --target web \
  --out-dir ../../../src/solver-demo/pkg
cd ..
python3 -m http.server 8765
```

Open `http://localhost:8765/src/solver-demo.html`, engage a foot without first
moving it, and hold it for one second. The knee/ankle should retain their pose
instead of going slack. Then drag the foot deliberately. Large translations,
knee-plane stability, and ankle roll are diagnostic observations for the next
tasks, not pass criteria for this first intervention.

## Reported and reproduced symptoms

- Arms appear to retain their authored pose better than legs.
- A foot can pronate/roll with little sense of ankle tension.
- Pulling a flexed foot toward the head produces little progressive resistance.
  Meaningful load arrives mainly when the hip-knee-ankle chain becomes taut, at
  which point the hip and body are dragged along.
- Moving a foot back and forth can send the knee medially/laterally too easily.
- The hip can internally rotate far beyond what feels plausible for the current
  pose.

The important qualitative signature is **slack followed by a hard reach limit**.
That is exactly what a distance-constrained chain produces when it has little or
no angular spring behavior.

## What the current model actually represents

Each grappler is a set of world-space point particles. Bones constrain distances
between those points. This represents segment length well, but a single line
segment has no orientation about its own axis. Consequently, femoral rotation,
tibial rotation, and joint torque are not represented directly.

The foot is a rigid-ish triangle formed by ankle, heel, and toe distances. It is
connected to the shin at only the ankle point. That shared point behaves like an
unrestricted ball joint unless another constraint restricts the foot relative
to the shin. There currently is no such ankle constraint.

The leg chain therefore has these protections:

- fixed bone lengths;
- a very permissive minimum knee angle;
- a knee hyperextension guard only near straight;
- collisions, floor contact, and damping;
- whole-body shape-matching tone, with graded foot/knee/hip attenuation while
  the leg is being driven.

It does **not** have:

- ankle dorsiflexion/plantarflexion limits;
- ankle inversion/eversion or axial-roll limits;
- knee varus/valgus resistance;
- persistent knee bend-plane resistance while flexed;
- hip internal/external rotation limits;
- a hip swing limit;
- a progressive angular/torsional tension curve for the lower limb.

## Primary causes

### 1. Driving a foot completely relaxed the leg (first intervention complete)

Before the first intervention, `solver/crates/gm-solver/src/solver.rs` grouped
knee, ankle, heel, and toe as one driven limb. If an effector targeted any
member of that group, every joint in the group received a tone scale of `0.0`;
the hip retained only `0.3` of normal tone.

This was the most direct explanation for the missing pose retention. While a
foot tracker was engaged, the only continuous spring that tried to preserve the
authored leg shape was intentionally removed from the entire leg. The remaining
rules mostly enforced validity, not a human-like resistance curve.

The completed first intervention replaces that binary lower-limb rule with the
graded policy proved above. This removes stationary-engagement collapse, but it
does not yet supply a human-like resistance curve for large motion.

### 2. The knee guard prevents only one narrow failure mode

The bend-memory constraint engages only when the knee is close to straight. It
prevents the knee from snapping through the straight line to the opposite side.
When the knee is clearly flexed, its remembered bend direction is immediately
updated from the current pose.

That means a flexed knee may sweep inward or outward and the solver treats the
new bend plane as valid, rather than resisting the change. This is why dragging
the foot laterally can rotate the knee inward without first building tension.
The guard is a hyperextension safety mechanism, not a knee-stability model.

### 3. Hip limits were deliberately omitted

The anatomy table has only neck and spine swing cones. Shoulder and hip cones
were removed because the full pose database contains legitimate grappling poses
in nearly every global direction.

That database-wide observation is valid but answers a different question. A hip
may need a very large overall workspace across all grappling poses while still
resisting rapid departure from the **currently authored** pose. A broad hard
validity envelope and a local soft pose-retention constraint are complementary,
not mutually exclusive.

With no hip cone, no axial rotation state, and reduced hip tone during a foot
drag, there is little to prevent apparent extreme internal rotation.

### 4. The ankle has geometry but no anatomy

The foot triangle preserves its own shape, but no hinge, cone, or twist
constraint relates that triangle to the shin. Heel and toe can therefore orbit
the ankle subject mainly to their fixed distances and collisions.

In the XR path, foot orientation targets the ankle and toe but deliberately does
not target the heel. The existing comment explains that targeting the heel made
the foot into a lever that swung the knee and hip. Omitting the heel avoids that
artifact, but leaves roll about the ankle-toe axis underconstrained. It treats a
symptom of the missing ankle/leg rotational model rather than supplying that
model.

Desktop manipulation targets ankle, toe, and heel, so it can command a full foot
orientation, but the solver still lacks anatomical resistance between that
orientation and the shin.

### 5. Damping is not pose retention

Velocity damping removes oscillation and slows motion. It does not create a
static restoring torque. Once a joint has been displaced and velocity reaches
zero, damping has nothing to say about whether that pose should be held or
resisted.

### 6. Validation mostly tests validity, not leg feel

The existing guarantees emphasize bone length, minimum angles, collisions,
topology, bounded speed, and deterministic behavior. A leg can satisfy all of
those checks while exhibiting unrealistic knee-plane motion or ankle roll. The
first intervention adds a stationary-tracker retention criterion, but there is
still no acceptance criterion such as "a 5 cm foot displacement must create
measurable knee/hip resistance before full extension."

## Why the arms can look better

The arm also relaxes under direct input, so the difference is not simply that
arms have tone and legs do not. Several structural details improve the arm's
appearance:

- the hand tracker targets hand, fingers, and wrist, providing a fuller distal
  orientation frame;
- the anatomy table contains wrist and hand hinge-angle constraints in addition
  to the elbow;
- the observed arm poses and contacts often give the distal chain more geometric
  cues than the underconstrained ankle-to-shin connection.

The arm is not a complete rotational anatomy model either. It is simply less
visibly underconstrained in the interactions being compared.

## Recommended direction

The likely fix is a combination of soft, pose-relative constraints and hard
safety limits. Merely raising global tone or adding one fixed hip cone is likely
to create new problems.

### A. Stop relaxing the entire lower limb as one binary unit

The first bounded version of this direction is complete. The implemented
gradient and its stationary-tracker regression are documented above. Further
tuning should be based on the translation and rotation probes, not on the
stationary criterion alone.

Use a graded response when the foot is driven:

- the directly targeted ankle/foot coordinates may relax strongly;
- the knee should retain a meaningful fraction of pose tone;
- the hip/pelvis should retain more tone than the knee;
- relaxation can fall off by graph distance from the effector;
- orientation-driving auxiliaries should not independently zero the whole limb.

This should create early, progressive resistance while still allowing a user to
move the foot deliberately.

### B. Add soft rest-relative lower-limb constraints

Capture useful relationships from the loaded pose, then resist deviations with
finite compliance:

- knee bend-plane orientation relative to the pelvis/femur;
- hip-to-knee direction relative to the pelvis;
- foot frame relative to the shin;
- optionally, a preferred knee flexion angle with a dead zone.

These should be soft and capable of yielding under sustained input. Their job is
"this leg initially wants to remain like the authored leg," not "the hip may
never enter a direction absent from standing anatomy."

### C. Add hard anatomical safety envelopes separately

Soft retention is not enough at extremes. Add limits for:

- knee varus/valgus and hyperextension;
- hip internal/external rotation and extreme adduction/abduction;
- ankle dorsiflexion/plantarflexion;
- ankle inversion/eversion.

Because a pure point chain cannot observe all axial rotations, these may require
virtual local frames or additional orientation state. Limits should be mined
against the database, but preferably in pelvis/femur/shin-local coordinates and
conditioned on flexion rather than as one global cone.

### D. Give the foot a real orientation relationship to the shin

The ankle needs more than a shared point. Two plausible paths are:

1. derive a foot frame from ankle/heel/toe and a shin frame from hip/knee/ankle,
   then constrain their relative swing and twist; or
2. introduce oriented segment/joint state (or carefully chosen virtual points)
   so axial rotation is represented explicitly.

Once that relationship exists, XR can drive a full foot orientation without
using the heel as an uncontrolled lever on the entire leg. Position and
orientation effectors should have separate compliance.

### E. Use progressive resistance

A leg should not be either slack or hard-blocked. A useful response curve has:

- a small neutral dead zone;
- moderate muscular resistance through ordinary displacement;
- increasing resistance near anatomical limits;
- a hard safety projection only at the final envelope.

This would make pulling a foot toward the head load the ankle, knee, and hip
before the chain reaches maximum length.

## Suggested diagnostic experiments for the remaining work

Add measurement probes before tuning constants so improvements can be compared
objectively:

1. **Foot translation (initial probe implemented):** Move a foot target in 2 cm
   increments toward the head. Record ankle residual, knee displacement, hip
   displacement, and pelvis displacement. The deterministic anchored-pelvis
   sweep now exists in `probe_lower_limb_tone.rs`; it remains a diagnostic, not
   an acceptance curve.
2. **Knee valgus/varus:** Move the ankle medially and laterally with hip position
   held approximately constant. Record knee-plane angle relative to the pelvis.
3. **Foot roll:** Rotate a foot tracker about its forward axis at fixed ankle
   position. Record heel orbit, foot-to-shin relative rotation, knee motion, and
   hip motion.
4. **Hip rotation:** Sweep the knee around the hip at several knee flexion
   angles. Measure pelvis-local internal/external rotation and the resistance
   proxy.
5. **Arm/leg comparison:** Apply matched normalized disturbances to a hand and a
   foot and compare how much motion reaches elbow/knee and shoulder/hip.
6. **Database compatibility:** Run every proposed hard envelope against all
   database frames. Run soft constraints as settle tests and ensure they do not
   noticeably rewrite authored grappling positions.

Effector residual is an available first resistance proxy, but it is not force.
For better tuning, diagnostics should expose the correction magnitude or XPBD
multiplier contributed by each soft anatomical constraint.

## Proposed order of attack

1. Expand the completed translation probe with knee-plane and foot-roll probes.
2. **Completed:** change lower-limb tone relaxation from binary to graded and
   add a stationary-tracker regression.
3. Add a soft knee bend-plane/rest-orientation constraint, using the new probe
   as its acceptance test.
4. Add a soft foot-to-shin orientation constraint and separate foot position
   from orientation compliance.
5. Add hard ankle, knee, and hip safety envelopes.
6. Tune against both the probes and the complete GrappleMap pose database.

The first two or three steps should address most of the "rope" sensation. The
orientation representation and safety-envelope work are what will address free
ankle roll and extreme internal rotation robustly.

## Bottom line

The current lower limb is anatomically valid only in a coarse positional sense.
The first intervention keeps graded tone active when the foot is driven and
prevents a stationary tracker from collapsing the leg. The remaining
constraints still mostly say "keep every bone the right length and do not
hyperextend through straight." Without local angular constraints, larger
motions can still produce a floppy chain whose first strong tension appears at
full reach.

The solution is not in the WASM boundary. It is to retain graded lower-limb tone,
model knee/hip/ankle orientation explicitly enough to observe the problematic
motions, and combine soft rest-relative tension with hard anatomical limits.
