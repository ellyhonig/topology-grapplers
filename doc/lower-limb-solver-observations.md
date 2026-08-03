# Lower-limb pose retention in the WASM grappler solver

## Scope

This note began as an observation pass. It identifies why foot dragging felt
more like pulling a rope than manipulating a leg and outlines the longer-term
constraint work. It now also records the first bounded implementation task,
the next diagnostic task, their proof obligations, measured results, and
reproduction steps.

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

## Next diagnostic task: measure knee-plane motion and foot roll

**Status: completed.** This task adds deterministic measurement tools without
changing solver behavior:

- `probe_lower_limb_knee_plane.rs` translates the complete left-foot frame from
  20 cm lateral to 20 cm medial and records pelvis-relative knee-plane angle,
  knee flexion, joint displacement, target residual, bone error, and rejected
  frame count.
- `probe_lower_limb_foot_roll.rs` commands roll from -30 through +30 degrees at
  a fixed ankle and records achieved tracker roll, foot-to-shin relative roll,
  heel orbit, proximal motion, target residual, bone error, and rejected frame
  count.
- `lower_limb_metrics.rs` contains the reusable geometry and five unit tests for
  its proof obligations.

Both probes pin `LeftHip`, `RightHip`, and `Core`, settle for 60 frames, reset to
that same reference state for every row, and hold each target for 60 frames at
60 Hz. Thus rows are independent and hip/core zeros verify the pins rather than
whole-body translation being mistaken for local joint motion. CSV is written to
standard output so revisions can be compared directly.

### Mathematical proof obligation for the diagnostics

For a unit measurement axis `u`, define the perpendicular projection

`P_u(v) = v - u(u dot v)`.

After normalizing projected directions `a` and `b`, both lie in the plane normal
to `u`. Therefore

`a dot b = cos(theta)`

and

`u dot (a cross b) = sin(theta)`.

The probes compute

`theta = atan2(u dot (a cross b), a dot b)`.

This recovers the signed angle in `(-pi, pi]`, unlike `acos`, which would lose
medial/lateral or clockwise/counterclockwise direction. The shortest difference
between two results uses `atan2(sin(theta - theta_0), cos(theta - theta_0))`, so
a transition across the +/-180-degree boundary does not report a false
358-degree change.

For the knee metric, `u` is the hip-to-ankle axis, `a` is pelvis-forward
projected normal to `u`, and `b` is the projected hip-to-knee bend vector. This
is zero when the knee bends toward pelvis-forward and is invariant under any
shared world translation or proper rotation: position differences cancel the
translation, while dot and cross products are preserved by an orthogonal
rotation. `knee_flexion_deg` is emitted beside the angle because bend-plane
azimuth becomes ill-conditioned as flexion approaches zero.

For the foot-to-shin metric, `u` is heel-to-toe, `a` is the normal of the
hip-knee-ankle plane, and `b` is the normal of the ankle-heel-toe plane. The leg
plane supplies the rotational reference that a point-only shin segment cannot
supply by itself.

Foot-roll targets use Rodrigues rotation about the line through the ankle:

`R_u(v) = v cos(alpha) + (u cross v) sin(alpha) + u(u dot v)(1 - cos(alpha))`.

For unit `u`, `R_u` is orthogonal, so
`|R_u(x) - R_u(y)| = |x - y|`. Applying the same rotation to heel and toe while
the ankle lies on the axis proves all three target edge lengths are unchanged;
the target is pure orientation, not a hidden foot deformation. The ankle is
fixed because `R_u(0) = 0`.

The unit tests verify signed quarter-turns and wraparound, all three rigid-foot
distance identities, fixed-axis behavior, knee-angle invariance under a known
rigid world transform, exact recovery of a known 23-degree pure roll, and safe
`None` results for degenerate axes or planes.

### Baseline diagnostic result

The knee probe now quantifies the missing bend-plane resistance. Around the
well-conditioned center of the sweep, lateral and medial inputs turn the knee
plane freely in opposite directions:

| Left-foot target | Knee-plane delta | Knee flexion | Knee motion | Final max residual |
|---|---:|---:|---:|---:|
| 6 cm lateral | -31.3613 deg | 13.7085 deg | 1.5372 cm | 0.1000 cm |
| stationary | +1.1581 deg | 15.1793 deg | 0.4546 cm | 0.3793 cm |
| 6 cm medial | +27.1525 deg | 16.4997 deg | 1.1553 cm | 0.1200 cm |

Larger inputs are deliberately retained in the CSV. They expose non-monotone
branches and near-straight conditioning: for example, 8 cm lateral produces a
-174.3196-degree delta at 9.0694 degrees of flexion. No row accumulated a
watchdog rejection. This is diagnostic evidence for the next soft knee-plane
constraint, not an acceptance curve for the current solver.

The foot-roll response is smooth but largely follows the commanded orientation
without transferring meaningful motion proximally:

| Roll command | Achieved roll | Foot-to-shin delta | Heel orbit | Knee motion |
|---|---:|---:|---:|---:|
| -30 deg | -25.8075 deg | -26.3127 deg | 4.2495 cm | 0.4463 cm |
| 0 deg | +0.0392 deg | -0.0081 deg | 0.3793 cm | 0.4546 cm |
| +30 deg | +25.7890 deg | +25.4220 deg | 4.2345 cm | 0.4288 cm |

Hip/core motion is exactly zero because of the pins, and no roll row accumulated
a watchdog rejection. This establishes a numerical baseline for the later
foot-to-shin orientation constraint; it does not claim that the existing free
roll is anatomically acceptable.

### How to verify the diagnostic task yourself

From `topology-grapplers/solver`, prove the measurement identities:

```sh
cargo test --release -p gm-tests lower_limb_metrics
```

Then reproduce both CSV sweeps:

```sh
cargo run --release -p gm-tests --example probe_lower_limb_knee_plane
cargo run --release -p gm-tests --example probe_lower_limb_foot_roll
```

The commands should reproduce the representative rows above with finite values
and `rejected_frames=0`. Small floating-point variation in the last digits is
acceptable. Finally, prove that the added diagnostics did not regress solver,
database, fuzz, determinism, or WASM behavior:

```sh
cargo test --release --workspace
```

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
2. **Knee valgus/varus (initial probe implemented):** Move the ankle medially and
   laterally with hip position held approximately constant. The deterministic
   sweep now records knee-plane angle relative to the pelvis and flexion so
   near-straight conditioning is visible.
3. **Foot roll (initial probe implemented):** Rotate a foot tracker about its
   forward axis at fixed ankle position. The deterministic sweep now records
   heel orbit, foot-to-shin relative rotation, knee motion, and hip motion.
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

1. **Completed:** expand the translation diagnostics with knee-plane and
   foot-roll probes.
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

## Historical handoff: prevent full planted-foot rotation

**Status: completed.** The handoff below records the requirements that guided
the implementation; the completed design, proof results, and reproduction
commands follow it.

### Completed work available to the next implementer

- Binary lower-limb relaxation was replaced by the proved
  `0/0.10/0.40/0.70` foot-to-hip tone gradient.
- `stationary_foot_tracker_does_not_collapse_the_driven_leg` proves that
  engaging a stationary foot tracker no longer turns the leg into a passive
  chain.
- Deterministic translation, knee-plane, and foot-roll probes now provide
  reproducible CSV baselines.
- Shared signed-angle and rigid-axis-rotation geometry has proof-oriented unit
  coverage, including rigid-motion invariance, wraparound, distance
  preservation, known-angle recovery, and degenerate inputs.
- The current WASM package was rebuilt and manually tested. That test confirmed
  the remaining failure below; none of the completed work claims to constrain
  ankle orientation yet.

### Newly confirmed required behavior

With the left ankle/foot planted on the floor, the current demo permits the toe
and rigid foot triangle to rotate a complete 360 degrees around the ankle. This
is not an acceptable range of motion. The next implementation must make such a
rotation impossible rather than merely damp it or make it slower.

The safety rule must be anatomical and local to the leg. It must not clamp the
foot to a fixed world direction: a grappler may legitimately turn, invert, or
occupy an unusual authored pose. Ordinary motion inside the allowed range
should remain compliant, while motion at the final envelope must saturate at a
documented hard limit. The exact limits must be calibrated against the complete
pose database in shin/foot-local coordinates rather than guessed from the
standing pose alone.

### Why the current constraints admit 360 degrees

Let the fixed ankle be `A`, heel be `H`, and toe be `T`. The current foot rules
preserve only the three triangle lengths

`|H - A|`, `|T - A|`, and `|T - H|`.

For any rotation matrix `R` in `SO(3)`, define

`H' = A + R(H - A)`

and

`T' = A + R(T - A)`.

Because `R` is orthogonal,

`|R x|^2 = x^T R^T R x = x^T x = |x|^2`.

It follows that all three foot lengths are identical before and after an
arbitrary rotation about `A`. Consequently, bone constraints cannot distinguish
an anatomical ankle pose from a 360-degree orbit. The floor is only a one-sided
positional constraint; it prevents points from passing below the floor but does
not define foot-to-shin swing or twist. Damping also cannot supply a static
limit.

### Required implementation sequence

1. Define a stable foot frame from ankle, heel, and toe. Define a corresponding
   shin frame using knee/ankle direction plus the hip-knee-ankle bend plane. If
   near-straight degeneracy makes that frame undefined, retain explicit local
   orientation state or a documented fallback; never silently emit a zero
   angle.
2. Express current foot orientation relative to the shin frame and decompose it
   into observable swing/twist coordinates. Keep these coordinates local so a
   rigid rotation of the entire grappler does not change the measurement.
3. Extend the foot-roll probe with a continuous commanded rotation from 0
   through 360 degrees. A single static 360-degree target is insufficient
   because it is identical to a zero-degree orientation; the test must unwrap
   the commanded and measured angle over successive frames.
4. Add a soft rest-relative foot-to-shin constraint with a neutral dead zone and
   progressive resistance through the ordinary range.
5. Add a separate hard ankle envelope. Once the relative orientation reaches
   the configured limit, project the rigid foot frame onto the admissible
   boundary even if the orientation effector continues to demand rotation.
6. Separate foot position and orientation compliance so a position target can
   remain accurate without granting unlimited orientation authority.
7. Calibrate the local-coordinate limits against every database frame, repeat
   the probes for both legs and both players, rebuild WASM, and manually repeat
   the planted-foot test.

This requirement takes priority over cosmetic tuning. It may be implemented
before or together with the previously listed soft knee-plane constraint, but
the local frames should be designed so both constraints can share them.

### Mathematical proof obligations for the fix

The implementation is not complete until tests establish all of the following:

1. **Rigid-motion invariance:** applying the same translation and proper
   rotation to the entire grappler leaves every measured foot-to-shin angle
   unchanged.
2. **Interior identity:** a pose strictly inside the hard envelope is unchanged
   by the hard projection.
3. **Boundary postcondition:** after projection, every constrained relative
   angle is at or inside its configured limit within a stated floating-point
   tolerance.
4. **Idempotence:** projecting an already projected pose a second time produces
   no additional correction within tolerance.
5. **Foot rigidity:** the correction preserves `|H-A|`, `|T-A|`, and `|T-H|`.
   Rotating heel and toe together about the ankle supplies this property because
   orthogonal transformations preserve distance.
6. **No full orbit:** under a continuous 0-to-360-degree commanded rotation with
   a fixed shin reference, the unwrapped foot-to-shin angle never crosses the
   configured hard bound. Effector residual must increase after saturation
   rather than the foot passing through the boundary.
7. **Degenerate safety:** straight or nearly straight leg configurations remain
   finite and deterministic and do not disable the safety envelope.
8. **Symmetry:** mirrored legs and the two players obey equivalent limits and
   sign conventions.

### Acceptance and verification checklist

- Add a failing-then-passing automated regression for the reported planted-foot
  360-degree interaction. Isolate the anatomical rule by fixing the shin
  reference, then add an end-to-end version with pelvis and planted ankle fixed
  while the knee remains solver-controlled.
- Assert the configured hard angle is far below a full half-turn and document
  how the database calibration selected it. The automated postcondition should
  compare measured angle to that configured value, not duplicate a magic
  number in the test.
- Assert bone errors remain within normal solver tolerance, all poses remain
  finite, pinned joints do not move, and no topology-watchdog frame is rejected.
- Prove ordinary in-range commands remain reachable and continuous on both
  sides of neutral; the hard fix must not turn the ankle into a permanently
  locked joint.
- Run the focused geometry and new behavioral regressions, both diagnostic
  probes, `cargo test --release --workspace`, the complete database compatibility
  pass, and the rebuilt browser demo.
- Manually plant the foot, continuously rotate the controller/gizmo past the
  limit in both directions, and verify that the foot stops at the envelope while
  residual/load builds. Confirm that releasing the command does not snap,
  explode, or leave a non-finite velocity.

## Completed implementation: local soft ankle tone and hard swing/twist envelope

**Status: completed and release-tested.** The solver now prevents the reported
full planted-foot orbit with a rest-relative constraint in shin-local
coordinates. This deliberately addresses ankle orientation before the separate
knee bend-plane task.

### Frames and degenerate fallback

For each leg, let `A`, `K`, and `H_p` be ankle, knee, and hip, and let `H_f`
and `T` be heel and toe. The foot frame uses

`f = normalize(T - H_f)`

as its longitudinal direction and

`n = normalize(f cross (A - H_f))`

as its triangle-plane normal. The shin frame uses

`y = normalize(K - A)`

and the normalized hip-knee-ankle plane normal as `x`, with
`z = x cross y`. The authored foot `f` and `n` axes are stored in this local
shin frame when the solver loads a pose.

Below three degrees of knee bend, the cross product defining the bend plane is
ill-conditioned. The solver does not substitute a zero angle or turn the rule
off. It stores the authored bend-plane normal in pelvis coordinates and
transports that direction through the current pelvis frame, projects it normal
to `y`, and re-orthonormalizes the shin frame. Unit tests exercise that path for
both legs and both players and prove finite, symmetric enforcement.

### Swing/twist decomposition and projection

Current and authored foot axes are first expressed in the current shin frame.
Swing is the geodesic direction angle

`sigma = acos(clamp(f_0 dot f, -1, 1))`.

The authored normal `n_0` is parallel-transported by the minimum rotation from
`f_0` to `f`. Twist is then the signed angle from that transported normal to
the current foot normal about `f`, computed with `atan2`. At an exactly
antipodal swing, where the minimum path is non-unique, the authored foot normal
selects a deterministic rotation axis.

The soft rule has an eight-degree neutral zone and applies one correction per
substep. For either angular coordinate `q`, dead-zone `d`, and default
stiffness `k_s = 0.03`, its requested magnitude outside the neutral zone is

`|q'| = |q| - k_s (|q| - d)`.

Thus it is exactly the identity inside the dead zone and its correction grows
linearly with displacement. The test case `q = 20 deg`, `d = 8 deg`, and
`k_s = 0.08` proves the expected `q' = 19.04 deg` result exactly within floating
point tolerance. Each coordinate correction is additionally capped at 0.02
radians per substep, and the controlled release correction is critically
damped instead of being re-integrated as momentum on the next substep. Together
these bound release speed after a saturated command.

The separate hard rule clamps both swing and twist to the configured
100-degree limits. It reconstructs the admissible local foot axes, transforms
them to world coordinates, and applies the unique corresponding proper
rotation to heel and toe about the unchanged ankle. An ankle position target
therefore remains independent of orientation compliance; once orientation
saturates, heel/toe effector residual grows instead of moving the ankle target
or passing through the boundary.

### Proof obligations discharged

The implementation and regressions establish:

- **Rigid-motion invariance:** local swing and twist are unchanged by a shared
  proper world rotation and translation.
- **Interior identity:** a foot strictly inside the envelope is bit-identical
  after hard projection.
- **Boundary postcondition:** projected swing and absolute twist are at their
  configured limits within `1e-9` radians in the isolated geometry test and
  within `2e-3` radians in the iterated end-to-end solver test.
- **Idempotence:** projecting the corrected foot a second time changes no point
  by more than `1e-9` meters.
- **Foot rigidity and position separation:** all three ankle/heel/toe edge
  lengths are preserved within `1e-9` meters and the ankle is bit-identical.
  This follows generally because the correction map has orthonormal source and
  destination bases, so its matrix `R` satisfies `R^T R = I`.
- **No full orbit:** two successive-target 0-through-360-degree regressions,
  one with the complete shin fixed and one with a solver-controlled knee, keep
  unwrapped local swing at or below the configured boundary. They also assert
  finite poses, exact pins, relative bone error below 1%, zero rejected frames,
  and a target residual at least 5 cm larger after saturation than in the
  interior.
- **Safe release:** releasing the saturated command for 120 frames keeps every
  velocity finite and at or below the configured speed cap, keeps every
  per-frame joint displacement below 5 cm, retains the angular postconditions,
  and causes no watchdog rejection.
- **Degenerate safety and symmetry:** the explicit straight-leg fallback stays
  finite and enforces the same bounds for left/right legs and both players.
- **Soft response:** the neutral zone is an exact identity and correction
  magnitude is proportional to excess displacement.

The `--orbit` probe makes the saturation directly visible. In the current
release build, commanded local rotation tracks from 0 to 99.9648 degrees with
effectively zero residual. A 105-degree command remains at 99.9725 degrees and
creates 1.7037 cm residual. At a 160-degree command the measured swing is
100.0000 degrees and residual has risen to 21.7578 cm. No probe frame is
rejected.

### Database calibration

`calibrate_ankle_envelope.rs` measured every adjacent frame pair using the same
rest-relative local coordinates used by the solver:

| Sample | Swing | Absolute twist |
|---|---:|---:|
| p50 | 14.7368 deg | 7.1773 deg |
| p90 | 42.9338 deg | 30.5104 deg |
| p95 | 53.1021 deg | 41.9360 deg |
| p99 | 75.1976 deg | 79.1079 deg |
| maximum transition jump | 146.6481 deg | 178.4933 deg |

The configured 100-degree limits therefore preserve more than 99% of 24,948
observed adjacent ankle changes with roughly 20 degrees of margin above the
p99 values, while remaining 80 degrees below a half-turn. Sparse larger
transition jumps are not silently declared anatomical ordinary motion. Every
one of 33,292 authored ankle frames is instead proved finite and accepted
bit-identically as its own rest-relative neutral, so unusual grappling poses
remain loadable without using a fixed world-facing foot direction. Five
database measurements exercised the near-straight fallback.

### Constraint-conflict safety proof

A hard foot rotation can be individually valid yet incompatible with the mat,
arena, or another body capsule. The coupled solver therefore evaluates every
hard-ankle candidate against scalar admissibility margins. For the floor the
margin is

`m_floor = y - radius + 0.004 m`,

for a collidable capsule pair it is

`m_contact = clearance - authored_clearance_floor + 0.012 m`,

and the arena margins are the remaining distances to each boundary. A hard
ankle candidate is committed only when every margin satisfies

`m_after >= min(m_before, 0)`.

This proves two cases directly: a valid constraint cannot become invalid, and
an inherited violation cannot deepen. If the candidate fails, the exact
pre-projection pose is restored. The 4 mm and 12 mm widths are the explicit mat
and flesh compression allowances used by validation, not hidden angular
tuning.

After the iterative solve, an accepted-step watchdog checks the same complete
contract as the validator: finite coordinates, floor and arena bounds, capsule
clearance, bone error, hinge and swing-cone limits, ankle swing/twist limits,
20 m/s transition speed, segment crossings, and the writhe-jump threshold. A
failed candidate is retried with doubled substeps and, if it remains
infeasible, the prior state is returned bit-identically apart from zeroed
velocity. Consequently, assuming the loaded state is admissible, admissibility
is inductive over every accepted or rejected step.

The final floor repair for an unpinned pose is also exact: all joints of both
players receive one common upward translation equal to the worst mat-bound
deficit. For any two points `p_i` and `p_j` and translation `t`,

`|(p_i + t) - (p_j + t)| = |p_i - p_j|`.

It therefore restores the floor bound while preserving every bone length,
angle, capsule clearance, and topology observable. If an application pin
exists, this translation yields to the pin and the watchdog handles any
remaining infeasible combination without moving it.

In addition to the saved property-test regressions, five independent runs of
512 random 40-step adversarial streams passed in release mode: 2,560 streams
and 102,400 checked transitions. These include far-outside-arena, below-floor,
multi-joint, and mutually opposing targets.

### Reproduction

From `topology-grapplers/solver`:

```sh
cargo test --release -p gm-solver ankle_constraints
cargo test --release -p gm-solver continuous_foot_orbit
cargo test --release -p gm-tests every_database_ankle_is_finite_and_unchanged_at_load
cargo test --release -p gm-tests default_ankle_envelope_covers_database_motion_without_allowing_a_half_turn
cargo run --release -p gm-tests --example calibrate_ankle_envelope
cargo run --release -p gm-tests --example probe_lower_limb_foot_roll -- --orbit
cargo test --release --workspace
```

All commands pass in the current release build, including the full database
settle test, deterministic input streams, fuzz invariants, topology tests,
validation, and WASM tests. The next independent anatomy task remains the soft
knee bend-plane constraint; the hard ankle rule prevents a foot-to-shin orbit
but is not intended to substitute for knee varus/valgus resistance.

### Final verification — 2026-07-21

- `cargo test --release --workspace` passes across every crate, database
  regression, deterministic stream, fuzz property, golden scenario, topology
  check, validator, and WASM test.
- Five additional 512-case fuzz runs pass, covering 102,400 adversarial solver
  transitions in addition to the saved regression corpus.
- The calibration and continuous-orbit probes reproduce the database counts
  and 100-degree saturation values documented above with zero rejected orbit
  frames.
- The web-target WASM was rebuilt, the solver demo loaded successfully in a
  browser, its two-player smoke test reported `valid` with a clean watchdog,
  and the browser console contained no warnings or errors.
