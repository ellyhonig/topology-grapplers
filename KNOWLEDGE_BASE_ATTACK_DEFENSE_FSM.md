# Knowledge Base Node: Attack-Defense FSM With Topology Coordinates

Parent: `KNOWLEDGE_BASE_ANIMATION_DRIVERS.md`

Source: `C:\Users\ellyh\Downloads\attackdefendpaper.pdf`

Paper: Edmond S. L. Ho and Taku Komura, "A finite state machine based on topology coordinates for wrestling games", Computer Animation and Virtual Worlds, 2011. DOI: `10.1002/cav.376`.

## Why This Paper Matters For GrappleMap

This paper is a direct sequel/application layer for Ho and Komura's topology-coordinate work. The earlier paper explains how to synthesize tangled motion by controlling writhe, center, and density. This paper explains how to put those coordinates into an interactive attack-and-defense system for two virtual wrestlers.

For GrappleMap, the important idea is not the game UI itself. The useful idea is the finite state machine (FSM) whose states are topological grappling configurations and whose transitions are attack/escape changes in those configurations.

GrappleMap already has a graph:

- nodes are grappling positions
- edges are transitions
- tags describe tactical concepts such as `kimura`, `back_control`, `triangle`, `body_lock`, and escapes

This paper suggests a way to add a second, derived graph layer where node identity and transition plausibility are based on topology descriptors rather than only names, tags, or raw joint coordinates.

## Core Idea

The system precomputes a finite state machine of wrestling attacks and defenses in topology-coordinate space.

Each state represents a way two characters are tangled. Each transition represents a change in the tangle, such as:

- creating a new wrap
- preserving an existing hold while moving another limb
- reducing writhe to escape
- switching to a different attack while reusing an existing entanglement

At runtime, the current topology coordinates of the characters are computed from their posture. The FSM node matching the current topological state is found, and the player is shown possible attacks or defensive transitions from that state. The characters can still be controlled kinematically with inverse kinematics, so the motions are not just canned clips.

## Topology Coordinates Review

The paper uses the same three topology-coordinate attributes as the earlier topology-coordinate paper:

- `writhe`: total amount of twisting between two curves or segment chains
- `center`: two scalar parameters locating where the twist is concentrated along the two chains
- `density`: scalar parameter describing how concentrated the twist is and which chain is playing the major wrapping role

The paper defines writhe using the Gauss Linking Integral (GLI). For two curves `gamma_1` and `gamma_2`:

```text
GLI(gamma_1, gamma_2)
  = (1 / 4*pi) * integral over gamma_1 integral over gamma_2
      ((d gamma_1 x d gamma_2) . (gamma_1 - gamma_2))
      / ||gamma_1 - gamma_2||^3
```

Here `x` is the cross product and `.` is the dot product. The GLI computes the average crossing count of the two curves over all viewing directions.

For GrappleMap, the curves are not mesh surfaces. They are selected chains of stick-figure body segments, such as:

- attacker right arm vs defender neck/head chain
- attacker left arm vs defender trapped arm
- leg chain vs opponent hip/leg chain
- torso/shoulder route vs opponent arm route

## Segment-Chain Writhe

The paper represents the character skeleton as line-segment chains. Let:

- `S_1` be a chain with `n_1` line segments
- `S_2` be a chain with `n_2` line segments
- `T_i,j` be the writhe contribution between segment `i` of `S_1` and segment `j` of `S_2`

Total writhe is:

```text
w = GLI(S_1, S_2)
  = sum from i=1 to n_1 sum from j=1 to n_2 T_i,j
```

The matrix:

```text
T = [T_i,j], with shape n_1 x n_2
```

is the `writhe matrix`. Its cells show where the topological interaction is happening. Large absolute values in particular cells mean those segment pairs contribute strongly to the tangle.

In GrappleMap terms, the matrix can distinguish cases that have similar endpoint positions but different threading:

- wrist wrapped around neck near the hand end of a chain
- elbow/upper-arm wrapping near the shoulder end
- twist spread across a whole limb
- twist concentrated at one local crossing

## Desired Writhe Matrix

The paper does not directly edit every entry of `T`. Instead, it builds an ideal desired writhe matrix from topology coordinates.

Let:

- `d` be density
- `c` be center
- `w` be writhe
- `I` be a base `n_1 x n_2` matrix
- `R(M, d)` rotate the distribution of matrix entries to change density
- `Tr(M, c)` translate the distribution to change center
- `S(M, w)` scale the matrix to change writhe

The desired matrix is:

```text
T_d = S(Tr(R(I, d - pi/4), c), w)
```

The `pi/4` offset is used because of the density definition in the topology-coordinate formulation.

The base matrix `I` puts equal mass in the center column or columns:

```text
If n_2 is odd:
  I_i,j = 1 / n_1  when j = (n_2 + 1) / 2
  I_i,j = 0        otherwise

If n_2 is even:
  I_i,j = 1 / (2*n_1) when j = n_2 / 2 or j = n_2 / 2 + 1
  I_i,j = 0           otherwise
```

This is close to what `src/topology-demo.js` does in `desiredWritheMatrix()`: start with a vertical unit distribution, rotate it by density, translate it by center, and scale it by target writhe.

## Topology-Control Quadratic Program

Once `T_d` is available, the character is moved by updating generalized coordinates so the current writhe matrix `T` approaches `T_d`.

For two chains with generalized coordinates `q_1` and `q_2`, the paper gives the topology-control problem:

```text
min over Delta q_1, Delta q_2, delta:
  ||Delta q_1||^2 + ||Delta q_2||^2 + ||delta||^2

subject to:
  Delta T = (partial T / partial q_1) Delta q_1
          + (partial T / partial q_2) Delta q_2

  |T_i,j + Delta T_i,j| <= sigma
    for 1 <= i <= n_1 and 1 <= j <= n_2

  T + Delta T - T_d + delta = 0

  r = J_1 Delta q_1 + J_2 Delta q_2
```

Terms:

- `Delta q_1`, `Delta q_2`: small joint/generalized-coordinate updates
- `Delta T`: resulting update to the writhe matrix
- `sigma`: per-segment-pair writhe threshold; the paper reports `sigma = 0.2` in experiments
- `delta`: slack vector that allows the solver to minimize, rather than perfectly satisfy, the desired matrix
- `r`: other kinematic constraints such as end-effector movement or center-of-mass constraints
- `J_1`, `J_2`: Jacobians for those kinematic constraints

The important GrappleMap interpretation: the solver tries to satisfy the desired topology while taking small motion steps and keeping segment-pair writhe values under a limit that helps prevent segments from getting too close or penetrating.

## Attack And Defense Split

The paper separates attacker and defender updates into two independent quadratic programs. This is important because it creates an interactive fight rather than a single global optimizer that helps the attacker regardless of defender movement.

### Attacker Update

The attacker moves toward the target topology while treating the defender's current posture as fixed for that step:

```text
min over Delta q_1, delta:
  ||Delta q_1||^2 + ||delta||^2

subject to:
  Delta T = (partial T / partial q_1) Delta q_1

  |T_i,j + Delta T_i,j| <= sigma
    for 1 <= i <= n_1 and 1 <= j <= n_2

  T + Delta T - T_d + delta = 0

  r_1 = J_1 Delta q_1
```

This means the attacker can fail to reach the target topology if the defender moves well. The paper frames this as a kind of physiological delay and as a gameplay feature.

### Defender Update

The defender is controlled by IK from user input, while the topology constraints are still considered:

```text
min over Delta q_2:
  ||Delta q_2||^2

subject to:
  Delta T = (partial T / partial q_2) Delta q_2

  |T_i,j + Delta T_i,j| <= sigma
    for 1 <= i <= n_1 and 1 <= j <= n_2

  r_2 = J_2 Delta q_2
```

Here `r_2` represents defender kinematic controls, such as dragging a limb or moving the torso.

The defender's best escape movements are those that reduce writhe efficiently with small movement. This maps well to grappling: a small posture change can nullify a choke or arm wrap if it changes the relevant topological relationship at the right time.

## Runtime Update Order

The runtime control loop is:

1. Evaluate current topology coordinates from the two characters' postures.
2. Find the matching FSM state.
3. Show available attacks or defensive changes from that state.
4. If the attacker chooses a target, compute the attacker's update toward the next target topology.
5. Update the attacker's posture.
6. Compute the defender's IK update using player input and topology constraints.
7. Update the defender's posture.
8. Advance time and repeat.

Because the attacker and defender are solved separately, the attacker does not get perfect foreknowledge of the defender's next move.

## FSM Construction

The paper's FSM is built offline from example topological postures:

1. An animator creates representative wrestling postures using topology-coordinate controls.
2. These designed postures become FSM states.
3. The topological status of each posture is evaluated using rational-tangle-style route comparisons.
4. The system finds shortest paths from designed tangled states back to untangled standing states.
5. Intermediate untangling states are added to the FSM.
6. States with similar topological status are connected.

The paper connects states when the absolute writhe differences between every pair of routes are below a threshold:

```text
connect states A and B if
  |w_A(route_k) - w_B(route_k)| < 0.5
for every compared route k
```

This thresholding idea is especially relevant for GrappleMap. It suggests a way to infer when two named positions are topologically equivalent even if joint coordinates differ.

## Mapping To GrappleMap

GrappleMap's existing graph is hand-authored and semantically named. This paper suggests adding derived topology information around that graph.

Possible derived state identity:

```json
{
  "node_id": 34,
  "topology_state": {
    "p0_right_arm_vs_p1_neck": {
      "writhe": 0.82,
      "center": [0.3, -0.1],
      "density": 0.18
    },
    "p0_left_arm_vs_p1_left_arm": {
      "writhe": 0.44,
      "center": [-0.2, 0.6],
      "density": -0.12
    }
  }
}
```

For transitions, a topology summary could describe what changes:

```json
{
  "transition_id": 912,
  "delta_topology": {
    "attacker_arm_vs_defender_neck": {
      "writhe_delta": 0.51,
      "center_delta": [0.2, -0.4],
      "role": "attack_tightening"
    }
  }
}
```

This should stay as derived analysis at first, not as canonical database data.

## Relevance To Grappling Tags

The paper is useful for reasoning about tags that imply attack/defense relationships:

- `kimura`: attacker arm chain and defender arm/shoulder chain should preserve a trapping topology; escapes reduce or shift that topology.
- `rear_naked_choke` / `choke`: attacker arm chain winds around defender neck/head route.
- `full_nelson`: attacker arm routes wrap around defender arm/neck routes from behind.
- `body_lock`: arm chains wrap around torso/hip routes.
- `triangle`: leg chain and defender neck/arm route form a strong loop-like topology.
- `leg_entanglement`: leg chains maintain or switch winding around opponent leg/hip chains.

The paper's attack/defense split is also useful for classifying transitions:

- attacker-increasing-writhe transitions
- defender-reducing-writhe transitions
- attack switches that preserve one existing entanglement while creating another
- failed escapes where writhe remains high
- successful escapes where writhe drops or the center leaves the controlled body part

## Engineering Use Cases

### 1. Topology-Aware Transition Validation

For every GrappleMap edge:

1. Compute selected chain-pair topology coordinates at each frame.
2. Detect abrupt jumps in writhe, center, or density.
3. Flag transitions where topology changes too much between sparse frames.
4. Compare tags against expected topology patterns.

This can catch transitions that visually appear to teleport a limb through another limb.

### 2. Derived Attack/Defense FSM

Build a derived FSM over GrappleMap nodes:

- group nodes by topological similarity
- classify edges by topology deltas
- identify attack choices available from a state
- identify defensive reductions of a state
- find intermediate states between tangled and untangled positions

This would not replace the existing graph. It would provide a topology-level view over it.

### 3. Agent-Assisted Suggestions

An agent could answer questions like:

- "From this Kimura state, which transitions preserve the arm trap?"
- "Which escapes reduce attacker-arm-vs-defender-arm writhe fastest?"
- "Which transitions look like attack switches rather than direct finishes?"
- "Where are there missing intermediate untangling states?"
- "Does this transition maintain the existing body lock while switching to back control?"

### 4. Interactive Repair

If a transition has clipping or impossible threading, an assistant could:

1. identify the relevant chain pair
2. compute start and target topology coordinates
3. interpolate topology coordinates instead of raw joint positions
4. propose intermediate frames that preserve segment clearance
5. leave the final GrappleMap pose edits for review in the editor

## Relationship To Existing Topology Demo

The existing browser topology demo already implements a simplified version of the earlier topology-coordinate solver:

- line-segment chain selection
- segment-pair writhe matrix
- topology coordinates
- desired writhe matrix
- iterative solver
- contact projection

This paper adds the higher-level control architecture:

- attack/defense state machine
- topological state matching
- attacker target topology
- defender IK response
- attack switching while preserving already tangled parts

So the natural progression for GrappleMap is:

1. keep `src/topology-demo.js` as a proof of local topology synthesis
2. add an offline analyzer that computes topology descriptors for real GrappleMap positions
3. build a topology-similarity graph/FSM report
4. only later expose attack/defense choices or repair suggestions in UI

## Limitations And Warnings

This paper is game-animation research, not a complete grappling physics model.

- The skeleton is approximated as line-segment chains.
- FSM states depend on which chains/routes are selected.
- Writhe does not prove a technique is tactically correct.
- The QP solver needs Jacobians and constraints that GrappleMap does not currently store.
- The original implementation used a 42-DOF human model and CPLEX; GrappleMap's current stick-figure/editor model is simpler.
- Topological similarity thresholds such as `0.5` are paper-specific starting points, not validated GrappleMap constants.
- Segment-pair writhe bounds help with closeness/penetration, but they are not a full capsule collision system.

For GrappleMap, this should be used as an analysis and assistance layer first.

## Suggested First Implementation

Add a standalone topology-FSM report generator:

1. Load `GrappleMap.txt`.
2. Define named chain pairs for common grappling controls.
3. Compute writhe, center, and density for each selected pair at each node.
4. Compute deltas for each transition.
5. Group topologically similar nodes with thresholded descriptors.
6. Export JSON and a Markdown report.
7. Compare inferred attack/defense topology changes against tags.

Suggested output file names:

- `topology_fsm_report.json`
- `topology_fsm_report.md`

Do not write topology data back into `GrappleMap.txt` until the chain definitions and thresholds have been tested.

## Agent Prompt Ideas

- "Build a derived attack/defense FSM for Kimura-tagged GrappleMap nodes."
- "Find transitions where attacker writhe increases while defender writhe-reduction options exist."
- "Identify topologically similar back-control states even if their descriptions differ."
- "Flag transitions where a choke tag appears but attacker-arm-vs-neck writhe is low."
- "Suggest intermediate untangling states between a triangle and open guard."
- "Find attack switches that preserve one entanglement while creating another."

## Bottom Line

This paper turns topology coordinates into an interactive attack/defense graph. For GrappleMap, its strongest use is a derived topology-level FSM that can classify positions, validate transitions, suggest missing intermediate frames, and reason about attack/escape choices without rewriting the canonical pose database.
