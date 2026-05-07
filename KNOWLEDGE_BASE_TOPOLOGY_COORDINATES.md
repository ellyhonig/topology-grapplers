# Knowledge Base Node: Topology Coordinates

Parent: `KNOWLEDGE_BASE_ANIMATION_DRIVERS.md`

Source: `C:\Users\ellyh\Downloads\Ho_Komura_EG2009 (2).pdf`

Paper: Edmond S. L. Ho and Taku Komura, "Character Motion Synthesis by Topology Coordinates", Eurographics 2009.

## Why This Paper Matters For GrappleMap

GrappleMap already represents grappling as two stick-figure skeletons moving through keyed poses. Many grappling exchanges depend on topological relationships: limbs crossing, arms threading under/over limbs, hooks, wraps, head-and-arm controls, back takes, body locks, leg entanglements, and escapes from tangled structures.

This paper gives a mathematical vocabulary for those relationships. Instead of treating motion only as joint coordinates, it describes motion in a topology space where the important state is how body segments wind around, pass through, or avoid one another.

For agent work, this is useful because GrappleMap agents could reason about entanglement quality and transition plausibility without needing full physics or motion capture.

## Core Idea

The paper proposes topology coordinates for synthesizing character motions with close contact. The method models selected body parts as chains of line segments, then tracks how those chains twist around each other.

The main coordinate is writhe, derived from the Gauss Linking Integral. Writhe measures how much two curves or segment chains wind around one another when averaged over viewing directions.

The paper adds two more attributes:

- `center`: where the twisting interaction is concentrated along the two chains.
- `density`: which chain contributes most to the twist, based on the principal axis of the writhe matrix.

Together, these form a compact control space for tangled motions. Interpolating in this topology space can produce threading or wrapping motions that ordinary linear interpolation in joint coordinates would turn into body-part penetration.

## Algorithm Shape

The paper's control loop is:

1. Choose two chains of segments whose topological relationship matters.
2. Compute their current topology coordinates.
3. Choose target topology coordinates.
4. Convert the target topology coordinates into a desired writhe matrix.
5. Solve for small generalized-coordinate updates that make the current writhe matrix approach the target.
6. Enforce constraints that prevent segment penetration.
7. Repeat until the topology coordinates reach the target.

The optimization is framed as a quadratic program. It minimizes motion magnitude while satisfying soft topology constraints, segment-pair writhe bounds, and any added kinematic constraints such as end-effector positions.

## Important Terms

- `Topology space`: state space where chain relationships are represented by writhe, center, and density.
- `Writhe`: scalar value describing how much two chains twist around each other.
- `Writhe matrix`: matrix whose elements store the writhe contribution of each segment pair between two chains.
- `Center`: approximate center of mass of high-contribution cells in the writhe matrix.
- `Density`: orientation of the dominant axis in the writhe matrix after normalization.
- `Loop passing`: special case where a chain is guided through a loop by increasing writhe toward about one.
- `Bundle tangling`: special case where one chain tangles with a group of chains while avoiding another group.

## Relevance To Grappling

The paper explicitly includes wrestling as a test domain. Its examples include characters changing holds, piggyback carrying, a full nelson-like interaction, hugging objects, and threading limbs through loops.

GrappleMap has many equivalent problems:

- Arm drags and underhooks require one limb path to pass around another limb or torso path.
- Back takes and seatbelt controls encode torso/arm/head wrapping.
- Darce, guillotine, triangle, kimura, and rear-naked-choke positions are topological limb/head/torso constraints.
- Leg entanglements can be described by chains passing around thighs, calves, hips, and torso.
- Escapes often require reducing writhe or moving the center of a twist along a chain.

This suggests an agent can use topology descriptors as additional semantic features over the existing keyframe graph.

## Possible GrappleMap Agent Uses

### 1. Entanglement Feature Extraction

Build a tool that reads each GrappleMap `Position` and computes topological descriptors for selected chain pairs:

- attacker arm chain vs defender neck-arm chain
- leg chain vs opponent hip-leg chain
- torso-spine path vs arm path
- left/right limb pairs for self-entanglement

Output could be stored as derived analysis, not committed into `GrappleMap.txt` at first.

### 2. Transition Validation

For every transition, compare start/end topology descriptors. Flag transitions where:

- topology changes dramatically with too few keyframes
- interpolated frames appear to pass through a chain
- claimed tags imply a wrap/control but writhe descriptors do not support it
- a transition loses an expected entanglement before the technique completes

### 3. Search And Tag Enrichment

Use topology measurements to suggest tags:

- `overhook`, `underhook`, `arm_drag`, `seatbelt`
- `triangle`, `kimura`, `darce`, `guillotine`
- `inside_position`, `outside_position`
- `leg_entanglement`, `body_lock`, `head_control`

This should be advisory. Grappling tags carry tactical meaning beyond raw geometry.

### 4. Agent-Assisted Animation

When generating or repairing a transition, an agent could ask for target topology changes rather than raw joint coordinates:

- "pass the right arm under the opponent's left arm"
- "increase wrap around neck-arm chain"
- "move the twist center from wrist area toward elbow/shoulder"
- "untangle the trapped leg without crossing through the opponent's thigh"

The existing GrappleMap editor could remain the pose authoring surface while an offline analyzer recommends keyframes or detects likely penetrations.

### 5. Drill Reasoning

A drill generator could search for paths that preserve or transform topology:

- maintain back control while changing seatbelt side
- increase leg entanglement before entering a submission
- reduce head-arm writhe as an escape goal
- alternate topological states for retention and recovery drills

## Integration Sketch

A conservative implementation path:

1. Add a standalone analysis utility, outside the core parser, that loads `GrappleMap.txt`.
2. Define named chains over the existing 23-joint player skeleton.
3. Compute approximate segment-pair writhe for selected chain pairs.
4. Export JSON with per-position and per-transition topology summaries.
5. Build a report that correlates topology summaries with tags/properties.
6. Only after validation, expose this data in browser search/explorer tools.

Do not change the database format in the first pass. Keep topology analysis as a derived artifact until the descriptors prove stable.

## Candidate Chain Definitions

Initial chains to test:

- `left_arm`: left shoulder, elbow, wrist, hand, fingers
- `right_arm`: right shoulder, elbow, wrist, hand, fingers
- `left_leg`: left hip, knee, ankle, heel/toe
- `right_leg`: right hip, knee, ankle, heel/toe
- `spine_head`: core, neck, head
- `shoulder_line`: left shoulder, neck, right shoulder
- `hip_line`: left hip, core, right hip
- `torso_loop_approx`: shoulder line plus hip line approximated as multiple segment chains

Because GrappleMap bodies are stick figures rather than closed meshes, most early work should use open-chain writhe and heuristic loop approximations.

## Limitations And Warnings

The paper's method is not a complete grappling simulator.

- It assumes models can be represented by line segments.
- It needs selected chains; the user or tool must decide which body paths matter.
- It uses local optimization, so conflicting constraints can get stuck.
- It does not solve all rigid-body collision issues.
- It does not decide the tactical order of tangles automatically.
- Writhe can detect topological winding, but not whether a grappling technique is mechanically valid.

For GrappleMap, topology coordinates should be treated as a validation and assistance layer, not as ground truth.

## Engineering Notes

Relevant existing files:

- `src/players.hpp`: player/joint definitions.
- `src/positions.hpp`: `Position`, joint coordinate helpers, reorientation operations.
- `src/positions.cpp`: body limits and related position utilities.
- `src/graph.hpp` / `src/graph.cpp`: graph nodes and transition edges.
- `src/persistence.cpp`: load/save `GrappleMap.txt`.
- `src/js_conversions.cpp`: route for exposing derived data to web UIs if needed later.

Likely new code locations:

- `src/topology.hpp`
- `src/topology.cpp`
- `src/topology_analyzer.cpp`

Potential standalone target:

- `grapplemap-topology-report`

Suggested output:

```json
{
  "nodes": [
    {
      "id": 34,
      "chains": {
        "p0_right_arm_vs_p1_spine_head": {
          "writhe": 0.73,
          "center": [0.4, 0.2],
          "density": -0.18
        }
      }
    }
  ],
  "transitions": [
    {
      "id": 217,
      "delta": {
        "p0_right_arm_vs_p1_spine_head": 0.51
      },
      "flags": []
    }
  ]
}
```

Numbers above are illustrative only.

## Agent Prompt Ideas

Useful future prompts:

- "Find transitions tagged `darce` and summarize their head-arm topology changes."
- "Flag transitions where limb chains likely pass through each other between keyframes."
- "Suggest missing entanglement tags from topology summaries."
- "Generate a drill path that preserves back-control topology while alternating attacks."
- "Compare two GrappleMap revisions by topology changes, not just pose-coordinate changes."

## Bottom Line

Ho and Komura's topology coordinates are a strong conceptual fit for GrappleMap because grappling is dominated by constrained threading, wrapping, and untangling. The safest first use is an offline topology analyzer that augments search, validation, and agent reasoning without rewriting the database or editor.
