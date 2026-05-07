# Knowledge Base Node: FreeMoCap

Parent: `KNOWLEDGE_BASE_INPUT_DRIVERS.md`

Source: https://github.com/freemocap/freemocap

Docs:

- https://freemocap.github.io/documentation/
- https://freemocap.org/

## Why This Repo Matters For GrappleMap

FreeMoCap is a free, open-source, markerless motion-capture system. It turns synchronized camera recordings into human movement data and downstream artifacts such as CSV files, NumPy arrays, Blender scenes, and animation-oriented exports.

For GrappleMap, the important fit is not replacing the current hand-authored pose graph. The useful path is using FreeMoCap-style capture and reconstruction as a source of real motion examples, pose priors, timing data, validation signals, and possible keyframe suggestions for grappling transitions.

GrappleMap already stores two-player stick-figure positions and animated transitions. FreeMoCap is a candidate upstream data source for observed movement that could help agents reason about whether generated transitions are anatomically plausible, temporally smooth, or grounded in recorded human motion.

## Project Shape

The main `freemocap/freemocap` repo is a Python desktop application distributed as the `freemocap` package.

Important source facts from the repo:

- Package name: `freemocap`.
- Runtime Python range: `>=3.10,<3.13`.
- Recommended quickstart path: install with `pip install freemocap`, then launch the GUI with `freemocap`.
- Source-development path: create a Python environment, install the repo editable with `pip install -e .`, then launch with `python -m freemocap`.
- Current project metadata in `pyproject.toml` identifies version line `v1.8.2`.
- License: AGPL-3.0-or-later.
- Main repo language mix is primarily Python, with Jupyter notebooks and some web UI code.

The package depends on a set of related FreeMoCap/Skelly components:

- `skellycam`: synchronized multi-camera capture backend.
- `skellytracker`: pose-estimation and object-tracking backend.
- `skellyforge`: postprocessing/reconstruction tooling.
- `skelly_viewer`: viewing/visualization tooling.
- `ajc27_freemocap_blender_addon`: Blender integration.
- `opencv-contrib-python`, `aniposelib`, `PySide6`, `pydantic`, `plotly`, and related desktop/scientific Python packages.

FreeMoCap should therefore be treated as a pipeline of cooperating tools, not just a single pose-estimation script.

## Motion-Capture Workflow

The user-facing workflow is:

1. Install and launch the FreeMoCap GUI.
2. Connect one or more cameras.
3. For multi-camera 3D capture, calibrate the capture volume with a ChArUco board.
4. Record motion.
5. Process recorded videos into tracked 2D/3D movement data.
6. Inspect or export outputs for analysis, animation, or downstream tooling.

The documentation recommends starting with a single-camera recording to verify the end-to-end pipeline before moving to multi-camera capture. Single-camera recordings are useful for 2D tracking, but reliable 3D reconstruction requires multiple calibrated camera views. The docs describe at least two cameras as viable and recommend three cameras for better 3D results.

## Data And Output Relevance

FreeMoCap outputs are useful to GrappleMap agents because they are closer to real human movement than synthetic interpolation.

Potentially relevant outputs:

- Time-indexed joint trajectories.
- 2D tracked skeleton data from each camera.
- 3D reconstructed skeleton points.
- CSV/NumPy data for analysis.
- Blender scenes or animation exports for visual review.
- Calibration data and reprojection-error diagnostics.

GrappleMap's current `Position` model has a fixed two-player joint skeleton. FreeMoCap data would need a retargeting layer before it can become GrappleMap frames. That layer should map FreeMoCap/Skelly joints into GrappleMap's 23-joint player representation, normalize scale/orientation, and preserve timing where possible.

## Fit With Existing GrappleMap Concepts

### GrappleMap Nodes

FreeMoCap recordings can help derive candidate named positions from real motion:

- detect stable holds or repeated posture clusters
- compare captured poses against existing GrappleMap nodes
- suggest missing intermediate positions in transitions
- identify body configurations that are hard to author by hand

This should be advisory at first. A captured frame is not automatically a grappling position; grappling semantics still need human review.

### GrappleMap Transitions

Captured movement can inform transitions by providing:

- realistic keyframe timing
- body-limit priors
- plausible limb arcs
- examples of balance shifts and torso rotation
- failure cases where simple interpolation crosses through bodies

The safest early integration is a sidecar report that compares captured motion to GrappleMap transitions instead of writing directly into `GrappleMap.txt`.

### Topology Coordinates

FreeMoCap pairs naturally with `KNOWLEDGE_BASE_TOPOLOGY_COORDINATES.md`.

FreeMoCap can provide observed joint trajectories. Topology-coordinate analysis can then measure how limb chains wrap, thread, or untangle during those trajectories. Together, they can support an agent workflow like:

1. Capture or import a real grappling movement.
2. Retarget skeleton data into GrappleMap-like chains.
3. Compute topology descriptors over time.
4. Compare descriptors against a GrappleMap transition.
5. Suggest missing keyframes or flag impossible topology changes.

## Candidate GrappleMap Agent Uses

### 1. Mocap Import Sidecar

Build a standalone importer that reads FreeMoCap output folders and emits a neutral JSON summary:

```json
{
  "recording": "example_session",
  "fps": 30,
  "people": [
    {
      "id": 0,
      "frames": [
        {
          "frame": 0,
          "joints": {
            "left_wrist": [0.12, 1.04, -0.22]
          }
        }
      ]
    }
  ]
}
```

Do not write GrappleMap frames in the first pass.

### 2. Retargeting Prototype

Create a small mapping layer from FreeMoCap skeleton points to GrappleMap's player joints.

Initial output:

- one normalized player pose per frame
- confidence/missing-data flags per joint
- scale and floor-contact estimates
- root orientation estimates

For two-person grappling, this will need identity tracking and contact-aware cleanup. Keep that out of the first prototype unless the recording data already contains reliable person IDs.

### 3. Transition Validation

Compare captured movement against GrappleMap transitions:

- frame count and timing differences
- joint velocity spikes
- limb-length drift after retargeting
- topology-coordinate changes across arms, legs, torso, and head
- likely penetration or impossible threading between keyframes

The first report can be descriptive: "this captured movement reaches a similar end pose but takes more frames and preserves a different arm-threading topology."

### 4. Pose Search And Clustering

Cluster captured frames and search for nearby GrappleMap nodes. Useful queries:

- "Which existing GrappleMap node is closest to this captured frame?"
- "Which captured frames look like guard, side control, turtle, or back control?"
- "Which transition in GrappleMap has the closest start/end shape to this capture segment?"

This requires a pose-distance metric that is invariant to translation, yaw, mirroring, and player swap, matching GrappleMap's existing reorientation concepts.

### 5. Authoring Assistance

Use captured trajectories to suggest intermediate keyframes in the editor:

- preserve real timing while simplifying to a small number of keyframes
- keep only frames where topology, contact, or direction changes
- present suggested frames for human approval

The output should be a reviewable patch or editor action list, not an automatic canonical data edit.

## Integration Sketch

A conservative implementation path:

1. Inspect one FreeMoCap output folder format and document the exact files needed.
2. Add a standalone script or utility outside the core GrappleMap parser.
3. Load FreeMoCap 3D skeleton data into a neutral intermediate structure.
4. Define a first-pass joint map into GrappleMap's `Position` representation.
5. Normalize scale, floor plane, facing direction, and origin.
6. Export derived JSON and visual proof frames.
7. Add a report comparing imported poses against existing GrappleMap nodes/transitions.
8. Only after validation, consider editor integration for assisted keyframe insertion.

Suggested local code locations:

- `scripts/import-freemocap-recording.py`
- `scripts/freemocap-to-grapplemap-report.py`
- `doc/freemocap-import.md`

If this becomes C++-native later, use:

- `src/positions.hpp`
- `src/graph.hpp`
- `src/metadata.cpp`
- `src/persistence.cpp`

But the first pass is better as a script because FreeMoCap's data ecosystem is Python-first.

## Risks And Warnings

- FreeMoCap's AGPL license is compatible with open research workflows, but any redistribution or integration plan should be checked before bundling code into GrappleMap.
- Single-camera data should not be treated as reliable 3D grappling motion.
- Grappling has heavy occlusion, close contact, and multiple bodies; markerless tracking can lose limbs or swap identities.
- Imported motion is not automatically tactically valid grappling data.
- Skeleton schemas will not match exactly; retargeting errors can be worse than hand-authored keyframes if not reviewed.
- Camera calibration, lighting, frame synchronization, and clothing/background conditions strongly affect output quality.

## Engineering Notes

Relevant GrappleMap files:

- `src/players.hpp`: joint definitions and player structure.
- `src/positions.hpp` / `src/positions.cpp`: pose representation, body limits, and reorientation.
- `src/graph.hpp` / `src/graph.cpp`: nodes and transitions.
- `src/persistence.cpp`: database load/save format.
- `src/gm.js`: browser skeleton rendering and interpolation.
- `src/topology-demo.js`: existing browser-side topology-coordinate proof-of-concept.

Relevant FreeMoCap entry points and references:

- `README.md`: install and launch workflows.
- `pyproject.toml`: Python version range, dependencies, package metadata, and CLI entry point.
- FreeMoCap docs: installation, recording, calibration, and workflow guidance.
- FreeMoCap website: high-level product workflow and output claims.

## Agent Prompt Ideas

Useful future prompts:

- "Inspect a FreeMoCap recording folder and summarize the available skeleton data files."
- "Prototype a FreeMoCap-to-GrappleMap joint mapping and export one normalized pose."
- "Compare this captured frame to GrappleMap nodes and list the nearest positions."
- "Use FreeMoCap motion to suggest intermediate keyframes for this GrappleMap transition."
- "Compute topology-coordinate changes over a FreeMoCap capture and compare them to a GrappleMap technique."

## Bottom Line

FreeMoCap is a promising upstream capture and validation source for GrappleMap, especially when paired with topology-coordinate analysis. The safest first step is not direct database import. It is a derived sidecar pipeline that loads FreeMoCap outputs, retargets them into GrappleMap-like skeletons, and produces reviewable reports or editor suggestions.
