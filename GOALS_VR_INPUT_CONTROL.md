# Goals for VR Input Control

## Objective

Let the user arrange the two-grappler scene relative to their real-world position before body tracking begins, then start tracking without causing a discontinuity in the controlled grappler's feet.

## 1. Orient the scene before tracking

- While VR is in setup mode and body trackers are not engaged, the user can rotate the complete grappler stage around themselves.
- Rotation applies to both grapplers and their shared scene as one unit; it does not alter their pose relative to one another.
- The user can see the result in the headset and continue adjusting it until they are standing in the desired position relative to the grapplers.
- Starting VR tracking locks the chosen scene transform in place. The menu or automatic setup recentering must not overwrite that orientation after tracking starts.
- Head and hand tracking snap to the user's current headset and controller poses only after the user explicitly starts tracking.

### Acceptance criteria

- Entering VR alone does not engage any body tracker.
- Before tracking, the user can change the scene's horizontal orientation without changing the selected grappling position.
- Both grapplers, the mat/grid, and all disengaged tracker targets rotate together.
- When tracking starts, the scene remains at the orientation selected by the user.
- The user can use the scene orientation to line their real body up with the controlled grappler before the one-time head-and-hand snap.

## 2. Preserve the foot when trigger control begins

- Pressing and holding a controller trigger changes that controller from hand control to foot control.
- On the trigger-down edge, create a temporary pivot/empty transform at the foot tracker's current world position and orientation.
- Preserve the foot tracker's current world transform while attaching it beneath that pivot. Capturing the foot must not move or rotate it, even for a single frame.
- After capture, drive the pivot from the controller's pose delta: the pivot follows the controller's translation and rotation exactly from their respective capture-time poses.
- Because motion is delta-based, the controller does not need to be physically located at or aligned with the foot when the clutch begins.
- Releasing the trigger ends the foot clutch, removes the temporary pivot, and returns that controller to hand control.

### Capture invariant

If `F0` is the foot transform and `C0` is the controller transform at trigger-down, capture must leave the rendered and solver-facing foot transform equal to `F0`:

```text
capture foot control: F(before) == F(after)
```

For later controller pose `Ct`, the temporary pivot follows the controller motion since capture:

```text
P(t) = Ct * inverse(C0) * P0
F(t) = P(t) * localFootAtCapture
```

The exact transform convention may differ in code, but the observable behavior must be the same.

### Acceptance criteria

- Trigger-down causes no positional snap of the foot.
- Trigger-down causes no orientation change of the foot.
- Holding the trigger and translating the controller moves the captured foot by the same translation delta.
- Holding the trigger and rotating the controller rotates the captured foot by the same rotation delta, starting from the foot's captured orientation.
- Controller rotation does not orbit the foot around the controller's physical location.
- Releasing and recapturing establishes a new pivot from the foot's then-current transform without a snap.
- The trigger press used to start overall VR tracking is still ignored for foot clutching until it has been released once.

## Non-goals for this change

- Changing the underlying two-grappler pose data.
- Full-body tracking beyond the existing headset and two-controller input model.
- Snapping a foot directly to a controller pose.
- Applying the controller's absolute orientation to the foot at capture time.

## Implementation note

Completed in `src/solver-demo/main.js` and `src/solver-demo.html`:

- Setup now includes a scene-turn control on the desktop panel plus 15-degree left/right controls in the headset panel. The yaw offset is applied to the shared stage root, so both grapplers, the grid, and disengaged trackers rotate together.
- The menu and stage are placed once when VR opens; looking away no longer triggers automatic recentering or scene rotation. Scene turn, scene distance, and floor height lock when tracking starts.
- The floating UI has a blue corner handle. Holding trigger on it and pointing around the user orbits only the panel at a fixed radius and height, keeps it facing the headset, and captures that trigger so it cannot start tracking or clutch a foot.
- Foot capture now creates a Babylon `TransformNode` at the foot's world pose, parents the foot beneath it at an identity local transform, and drives that pivot from independent controller translation and rotation deltas.
- `gmDebug.footPivotProbe()` verifies capture continuity in world and solver-stage coordinates, one-to-one translation, delta rotation, the pivot hierarchy, and the absence of controller-position orbiting.
