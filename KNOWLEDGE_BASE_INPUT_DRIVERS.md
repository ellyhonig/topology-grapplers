# Knowledge Base Parent: Input Drivers

Input drivers are upstream sources of observed or captured movement data for GrappleMap. These nodes describe systems that can provide recordings, tracked poses, timing, calibration data, or other external signals that may inform GrappleMap analysis and authoring.

Child nodes:

- `KNOWLEDGE_BASE_FREEMOCAP.md`: webcam and multi-camera markerless motion capture as sidecar input for pose validation, retargeting, and keyframe assistance.

## Role In GrappleMap

Input drivers should not replace GrappleMap's hand-authored graph. Their safest role is to provide sidecar evidence that agents can use when checking plausibility, suggesting transitions, estimating timing, or comparing authored motion against captured human movement.

Good input-driver tasks:

- import captured joint trajectories into a sidecar format
- retarget observed motion to GrappleMap's two-player skeleton
- compare captured poses to existing nodes
- identify stable holds or useful intermediate frames
- provide timing and smoothness priors for transition repair
