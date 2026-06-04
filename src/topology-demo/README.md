# Topology Demo Module Map

This folder splits the topology demo into subsystem files loaded by `../topology-demo.html`.
The files are plain browser scripts, so load order matters and is declared explicitly in the HTML.

- `00-shared.js`: shared constants, mutable app state, and tiny math/DOM helpers.
- `01-position-store.js`: `GrappleMap.txt` decoding, parsing, and fallback pose data.
- `02-body-topology-chains.js`: limb chains, body segment inventories, joint lookup, and drag-chain selection.
- `03-ik-body-rules.js`: IK/body rules, bone length projection, pins, foot triangle constraints, and bend planes.
- `04-contact-space.js`: body capsule contact projection and clearance measurement.
- `05-topology-coordinates.js`: writhe matrices, topology coordinate extraction, target matrix synthesis, and matrix stepping.
- `06-topology-solver.js`: damped linearized topology-space solve and post-solve body/contact restoration.
- `07-proof-rendering.js`: matrix canvases, contact proof lines, and kosher movement preview rendering.
- `08-motion-input-drivers.js`: slider auto-solve, drag state, drag-step integration, and contact relaxation.
- `09-scene-renderer.js`: Babylon scene setup, player rendering, grid, and draggable joint handles.
- `10-ui-app.js`: DOM wiring, proof text, position loading, and boot sequence.
- `08-ik-only-motion-input-drivers.js`: direct IK dragging without topology or contact systems.
- `10-ik-only-ui-app.js`: smaller IK-only boot file for `../ik-demo.html`.

When changing behavior, prefer editing the smallest subsystem file that owns it. Keep cross-subsystem calls explicit rather than hiding them behind new global helpers.
