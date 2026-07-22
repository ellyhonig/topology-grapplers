# Solver Demo

This browser demo renders two GrappleMap grapplers and drives the Rust XPBD
solver with mouse gizmos or WebXR input. The repository includes the compiled
`gm-wasm` web package, so a fresh clone can run without installing Rust.

## Run locally

On Windows, start SteamVR and double-click `start-steamvr-demo.bat`. This starts
the local tracker bridge and correctly rooted static server together, then
opens `http://localhost:8766/src/solver-demo.html`.

Start the static server from the repository root:

```sh
python3 -m http.server 8765
```

Open:

```text
http://localhost:8765/src/solver-demo.html
```

The server must start at the repository root. The demo loads
`/GrappleMap.txt`, shared files under `/src`, and the WASM package under
`/src/solver-demo/pkg`.

Python's static server sends `.wasm` as `application/wasm` on current Python
versions. Confirm the package is reachable if the page shows only HTML:

```sh
curl -I http://localhost:8765/src/solver-demo/pkg/gm_wasm.js
curl -I http://localhost:8765/src/solver-demo/pkg/gm_wasm_bg.wasm
curl -I http://localhost:8765/GrappleMap.txt
```

All three requests should return `200`. The WASM response should use
`Content-Type: application/wasm`.

## VR controls

Chromium does not expose generic SteamVR trackers through WebXR. The included
loopback bridge supplies their poses separately:

```sh
python -m pip install openvr cryptography
python scripts/steamvr_tracker_bridge.py
```

Use `start-steamvr-firebase.bat` to start the bridge and open the hosted site in
one step. Local pages connect to `ws://127.0.0.1:17373`; the HTTPS Firebase page
uses `wss://127.0.0.1:17374`. On first run, the bridge creates and trusts a
GrappleMap-only localhost server certificate for the current Windows user.
Both listeners remain loopback-only. Allow Chrome's Local Network Access prompt.

1. Open the demo in a WebXR-capable headset browser and choose **Enter VR**.
2. Choose a starting GrappleMap position and either the red or blue grappler.
   No body trackers are active during setup.
3. The setup panel and grappler stage are placed once when VR opens. Looking
   away does not move or rotate either one. Use **Turn left** and **Turn right**
   to rotate both grapplers and the mat together until the controlled grappler
   is aligned with your real body.
4. To move only the UI, point at its blue upper corner, hold trigger, and point
   around yourself. The panel orbits at a constant distance and keeps facing
   you; release trigger to leave it in place.
5. Adjust scene distance, manual floor height, or tracker stiffness if needed.
   Point at a panel control and press any controller button to activate it.
6. Point away from the panel and press either trigger once. This locks the
   setup placement, snaps the head and both hands to the headset/controllers,
   and hides the controlled grappler's tracker-box visuals.
7. With two SteamVR trackers connected, they are assigned to the feet and both
   controllers remain on the hands. Without them, release the starting trigger,
   then hold a controller trigger to clutch that side from hand to foot.

At the initial tracking moment, head and hand trackers snap directly onto the
headset and controllers. A foot clutch instead creates a temporary pivot at
the foot's existing world position and orientation, then parents the foot to
that pivot without changing its pose. Controller translation and rotation
deltas drive the pivot from the capture pose, so controller rotation cannot
orbit the ankle around the hand. Foot rotation aims the toe around the pinned
ankle and does not pull on the heel/whole leg.
The solver reads all tracker world poses in stage coordinates.

The floating panel also provides **Floor -5 cm**, **Floor +5 cm**, **Reset
pose**, and **Release trackers**. Releasing trackers returns to setup; the next
off-panel trigger press resumes tracking with a fresh one-time snap. The setup
menu and grappler stage remain fixed after their initial placement, both before
and after tracking starts. Scene turn, distance, and floor height also lock when
tracking starts.

## Headset access and port forwarding

WebXR requires a secure browser context. `http://localhost` is accepted for
local development, but a headset opening `http://<computer-LAN-IP>:8765` may
not expose WebXR. Use an HTTPS port-forwarding/tunnel URL, HTTPS hosting, or a
development workflow that presents the server as localhost on the headset.

The tunnel must forward the repository-root server on port `8765`. Opening a
URL that serves only the `src` directory breaks the `/GrappleMap.txt` request.

## Rebuild the WASM package

The complete Rust workspace lives in this repository under `solver/`:

```text
topology-grapplers/
├── solver/
└── src/solver-demo/pkg/
```

Install Rust and `wasm-pack`, then run from `solver/`:

```sh
wasm-pack build crates/gm-wasm --target web \
  --out-dir ../../../src/solver-demo/pkg
```

Commit the regenerated files in `src/solver-demo/pkg` whenever the Rust solver
API changes. Do not restore wasm-pack's default catch-all ignore rule without
keeping the package allowlist in `pkg/.gitignore`; fresh clones depend on these
compiled files.

## Troubleshooting

### The page looks like plain HTML

Open the browser developer console and network panel. A 404 for
`solver-demo/pkg/gm_wasm.js` or `gm_wasm_bg.wasm` means the compiled package is
missing or the server root is wrong. A JavaScript module or WASM error prevents
Babylon from creating the scene, leaving only the HTML overlay visible.

### VR is unavailable

Confirm that the browser supports `immersive-vr`, the headset is connected,
and the page is in a secure context. Desktop browsers without an attached
headset will correctly show `no headset found` while the mouse demo continues
to work.

### Babylon or the GUI does not load

The demo loads Babylon.js and Babylon GUI from `cdn.babylonjs.com`. The browser
therefore needs outbound internet access unless those scripts are vendored
locally in a future change.
