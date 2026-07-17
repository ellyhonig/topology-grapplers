# Solver Demo

This browser demo renders two GrappleMap grapplers and drives the Rust XPBD
solver with mouse gizmos or WebXR input. The repository includes the compiled
`gm-wasm` web package, so a fresh clone can run without installing Rust.

## Run locally

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

1. Connect and track two foot trackers in SteamVR, then open the demo in a
   WebXR-capable headset browser and choose **Enter VR**.
2. Choose a starting GrappleMap position and either the red or blue grappler.
   No body trackers are active during setup.
3. Move and turn your head to position yourself. Whenever the setup panel
   recenters, the grappler stage recenters with it.
4. Adjust scene distance, manual floor height, or tracker stiffness if needed.
   Point at a panel control and press any controller button to activate it.
5. Look forward and stand with the tracker on your left foot physically to
   your left and the tracker on your right foot physically to your right.
6. Point away from the panel and press either controller trigger once. This
   locks the setup placement and performs the only start/calibration action.
   The head and hands attach to the headset/controllers, while the two lowest
   unhanded 6-DOF WebXR sources are classified left/right along the headset's
   horizontal axis and assigned to the matching foot gizmos.

At calibration, each foot gizmo preserves its existing position and orientation
relative to its assigned SteamVR tracker. Tracker translation then moves the
ankle one-to-one, while tracker rotation changes the calibrated foot orientation
without orbiting the ankle target. The controllers always drive the hands;
holding a trigger no longer switches a controller to a foot. The solver reads
all tracker world poses in stage coordinates.

The floating panel also provides **Floor -5 cm**, **Floor +5 cm**, **Reset
pose**, and **Release trackers**. Releasing trackers returns to setup; the next
off-panel trigger press detects, assigns, and recalibrates the feet. Before
tracking starts, the setup menu and grappler stage recenter together when the
headset turns more than 30 degrees away. After tracking starts, both stay
locked in the play space.

SteamVR tracker discovery uses WebXR input sources with handedness `none` and
target-ray mode `tracked-pointer`. If the status does not reach `2/2 feet
found`, confirm both trackers are powered, tracked, and exposed to the browser's
WebXR session. When more than two unhanded tracked devices are exposed, the two
lowest devices at calibration are used.

## Headset access and port forwarding

WebXR requires a secure browser context. `http://localhost` is accepted for
local development, but a headset opening `http://<computer-LAN-IP>:8765` may
not expose WebXR. Use an HTTPS port-forwarding/tunnel URL, HTTPS hosting, or a
development workflow that presents the server as localhost on the headset.

The tunnel must forward the repository-root server on port `8765`. Opening a
URL that serves only the `src` directory breaks the `/GrappleMap.txt` request.

## Rebuild the WASM package

The Rust source currently lives in the sibling `grapple-solver` workspace. If
both directories share the same parent:

```text
grapplemapapp/
├── grapple-solver/
└── topology-grapplers/
```

install Rust and `wasm-pack`, then run from `grapple-solver`:

```sh
wasm-pack build crates/gm-wasm --target web \
  --out-dir ../../../topology-grapplers/src/solver-demo/pkg
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
