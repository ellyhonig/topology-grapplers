# GrappleMap

## Introduction

The GrappleMap is:

1. A database of interconnected grappling positions and transitions,
   animated with stick figures.
2. A set of tools to build and explore this database.

The main interface for the database is the [website](http://eel.is/GrappleMap/), which has:

- A [search page](http://eel.is/GrappleMap/):

  ![screenshot](http://eel.is/GrappleMap-extra/search.png)

- Per-position pages:

  ![screenshot](http://eel.is/GrappleMap-extra/pospage.png)

- [Composer](http://eel.is/GrappleMap/composer/):

  ![screenshot](http://eel.is/GrappleMap-extra/composer.png)

- [Explorer](http://eel.is/GrappleMap/explorer/):

  ![screenshot](http://eel.is/GrappleMap-extra/explorer.png)

- [Editor](http://eel.is/GrappleMap/editor/):

  ![screenshot](http://eel.is/GrappleMap-extra/editor.png)

In addition, there is:

- a native editor (only built and tested on Linux)

- a VR interface (only built and tested on Linux):

  [![screenshot](https://img.youtube.com/vi/MAeBgGZ1GdM/0.jpg)](https://www.youtube.com/watch?v=MAeBgGZ1GdM)

- some utilities for making videos of scripted or randomly generated matches, like these:

  [![demo](https://img.youtube.com/vi/sdygmrlm-ck/0.jpg)](https://www.youtube.com/watch?v=sdygmrlm-ck)
  [![blenderdemo](https://img.youtube.com/vi/rC7zTBMPj1Y/0.jpg)](https://www.youtube.com/watch?v=rC7zTBMPj1Y)

- a diff utility for summarizing changesets, e.g.:

  [![diff](http://eel.is/GrappleMap-extra/diff-small.png)](http://eel.is/GrappleMap-extra/diff-big.png)

## Rust Solver Demo (WebXR)

`src/solver-demo.html` is a browser demo that renders two grapplers with
[Babylon.js](https://www.babylonjs.com/) and drives their bodies through a Rust
XPBD constraint solver compiled to WebAssembly (`src/solver-demo/pkg/`, built
with `wasm-pack`). Its editable Rust workspace, including muscle-tone and
constraint code, is in `solver/`. Each grappler carries five 6-DOF trackers —
hands, feet, and head, like a VR tracker rig. On desktop you engage a tracker and move it with
the gizmo; in a headset the head and hand gizmos follow the HMD/controllers,
with each active gizmo parented to its XR transform. Because Chromium does not
expose SteamVR generic trackers through WebXR, the SteamVR variant receives two
dedicated foot poses from `scripts/steamvr_tracker_bridge.py` on loopback. The
body follows under gravity, muscle tone, and anatomy limits. Positions
are loaded from the top-level `GrappleMap.txt`. In VR, choose whether to drive
the red or blue grappler, position the shared menu/grappler stage, adjust the
manual floor height if needed, then press either trigger once to calibrate and
start tracking. The first press snaps the hands and head to the XR hardware,
assigns the two lowest SteamVR tracker poses left/right from their positions,
captures the foot offsets/orientations, and hides the tracker visuals.
Controllers remain assigned to the hands. The floating panel accepts any
controller button and includes pose reset, tracker release, stiffness,
floor-height, and scene-distance controls.

**Live build:** https://grapplemap-solver-vr.web.app

### Running it locally

On Windows, start SteamVR, then double-click `start-steamvr-demo.bat`. It starts
both the local tracker bridge and the correctly rooted web server, opens the
demo at `http://localhost:8766/src/solver-demo.html`, and keeps running until
you close its console window. Make sure **SteamVR Settings > OpenXR** reports
SteamVR as the current OpenXR runtime.

The page is fully client-side but has two path requirements: assets resolve
relative to `src/`, and the page fetches `../GrappleMap.txt` (the database at the
repo root). Serve the **repo root** and open the page under `/src/`:

```sh
python -m http.server 8000      # from the repo root
# then open http://localhost:8000/src/solver-demo.html
```

The generic foot trackers require the bridge as well. The one-command manual
equivalent is `python scripts/steamvr_tracker_bridge.py --serve`.

Babylon.js and Babylon GUI are vendored under `src/solver-demo/vendor`, so the
local demo does not depend on a CDN. The `.wasm` module must be served with
`Content-Type: application/wasm`.

### VR requires HTTPS

WebXR immersive sessions only run in a [secure
context](https://developer.mozilla.org/docs/Web/Security/Secure_Contexts):
`https://` or `localhost`. Loading the page over a plain-HTTP LAN/public IP
(e.g. `http://192.168.x.x:8080`) lets the desktop 3D scene render but silently
blocks "Open in VR". Use the HTTPS build above (or any HTTPS host) from the
headset.

## FAQ

### Which grappling techniques are included?

The intent is to map as many proven MMA-applicable grappling techniques as possible,
regardless of whether they come from wrestling, Brazilian Jiu-Jitsu, Judo,
Sambo, etc etc.


### What is included per technique?

The map only demonstrates techniques in their bare textbook form, and does not
include any discussion or explanation. However, the map almost always
cites source instructional materials, which readers are strongly
encouraged to consult for detailed breakdowns and tactics.


### How is the map organized?

#### 1) as a graph

Techniques are modeled in a big [directed graph](https://en.wikipedia.org/wiki/Graph_%28discrete_mathematics%29),
where each vertex is
a concrete pose of the two stick figure players, and each edge is a transition from one such position to another.

#### 2) by tags

In addition to being interconnected, positions and transitions are also named and tagged.
Tags are used for categorization into domains (e.g. deep_half, side\_control),
but also for more detailed information about poses (e.g. bottom\_supine, top\_posture\_broken),
and also for presence of any specific grips or controls (e.g. lockdown, kimura, crossface, arm_drag).


### What is the purpose of the map?

- To summarize proven techniques that address various situations
- To serve as an index into the literature, giving suggestions for specifically relevant instructional materials
- To give concrete ideas for drills to practice
- To (maybe one day) serve as training data set for machine learning algorithms to discover new jiu jitsu
- To be beautiful


### Won't it take forever to get decent coverage and quality?

If I have to do it all by myself, probably yes.
But I hope that once word gets around about the GrappleMap,
other grappling nerds will want to chip in and help me make it
the most awesome (no-gi) grappling map ever. :)

See also [Can I help map techniques?](#can-i-help-map-techniques).




### What license is the code and data under?

None; the GrappleMap code and data is released into the public domain.


### How well can the map model grappling technique?

There are two aspects to this: the graph structure itself, and the
quality of the individual transition animations.

The graph structure itself is obviously a gross simplification.
Real grappling is a continuous space where
even individual positions and transitions occur in infinite variety.
Still, all grappling instructionals include examples
of forms that a technique can or should take.
The GrappleMap tries to connect and integrate all these fragmented
examples, in order to form a more complete picture.

The transition animations are pretty rudimentary, in part because
they are manually edited rather than motion-captured, and in part
because the animation system itself is currently based on
very simplistic fixed-interval keyframes, which limits joints to
5 direction changes per second.
This means that things like small-scale hand fighting and battling
for grips is barely modeled at all.
Usually the level of detail in the map is that a grip is either
acquired without great strain, or not at all, and that's it.
The timing of techniques is also not really captured at all.
The animations are really just schematics showing how the different
entanglements and postures flow into eachother, not live action.

### What about gi techniques?

I personally have no interest in gi-specific techniques, there are no such
techniques in the database, and clothes are not modeled in the software
(doing it right would be a lot of additional work).

If someone else wants to make GiGrappleMap, power to them!


### Wouldn't more realistic bodies/skeletons be better?

To make creating and editing transitions as simple as possible, the stick figures
originally had even fewer "joints"/"bones". I added more only as
necessary to allow execution of techniques I wanted to map. This
process resulted in the current stick figures, which I think strike a good
balance that fits the purpose of the GrappleMap.


### What about other body types?

I'm not really interested in mapping those few rare techniques that only work for
specific body types.


### What about the fence/cage (in MMA)?

It's on the [todo list](todo.txt).


### How about some nicer graphics?

Maybe later. (Patches welcome!)


### How come the animations run so poorly?

Try using Chrome. It's the only browser that I've ever gotten good WebGL performance out of. :/


### What is the database format?

The database is a [plain text file](https://github.com/Eelis/GrappleMap/blob/master/GrappleMap.txt) that declares:

1. Positions (with name, tags, description/reference, and joint coordinates)

2. Transitions (with name, tags, description/reference, and joint coordinates for each frame)

It's a pretty ad-hoc and minimalistic format, but it has the nice property that
textual diffs show which positions and transitions differ, and that an ordinary
text editor can be used to edit the names, descriptions, and tags.

I fully expect that the format will evolve into something different to accommodate future features.


### Do you train personally?

Back in university, I trained with their MMA club for a year or so, but it was super
casual, and a long time ago. I basically have no martial arts credentials whatsoever.


### Can I help map techniques?

Absolutely! There is *way* too much grappling technique for me to do this by myself,
so nothing would please me more than to see the GrappleMap become a collaborative effort.

Fire up the [editor](http://eel.is/GrappleMap/editor/), read its [manual](https://github.com/Eelis/GrappleMap/blob/master/doc/web-editor.md), and have a go. If you get stuck, [drop by](https://webchat.freenode.net/) in #grapplemap on Freenode IRC or mail me at grapplemap@contacts.eelis.net.

### Can I help improve the software/website?

Absolutely! For ideas for things you could work on, see the [todo list](doc/todo.txt).
