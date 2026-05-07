# GrappleMap Knowledge Base

This document is a working map for agent-assisted development on GrappleMap. It summarizes the codebase shape, the database model, important invariants, and the safest entry points for future automation.

## Project Purpose

GrappleMap is a public-domain database and toolset for grappling positions and transitions. The data is a directed graph:

- Nodes are named grappling positions.
- Edges are animated transitions between positions.
- Tags and properties classify both nodes and transitions.
- The same database powers static pages, browser tools, native tools, VR tools, video generation, graph export, and diffing.

The canonical source of grappling data is `GrappleMap.txt`. Most software in the repo either parses this file, edits it, renders it, exports it, or builds derived artifacts from it.

Related knowledge-base nodes:

- `KNOWLEDGE_BASE_ANIMATION_DRIVERS.md`: parent node for motion-generation, transition-repair, and animation-validation methods.
  - `KNOWLEDGE_BASE_TOPOLOGY_COORDINATES.md`: research note on Ho and Komura's topology-coordinate method and how it could support GrappleMap agents.
  - `KNOWLEDGE_BASE_ATTACK_DEFENSE_FSM.md`: research note on Ho and Komura's topology-coordinate finite-state-machine method for interactive wrestling attacks and defenses.
- `KNOWLEDGE_BASE_INPUT_DRIVERS.md`: parent node for upstream capture, sensor, and observed-motion sources.
  - `KNOWLEDGE_BASE_FREEMOCAP.md`: source note on using FreeMoCap webcam/motion-capture data as a sidecar input for GrappleMap validation, retargeting, and keyframe assistance.

## Top-Level Layout

- `GrappleMap.txt`: primary grappling database. It is large, plain text, and intentionally diffable.
- `src/`: C++ core, Emscripten bindings, browser JavaScript, HTML/CSS, native editor/playback tools, renderers, and utility binaries.
- `doc/`: human documentation for development and editor usage.
- `drills/`: scripted drill paths by position/transition name.
- `scripts/`: deployment, packaging, object-store, video, and dependency helper scripts.
- `proofs/`: Coq formalizations for math/position logic.
- `blender/`: Blender animation helper script.
- `grappleman.mhm`: extra project data asset; not part of the main parser path inspected here.

## Main Data Model

### Database Format

`GrappleMap.txt` is a sequence of records. Each record has metadata lines followed by one or more encoded positions:

- A record with exactly one encoded position becomes a named graph node.
- A record with two or more encoded positions becomes a transition edge.
- Metadata lines are unindented.
- Encoded position lines are indented with spaces.
- One encoded position is stored as four indented base62 lines.

Metadata conventions:

- First metadata line is the display name. It often uses literal `\n` to represent line breaks in labels.
- `tags:` declares search/categorization tags.
- `properties:` declares behavior/classification flags.
- `ref:` points to source instructional material.

Known properties:

- `top`: top player initiative/classification.
- `bottom`: bottom player initiative/classification.
- `bidirectional`: transition can be traversed in both directions.
- `detailed`: doubles keyframe density from 5 segments/second to 10 segments/second.

### C++ Types

Core types live in:

- `src/positions.hpp`
- `src/graph.hpp`
- `src/persistence.cpp`
- `src/metadata.cpp`
- `src/paths.hpp` / `src/paths.cpp`
- `src/editor.hpp` / `src/editor.cpp`

Important concepts:

- `Position`: two players, each with a fixed set of 23 joints, each joint as a `V3`.
- `Sequence`: transition metadata plus two or more `Position` frames.
- `NamedPosition`: position metadata plus one `Position`.
- `Graph::Node`: named position plus incoming/outgoing transition lists.
- `Graph::Edge`: sequence plus `from` and `to` node references.
- `PositionReorientation`: rotation/translation, optional mirror, optional player swap.
- `Reoriented<T>` / `Reversible<T>`: wrappers used to track orientation while traversing paths.

The graph intentionally treats positions as equivalent under reorientation when possible. This keeps the map connected even when database records encode the same physical position with different rotation, translation, mirror, or player ordering.

## Important Invariants

These are the invariants future agents should preserve:

- A transition must have at least two frames.
- A transition must not start and end at the same position modulo reorientation.
- The first and last transition frames correspond to graph nodes.
- Editing a connecting position can affect every transition connected to that node unless the edit is only a reorientation.
- Local edits to intermediate frames only affect the current transition.
- A `bidirectional` transition appears as reverse-traversable in node incoming/outgoing lists.
- The joint order in browser JavaScript must stay aligned with the C++ joint tables.
- `GrappleMap.txt` should keep Unix-style text formatting because it is the canonical diffable data source.

## Parser And Persistence Flow

`src/persistence.cpp` owns the text database format.

Read path:

1. `loadGraph(filename)` reads the whole file.
2. `readSeqs()` groups metadata and encoded position blocks.
3. One-frame sequences are extracted as named positions.
4. Multi-frame sequences become transition edges.
5. `Graph` connects transition endpoints to existing or newly discovered nodes.
6. A sidecar `GrappleMap.txt.index` may cache edge endpoint node numbers using an MD5 hash of the database.

Write path:

1. `save(Graph, ostream)` writes named nodes first.
2. It then writes all transition sequences.
3. `Position` encoding uses base62 pairs for scaled coordinates.

Agent note: avoid hand-editing encoded position blocks unless the task is explicitly data authoring. It is safer to edit metadata, tags, properties, docs, UI code, or use the editor paths for pose changes.

## Browser Runtime

The browser tools use Emscripten-generated C++ bindings plus hand-written JavaScript.

Shared runtime:

- `src/web_db_loader.cpp`: exposes `Module.loadDB()`.
- `src/js_conversions.cpp`: converts `Graph`, nodes, transitions, frames, tags, and reorientations to JS values.
- `src/gm.js`: shared Babylon.js body rendering, reorientation utilities, interpolation, random paths, and DB prep.
- `src/graphdisplay.js`: D3 graph visualization helpers.

Browser pages:

- `src/search.html` / `src/search.js`: search/tag browsing.
- `src/explorer.html` / `src/explorer.js`: graph neighborhood explorer.
- `src/composer.html` / `src/composer.js`: build and play drill paths.
- `src/editor.html` / `src/editor.js`: web editor UI around Emscripten editor bindings.
- `src/example-drills.html`: drill examples.

Data shape exposed to JS:

- `db.nodes[]`: `id`, `incoming`, `outgoing`, `position`, `description`, `tags`, optional `line_nr`.
- `db.transitions[]`: `id`, `from`, `to`, `frames`, `description`, `tags`, `properties`, optional `line_nr`.
- Transition endpoints include node IDs and reorientation objects.
- Reorientation objects contain `mirror`, `swap_players`, `angle`, and `offset`.

## Editor Architecture

The C++ editor model is in `src/editor.cpp` and `src/editor.hpp`. It is wrapped for the web by `src/editor_canvas.cpp`, and for native GLFW by `src/glfw_editor.cpp`.

Editor responsibilities:

- Track current graph, selection path, current location, playback state, selection lock, and undo stack.
- Move to named entities (`p34`, `t123`, `l31432`, `last-trans`, or display names).
- Insert/delete keyframes.
- Split transitions at a keyframe.
- Add new prepend/append transitions to a selected path.
- Replace current positions with local/propagating/unintended node-modification policy.
- Update node and transition metadata.

Web editor bindings from `editor_canvas.cpp` include:

- `editor_main`, `editor_loadDB`, `getDB`
- `get_selection`, `get_pre_choices`, `get_post_choices`, `get_dirty`
- `insert_keyframe`, `delete_keyframe`, `split_seq`, `undo`
- `prepend_new`, `append_new`, `set_selected`, `browseto`
- `set_node_desc`, `set_seq_desc`
- mode, transform, joint-selection, confinement, mirroring, resolution, and view controls

Agent note: if the goal is a new agent workflow around data editing, start by wrapping existing editor operations rather than reimplementing graph mutation rules.

## Build System

The active build file is `src/SConstruct` and uses SCons.

Native build dependencies are listed in `scripts/apt-install-devtools.sh`:

- `g++`, `scons`, `pkg-config`
- Boost headers and Boost Program Options/Regex
- Graphviz development libraries
- FTGL, Xine, GLFW, OpenGL/GLU

Useful targets:

- `grapplemap-glfw-editor`: native editor.
- `grapplemap-glfw-playback`: native playback.
- `grapplemap-indexer`: writes `GrappleMap.txt.index`.
- `grapplemap-dbtojs`: exports JS database.
- `grapplemap-todot`: emits Graphviz DOT.
- `grapplemap-diff`: compares database revisions.
- `grapplemap-mkpospages`: static position pages.
- `grapplemap-mkvid`: video generation.
- `libgrapplemap.js`: Emscripten browser runtime.
- `noX`: alias for non-X11/headless-ish outputs listed in `SConstruct`.

Basic documented native editor build on Ubuntu:

```sh
cd src
scons -j8 grapplemap-glfw-editor
./grapplemap-glfw-editor --db ../GrappleMap.txt p34
```

This repository was inspected from Windows, but the documented development path is Linux/Ubuntu.

## Utility Programs

- `src/indexer.cpp`: generate/update database index cache.
- `src/dbtojs.cpp`: load database and write `transitions.js`.
- `src/todot.cpp`: output a DOT graph.
- `src/diff.cpp`: compare two graph databases with position/transition awareness.
- `src/mkpospages.cpp`: generate per-position web pages and images.
- `src/makevideo.cpp`: create videos from scenes or generated paths.
- `src/glfw_playback.cpp`, `src/vr_playback.cpp`: playback frontends.
- `src/vr_editor.cpp`, `src/vr_joint_editor.cpp`, `src/vr_joint_browser.cpp`: VR editor tooling.

## Rendering

Native rendering:

- `src/rendering.cpp`
- `src/playerdrawer.cpp`
- `src/images.cpp`
- `src/icosphere.cpp`
- `src/camera.hpp`

Browser rendering:

- `src/gm.js` uses Babylon.js.
- Stick figures are rendered from fixed joint and segment tables.
- Camera helpers follow player cores/head/hands for external or first-person views.

Agent note: the browser joint constants in `gm.js` are manually synchronized with C++ tables. Any body model change must update both sides together.

## Tags, Search, And Metadata

Metadata helpers live in `src/metadata.cpp`.

Capabilities:

- Extract properties from description lines.
- Extract tags from nodes and edges.
- Find nodes/transitions by description or ID-like string.
- Build tag queries for search/explorer behavior.
- Resolve named entities such as `p34`, `t1383`, `l31432`, and `last-trans`.

For metadata-only work, prefer edits to names, `tags:`, `properties:`, and `ref:` lines in `GrappleMap.txt`. Do not rewrap encoded position lines.

## Agent Workflows Worth Building

Good near-term agent tasks:

- Metadata linting: duplicate tags, inconsistent tags, missing refs, unknown properties, capitalization drift.
- Database summarization: extract techniques by domain/tag/property and produce drill/query docs.
- Search improvements: better query syntax, tag facets, negative tags, saved searches.
- Contribution assistant: guide users from an editor change to a clean diff and PR-ready summary.
- Drill generator: build paths from tags, start/end positions, frame length, or player role.
- Data validation: detect identity transitions, orphan nodes, disconnected domains, missing descriptions, suspicious frame counts.
- JS modernization: replace legacy globals gradually while preserving generated Emscripten `Module` contracts.
- Build/dependency containerization: codify the documented Ubuntu setup in Docker or devcontainer files.

Riskier tasks:

- Pose/animation generation directly into `GrappleMap.txt`.
- Changing joint schema or coordinate encoding.
- Editing `Graph::replace`, reorientation, or path traversal without tests.
- Browser UI rewrites that break the Emscripten module load lifecycle.

## Suggested Agent Guardrails

When an agent modifies this repo:

1. Run `git status --short` before editing.
2. Treat `GrappleMap.txt` as high-risk data.
3. Prefer C++ graph/editor APIs for structural data changes.
4. Keep generated files and sidecar indexes out of commits unless explicitly requested.
5. Preserve public URL/query formats such as `?p34`, transition lists, and selected node lists.
6. Preserve `Module.*` names used by the HTML/JS unless updating all call sites and generated bindings.
7. For browser changes, verify each affected HTML entry point loads.
8. For graph changes, verify parser/indexer/diff tools still build and run.

## Quick Orientation Checklist

Before taking a new task, inspect:

- `README.md` for product intent.
- `doc/dev.md` for build notes.
- `doc/web-editor.md` for editor behavior and user-facing invariants.
- `src/SConstruct` for the relevant build target.
- `src/persistence.cpp` if touching database format.
- `src/graph.cpp` if touching graph mutation.
- `src/editor.cpp` and `src/editor_canvas.cpp` if touching editing behavior.
- `src/gm.js`, `src/composer.js`, `src/explorer.js`, or `src/editor.js` if touching browser behavior.

## Open Questions For Future Agent Work

- Which agent surface is desired first: code assistant, data curation assistant, browser UI assistant, or autonomous graph/drill explorer?
- Should agent-generated changes write directly to `GrappleMap.txt`, or produce reviewable patches/instructions first?
- Is the target runtime still the original Emscripten/SCons stack, or should a modern build wrapper be added around it?
- Should validation live as a C++ utility, a standalone script, or browser/editor warnings?
