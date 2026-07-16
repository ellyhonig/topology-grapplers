// Rust solver demo: Babylon rendering + tracker-based input driving the
// gm-wasm XPBD engine.
//
// Input model (deliberately identical to a VR tracker rig): each player has
// five 6-DOF trackers - left hand, right hand, left foot, right foot, and
// head. On the mouse version they are draggable/rotatable gizmo boxes. A
// tracker that has not been touched simply follows its body part; grabbing
// it *engages* it, and from then on the tracker's position + orientation
// drive that body part through compliant solver constraints. Joints can NOT be
// grabbed arbitrarily - all input flows through the trackers, exactly like
// real hardware.

import init, { Engine } from "./pkg/gm_wasm.js";

var engine3d, scene, camera;
var wasmEngine;
var updatePlayers;
var currentPose = null; // [[V3;23];2]
var entries = [];
var jointNames = [];
var uiTick = 0;
var dualTest = null; // { t } while the scripted two-player test runs
var scriptedEffectors = new Map(); // debug hook -> { player, joint, target }

// ---------------------------------------------------------------- trackers

// joint: the joint the tracker position drives.
// aux:   an optional second joint driven by the tracker's forward axis,
//        conveying the tracker's *orientation* to the solver (fingers for
//        hands, toes for feet). auxBack mirrors it behind (wrist/heel/neck).
var TRACKER_DEFS = [
	{ label: "left hand", joint: LeftHand, aux: LeftFingers, auxBack: LeftWrist },
	{ label: "right hand", joint: RightHand, aux: RightFingers, auxBack: RightWrist },
	{ label: "left foot", joint: LeftAnkle, aux: LeftToe, auxBack: LeftHeel },
	{ label: "right foot", joint: RightAnkle, aux: RightToe, auxBack: RightHeel },
	{ label: "head", joint: Head, auxBack: Neck }
];

var trackers = []; // { player, def, mesh, engaged, auxDist, auxBackDist }
var gizmoManager = null;
var selectedTracker = null;

function byId(id) { return document.getElementById(id); }

function flatToPose(flat) {
	var p = [[], []];
	for (var i = 0; i < 46; ++i) {
		var player = Math.floor(i / 23);
		p[player].push(v3(flat[i * 3], flat[i * 3 + 1], flat[i * 3 + 2]));
	}
	return p;
}

// ---------------------------------------------------------------- scene

function initScene() {
	var canvas = byId("renderCanvas");
	engine3d = new BABYLON.Engine(canvas, true);
	scene = new BABYLON.Scene(engine3d);
	scene.clearColor = new BABYLON.Color4(0.91, 0.93, 0.95, 1);
	camera = new BABYLON.ArcRotateCamera("camera", -Math.PI / 2.35, Math.PI / 2.8, 4.2, v3(0, 0.85, 0), scene);
	camera.attachControl(canvas, true);
	camera.wheelPrecision = 55;
	var light = new BABYLON.HemisphericLight("hemi", v3(0.4, 1, 0.2), scene);
	light.intensity = 0.9;

	var red = new BABYLON.StandardMaterial("redskin", scene);
	var blue = new BABYLON.StandardMaterial("blueskin", scene);
	red.diffuseColor = new BABYLON.Color3(0.62, 0.06, 0.04);
	blue.diffuseColor = new BABYLON.Color3(0.04, 0.08, 0.58);
	red.specularPower = 0;
	blue.specularPower = 0;

	var updaters = [
		animated_player_from_array(currentPose[0], red, scene),
		animated_player_from_array(currentPose[1], blue, scene)
	];
	updatePlayers = function (p) { updaters[0](p[0]); updaters[1](p[1]); };

	var grey = new BABYLON.Color3(0.68, 0.7, 0.72);
	for (var i = -6; i <= 6; ++i) {
		BABYLON.MeshBuilder.CreateLines("grid-x", { points: [v3(i / 2, 0, -3), v3(i / 2, 0, 3)] }, scene).color = grey;
		BABYLON.MeshBuilder.CreateLines("grid-z", { points: [v3(-3, 0, i / 2), v3(3, 0, i / 2)] }, scene).color = grey;
	}

	createTrackers();
	installTrackerControls();

	var lastTime = performance.now();
	engine3d.runRenderLoop(function () {
		var now = performance.now();
		var dt = Math.min((now - lastTime) / 1000, 0.05);
		lastTime = now;
		stepSolver(dt);
		scene.render();
	});
	window.addEventListener("resize", function () { engine3d.resize(); });
}

function trackerMaterial(player, engaged) {
	var name = "tracker-p" + player + (engaged ? "-on" : "-off");
	var existing = scene.getMaterialByName(name);
	if (existing) return existing;
	var m = new BABYLON.StandardMaterial(name, scene);
	if (player === 0) m.diffuseColor = engaged ? new BABYLON.Color3(1, 0.45, 0.35) : new BABYLON.Color3(0.75, 0.4, 0.36);
	else m.diffuseColor = engaged ? new BABYLON.Color3(0.4, 0.6, 1) : new BABYLON.Color3(0.4, 0.46, 0.72);
	m.emissiveColor = engaged
		? (player === 0 ? new BABYLON.Color3(0.35, 0.08, 0.03) : new BABYLON.Color3(0.03, 0.1, 0.35))
		: new BABYLON.Color3(0.05, 0.05, 0.05);
	m.alpha = engaged ? 0.95 : 0.55;
	return m;
}

function createTrackers() {
	for (var player = 0; player < 2; ++player) {
		TRACKER_DEFS.forEach(function (def) {
			var mesh = BABYLON.MeshBuilder.CreateBox("tracker", { width: 0.07, height: 0.045, depth: 0.11 }, scene);
			mesh.rotationQuaternion = BABYLON.Quaternion.Identity();
			mesh.isPickable = true;
			// A small nose marks the tracker's forward (+z) axis.
			var nose = BABYLON.MeshBuilder.CreateBox("tracker-nose", { width: 0.02, height: 0.02, depth: 0.035 }, scene);
			nose.parent = mesh;
			nose.position.z = 0.065;
			nose.isPickable = false;
			var tracker = {
				player: player,
				def: def,
				mesh: mesh,
				nose: nose,
				engaged: false,
				auxDist: 0.08,
				auxBackDist: 0.08
			};
			mesh.metadata = { tracker: tracker };
			mesh.material = trackerMaterial(player, false);
			nose.material = mesh.material;
			trackers.push(tracker);
		});
	}
	syncDisengagedTrackers();
}

// Point a disengaged tracker at its body part: position on the driven joint,
// forward axis along joint->aux (hand->fingers, ankle->toe) or away from the
// back joint (neck->head for the head tracker).
function syncDisengagedTrackers() {
	trackers.forEach(function (t) {
		if (t.engaged) return;
		var joint = currentPose[t.player][t.def.joint];
		var aux = t.def.aux === undefined ? null : currentPose[t.player][t.def.aux];
		var auxBack = t.def.auxBack === undefined ? null : currentPose[t.player][t.def.auxBack];
		t.mesh.position.copyFrom(joint);
		var fwd = aux ? aux.subtract(joint)
			: auxBack ? joint.subtract(auxBack)
			: v3(0, 0, 1);
		if (aux) t.auxDist = Math.max(fwd.length(), 0.03);
		if (auxBack) t.auxBackDist = Math.max(joint.subtract(auxBack).length(), 0.03);
		if (fwd.length() > 1e-6) {
			var dir = fwd.normalize();
			var up = Math.abs(dir.y) > 0.95 ? v3(1, 0, 0) : v3(0, 1, 0);
			var m = BABYLON.Matrix.Zero();
			BABYLON.Matrix.LookAtLHToRef(v3(0, 0, 0), dir, up, m);
			m.invert();
			BABYLON.Quaternion.FromRotationMatrixToRef(m, t.mesh.rotationQuaternion);
		}
	});
}

function setTrackerEngaged(t, engaged) {
	if (t.engaged === engaged) return;
	t.engaged = engaged;
	t.mesh.material = trackerMaterial(t.player, engaged);
	t.nose.material = t.mesh.material;
	refreshTrackerStatus();
}

function releaseAllTrackers() {
	trackers.forEach(function (t) { setTrackerEngaged(t, false); });
	if (gizmoManager) gizmoManager.attachToMesh(null);
	selectedTracker = null;
	syncDisengagedTrackers();
}

function refreshTrackerStatus() {
	var active = trackers.filter(function (t) { return t.engaged; });
	byId("trackerStatus").textContent = active.length
		? active.map(function (t) { return "p" + t.player + " " + t.def.label; }).join(", ")
		: "none engaged";
}

// ---------------------------------------------------------------- input

function installTrackerControls() {
	gizmoManager = new BABYLON.GizmoManager(scene);
	gizmoManager.positionGizmoEnabled = true;
	gizmoManager.rotationGizmoEnabled = true;
	gizmoManager.usePointerToAttachGizmos = false;
	gizmoManager.attachableMeshes = trackers.map(function (t) { return t.mesh; });

	// Engage + select on click; the gizmos then handle 6-DOF manipulation.
	scene.onPointerObservable.add(function (pointerInfo) {
		if (pointerInfo.type !== BABYLON.PointerEventTypes.POINTERDOWN) return;
		var evt = pointerInfo.event;
		// scene.pointerX/Y are in render-buffer space (offsetX is CSS space,
		// which diverges once the canvas is scaled), so picks land correctly.
		var pick = scene.pick(scene.pointerX, scene.pointerY, function (mesh) {
			return mesh.metadata && mesh.metadata.tracker;
		});
		if (!pick || !pick.hit) return;
		var tracker = pick.pickedMesh.metadata.tracker;
		if (evt.detail >= 2 || evt.shiftKey) {
			// Double-click / shift-click: release this tracker.
			setTrackerEngaged(tracker, false);
			if (selectedTracker === tracker) {
				gizmoManager.attachToMesh(null);
				selectedTracker = null;
			}
			syncDisengagedTrackers();
			return;
		}
		selectedTracker = tracker;
		setTrackerEngaged(tracker, true);
		gizmoManager.attachToMesh(tracker.mesh);
	});

	// Dragging a gizmo must not orbit the camera underneath. The aggregate
	// gizmos expose drag start/end observables (the per-axis sub-gizmos
	// don't in current Babylon).
	var hookGizmo = function (gizmo) {
		if (!gizmo || !gizmo.onDragStartObservable) return;
		gizmo.onDragStartObservable.add(function () { camera.detachControl(); });
		gizmo.onDragEndObservable.add(function () { camera.attachControl(byId("renderCanvas"), true); });
	};
	hookGizmo(gizmoManager.gizmos.positionGizmo);
	hookGizmo(gizmoManager.gizmos.rotationGizmo);
}

// ---------------------------------------------------------------- solve loop

function trackerEffectors(stiffness) {
	var list = [];
	trackers.forEach(function (t) {
		if (!t.engaged) return;
		var pos = t.mesh.position;
		// Keep targets above the mat: the solver would refuse anyway, but a
		// clamped target keeps the pull direction sensible.
		var y = Math.max(pos.y, 0.02);
		list.push([t.player, t.def.joint, pos.x, y, pos.z, stiffness]);
		// Orientation: forward axis places the optional aux joint (fingers/toe),
		// and the opposite side places the back joint (wrist/heel/neck), so
		// tracker rotation turns its associated body part.
		var fwd = t.mesh.forward ? t.mesh.forward : t.mesh.getDirection(BABYLON.Axis.Z);
		if (t.def.aux !== undefined) {
			var aux = pos.add(fwd.scale(t.auxDist));
			list.push([t.player, t.def.aux, aux.x, Math.max(aux.y, 0.02), aux.z, stiffness * 0.7]);
		}
		if (t.def.auxBack !== undefined) {
			var auxBack = pos.subtract(fwd.scale(t.auxBackDist));
			list.push([t.player, t.def.auxBack, auxBack.x, Math.max(auxBack.y, 0.02), auxBack.z, stiffness * 0.5]);
		}
	});
	return list;
}

function collectEffectors(dt) {
	var stiffness = parseFloat(byId("stiffness").value);
	var list = trackerEffectors(stiffness);

	scriptedEffectors.forEach(function (d) {
		list.push([d.player, d.joint, d.target.x, d.target.y, d.target.z, stiffness]);
	});

	// Scripted dual-input test: both players' right hands circle their own
	// opposite shoulders simultaneously, proving concurrent two-player input.
	if (dualTest) {
		dualTest.t += dt;
		var t = dualTest.t;
		for (var player = 0; player < 2; ++player) {
			var shoulder = currentPose[player][LeftShoulder];
			var phase = t * 1.6 + player * Math.PI;
			list.push([
				player, RightHand,
				shoulder.x + 0.35 * Math.cos(phase),
				Math.max(0.05, shoulder.y + 0.25 * Math.sin(phase * 0.7)),
				shoulder.z + 0.35 * Math.sin(phase),
				0.85
			]);
		}
		if (t > 12) stopDualTest();
	}

	var flat = new Float64Array(list.length * 6);
	list.forEach(function (e, i) { flat.set(e, i * 6); });
	return flat;
}

function stepSolver(dt) {
	var effectors = collectEffectors(dt);
	var t0 = performance.now();
	var flat = wasmEngine.step(effectors, dt);
	var solveMs = performance.now() - t0;
	currentPose = flatToPose(flat);
	updatePlayers(currentPose);
	syncDisengagedTrackers();

	if (++uiTick % 6 === 0) refreshHud(solveMs);
}

function startDualTest() {
	dualTest = { t: 0 };
	byId("dualTestBtn").textContent = "Stop two-player test";
}

function stopDualTest() {
	dualTest = null;
	byId("dualTestBtn").textContent = "Two-player input test";
}

function refreshHud(solveMs) {
	byId("solveMs").textContent = solveMs.toFixed(1) + " ms";
	var diagText = wasmEngine.diagnostics();
	if (diagText) {
		var diag = JSON.parse(diagText);
		byId("minClearance").textContent = diag.min_clearance.toFixed(3);
		byId("boneError").textContent = diag.max_bone_error.toFixed(4);
		byId("contactCount").textContent = diag.contact_count;
		byId("writheJump").textContent = diag.max_writhe_jump.toFixed(3);
		byId("watchdog").textContent = diag.rejected ? "step rejected"
			: diag.retries > 0 ? (diag.retries + " re-solve") : "clean";
	}

	var report = JSON.parse(wasmEngine.validate());
	var state = byId("validState");
	if (report.violations.length === 0) {
		state.textContent = "valid";
		state.className = "ok";
		byId("validationLog").textContent =
			"all invariants hold\n" +
			"min clearance: " + report.min_clearance.toFixed(4) + " m\n" +
			"max bone error: " + (report.max_bone_error * 100).toFixed(2) + " %";
	} else {
		state.textContent = report.violations.length + " violations";
		state.className = "bad";
		byId("validationLog").textContent = report.violations.slice(0, 8).map(function (violation) {
			return violation.kind + " " + violation.detail + " (" + violation.amount.toFixed(4) + ")";
		}).join("\n");
	}

	var topo = JSON.parse(wasmEngine.topologySummary());
	byId("topologyLog").textContent = topo.length
		? topo.slice(0, 8).map(function (row) {
			return row.a + " x " + row.b + ": " + row.writhe.toFixed(3);
		}).join("\n")
		: "no significant entanglement";
}

// ---------------------------------------------------------------- boot

function populatePositions() {
	var select = byId("positionSelect");
	entries.forEach(function (e) {
		if (e.frames !== 1) return; // named positions only
		var option = document.createElement("option");
		option.value = e.index;
		option.textContent = e.name;
		select.appendChild(option);
	});
	select.addEventListener("change", function () { loadEntry(parseInt(select.value, 10)); });
}

function loadEntry(index) {
	wasmEngine.loadEntry(index);
	currentPose = flatToPose(wasmEngine.pose());
	updatePlayers(currentPose);
	releaseAllTrackers();
}

async function boot() {
	await init();
	wasmEngine = new Engine();
	jointNames = JSON.parse(wasmEngine.jointNames());

	var response = await fetch("../GrappleMap.txt");
	var text = await response.text();
	entries = JSON.parse(wasmEngine.loadDatabase(text));
	populatePositions();

	// Start from the classic first position.
	var first = entries.find(function (e) { return e.frames === 1; });
	wasmEngine.loadEntry(first.index);
	byId("positionSelect").value = first.index;
	currentPose = flatToPose(wasmEngine.pose());

	initScene();
	refreshTrackerStatus();

	byId("resetBtn").addEventListener("click", function () {
		loadEntry(parseInt(byId("positionSelect").value, 10));
	});
	byId("releaseBtn").addEventListener("click", releaseAllTrackers);
	byId("dualTestBtn").addEventListener("click", function () {
		if (dualTest) stopDualTest();
		else startDualTest();
	});
	var stiffness = byId("stiffness");
	var stiffnessOut = byId("stiffnessOut");
	function syncStiffness() { stiffnessOut.textContent = stiffness.value; }
	stiffness.addEventListener("input", syncStiffness);
	syncStiffness();
}

// Debug/testing hook: lets automation and the console inspect and drive the
// module state (e.g. window.gmDebug.pose(0, RightHand)).
window.gmDebug = {
	pose: function (player, joint) {
		var p = currentPose[player][joint];
		return { x: p.x, y: p.y, z: p.z };
	},
	diagnostics: function () { return wasmEngine ? wasmEngine.diagnostics() : ""; },
	validate: function () { return wasmEngine ? wasmEngine.validate() : ""; },
	trackerCount: function () { return trackers.filter(function (t) { return t.engaged; }).length; },
	dualTestActive: function () { return !!dualTest; },
	startDualTest: function () { startDualTest(); },
	// Engage tracker i (0..4 player 0, 5..9 player 1) and move it.
	setTracker: function (index, x, y, z) {
		var t = trackers[index];
		setTrackerEngaged(t, true);
		t.mesh.position.copyFromFloats(x, y, z);
	},
	releaseTrackers: function () { releaseAllTrackers(); },
	setDrag: function (player, joint, x, y, z) {
		scriptedEffectors.set("debug", { player: player, joint: joint, target: v3(x, y, z) });
	},
	clearDrag: function () { scriptedEffectors.delete("debug"); },
	// Drive n solver steps synchronously (headless testing without rAF).
	tick: function (dt, n) {
		for (var i = 0; i < (n || 1); ++i) stepSolver(dt);
	}
};

boot();
