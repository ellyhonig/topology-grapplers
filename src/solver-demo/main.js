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

var engine3d, scene, camera, stageRoot;
var wasmEngine;
var updatePlayers;
var currentPose = null; // [[V3;23];2]
var entries = [];
var namedEntries = [];
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
var headTranslationAlignment = null;
var headRotationAlignment = null;

// WebXR controls one selected grappler. All source poses are calibrated against
// the current tracker pose, so a standing player can control a grappler in any
// orientation without snapping the body upright.
var vr = {
	supported: false,
	active: false,
	started: false,
	trackingPaused: false,
	stagePlaced: false,
	menuLocked: false,
	player: 0,
	stageDistance: 1.6,
	experience: null,
	menu: null,
	headCalibration: null,
	controllers: {
		left: { source: null, mode: "hand", calibration: null },
		right: { source: null, mode: "hand", calibration: null }
	},
	lastStatus: ""
};

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
	stageRoot = new BABYLON.TransformNode("grappler-stage", scene);
	stageRoot.rotationQuaternion = BABYLON.Quaternion.Identity();
	// HMD local +Y (head-up) maps to the head gizmo's local +Z
	// (neck-to-head). Translation keeps HMD left/right unchanged. Rotation uses
	// the opposite quarter-turn so HMD roll bends the neck in the same direction
	// instead of mirroring the user's lean.
	headTranslationAlignment = BABYLON.Quaternion.RotationAxis(BABYLON.Axis.X, Math.PI / 2);
	headRotationAlignment = BABYLON.Quaternion.RotationAxis(BABYLON.Axis.X, -Math.PI / 2);

	var red = new BABYLON.StandardMaterial("redskin", scene);
	var blue = new BABYLON.StandardMaterial("blueskin", scene);
	red.diffuseColor = new BABYLON.Color3(0.62, 0.06, 0.04);
	blue.diffuseColor = new BABYLON.Color3(0.04, 0.08, 0.58);
	red.specularPower = 0;
	blue.specularPower = 0;

	var firstPlayerMesh = scene.meshes.length;
	var updaters = [
		animated_player_from_array(currentPose[0], red, scene),
		animated_player_from_array(currentPose[1], blue, scene)
	];
	scene.meshes.slice(firstPlayerMesh).forEach(function (mesh) {
		if (!mesh.parent) mesh.parent = stageRoot;
	});
	updatePlayers = function (p) { updaters[0](p[0]); updaters[1](p[1]); };

	var grey = new BABYLON.Color3(0.68, 0.7, 0.72);
	for (var i = -6; i <= 6; ++i) {
		var gridX = BABYLON.MeshBuilder.CreateLines("grid-x", { points: [v3(i / 2, 0, -3), v3(i / 2, 0, 3)] }, scene);
		var gridZ = BABYLON.MeshBuilder.CreateLines("grid-z", { points: [v3(-3, 0, i / 2), v3(3, 0, i / 2)] }, scene);
		gridX.color = grey;
		gridZ.color = grey;
		gridX.parent = stageRoot;
		gridZ.parent = stageRoot;
	}

	createTrackers();
	installTrackerControls();

	var lastTime = performance.now();
	engine3d.runRenderLoop(function () {
		var now = performance.now();
		var dt = Math.min((now - lastTime) / 1000, 0.05);
		lastTime = now;
		updateVrInput();
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
			mesh.parent = stageRoot;
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
			var up;
			if (t.def.label === "head") {
				// Anchor local +X to the grappler's anatomical right. HMD local
				// +X then stays left/right-correct even when the torso is lying
				// sideways or upside down relative to the room.
				var shoulderRight = currentPose[t.player][RightShoulder]
					.subtract(currentPose[t.player][LeftShoulder]);
				shoulderRight = shoulderRight.subtract(dir.scale(BABYLON.Vector3.Dot(shoulderRight, dir)));
				if (shoulderRight.lengthSquared() > 1e-6) {
					shoulderRight.normalize();
					up = BABYLON.Vector3.Cross(dir, shoulderRight).normalize();
				}
			}
			if (!up) up = Math.abs(dir.y) > 0.95 ? v3(1, 0, 0) : v3(0, 1, 0);
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

function releasePlayerTrackers(player) {
	trackers.forEach(function (t) {
		if (t.player === player) setTrackerEngaged(t, false);
	});
	if (selectedTracker && selectedTracker.player === player) {
		gizmoManager.attachToMesh(null);
		selectedTracker = null;
	}
	syncDisengagedTrackers();
}

function trackerFor(player, label) {
	return trackers.find(function (t) {
		return t.player === player && t.def.label === label;
	});
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

// ---------------------------------------------------------------- WebXR

function copyPose(pose) {
	return {
		position: pose.position.clone(),
		rotation: pose.rotation.clone()
	};
}

function nodeWorldPose(node) {
	if (!node) return null;
	node.computeWorldMatrix(true);
	var scale = BABYLON.Vector3.One();
	var rotation = BABYLON.Quaternion.Identity();
	var position = BABYLON.Vector3.Zero();
	if (!node.getWorldMatrix().decompose(scale, rotation, position)) return null;
	rotation.normalize();
	return { position: position, rotation: rotation };
}

function nodePoseInStage(node) {
	var worldPose = nodeWorldPose(node);
	if (!worldPose || !stageRoot) return null;
	stageRoot.computeWorldMatrix(true);
	var stageScale = BABYLON.Vector3.One();
	var stageRotation = BABYLON.Quaternion.Identity();
	var stagePosition = BABYLON.Vector3.Zero();
	stageRoot.getWorldMatrix().decompose(stageScale, stageRotation, stagePosition);
	var inverseStage = stageRoot.getWorldMatrix().clone();
	inverseStage.invert();
	var localPosition = BABYLON.Vector3.TransformCoordinates(worldPose.position, inverseStage);
	var localRotation = BABYLON.Quaternion.Inverse(stageRotation).multiply(worldPose.rotation);
	localRotation.normalize();
	return { position: localPosition, rotation: localRotation };
}

function rotateVector(vector, rotation) {
	var result = BABYLON.Vector3.Zero();
	vector.rotateByQuaternionToRef(rotation, result);
	return result;
}

function remapRelativeRotation(relative, axisAlignment) {
	var alignment = axisAlignment || BABYLON.Quaternion.Identity();
	return alignment.multiply(relative).multiply(BABYLON.Quaternion.Inverse(alignment));
}

function createCalibration(sourcePose, tracker, translationAlignment, rotationAlignment) {
	var translation = translationAlignment || BABYLON.Quaternion.Identity();
	return {
		source: copyPose(sourcePose),
		target: {
			position: tracker.mesh.position.clone(),
			rotation: tracker.mesh.rotationQuaternion.clone()
		},
		translationAlignment: translation.clone(),
		rotationAlignment: (rotationAlignment || translation).clone()
	};
}

// Map motion in the source's calibration frame into the target tracker's
// anatomical frame. Translation and rotation both remain relative, which is
// the key to controlling a lying character from a standing play space.
function mapCalibratedPose(sourcePose, calibration) {
	var inverseSource = BABYLON.Quaternion.Inverse(calibration.source.rotation);
	var sourceDelta = sourcePose.position.subtract(calibration.source.position);
	var sourceLocalDelta = rotateVector(sourceDelta, inverseSource);
	var alignedDelta = rotateVector(sourceLocalDelta, calibration.translationAlignment);
	var targetDelta = rotateVector(alignedDelta, calibration.target.rotation);

	var sourceRelativeRotation = inverseSource.multiply(sourcePose.rotation);
	var targetRelativeRotation = remapRelativeRotation(sourceRelativeRotation, calibration.rotationAlignment);
	var targetRotation = calibration.target.rotation.multiply(targetRelativeRotation);
	targetRotation.normalize();

	return {
		position: calibration.target.position.add(targetDelta),
		rotation: targetRotation
	};
}

function applyVrPose(tracker, sourcePose, calibration) {
	if (!tracker || !sourcePose || !calibration) return;
	var mapped = mapCalibratedPose(sourcePose, calibration);
	tracker.mesh.position.copyFrom(mapped.position);
	tracker.mesh.rotationQuaternion.copyFrom(mapped.rotation);
	setTrackerEngaged(tracker, true);
}

function horizontalForward(rotation) {
	var forward = rotateVector(BABYLON.Axis.Z, rotation);
	forward.y = 0;
	if (forward.lengthSquared() < 1e-6) return v3(0, 0, 1);
	return forward.normalize();
}

function recenterVrMenu(force) {
	if (!vr.active || !vr.menu || vr.menuLocked) return;
	var cameraPose = nodeWorldPose(vr.experience.baseExperience.camera);
	if (!cameraPose) return;
	var forward = horizontalForward(cameraPose.rotation);
	var toMenu = vr.menu.plane.position.subtract(cameraPose.position);
	toMenu.y = 0;
	var outsideFollowCone = toMenu.lengthSquared() < 1e-6 ||
		BABYLON.Vector3.Dot(forward, toMenu.normalize()) < Math.cos(Math.PI / 6);
	if (!force && !outsideFollowCone) return;
	vr.menu.plane.position.copyFrom(cameraPose.position.add(forward.scale(1.15)));
	vr.menu.plane.position.y = cameraPose.position.y - 0.12;
	vr.menu.plane.lookAt(cameraPose.position, 0, 0, 0, BABYLON.Space.WORLD);
}

function placeStageInFront() {
	var xrCamera = vr.experience.baseExperience.camera;
	var cameraPose = nodeWorldPose(xrCamera);
	if (!cameraPose) return false;
	var forward = horizontalForward(cameraPose.rotation);
	var yaw = Math.atan2(forward.x, forward.z);
	var eyeHeight = xrCamera.realWorldHeight;
	if (!Number.isFinite(eyeHeight) || eyeHeight < 0.8) eyeHeight = 1.65;
	stageRoot.position.copyFrom(cameraPose.position.add(forward.scale(vr.stageDistance)));
	stageRoot.position.y = cameraPose.position.y - eyeHeight;
	stageRoot.rotationQuaternion.copyFrom(BABYLON.Quaternion.RotationAxis(BABYLON.Axis.Y, yaw));
	stageRoot.computeWorldMatrix(true);
	vr.stagePlaced = true;
	return true;
}

function setStageDistance(value) {
	var next = Math.max(1, Math.min(3, Math.round(Number(value) * 10) / 10));
	if (!Number.isFinite(next)) return;
	var changed = next !== vr.stageDistance;
	vr.stageDistance = next;
	var input = byId("sceneDistance");
	var output = byId("sceneDistanceOut");
	if (input && Number(input.value) !== next) input.value = next;
	if (output) output.textContent = next.toFixed(1) + " m";
	if (vr.menu) {
		if (vr.menu.distance && Math.abs(vr.menu.distance.value - next) > 0.001) {
			vr.menu.distance.value = next;
		}
		if (vr.menu.distanceLabel) vr.menu.distanceLabel.text = "Scene distance: " + next.toFixed(1) + " m";
	}
	if (!changed || !vr.active || !vr.experience) return;
	if (placeStageInFront() && !vr.trackingPaused) calibrateVrTracking(false);
}

function controllerNode(source) {
	return source ? (source.grip || source.pointer || null) : null;
}

function triggerPressed(source) {
	if (!source) return false;
	var motionController = source.motionController;
	if (motionController) {
		var component = motionController.getComponent("xr-standard-trigger");
		if (component) return !!component.pressed || component.value >= 0.55;
	}
	var gamepad = source.inputSource && source.inputSource.gamepad;
	return !!(gamepad && gamepad.buttons && gamepad.buttons[0] &&
		(gamepad.buttons[0].pressed || gamepad.buttons[0].value >= 0.55));
}

function controllerTargetSide(side) {
	// The GrappleMap rig is presented face-to-face in third person, so WebXR
	// handedness must be mirrored to reach the limb on the user's same side.
	return side === "left" ? "right" : "left";
}

function trackerForControllerMode(side, mode) {
	return trackerFor(vr.player, controllerTargetSide(side) + (mode === "foot" ? " foot" : " hand"));
}

function calibrateController(side, mode) {
	var state = vr.controllers[side];
	var sourcePose = nodePoseInStage(controllerNode(state.source));
	if (!sourcePose) {
		state.calibration = null;
		return;
	}
	var previousTracker = trackerForControllerMode(side, state.mode);
	var targetTracker = trackerForControllerMode(side, mode);
	if (previousTracker !== targetTracker) setTrackerEngaged(previousTracker, false);
	syncDisengagedTrackers();
	state.mode = mode;
	state.calibration = createCalibration(sourcePose, targetTracker, BABYLON.Quaternion.Identity());
	setTrackerEngaged(targetTracker, true);
}

function updateController(side) {
	var state = vr.controllers[side];
	if (!state.source) return;
	var desiredMode = triggerPressed(state.source) ? "foot" : "hand";
	if (!state.calibration || desiredMode !== state.mode) calibrateController(side, desiredMode);
	var sourcePose = nodePoseInStage(controllerNode(state.source));
	applyVrPose(trackerForControllerMode(side, state.mode), sourcePose, state.calibration);
}

function calibrateVrTracking(moveStage) {
	if (!vr.active || !vr.experience) return false;
	if (moveStage && !placeStageInFront()) return false;
	releasePlayerTrackers(vr.player);
	var headTracker = trackerFor(vr.player, "head");
	var headPose = nodePoseInStage(vr.experience.baseExperience.camera);
	if (!headPose) return false;
	vr.headCalibration = createCalibration(
		headPose,
		headTracker,
		headTranslationAlignment,
		headRotationAlignment
	);
	setTrackerEngaged(headTracker, true);
	["left", "right"].forEach(function (side) {
		var state = vr.controllers[side];
		state.calibration = null;
		state.mode = triggerPressed(state.source) ? "foot" : "hand";
		if (state.source) calibrateController(side, state.mode);
	});
	return true;
}

function startVrTracking() {
	if (!vr.active) return;
	vr.trackingPaused = false;
	if (!calibrateVrTracking(!vr.stagePlaced)) {
		vr.trackingPaused = true;
		setVrStatus("Move the headset, then try Start again", "unavailable");
		return;
	}
	vr.started = true;
	vr.menuLocked = true;
	byId("positionSelect").disabled = true;
	refreshVrStatus();
}

function clearVrCalibrations() {
	vr.headCalibration = null;
	["left", "right"].forEach(function (side) {
		vr.controllers[side].calibration = null;
		vr.controllers[side].mode = "hand";
	});
}

function releaseVrTracking() {
	vr.started = false;
	vr.trackingPaused = true;
	vr.menuLocked = false;
	clearVrCalibrations();
	byId("positionSelect").disabled = false;
	releaseAllTrackers();
	refreshVrStatus();
}

function stopVrTracking() {
	vr.started = false;
	vr.trackingPaused = false;
	vr.stagePlaced = false;
	vr.menuLocked = false;
	clearVrCalibrations();
	byId("positionSelect").disabled = false;
	releasePlayerTrackers(vr.player);
}

function restoreDesktopStage() {
	stageRoot.position.copyFromFloats(0, 0, 0);
	stageRoot.rotationQuaternion.copyFrom(BABYLON.Quaternion.Identity());
	stageRoot.computeWorldMatrix(true);
}

function updateVrInput() {
	if (!vr.active || !vr.experience) return;
	if (!vr.started) recenterVrMenu(false);
	if (vr.trackingPaused) return;
	if (!vr.headCalibration && !calibrateVrTracking(!vr.stagePlaced)) return;
	var headPose = nodePoseInStage(vr.experience.baseExperience.camera);
	applyVrPose(trackerFor(vr.player, "head"), headPose, vr.headCalibration);
	updateController("left");
	updateController("right");
	refreshVrStatus();
}

function setVrStatus(text, className) {
	var status = byId("vrStatus");
	status.textContent = text;
	status.className = className || "";
	if (vr.menu && vr.menu.status) vr.menu.status.text = text;
}

function refreshVrPositionLabel() {
	if (!vr.menu || !vr.menu.position) return;
	var select = byId("positionSelect");
	var option = select.options[select.selectedIndex];
	vr.menu.position.text = option ? option.textContent : "No position";
}

function setControlledPlayer(value) {
	var next = Number(value) === 1 ? 1 : 0;
	var previous = vr.player;
	vr.player = next;
	var select = byId("vrPlayerSelect");
	if (select) select.value = String(next);
	if (next !== previous) {
		clearVrCalibrations();
		releasePlayerTrackers(previous);
		if (vr.active && !vr.trackingPaused) calibrateVrTracking(false);
	}
	refreshVrStatus();
}

function setTrackerStiffness(value) {
	var next = Math.max(0.1, Math.min(1, Math.round(Number(value) * 20) / 20));
	if (!Number.isFinite(next)) return;
	var input = byId("stiffness");
	var output = byId("stiffnessOut");
	if (input && Number(input.value) !== next) input.value = next;
	if (output) output.textContent = next.toFixed(2).replace(/0$/, "");
	if (vr.menu) {
		if (vr.menu.stiffness && Math.abs(vr.menu.stiffness.value - next) > 0.001) {
			vr.menu.stiffness.value = next;
		}
		if (vr.menu.stiffnessLabel) vr.menu.stiffnessLabel.text = "Tracker stiffness: " + next.toFixed(2).replace(/0$/, "");
	}
}

function refreshVrStatus() {
	var text;
	var className;
	var playerName = vr.player === 0 ? "Red" : "Blue";
	if (!vr.supported) {
		text = "unavailable";
		className = "unavailable";
	} else if (!vr.active) {
		text = "ready - enter VR";
		className = "ready";
	} else if (vr.trackingPaused) {
		text = "trackers released - Start to resume";
		className = "ready";
	} else if (!vr.started) {
		text = "live preview - controlling " + playerName;
		className = "active";
	} else {
		var left = vr.controllers.left.source ? vr.controllers.left.mode : "waiting";
		var right = vr.controllers.right.source ? vr.controllers.right.mode : "waiting";
		text = "active - " + playerName + ", head, L " + left + ", R " + right;
		className = "active";
	}
	if (text !== vr.lastStatus) {
		vr.lastStatus = text;
		setVrStatus(text, className);
	}
	if (vr.menu) {
		var positionEnabled = vr.active && !vr.started;
		vr.menu.start.isEnabled = vr.active && !vr.started;
		vr.menu.previous.isEnabled = vr.active && !vr.started;
		vr.menu.next.isEnabled = vr.active && !vr.started;
		vr.menu.start.alpha = vr.started ? 0.45 : 1;
		vr.menu.start.textBlock.text = vr.started ? "Controls active" : (vr.trackingPaused ? "Resume controls" : "Start");
		vr.menu.previous.alpha = positionEnabled ? 1 : 0.45;
		vr.menu.next.alpha = positionEnabled ? 1 : 0.45;
		vr.menu.reset.isEnabled = vr.active;
		vr.menu.release.isEnabled = vr.active && !vr.trackingPaused;
		vr.menu.reset.alpha = vr.active ? 1 : 0.45;
		vr.menu.release.alpha = vr.active && !vr.trackingPaused ? 1 : 0.45;
		vr.menu.playerRed.background = vr.player === 0 ? "#a4261b" : "#35475a";
		vr.menu.playerBlue.background = vr.player === 1 ? "#234eac" : "#35475a";
	}
	var startButton = byId("vrStartBtn");
	startButton.disabled = !vr.active || vr.started;
	startButton.textContent = vr.started ? "VR controls active"
		: vr.trackingPaused ? "Resume VR controls" : "Start VR controls";
	byId("positionSelect").disabled = vr.started;
	refreshVrPositionLabel();
}

function cyclePosition(direction) {
	if (vr.started || namedEntries.length === 0) return;
	var select = byId("positionSelect");
	var currentIndex = namedEntries.findIndex(function (entry) {
		return String(entry.index) === select.value;
	});
	var nextIndex = (currentIndex + direction + namedEntries.length) % namedEntries.length;
	select.value = namedEntries[nextIndex].index;
	loadEntry(namedEntries[nextIndex].index);
	refreshVrPositionLabel();
}

function createVrMenu() {
	if (!BABYLON.GUI) return;
	var plane = BABYLON.MeshBuilder.CreatePlane("vr-setup-menu", {
		width: 1.05,
		height: 0.98,
		sideOrientation: BABYLON.Mesh.DOUBLESIDE
	}, scene);
	plane.isPickable = true;
	plane.setEnabled(false);
	var texture = BABYLON.GUI.AdvancedDynamicTexture.CreateForMesh(plane, 1100, 1030, false);
	var background = new BABYLON.GUI.Rectangle("vr-menu-background");
	background.background = "#101720";
	background.color = "#91a5bb";
	background.thickness = 4;
	background.cornerRadius = 28;
	texture.addControl(background);
	var panel = new BABYLON.GUI.StackPanel("vr-menu-panel");
	panel.width = "970px";
	panel.paddingTop = "28px";
	panel.paddingBottom = "22px";
	background.addControl(panel);

	function addText(name, text, height, size, color) {
		var control = new BABYLON.GUI.TextBlock(name, text);
		control.height = height + "px";
		control.fontSize = size;
		control.color = color || "white";
		control.textWrapping = true;
		panel.addControl(control);
		return control;
	}
	function makeButton(name, text) {
		var button = BABYLON.GUI.Button.CreateSimpleButton(name, text);
		button.height = "70px";
		button.color = "white";
		button.background = "#245f9e";
		button.thickness = 2;
		button.cornerRadius = 15;
		button.fontSize = 25;
		return button;
	}
	function makeTwoColumnRow(name, left, right, height) {
		var row = new BABYLON.GUI.Grid(name);
		row.height = (height || 78) + "px";
		row.addColumnDefinition(0.5);
		row.addColumnDefinition(0.5);
		left.paddingRight = "7px";
		right.paddingLeft = "7px";
		row.addControl(left, 0, 0);
		row.addControl(right, 0, 1);
		panel.addControl(row);
		return row;
	}
	function makeSlider(name, minimum, maximum, value, step) {
		var slider = new BABYLON.GUI.Slider(name);
		slider.minimum = minimum;
		slider.maximum = maximum;
		slider.value = value;
		slider.step = step;
		slider.height = "54px";
		slider.width = "860px";
		slider.color = "#65b5ff";
		slider.background = "#34495e";
		slider.borderColor = "#b9cce0";
		slider.thumbWidth = "42px";
		panel.addControl(slider);
		return slider;
	}

	addText("vr-menu-title", "GrappleMap VR", 60, 40, "#ffffff");
	var status = addText("vr-menu-status", "Live preview", 56, 23, "#b9cce0");
	var position = addText("vr-menu-position", "Position", 58, 28, "#ffffff");
	var previous = makeButton("vr-menu-previous", "Previous");
	var next = makeButton("vr-menu-next", "Next");
	makeTwoColumnRow("vr-menu-position-buttons", previous, next);
	var playerLabel = addText("vr-menu-player-label", "Controlled grappler", 48, 25, "#b9cce0");
	var playerRed = makeButton("vr-menu-player-red", "Red");
	var playerBlue = makeButton("vr-menu-player-blue", "Blue");
	makeTwoColumnRow("vr-menu-player-buttons", playerRed, playerBlue);
	var distanceLabel = addText("vr-menu-distance-label", "Scene distance: " + vr.stageDistance.toFixed(1) + " m", 46, 24, "#b9cce0");
	var distance = makeSlider("vr-menu-distance", 1, 3, vr.stageDistance, 0.1);
	var stiffnessValue = parseFloat(byId("stiffness").value);
	var stiffnessLabel = addText("vr-menu-stiffness-label", "Tracker stiffness: " + stiffnessValue, 46, 24, "#b9cce0");
	var stiffness = makeSlider("vr-menu-stiffness", 0.1, 1, stiffnessValue, 0.05);
	var reset = makeButton("vr-menu-reset", "Reset pose");
	var release = makeButton("vr-menu-release", "Release trackers");
	makeTwoColumnRow("vr-menu-action-buttons", reset, release);
	var start = makeButton("vr-menu-start", "Start");
	start.height = "82px";
	start.background = "#18864b";
	start.paddingTop = "8px";
	panel.addControl(start);
	previous.onPointerUpObservable.add(function () { cyclePosition(-1); });
	next.onPointerUpObservable.add(function () { cyclePosition(1); });
	playerRed.onPointerUpObservable.add(function () { setControlledPlayer(0); });
	playerBlue.onPointerUpObservable.add(function () { setControlledPlayer(1); });
	distance.onValueChangedObservable.add(setStageDistance);
	stiffness.onValueChangedObservable.add(setTrackerStiffness);
	reset.onPointerUpObservable.add(resetSelectedPose);
	release.onPointerUpObservable.add(releaseVrTracking);
	start.onPointerUpObservable.add(startVrTracking);
	vr.menu = {
		plane: plane,
		texture: texture,
		status: status,
		position: position,
		previous: previous,
		next: next,
		playerLabel: playerLabel,
		playerRed: playerRed,
		playerBlue: playerBlue,
		distanceLabel: distanceLabel,
		distance: distance,
		stiffnessLabel: stiffnessLabel,
		stiffness: stiffness,
		reset: reset,
		release: release,
		start: start
	};
	refreshVrStatus();
}

function registerVrController(source) {
	var side = source.inputSource && source.inputSource.handedness;
	if (side !== "left" && side !== "right") return;
	vr.controllers[side].source = source;
	vr.controllers[side].calibration = null;
	if (vr.active && !vr.trackingPaused) calibrateController(side, triggerPressed(source) ? "foot" : "hand");
	refreshVrStatus();
}

function unregisterVrController(source) {
	["left", "right"].forEach(function (side) {
		var state = vr.controllers[side];
		if (state.source !== source) return;
		setTrackerEngaged(trackerForControllerMode(side, "hand"), false);
		setTrackerEngaged(trackerForControllerMode(side, "foot"), false);
		state.source = null;
		state.calibration = null;
		state.mode = "hand";
	});
	refreshVrStatus();
}

async function initXR() {
	setVrStatus("checking...", "");
	if (!navigator.xr || !navigator.xr.isSessionSupported) {
		setVrStatus("unavailable in this browser", "unavailable");
		return;
	}
	try {
		vr.supported = await navigator.xr.isSessionSupported("immersive-vr");
		if (!vr.supported) {
			setVrStatus("no headset found", "unavailable");
			return;
		}
		vr.experience = await scene.createDefaultXRExperienceAsync({
			disableTeleportation: true,
			uiOptions: { sessionMode: "immersive-vr", referenceSpaceType: "local-floor" }
		});
		createVrMenu();
		vr.experience.input.onControllerAddedObservable.add(registerVrController);
		vr.experience.input.onControllerRemovedObservable.add(unregisterVrController);
		vr.experience.baseExperience.onStateChangedObservable.add(function (state) {
			if (state === BABYLON.WebXRState.IN_XR) {
				vr.active = true;
				vr.started = false;
				vr.trackingPaused = false;
				vr.stagePlaced = false;
				vr.menuLocked = false;
				clearVrCalibrations();
				if (vr.menu) vr.menu.plane.setEnabled(true);
				recenterVrMenu(true);
				// Entering VR immediately starts a live pose preview. Start only
				// locks the chosen position and menu placement.
				calibrateVrTracking(true);
			} else if (state === BABYLON.WebXRState.NOT_IN_XR) {
				stopVrTracking();
				vr.active = false;
				if (vr.menu) vr.menu.plane.setEnabled(false);
				restoreDesktopStage();
			}
			refreshVrStatus();
		});
		refreshVrStatus();
	} catch (error) {
		console.error("WebXR setup failed", error);
		vr.supported = false;
		setVrStatus("VR setup failed", "unavailable");
	}
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
		// Tracker transforms are local to the movable VR stage. Use the local
		// rotation here so a stage yaw never leaks into solver coordinates.
		var fwd = rotateVector(BABYLON.Axis.Z, t.mesh.rotationQuaternion);
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
	namedEntries = entries.filter(function (e) { return e.frames === 1; });
	namedEntries.forEach(function (e) {
		var option = document.createElement("option");
		option.value = e.index;
		option.textContent = e.name;
		select.appendChild(option);
	});
	select.addEventListener("change", function () {
		loadEntry(parseInt(select.value, 10));
		refreshVrPositionLabel();
	});
}

function loadEntry(index) {
	wasmEngine.loadEntry(index);
	currentPose = flatToPose(wasmEngine.pose());
	updatePlayers(currentPose);
	releaseAllTrackers();
	if (vr.active && !vr.trackingPaused) calibrateVrTracking(false);
	refreshVrPositionLabel();
}

function resetSelectedPose() {
	loadEntry(parseInt(byId("positionSelect").value, 10));
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

	byId("resetBtn").addEventListener("click", resetSelectedPose);
	byId("releaseBtn").addEventListener("click", function () {
		if (vr.active) releaseVrTracking();
		else releaseAllTrackers();
	});
	byId("dualTestBtn").addEventListener("click", function () {
		if (dualTest) stopDualTest();
		else startDualTest();
	});
	byId("vrStartBtn").addEventListener("click", startVrTracking);
	byId("vrPlayerSelect").addEventListener("change", function (event) {
		setControlledPlayer(event.target.value);
	});
	byId("sceneDistance").addEventListener("input", function (event) {
		setStageDistance(event.target.value);
	});
	var stiffness = byId("stiffness");
	stiffness.addEventListener("input", function (event) { setTrackerStiffness(event.target.value); });
	setControlledPlayer(byId("vrPlayerSelect").value);
	setStageDistance(byId("sceneDistance").value);
	setTrackerStiffness(stiffness.value);
	await initXR();
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
	vrState: function () {
		return {
			supported: vr.supported,
			active: vr.active,
			started: vr.started,
			trackingPaused: vr.trackingPaused,
			menuLocked: vr.menuLocked,
			player: vr.player,
			stageDistance: vr.stageDistance,
			leftTargetSide: controllerTargetSide("left"),
			rightTargetSide: controllerTargetSide("right"),
			leftMode: vr.controllers.left.mode,
			rightMode: vr.controllers.right.mode
		};
	},
	// Pure mapping probe used by automated desktop tests: an HMD roll must
	// change the neck-to-head direction rather than spinning invisibly.
	headTiltProbe: function (degrees) {
		var tracker = trackerFor(vr.player, "head");
		var before = rotateVector(BABYLON.Axis.Z, tracker.mesh.rotationQuaternion);
		var relative = BABYLON.Quaternion.RotationAxis(BABYLON.Axis.Z, degrees * Math.PI / 180);
		var mapped = remapRelativeRotation(relative, headRotationAlignment);
		var afterRotation = tracker.mesh.rotationQuaternion.multiply(mapped);
		var after = rotateVector(BABYLON.Axis.Z, afterRotation);
		return {
			before: { x: before.x, y: before.y, z: before.z },
			after: { x: after.x, y: after.y, z: after.z },
			change: BABYLON.Vector3.Distance(before, after)
		};
	},
	// Drive n solver steps synchronously (headless testing without rAF).
	tick: function (dt, n) {
		for (var i = 0; i < (n || 1); ++i) stepSolver(dt);
	}
};

boot();
