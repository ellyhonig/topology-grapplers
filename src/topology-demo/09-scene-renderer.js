// Babylon scene setup, player rendering, joint handles, and active-handle presentation.
// Keep this file focused: future agents should change this module for this system only.

function initScene() {
	var canvas = byId("renderCanvas");
	engine = new BABYLON.Engine(canvas, true);
	scene = new BABYLON.Scene(engine);
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
	updatePlayers = makePlayerUpdater([fallbackPositions()[0].position[0], fallbackPositions()[0].position[1]], [red, blue], scene);
	createJointHandles(scene);
	installDragControls(canvas);
	engine.runRenderLoop(function () {
		if (drag) dragStep();
		else autoStep();
		scene.render();
	});
	window.addEventListener("resize", function () { engine.resize(); });
}

function makePlayerUpdater(position, materials, scene) {
	var updaters = [
		animated_player_from_array(position[0], materials[0], scene),
		animated_player_from_array(position[1], materials[1], scene)
	];
	var grey = new BABYLON.Color3(0.68, 0.7, 0.72);
	for (var i = -6; i <= 6; ++i) {
		BABYLON.MeshBuilder.CreateLines("grid-x", { points: [v3(i / 2, 0, -3), v3(i / 2, 0, 3)] }, scene).color = grey;
		BABYLON.MeshBuilder.CreateLines("grid-z", { points: [v3(-3, 0, i / 2), v3(3, 0, i / 2)] }, scene).color = grey;
	}
	return function (p) {
		updaters[0](p[0]);
		updaters[1](p[1]);
	};
}

function updateHandles(position) {
	if (!position || !handleMeshes.length) return;
	handleMeshes.forEach(function (mesh) {
		mesh.position.copyFrom(position[mesh.metadata.player][mesh.metadata.joint]);
	});
}

function createJointHandles(scene) {
	var handleMat = new BABYLON.StandardMaterial("jointHandle", scene);
	handleMat.diffuseColor = new BABYLON.Color3(1, 0.78, 0.12);
	handleMat.emissiveColor = new BABYLON.Color3(0.18, 0.12, 0.02);
	handleMat.alpha = 0.72;
	var activeMat = new BABYLON.StandardMaterial("activeJointHandle", scene);
	activeMat.diffuseColor = new BABYLON.Color3(1, 1, 1);
	activeMat.emissiveColor = new BABYLON.Color3(0.35, 0.25, 0.04);
	for (var player = 0; player < 2; ++player) {
		for (var joint = 0; joint < jointNames.length; ++joint) {
			var radius = Math.max(joints[joint][0] * 2.2, 0.055);
			var handle = BABYLON.MeshBuilder.CreateSphere("drag-joint", { segments: 12, diameter: radius }, scene);
			handle.material = handleMat;
			handle.isPickable = true;
			handle.metadata = { dragHandle: true, player: player, joint: joint, normalMaterial: handleMat, activeMaterial: activeMat };
			handleMeshes.push(handle);
		}
	}
}

function setActiveHandle(active) {
	handleMeshes.forEach(function (mesh) {
		mesh.material = active && mesh.metadata.player === active.player && mesh.metadata.joint === active.joint ? mesh.metadata.activeMaterial : mesh.metadata.normalMaterial;
	});
}

