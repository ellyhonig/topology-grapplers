// IK-only input-driven motion: direct joint dragging with body-length projection.
// This page intentionally skips topology solving and contact projection, so bodies may overlap.

function autoStep() {
	// The shared renderer calls autoStep each frame when no drag is active.
}

function chainContaining(player, joint) {
	var found = chainDefs.find(function (chain) {
		return chain.player === player && chain.joints.indexOf(joint) !== -1;
	});
	if (found) return found;
	var side = legSideForJoint(joint);
	if (!side || joint !== side.heel) return null;
	return chainDefs.find(function (chain) {
		return chain.player === player && chain.joints[0] === side.hip;
	});
}

function allPlayerJoints() {
	var out = [];
	for (var joint = 0; joint < jointNames.length; ++joint) out.push(joint);
	return out;
}

function upperBodyJoints() {
	return [
		Core, Neck, Head,
		LeftShoulder, LeftElbow, LeftWrist, LeftHand, LeftFingers,
		RightShoulder, RightElbow, RightWrist, RightHand, RightFingers
	];
}

function armRootJoints(joint) {
	if (joint === LeftShoulder) return [LeftShoulder, LeftElbow, LeftWrist, LeftHand, LeftFingers];
	if (joint === RightShoulder) return [RightShoulder, RightElbow, RightWrist, RightHand, RightFingers];
	return null;
}

function legStiffDragJoints(joint) {
	if (joint === LeftHip) return [LeftHip, LeftKnee, LeftAnkle, LeftToe, LeftHeel];
	if (joint === RightHip) return [RightHip, RightKnee, RightAnkle, RightToe, RightHeel];
	if (joint === LeftKnee) return [LeftKnee, LeftAnkle, LeftToe, LeftHeel];
	if (joint === RightKnee) return [RightKnee, RightAnkle, RightToe, RightHeel];
	if (joint === LeftAnkle) return [LeftAnkle, LeftToe, LeftHeel];
	if (joint === RightAnkle) return [RightAnkle, RightToe, RightHeel];
	return null;
}

function ikGroupDragForJoint(joint) {
	if (joint === Core) {
		return { mode: "whole-body", label: "whole body", joints: allPlayerJoints(), preserveExactly: true };
	}
	if (joint === Neck || joint === LeftShoulder || joint === RightShoulder) {
		return { mode: "upper-body", label: "upper body", joints: upperBodyJoints(), preserveExactly: false };
	}
	var armJoints = armRootJoints(joint);
	if (armJoints) {
		return { mode: "limb-root", label: "stiff arm root", joints: armJoints, pinnedJoints: armJoints, preserveExactly: false };
	}
	var legJoints = legStiffDragJoints(joint);
	if (legJoints) {
		return { mode: "limb-root", label: "stiff leg section", joints: legJoints, pinnedJoints: legJoints, preserveExactly: false };
	}
	return null;
}

function translateJoints(position, player, joints, delta) {
	joints.forEach(function (joint) {
		position[player][joint] = position[player][joint].add(delta);
	});
}

function syncDragTargetFromPointer() {
	if (!drag || !scene || !camera) return;
	var ray = scene.createPickingRay(scene.pointerX, scene.pointerY, BABYLON.Matrix.Identity(), camera);
	var distanceOnPlane = ray.intersectsPlane(drag.plane);
	if (distanceOnPlane === null || !Number.isFinite(distanceOnPlane)) return;
	drag.target = ray.origin.add(ray.direction.scale(distanceOnPlane)).add(drag.offset);
	drag.target.y = Math.max(0.02, drag.target.y);
}

function startDrag(mesh) {
	if (!currentPosition || !mesh || !mesh.metadata || !mesh.metadata.dragHandle) return;
	var player = mesh.metadata.player;
	var joint = mesh.metadata.joint;
	var baseChain = chainContaining(player, joint);
	if (!baseChain) return;
	var dragChain = dragChainForJoint(baseChain, player, joint);
	var groupDrag = ikGroupDragForJoint(joint);
	var origin = currentPosition[player][joint].clone();
	var normal = camera.position.subtract(origin).normalize();
	drag = {
		player: player,
		joint: joint,
		chain: dragChain,
		group: groupDrag,
		target: origin.clone(),
		offset: v3(0, 0, 0),
		plane: BABYLON.Plane.FromPositionAndNormal(origin, normal),
		referencePosition: clonePosition(currentPosition),
		chainLengths: chainLengths(currentPosition, dragChain),
		bodyLengths: (groupDrag || dragChain.projectBody) ? segmentLengthsFrom(currentPosition) : null,
		footLengths: footLengthsFrom(currentPosition, player, footSideForChain(dragChain))
	};
	syncDragTargetFromPointer();
	drag.offset = origin.subtract(drag.target);
	syncDragTargetFromPointer();
	autoSolving = false;
	setActiveHandle(drag);
	byId("renderCanvas").classList.add("dragging");
	camera.detachControl(byId("renderCanvas"));
	updateProof();
}

function endDrag() {
	drag = null;
	setActiveHandle(null);
	byId("renderCanvas").classList.remove("dragging");
	if (camera) camera.attachControl(byId("renderCanvas"), true);
	updateProof();
}

function dragGroupStep(p, draggedPosition) {
	var delta = draggedPosition.subtract(p[drag.player][drag.joint]);
	translateJoints(p, drag.player, drag.group.joints, delta);
	if (!drag.group.preserveExactly) {
		var pins = (drag.group.pinnedJoints || []).map(function (joint) {
			return { player: drag.player, joint: joint, position: null };
		});
		projectBodyLengths(p, drag.bodyLengths, pins, 18);
		projectBendPlanes(p, drag.referencePosition, pins);
		projectShoulderHipRangeLimits(p, drag.referencePosition, drag.player);
		projectNeckRangeLimits(p, drag.referencePosition, drag.player);
		projectBodyLengths(p, drag.bodyLengths, pins, 18);
		projectNeckRangeLimits(p, drag.referencePosition, drag.player);
	}
}

function dragStep() {
	if (!drag || !currentPosition) return;
	var p = clonePosition(currentPosition);
	var grabbed = p[drag.player][drag.joint];
	var delta = drag.target.subtract(grabbed);
	var deltaLen = delta.length();
	if (deltaLen > maxDragStep) delta = delta.scale(maxDragStep / deltaLen);
	var draggedPosition = grabbed.add(delta);
	draggedPosition.y = Math.max(0.02, draggedPosition.y);

	if (drag.group) {
		dragGroupStep(p, draggedPosition);
	} else {
		projectAnchoredChainToTarget(p, drag.chain, drag.chainLengths, drag.joint, draggedPosition);
		projectFootTriangle(p, drag.chain, drag.footLengths, drag.joint, drag.referencePosition);
		projectSelectedLimbLengths(p, drag);
		if (drag.bodyLengths) projectBodyLengths(p, drag.bodyLengths, [chainRootPin(drag.chain)]);
		projectShoulderHipRangeLimits(p, drag.referencePosition, drag.player);
		projectNeckRangeLimits(p, drag.referencePosition, drag.player);
	}

	currentPosition = p;
	contactProof = emptyContactProof();
	updatePlayers(currentPosition);
	updateHandles(currentPosition);
	updateProof();
}

function installDragControls(canvas) {
	scene.onPointerObservable.add(function (pointerInfo) {
		if (pointerInfo.type === BABYLON.PointerEventTypes.POINTERDOWN) {
			var pick = scene.pick(scene.pointerX, scene.pointerY, function (mesh) {
				return mesh.metadata && mesh.metadata.dragHandle;
			});
			if (pick && pick.hit) {
				pointerInfo.event.preventDefault();
				startDrag(pick.pickedMesh);
			}
		} else if (pointerInfo.type === BABYLON.PointerEventTypes.POINTERMOVE) {
			syncDragTargetFromPointer();
		} else if (pointerInfo.type === BABYLON.PointerEventTypes.POINTERUP) {
			if (drag) endDrag();
		}
	});
	canvas.addEventListener("pointerleave", function () {
		if (drag) endDrag();
	});
}
