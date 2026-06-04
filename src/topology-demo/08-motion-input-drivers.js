// Input-driven motion: auto-solve sliders, drag state, drag-step integration, and contact relaxation.
// Keep this file focused: future agents should change this module for this system only.

function queueAutoSolve(resetSteps) {
	setOutputs();
	autoSolving = true;
	autoStepBudget = resetSteps ? parseInt(el.frames.value, 10) : Math.max(autoStepBudget, 8);
	if (currentPosition) updateProof();
}

function autoStep() {
	if (!currentPosition || !autoSolving) return;
	var chainA = selectedChain(el.chainA);
	var chainB = selectedChain(el.chainB);
	var current = writheMatrix(currentPosition, chainA, chainB);
	var target = readTarget();
	var finalDesired = desiredWritheMatrix(current.length, current[0].length, target);
	var desired = steppedMatrix(current, finalDesired, maxMatrixStep);
	var solved = solveToward(currentPosition, chainA, chainB, desired, {
		affectA: el.affectA.checked,
		affectB: el.affectB.checked,
		iterations: 1,
		maxDelta: maxJointStep
	});
	currentPosition = solved.position;
	contactProof = solved.proof || emptyContactProof();
	updatePlayers(currentPosition);
	updateHandles(currentPosition);
	renderContactProof(contactProof);
	updateProof();

	--autoStepBudget;
	var finalLoss = matrixLoss(writheMatrix(currentPosition, chainA, chainB), finalDesired);
	if (Math.sqrt(finalLoss) < 0.015 || autoStepBudget <= 0) autoSolving = false;
}

function dragMoveList(chains, pin) {
	var seen = {};
	var out = [];
	function add(player, joint) {
		var key = player + ":" + joint;
		if (pin && key === pin.player + ":" + pin.joint) return;
		if (seen[key]) return;
		seen[key] = true;
		out.push({ player: player, joint: joint });
	}
	chains.forEach(function (chain) {
		chain.joints.forEach(function (joint) { add(chain.player, joint); });
		var side = footSideForChain(chain);
		if (side) {
			add(chain.player, side.toe);
			add(chain.player, side.heel);
		}
	});
	return out;
}

function relaxContacts(position, chains, lengths, pinned, passes) {
	var proof = emptyContactProof();
	var move = dragMoveList(chains, null);
	var chain = chains[0];
	var targetJoint = drag && drag.chain === chain ? drag.joint : chain.joints[chain.joints.length - 1];
	for (var pass = 0; pass < passes; ++pass) {
		var before = clonePosition(position);
		var passProof = projectContacts(before, position, chains, move, drag && drag.chain === chain ? drag.contactClearanceTarget : undefined);
		mergeProofs(proof, passProof);
		if (drag && drag.chain === chain) projectSelectedLimbLengths(position, drag);
		else projectSelectedChainLengths(position, chain, lengths, targetJoint);
		if (!passProof.count) break;
	}
	if (!Number.isFinite(proof.minClearance)) proof.minClearance = 0;
	return proof;
}

function relaxAllContacts(position, lengths, pinned, passes) {
	var proof = emptyContactProof();
	var move = allMovableJoints(pinned || []);
	for (var pass = 0; pass < passes; ++pass) {
		var before = clonePosition(position);
		var passProof = projectAllContacts(before, position, move);
		mergeProofs(proof, passProof);
		projectBodyLengths(position, lengths, pinned || []);
		if (!passProof.count) break;
	}
	if (!Number.isFinite(proof.minClearance)) proof.minClearance = 0;
	return proof;
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
	var chain = chainContaining(player, joint);
	if (chain) {
		el.chainA.value = chain.id;
		if (chainById(el.chainB.value).player === chain.player) {
			var other = chainDefs.find(function (candidate) { return candidate.player !== chain.player; });
			if (other) el.chainB.value = other.id;
		}
	}
	var chainA = selectedChain(el.chainA);
	var chainB = selectedChain(el.chainB);
	var dragChain = dragChainForJoint(chain || chainA, player, joint);
	var topologyOther = chainB.player === dragChain.player ? chainDefs.find(function (candidate) { return candidate.player !== dragChain.player; }) || chainB : chainB;
	var startClearance = measureMinClearance(currentPosition, [dragChain]);
	var contactClearanceTarget = Math.min(minAcceptedClearance, startClearance);
	var origin = currentPosition[player][joint].clone();
	var normal = camera.position.subtract(origin).normalize();
	drag = {
		player: player,
		joint: joint,
		chain: dragChain,
		topologyOther: topologyOther,
		target: origin.clone(),
		offset: v3(0, 0, 0),
		plane: BABYLON.Plane.FromPositionAndNormal(origin, normal),
		topologyMatrix: writheMatrix(currentPosition, dragChain, topologyOther),
		referencePosition: clonePosition(currentPosition),
		chainLengths: chainLengths(currentPosition, dragChain),
		bodyLengths: dragChain.projectBody ? segmentLengthsFrom(currentPosition) : null,
		footLengths: footLengthsFrom(currentPosition, player, footSideForChain(dragChain)),
		clearanceFloor: Math.min(-0.001, startClearance - 0.002),
		contactClearanceTarget: contactClearanceTarget,
		previewTick: 0
	};
	syncDragTargetFromPointer();
	drag.offset = origin.subtract(drag.target);
	syncDragTargetFromPointer();
	autoSolving = false;
	setActiveHandle(drag);
	byId("renderCanvas").classList.add("dragging");
	camera.detachControl(byId("renderCanvas"));
	renderKosherMovementArea();
	updateProof();
}

function endDrag() {
	drag = null;
	disposeKosherVisualization();
	setActiveHandle(null);
	byId("renderCanvas").classList.remove("dragging");
	if (camera) camera.attachControl(byId("renderCanvas"), true);
	updateProof();
}

function commitDragStep(position, proof, grabbed, reached, clearance, lengthError) {
	proof = proof || emptyContactProof();
	proof.projected.push({ from: grabbed, to: reached });
	proof.minClearance = Math.min(proof.minClearance, clearance);
	proof.lengthError = lengthError;
	currentPosition = position;
	contactProof = proof;
	updatePlayers(currentPosition);
	updateHandles(currentPosition);
	renderContactProof(contactProof);
	drag.previewTick = (drag.previewTick || 0) + 1;
	if (drag.previewTick % 4 === 0) renderKosherMovementArea();
	updateProof();
}

function dragClearanceValid(clearance) {
	return clearance >= (drag && Number.isFinite(drag.clearanceFloor) ? drag.clearanceFloor : -0.001);
}

function dragStep() {
	if (!drag || !currentPosition) return;
	var chainA = drag.chain;
	var chainB = drag.topologyOther;
	var previous = clonePosition(currentPosition);
	var p = clonePosition(currentPosition);
	var grabbed = p[drag.player][drag.joint];
	var delta = drag.target.subtract(grabbed);
	var deltaLen = delta.length();
	if (deltaLen > maxDragStep) delta = delta.scale(maxDragStep / deltaLen);
	var draggedPosition = grabbed.add(delta);
	draggedPosition.y = Math.max(0.02, draggedPosition.y);

	projectAnchoredChainToTarget(p, chainA, drag.chainLengths, drag.joint, draggedPosition);
	projectFootTriangle(p, chainA, drag.footLengths, drag.joint, drag.referencePosition);
	var limbMove = dragMoveList([chainA], null);
	var directProof = projectContacts(previous, p, [chainA], limbMove, drag.contactClearanceTarget);
	projectSelectedLimbLengths(p, drag);
	if (drag.bodyLengths) projectBodyLengths(p, drag.bodyLengths, [chainRootPin(chainA)]);
	var directCandidate = clonePosition(p);
	var directClearance = measureMinClearance(directCandidate, [chainA]);
	var directLengthError = maxChainLengthError(directCandidate, chainA, drag.chainLengths, drag.footLengths);
	var directValid = dragClearanceValid(directClearance) && directLengthError <= 0.002;
	if (isLegChain(chainA) && directValid) {
		directProof.rejected.push({ from: grabbed, to: draggedPosition });
		commitDragStep(directCandidate, directProof, grabbed, directCandidate[drag.player][drag.joint], directClearance, directLengthError);
		return;
	}
	var current = writheMatrix(p, chainA, chainB);
	var desired = steppedMatrix(current, drag.topologyMatrix, maxMatrixStep);
	var solved = solveToward(p, chainA, chainB, desired, {
		affectA: true,
		affectB: false,
		pinned: [chainRootPin(chainA)],
		iterations: 1,
		maxDelta: 0.012,
		clearanceFloor: drag.clearanceFloor,
		minClearanceTarget: drag.contactClearanceTarget
	});

	p = solved.position;
	projectSelectedLimbLengths(p, drag);
	if (drag.bodyLengths) projectBodyLengths(p, drag.bodyLengths, [chainRootPin(chainA)]);
	var projectionProof = relaxContacts(p, [chainA], drag.chainLengths, [chainRootPin(chainA)], 5);
	projectSelectedLimbLengths(p, drag);
	if (drag.bodyLengths) projectBodyLengths(p, drag.bodyLengths, [chainRootPin(chainA)]);
	mergeProofs(solved.proof, directProof);
	mergeProofs(solved.proof, projectionProof);

	var nextClearance = measureMinClearance(p, [chainA]);
	var lengthError = maxChainLengthError(p, chainA, drag.chainLengths, drag.footLengths);
	var solvedTargetError = dist(p[drag.player][drag.joint], draggedPosition);
	var directTargetError = dist(directCandidate[drag.player][drag.joint], draggedPosition);
	var solvedValid = dragClearanceValid(nextClearance) && lengthError <= 0.002 && solvedTargetError <= directTargetError + maxDragStep * 0.35;
	solved.proof.rejected.push({ from: grabbed, to: draggedPosition });
	if (!solvedValid && directValid) {
		p = directCandidate;
		nextClearance = directClearance;
		lengthError = directLengthError;
	} else if (!solvedValid) {
		p = previous;
		nextClearance = measureMinClearance(p, [chainA]);
		lengthError = maxChainLengthError(p, chainA, drag.chainLengths, drag.footLengths);
	}
	commitDragStep(p, solved.proof, grabbed, p[drag.player][drag.joint], nextClearance, lengthError);
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

