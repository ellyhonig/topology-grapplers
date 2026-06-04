// DOM controls, proof text, position loading, and application boot sequence.
// Keep this file focused: future agents should change this module for this system only.

function selectedChain(select) {
	return chainDefs.find(function (chain) { return chain.id === select.value; });
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

function chainById(id) {
	return chainDefs.find(function (chain) { return chain.id === id; });
}

function draggedLabel() {
	if (!drag) return "none";
	return (drag.player === 0 ? "Red " : "Blue ") + jointNames[drag.joint];
}

function readTarget() {
	return {
		writhe: parseFloat(el.writhe.value),
		density: parseFloat(el.density.value),
		centerA: parseFloat(el.centerA.value),
		centerB: parseFloat(el.centerB.value)
	};
}

function setOutputs() {
	el.writheOut.textContent = fmt(parseFloat(el.writhe.value));
	el.densityOut.textContent = fmt(parseFloat(el.density.value));
	el.centerAOut.textContent = fmt(parseFloat(el.centerA.value));
	el.centerBOut.textContent = fmt(parseFloat(el.centerB.value));
	el.framesOut.textContent = el.frames.value;
	if (el.kosherStepsOut) el.kosherStepsOut.textContent = String(kosherPreviewSteps);
}

function syncSlidersToCurrentTopology() {
	if (!currentPosition || !el.chainA || !el.chainB) return;
	var topo = topologyCoordinates(writheMatrix(currentPosition, selectedChain(el.chainA), selectedChain(el.chainB)));
	el.writhe.value = String(clamp(topo.writhe, parseFloat(el.writhe.min), parseFloat(el.writhe.max)));
	el.density.value = String(clamp(topo.density, parseFloat(el.density.min), parseFloat(el.density.max)));
	el.centerA.value = String(clamp(topo.centerA, parseFloat(el.centerA.min), parseFloat(el.centerA.max)));
	el.centerB.value = String(clamp(topo.centerB, parseFloat(el.centerB.min), parseFloat(el.centerB.max)));
	setOutputs();
}

function updateProof() {
	var chainA = drag ? drag.chain : selectedChain(el.chainA);
	var chainB = drag ? drag.topologyOther : selectedChain(el.chainB);
	var current = writheMatrix(currentPosition, chainA, chainB);
	var topo = topologyCoordinates(current);
	var target = readTarget();
	var desired = drag ? drag.topologyMatrix : desiredWritheMatrix(current.length, current[0].length, target);
	var desiredTopo = topologyCoordinates(desired);
	var loss = matrixLoss(current, desired);
	var acceptedClearance = drag ? measureMinClearance(currentPosition, [chainA]) : measureMinClearance(currentPosition, [chainA, chainB]);
	if (acceptedClearance > -0.005) acceptedClearance = 0;
	drawMatrix(el.currentMatrix, current);
	drawMatrix(el.desiredMatrix, desired);
	el.currentWrithe.textContent = fmt(topo.writhe);
	el.targetWrithe.textContent = fmt(drag ? desiredTopo.writhe : target.writhe);
	el.residual.textContent = fmt(Math.sqrt(loss));
	el.solverStep.textContent = String(solverStep);
	el.contactProjectionCount.textContent = String(contactProof.count || 0);
	el.minClearance.textContent = fmt(acceptedClearance);
	el.dragStatus.textContent = draggedLabel();
	el.proofLog.textContent = [
		"GLI: analytical line-segment Gauss linking integral from Klenin-Langowski appendix.",
		"Writhe matrix: " + current.length + " x " + current[0].length + " segment-pair Ti,j values.",
		"Topology coordinates: w=" + fmt(topo.writhe) + ", density=" + fmt(topo.density) + ", center=(" + fmt(topo.centerA) + ", " + fmt(topo.centerB) + ").",
		"Drag mode: clicked joint selects one GrappleMap limb; only that limb's root-to-tip chain is movable, with its root anchored and the other body segments fixed as collision obstacles.",
		"Slider mode: vertical unit matrix, density rotation, center translation, scaled by target writhe.",
		"Generalized-coordinate solve: damped linearized QP/least-squares solve on ||J*dq-(Td-T)||^2 with per-frame target clamping and bone-length projection.",
		"Character control projection: every drag frame advances the grabbed joint by at most " + fmt(maxDragStep) + " m, restores the selected limb's bone lengths with anchored IK, then projects selected-limb capsules away from every non-adjacent body capsule.",
		"Kosher movement field: green dots are sampled drag-plane targets reachable in " + kosherPreviewSteps + " bounded steps while staying inside the same contact and topology limits; the green ring is the raw step-distance boundary.",
		"Step bounds: each frame can change a writhe-matrix cell by at most " + fmt(maxMatrixStep) + ", each topology-solver joint axis by at most " + fmt(maxJointStep) + " meters, drag input by at most " + fmt(maxDragStep) + " meters, and each collision/length projection is capped.",
		"Visual proof: orange lines are raw attempted vectors; green lines are the projected kosher vectors that replace them.",
		"Segment-length proof: selected limb max length error=" + fmt(drag ? maxChainLengthError(currentPosition, chainA, drag.chainLengths, drag.footLengths) : 0) + " m; foot drags include toe-heel, toe-ankle, and heel-ankle triangle constraints; any frame above 0.002 m is rejected.",
		"Contact projections=" + (contactProof.count || 0) + ", accepted min clearance=" + fmt(acceptedClearance) + ".",
		"Selected chains: " + chainA.label + " vs " + chainB.label + "."
	].join("\n");
}

function loadPosition(index) {
	basePosition = clonePosition(dbPositions[index].position);
	currentPosition = clonePosition(basePosition);
	autoSolving = false;
	autoStepBudget = 0;
	contactProof = emptyContactProof();
	solverStep = 0;
	syncSlidersToCurrentTopology();
	contactProof = relaxAllContacts(currentPosition, segmentLengthsFrom(currentPosition), [], 8);
	syncSlidersToCurrentTopology();
	updatePlayers(currentPosition);
	updateHandles(currentPosition);
	renderContactProof(contactProof);
	updateProof();
}

function populateControls() {
	el.positionSelect.innerHTML = "";
	dbPositions.slice(0, 420).forEach(function (record, index) {
		var option = document.createElement("option");
		option.value = String(index);
		option.textContent = (record.description[0] || "Position " + index).replace(/^#\S+\s*/, "").replace(/\\n/g, " ");
		el.positionSelect.appendChild(option);
	});

	[el.chainA, el.chainB].forEach(function (select) {
		select.innerHTML = "";
		chainDefs.forEach(function (chain) {
			var option = document.createElement("option");
			option.value = chain.id;
			option.textContent = chain.label;
			select.appendChild(option);
		});
	});
	el.chainA.value = "p0-left-arm";
	el.chainB.value = "p1-right-arm";
}

function initDom() {
	[
		"positionSelect", "chainA", "chainB", "affectA", "affectB", "writhe", "density",
		"centerA", "centerB", "frames", "kosherSteps", "writheOut", "densityOut", "centerAOut",
		"centerBOut", "framesOut", "kosherStepsOut", "resetBtn",
		"currentMatrix", "desiredMatrix", "proofLog", "currentWrithe", "targetWrithe",
		"residual", "solverStep", "contactProjectionCount", "minClearance", "dragStatus"
	].forEach(function (id) { el[id] = byId(id); });

	["writhe", "density", "centerA", "centerB", "frames"].forEach(function (id) {
		if (!el[id]) return;
		el[id].addEventListener("input", function () {
			queueAutoSolve(true);
		});
	});
	if (el.kosherSteps) {
		el.kosherSteps.addEventListener("input", function () {
			kosherPreviewSteps = parseInt(el.kosherSteps.value, 10);
			setOutputs();
			renderKosherMovementArea();
			updateProof();
		});
	}
	["chainA", "chainB", "affectA", "affectB"].forEach(function (id) {
		if (!el[id]) return;
		el[id].addEventListener("change", function () {
			autoSolving = false;
			syncSlidersToCurrentTopology();
			updateProof();
		});
	});
	el.positionSelect.addEventListener("change", function () {
		loadPosition(parseInt(el.positionSelect.value, 10));
	});
	el.resetBtn.addEventListener("click", function () { loadPosition(parseInt(el.positionSelect.value, 10)); });
	setOutputs();
}

function boot() {
	initDom();
	initScene();
	fetch("../GrappleMap.txt")
		.then(function (r) {
			if (!r.ok) throw new Error("HTTP " + r.status);
			return r.text();
		})
		.then(function (text) {
			dbPositions = parseGrappleMap(text);
			if (!dbPositions.length) dbPositions = fallbackPositions();
			populateControls();
			loadPosition(0);
		})
		.catch(function (err) {
			dbPositions = fallbackPositions();
			populateControls();
			loadPosition(0);
			el.proofLog.textContent += "\n\nGrappleMap.txt could not be fetched: " + err.message + "\nRun this page through a local web server from the repo root.";
		});
}

window.addEventListener("DOMContentLoaded", boot);

