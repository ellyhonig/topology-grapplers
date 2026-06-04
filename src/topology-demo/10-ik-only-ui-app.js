// IK-only DOM controls, position loading, and application boot sequence.
// This boot file avoids topology-space UI and solver/contact proof dependencies.

function draggedLabel() {
	if (!drag) return "none";
	return (drag.player === 0 ? "Red " : "Blue ") + jointNames[drag.joint];
}

function selectedIkChainLabel() {
	if (!drag) return "none";
	if (drag.group) return drag.group.label;
	return drag.chain.label;
}

function currentIkLengthError() {
	if (!drag || !currentPosition) return 0;
	if (drag.bodyLengths) return maxBodyLengthError(currentPosition, drag.bodyLengths);
	return maxChainLengthError(currentPosition, drag.chain, drag.chainLengths, drag.footLengths);
}

function updateProof() {
	if (el.dragStatus) el.dragStatus.textContent = draggedLabel();
	if (el.activeLimb) el.activeLimb.textContent = selectedIkChainLabel();
	if (el.lengthError) el.lengthError.textContent = fmt(currentIkLengthError());
	if (el.motionMode) el.motionMode.textContent = "IK only";
	if (el.proofLog) {
		el.proofLog.textContent = [
			"Mode: IK-only body rules.",
			"Loaded modules: shared state, position store, body topology chains, IK body rules, IK input, scene renderer, IK UI boot.",
			"Skipped modules: contact space, topology coordinates, topology solver, and proof rendering.",
			"Drag behavior: neck and shoulder drags move the upper body, head drags solve only the spine/head chain, hip/knee/ankle drags move the attached leg section stiffly, core drags translate the full body without stretching bones, and distal hand/foot-tip drags solve their owning IK chain.",
			"Range behavior: shoulder and hip motion is clamped against the drag-start pose, with elbows/knees carrying the rest of the limb when the limit is reached.",
			"Neck behavior: shoulder-center to neck and neck to head are clamped to drag-start ranges and fixed lengths, so the head cannot spin through itself.",
			"Shoulder behavior: left and right shoulders are connected by a structural bone, so body-length projection preserves their span without a custom shoulder solver.",
			"Collision behavior: no inter-character clearance checks are run, so characters can clip through each other."
		].join("\n");
	}
}

function loadPosition(index) {
	basePosition = clonePosition(dbPositions[index].position);
	currentPosition = clonePosition(basePosition);
	autoSolving = false;
	autoStepBudget = 0;
	contactProof = emptyContactProof();
	solverStep = 0;
	updatePlayers(currentPosition);
	updateHandles(currentPosition);
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
}

function initDom() {
	[
		"positionSelect", "resetBtn", "dragStatus", "activeLimb", "lengthError", "motionMode", "proofLog"
	].forEach(function (id) { el[id] = byId(id); });

	el.positionSelect.addEventListener("change", function () {
		loadPosition(parseInt(el.positionSelect.value, 10));
	});
	el.resetBtn.addEventListener("click", function () {
		loadPosition(parseInt(el.positionSelect.value, 10));
	});
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
