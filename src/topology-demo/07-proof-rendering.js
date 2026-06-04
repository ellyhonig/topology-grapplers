// Visual proof rendering for matrices, contact vectors, and kosher movement previews.
// Keep this file focused: future agents should change this module for this system only.

function drawMatrix(canvas, matrix) {
	var ctx = canvas.getContext("2d");
	var w = canvas.width;
	var h = canvas.height;
	ctx.clearRect(0, 0, w, h);
	var rows = matrix.length;
	var cols = matrix[0].length;
	var max = 0;
	for (var i = 0; i < rows; ++i) {
		for (var j = 0; j < cols; ++j) max = Math.max(max, Math.abs(matrix[i][j]));
	}
	max = max || 1;
	var cellW = w / cols;
	var cellH = h / rows;
	for (var r = 0; r < rows; ++r) {
		for (var c = 0; c < cols; ++c) {
			var val = matrix[r][c] / max;
			var red = val > 0 ? Math.round(40 + 200 * val) : 35;
			var blue = val < 0 ? Math.round(40 - 200 * val) : 35;
			var green = Math.round(235 - 130 * Math.abs(val));
			ctx.fillStyle = "rgb(" + red + "," + green + "," + blue + ")";
			ctx.fillRect(c * cellW, r * cellH, cellW + 1, cellH + 1);
		}
	}
	ctx.strokeStyle = "#222";
	ctx.strokeRect(0.5, 0.5, w - 1, h - 1);
}

function makeLine(name, from, to, color) {
	var line = BABYLON.MeshBuilder.CreateLines(name, { points: [from, to] }, scene);
	line.color = color;
	line.alwaysSelectAsActiveMesh = true;
	proofMeshes.push(line);
	return line;
}

function renderContactProof(proof) {
	proofMeshes.forEach(function (mesh) { mesh.dispose(); });
	proofMeshes = [];
	if (!proof || !proof.rejected) return;
	var rejectedColor = new BABYLON.Color3(0.94, 0.42, 0.08);
	var projectedColor = new BABYLON.Color3(0.08, 0.62, 0.28);
	var contactColor = new BABYLON.Color3(0.05, 0.05, 0.05);
	var maxLines = Math.min(16, proof.rejected.length);
	for (var i = 0; i < maxLines; ++i) {
		makeLine("rejected-contact-vector", proof.rejected[i].from, proof.rejected[i].to, rejectedColor);
		makeLine("projected-contact-vector", proof.projected[i].from, proof.projected[i].to, projectedColor);
	}
	var maxContacts = Math.min(10, proof.contacts.length);
	for (var j = 0; j < maxContacts; ++j) {
		makeLine("capsule-clearance", proof.contacts[j].at, proof.contacts[j].other, contactColor);
	}
}

function disposeKosherVisualization() {
	kosherMeshes.forEach(function (mesh) { mesh.dispose(); });
	kosherMeshes = [];
}

function ensureKosherMaterials() {
	if (kosherMat) return;
	kosherMat = new BABYLON.StandardMaterial("kosher-move-valid", scene);
	kosherMat.diffuseColor = new BABYLON.Color3(0.08, 0.78, 0.28);
	kosherMat.emissiveColor = new BABYLON.Color3(0.02, 0.22, 0.06);
	kosherMat.alpha = 0.48;
	kosherEdgeMat = new BABYLON.StandardMaterial("kosher-move-edge", scene);
	kosherEdgeMat.diffuseColor = new BABYLON.Color3(0.04, 0.44, 0.18);
	kosherEdgeMat.emissiveColor = new BABYLON.Color3(0.02, 0.14, 0.05);
	kosherEdgeMat.alpha = 0.7;
}

function dragPlaneBasis(d) {
	var normal = d.plane.normal.clone();
	if (normal.length() < 1e-6) normal = v3(0, 0, 1);
	normal = normal.normalize();
	var up = Math.abs(dot(normal, v3(0, 1, 0))) > 0.92 ? v3(1, 0, 0) : v3(0, 1, 0);
	var u = cross(up, normal).normalize();
	var v = cross(normal, u).normalize();
	return { u: u, v: v };
}

function simulateKosherTarget(source, d, target, steps) {
	var p = clonePosition(source);
	var chain = d.chain;
	var targetJoint = d.joint;
	var proof = emptyContactProof();
	var stepTarget = p[d.player][targetJoint].clone();
	for (var i = 0; i < steps; ++i) {
		var delta = target.subtract(stepTarget);
		if (delta.length() < 0.001) break;
		stepTarget = stepTarget.add(clampVector(delta, maxDragStep));
		var before = clonePosition(p);
		projectAnchoredChainToTarget(p, chain, d.chainLengths, targetJoint, stepTarget);
		projectFootTriangle(p, chain, d.footLengths, targetJoint, d.referencePosition);
		if (d.bodyLengths) projectBodyLengths(p, d.bodyLengths, [chainRootPin(chain)]);
		var passProof = projectContacts(before, p, [chain], dragMoveList([chain], null), d.contactClearanceTarget);
		mergeProofs(proof, passProof);
		projectSelectedLimbLengths(p, d);
		if (d.bodyLengths) projectBodyLengths(p, d.bodyLengths, [chainRootPin(chain)]);
	}
	var clearance = measureMinClearance(p, [chain]);
	var clearanceFloor = Number.isFinite(d.clearanceFloor) ? d.clearanceFloor : -0.001;
	proof.minClearance = Math.min(proof.minClearance, clearance);
	return {
		position: p,
		accepted: clearance >= clearanceFloor && dist(p[d.player][targetJoint], target) <= maxDragStep * 1.25,
		clearance: clearance,
		proof: proof
	};
}

function renderKosherMovementArea() {
	disposeKosherVisualization();
	if (!drag || !currentPosition || !scene) return;
	ensureKosherMaterials();
	var center = currentPosition[drag.player][drag.joint];
	var basis = dragPlaneBasis(drag);
	var radius = maxDragStep * kosherPreviewSteps;
	var validPoints = [];
	var samples = [{ x: 0, y: 0 }];
	for (var ring = 1; ring <= 4; ++ring) {
		var ringRadius = radius * ring / 4;
		var count = ring * 8;
		for (var i = 0; i < count; ++i) {
			var angle = (Math.PI * 2 * i) / count;
			samples.push({ x: Math.cos(angle) * ringRadius, y: Math.sin(angle) * ringRadius });
		}
	}

	samples.forEach(function (sample) {
		var candidate = center.add(basis.u.scale(sample.x)).add(basis.v.scale(sample.y));
		candidate.y = Math.max(0.02, candidate.y);
		var result = simulateKosherTarget(currentPosition, drag, candidate, kosherPreviewSteps);
		if (!result.accepted) return;
		validPoints.push(candidate);
		var dotMesh = BABYLON.MeshBuilder.CreateSphere("kosher-reachable-dot", { segments: 8, diameter: 0.025 }, scene);
		dotMesh.position.copyFrom(candidate);
		dotMesh.material = kosherMat;
		dotMesh.isPickable = false;
		kosherMeshes.push(dotMesh);
	});

	if (validPoints.length > 2) {
		var linePoints = [];
		var edgeCount = 64;
		for (var e = 0; e <= edgeCount; ++e) {
			var edgeAngle = (Math.PI * 2 * e) / edgeCount;
			linePoints.push(center.add(basis.u.scale(Math.cos(edgeAngle) * radius)).add(basis.v.scale(Math.sin(edgeAngle) * radius)));
		}
		var edge = BABYLON.MeshBuilder.CreateLines("kosher-step-boundary", { points: linePoints }, scene);
		edge.color = new BABYLON.Color3(0.02, 0.42, 0.12);
		edge.isPickable = false;
		kosherMeshes.push(edge);
	}
}

