// Topology space solver: linearized damped solve with body-length/contact restoration.
// Keep this file focused: future agents should change this module for this system only.

function mergeProofs(into, proof) {
	into.contacts = into.contacts.concat(proof.contacts);
	into.rejected = into.rejected.concat(proof.rejected);
	into.projected = into.projected.concat(proof.projected);
	into.count += proof.count;
	into.minClearance = Math.min(into.minClearance, proof.minClearance);
}

function solveLinearSystem(a, b) {
	var n = b.length;
	var m = a.map(function (row, i) { return row.slice().concat([b[i]]); });
	for (var col = 0; col < n; ++col) {
		var pivot = col;
		for (var r = col + 1; r < n; ++r) {
			if (Math.abs(m[r][col]) > Math.abs(m[pivot][col])) pivot = r;
		}
		if (Math.abs(m[pivot][col]) < 1e-10) continue;
		var tmp = m[col];
		m[col] = m[pivot];
		m[pivot] = tmp;
		var div = m[col][col];
		for (var c = col; c <= n; ++c) m[col][c] /= div;
		for (var rr = 0; rr < n; ++rr) {
			if (rr === col) continue;
			var factor = m[rr][col];
			for (var cc = col; cc <= n; ++cc) m[rr][cc] -= factor * m[col][cc];
		}
	}
	return m.map(function (row) { return Number.isFinite(row[n]) ? row[n] : 0; });
}

function projectChainLengths(position, chain, lengths, anchorFirst) {
	var player = position[chain.player];
	if (anchorFirst) {
		for (var i = 1; i < chain.joints.length; ++i) {
			var prev = player[chain.joints[i - 1]];
			var cur = player[chain.joints[i]];
			var dir = cur.subtract(prev);
			var len = dir.length();
			if (len > 1e-6) player[chain.joints[i]] = prev.add(dir.scale(lengths[i - 1] / len));
		}
	} else {
		for (var j = chain.joints.length - 2; j >= 0; --j) {
			var next = player[chain.joints[j + 1]];
			var now = player[chain.joints[j]];
			var back = now.subtract(next);
			var backLen = back.length();
			if (backLen > 1e-6) player[chain.joints[j]] = next.add(back.scale(lengths[j] / backLen));
		}
	}
}

function restoreSelectedChainLengths(position, chainA, chainB, lenA, lenB, options) {
	if (options.affectA) projectChainLengths(position, chainA, lenA, true);
	if (options.affectB) projectChainLengths(position, chainB, lenB, true);
}

function movableJoints(chain, moveIt, pinned) {
	if (!moveIt) return [];
	var pins = pinnedJointMap(pinned);
	return chain.joints.slice(1).map(function (joint) {
		return { player: chain.player, joint: joint };
	}).filter(function (item) {
		return !pins[item.player + ":" + item.joint];
	});
}

function solveToward(position, chainA, chainB, desired, options) {
	var p = clonePosition(position);
	var move = movableJoints(chainA, options.affectA, options.pinned).concat(movableJoints(chainB, options.affectB, options.pinned));
	var lenA = chainLengths(p, chainA);
	var lenB = chainLengths(p, chainB);
	var aggregateProof = emptyContactProof();
	var clearanceFloor = Number.isFinite(options.clearanceFloor) ? options.clearanceFloor : -0.001;
	var minClearanceTarget = Number.isFinite(options.minClearanceTarget) ? options.minClearanceTarget : minAcceptedClearance;
	var eps = 0.012;
	var maxDelta = options.maxDelta || maxJointStep;
	var damping = 0.045;
	var axes = ["x", "y", "z"];
	var lastLoss = matrixLoss(writheMatrix(p, chainA, chainB), desired);

	for (var iter = 0; iter < options.iterations; ++iter) {
		var current = writheMatrix(p, chainA, chainB);
		var currentVec = matrixVector(current);
		var desiredVec = matrixVector(desired);
		var residual = desiredVec.map(function (v, i) { return v - currentVec[i]; });
		var columns = [];
		var dofs = [];

		for (var m = 0; m < move.length; ++m) {
			var target = p[move[m].player][move[m].joint];
			for (var a = 0; a < axes.length; ++a) {
				var axis = axes[a];
				target[axis] += eps;
				var plus = matrixVector(writheMatrix(p, chainA, chainB));
				target[axis] -= eps;
				var col = plus.map(function (value, idx) { return (value - currentVec[idx]) / eps; });
				columns.push(col);
				dofs.push({ player: move[m].player, joint: move[m].joint, axis: axis });
			}
		}

		var n = columns.length;
		if (!n) break;
		var lhs = [];
		var rhs = [];
		for (var row = 0; row < n; ++row) {
			lhs[row] = [];
			for (var col = 0; col < n; ++col) {
				var sum = row === col ? damping : 0;
				for (var ri = 0; ri < residual.length; ++ri) sum += columns[row][ri] * columns[col][ri];
				lhs[row][col] = sum;
			}
			var right = 0;
			for (var r = 0; r < residual.length; ++r) right += columns[row][r] * residual[r];
			rhs[row] = right;
		}

		var dq = solveLinearSystem(lhs, rhs);
		var beforeProjection = clonePosition(p);
		for (var k = 0; k < dofs.length; ++k) {
			var v = p[dofs[k].player][dofs[k].joint];
			v[dofs[k].axis] += clamp(dq[k], -maxDelta, maxDelta);
			v.y = Math.max(0.02, v.y);
		}

		var projectedChains = [];
		if (options.affectA) projectedChains.push(chainA);
		if (options.affectB) projectedChains.push(chainB);
		for (var pass = 0; pass < 3; ++pass) {
			restoreSelectedChainLengths(p, chainA, chainB, lenA, lenB, options);
			var iterationProof = projectContacts(beforeProjection, p, projectedChains, move, minClearanceTarget);
			mergeProofs(aggregateProof, iterationProof);
		}
		restoreSelectedChainLengths(p, chainA, chainB, lenA, lenB, options);
		var remainingClearance = projectedChains.length ? measureMinClearance(p, projectedChains) : 0;
		if (remainingClearance < clearanceFloor) {
			p = clonePosition(position);
			aggregateProof.minClearance = Math.min(aggregateProof.minClearance, remainingClearance);
			lastLoss = matrixLoss(writheMatrix(p, chainA, chainB), desired);
			break;
		}
		lastLoss = matrixLoss(writheMatrix(p, chainA, chainB), desired);
		++solverStep;
	}
	if (!Number.isFinite(aggregateProof.minClearance)) aggregateProof.minClearance = 0;
	return { position: p, loss: lastLoss, proof: aggregateProof };
}

