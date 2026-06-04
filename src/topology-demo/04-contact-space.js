// Contact projection and clearance rules for body capsules in topology space.
// Keep this file focused: future agents should change this module for this system only.

function closestSegmentPoints(p1, q1, p2, q2) {
	var d1 = q1.subtract(p1);
	var d2 = q2.subtract(p2);
	var r = p1.subtract(p2);
	var a = dot(d1, d1);
	var e = dot(d2, d2);
	var f = dot(d2, r);
	var s, t;

	if (a <= 1e-8 && e <= 1e-8) return { c1: p1, c2: p2, s: 0, t: 0 };
	if (a <= 1e-8) {
		s = 0;
		t = clamp(f / e, 0, 1);
	} else {
		var c = dot(d1, r);
		if (e <= 1e-8) {
			t = 0;
			s = clamp(-c / a, 0, 1);
		} else {
			var b = dot(d1, d2);
			var denom = a * e - b * b;
			s = denom ? clamp((b * f - c * e) / denom, 0, 1) : 0;
			t = (b * s + f) / e;
			if (t < 0) {
				t = 0;
				s = clamp(-c / a, 0, 1);
			} else if (t > 1) {
				t = 1;
				s = clamp((b - c) / a, 0, 1);
			}
		}
	}

	return {
		c1: p1.add(d1.scale(s)),
		c2: p2.add(d2.scale(t)),
		s: s,
		t: t
	};
}

function contactDirection(candidate, obstacle) {
	var delta = candidate.c1.subtract(candidate.c2);
	var len = delta.length();
	if (len > 1e-6) return delta.scale(1 / len);
	var fallback = cross(obstacle.b.subtract(obstacle.a), v3(0, 1, 0));
	if (fallback.length() < 1e-6) fallback = v3(1, 0, 0);
	return fallback.normalize();
}

function distributeCorrection(position, moving, seg, correction, weightOnTo) {
	var fromKey = seg.player + ":" + seg.from;
	var toKey = seg.player + ":" + seg.to;
	var moveFrom = moving[fromKey];
	var moveTo = moving[toKey];
	if (moveFrom && moveTo) {
		position[seg.player][seg.from] = position[seg.player][seg.from].add(correction);
		position[seg.player][seg.to] = position[seg.player][seg.to].add(correction);
	} else if (moveFrom) {
		position[seg.player][seg.from] = position[seg.player][seg.from].add(correction);
	} else if (moveTo) {
		position[seg.player][seg.to] = position[seg.player][seg.to].add(correction);
	}
}

function segmentHasMovableJoint(moving, seg) {
	return moving[seg.player + ":" + seg.from] || moving[seg.player + ":" + seg.to];
}

function distributePairCorrection(position, moving, candidate, obstacle, correction, s, t) {
	var candidateCanMove = segmentHasMovableJoint(moving, candidate);
	var obstacleCanMove = segmentHasMovableJoint(moving, obstacle);
	if (candidateCanMove && obstacleCanMove) {
		distributeCorrection(position, moving, candidate, correction.scale(0.5), s);
		distributeCorrection(position, moving, obstacle, correction.scale(-0.5), t);
	} else if (candidateCanMove) {
		distributeCorrection(position, moving, candidate, correction, s);
	} else if (obstacleCanMove) {
		distributeCorrection(position, moving, obstacle, correction.scale(-1), t);
	}
}

function segmentsAreAdjacent(a, b) {
	return a.player === b.player && (a.from === b.from || a.from === b.to || a.to === b.from || a.to === b.to);
}

function projectContacts(previous, position, chains, move, minClearanceTarget) {
	var proof = emptyContactProof();
	var moving = movingJointMap(move);
	var acceptedClearance = Number.isFinite(minClearanceTarget) ? minClearanceTarget : minAcceptedClearance;
	var chainSegmentsNow = [];
	chains.forEach(function (chain) {
		chainSegmentsNow = chainSegmentsNow.concat(selectedChainBodySegments(position, chain));
	});
	var chainKeys = {};
	chainSegmentsNow.forEach(function (seg) { chainKeys[seg.key] = true; });

	for (var pass = 0; pass < 4; ++pass) {
		var changed = false;
		var body = drawableBodySegments(position);
		chainSegmentsNow = [];
		chains.forEach(function (chain) {
			chainSegmentsNow = chainSegmentsNow.concat(selectedChainBodySegments(position, chain));
		});

		for (var i = 0; i < chainSegmentsNow.length; ++i) {
			var candidate = chainSegmentsNow[i];
			for (var j = 0; j < body.length; ++j) {
				var obstacle = body[j];
				if (candidate.key === obstacle.key) continue;
				if (candidate.player === obstacle.player && (candidate.from === obstacle.from || candidate.from === obstacle.to || candidate.to === obstacle.from || candidate.to === obstacle.to)) continue;
				if (chainKeys[obstacle.key]) continue;

				var closest = closestSegmentPoints(candidate.a, candidate.b, obstacle.a, obstacle.b);
				var clearance = closest.c1.subtract(closest.c2).length() - candidate.radius - obstacle.radius;
				proof.minClearance = Math.min(proof.minClearance, clearance);
				var minClearance = acceptedClearance;
				if (clearance >= minClearance) continue;

				var normal = contactDirection(closest, obstacle);
				var correction = clampVector(normal.scale(minClearance - clearance), maxContactProjectionStep);
				var prevA = previous[candidate.player][candidate.from];
				var prevB = previous[candidate.player][candidate.to];
				var attemptedMid = candidate.a.add(candidate.b).scale(0.5);
				var previousMid = prevA.add(prevB).scale(0.5);
				var correctedMid = attemptedMid.add(correction);
				distributeCorrection(position, moving, candidate, correction, closest.s);
				proof.contacts.push({
					at: closest.c1,
					other: closest.c2,
					clearance: clearance,
					correction: correction.length()
				});
				proof.rejected.push({ from: previousMid, to: attemptedMid });
				proof.projected.push({ from: attemptedMid, to: correctedMid });
				proof.count += 1;
				changed = true;
			}
		}
		if (!changed) break;
	}

	if (!Number.isFinite(proof.minClearance)) proof.minClearance = 0;
	return proof;
}

function projectAllContacts(previous, position, move) {
	var proof = emptyContactProof();
	var moving = movingJointMap(move);

	for (var pass = 0; pass < 6; ++pass) {
		var changed = false;
		var body = drawableBodySegments(position);

		for (var i = 0; i < body.length; ++i) {
			var candidate = body[i];
			for (var j = i + 1; j < body.length; ++j) {
				var obstacle = body[j];
				if (candidate.key === obstacle.key) continue;
				if (segmentsAreAdjacent(candidate, obstacle)) continue;

				var closest = closestSegmentPoints(candidate.a, candidate.b, obstacle.a, obstacle.b);
				var clearance = closest.c1.subtract(closest.c2).length() - candidate.radius - obstacle.radius;
				proof.minClearance = Math.min(proof.minClearance, clearance);
				if (clearance >= minAcceptedClearance) continue;

				var normal = contactDirection(closest, obstacle);
				var correction = clampVector(normal.scale(minAcceptedClearance - clearance), maxContactProjectionStep);
				var prevA = previous[candidate.player][candidate.from];
				var prevB = previous[candidate.player][candidate.to];
				var attemptedMid = candidate.a.add(candidate.b).scale(0.5);
				var previousMid = prevA.add(prevB).scale(0.5);
				var correctedMid = attemptedMid.add(correction);
				distributePairCorrection(position, moving, candidate, obstacle, correction, closest.s, closest.t);
				proof.contacts.push({
					at: closest.c1,
					other: closest.c2,
					clearance: clearance,
					correction: correction.length()
				});
				proof.rejected.push({ from: previousMid, to: attemptedMid });
				proof.projected.push({ from: attemptedMid, to: correctedMid });
				proof.count += 1;
				changed = true;
			}
		}
		if (!changed) break;
	}

	if (!Number.isFinite(proof.minClearance)) proof.minClearance = 0;
	return proof;
}

function measureMinClearance(position, chains) {
	var minSeen = Infinity;
	var body = drawableBodySegments(position);
	var selected = [];
	var selectedKeys = {};
	chains.forEach(function (chain) {
		selected = selected.concat(selectedChainBodySegments(position, chain));
	});
	selected.forEach(function (seg) { selectedKeys[seg.key] = true; });
	for (var i = 0; i < selected.length; ++i) {
		for (var j = 0; j < body.length; ++j) {
			var candidate = selected[i];
			var obstacle = body[j];
			if (candidate.key === obstacle.key) continue;
			if (candidate.player === obstacle.player && (candidate.from === obstacle.from || candidate.from === obstacle.to || candidate.to === obstacle.from || candidate.to === obstacle.to)) continue;
			if (selectedKeys[obstacle.key]) continue;
			var closest = closestSegmentPoints(candidate.a, candidate.b, obstacle.a, obstacle.b);
			minSeen = Math.min(minSeen, closest.c1.subtract(closest.c2).length() - candidate.radius - obstacle.radius);
		}
	}
	return Number.isFinite(minSeen) ? minSeen : 0;
}

function measureAllMinClearance(position) {
	var minSeen = Infinity;
	var body = drawableBodySegments(position);
	for (var i = 0; i < body.length; ++i) {
		for (var j = i + 1; j < body.length; ++j) {
			var candidate = body[i];
			var obstacle = body[j];
			if (candidate.key === obstacle.key) continue;
			if (segmentsAreAdjacent(candidate, obstacle)) continue;
			var closest = closestSegmentPoints(candidate.a, candidate.b, obstacle.a, obstacle.b);
			minSeen = Math.min(minSeen, closest.c1.subtract(closest.c2).length() - candidate.radius - obstacle.radius);
		}
	}
	return Number.isFinite(minSeen) ? minSeen : 0;
}

