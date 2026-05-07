(function () {
	"use strict";

	var base62 = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
	var jointNames = [
		"Left toe", "Right toe", "Left heel", "Right heel", "Left ankle", "Right ankle",
		"Left knee", "Right knee", "Left hip", "Right hip", "Left shoulder", "Right shoulder",
		"Left elbow", "Right elbow", "Left wrist", "Right wrist", "Left hand", "Right hand",
		"Left fingers", "Right fingers", "Core", "Neck", "Head"
	];

	var chainDefs = [
		{ id: "p0-left-arm", label: "Red left arm", player: 0, joints: [LeftShoulder, LeftElbow, LeftWrist, LeftHand, LeftFingers] },
		{ id: "p0-right-arm", label: "Red right arm", player: 0, joints: [RightShoulder, RightElbow, RightWrist, RightHand, RightFingers] },
		{ id: "p0-left-leg", label: "Red left leg", player: 0, joints: [LeftHip, LeftKnee, LeftAnkle, LeftToe] },
		{ id: "p0-right-leg", label: "Red right leg", player: 0, joints: [RightHip, RightKnee, RightAnkle, RightToe] },
		{ id: "p0-spine", label: "Red spine/head", player: 0, joints: [Core, Neck, Head] },
		{ id: "p1-left-arm", label: "Blue left arm", player: 1, joints: [LeftShoulder, LeftElbow, LeftWrist, LeftHand, LeftFingers] },
		{ id: "p1-right-arm", label: "Blue right arm", player: 1, joints: [RightShoulder, RightElbow, RightWrist, RightHand, RightFingers] },
		{ id: "p1-left-leg", label: "Blue left leg", player: 1, joints: [LeftHip, LeftKnee, LeftAnkle, LeftToe] },
		{ id: "p1-right-leg", label: "Blue right leg", player: 1, joints: [RightHip, RightKnee, RightAnkle, RightToe] },
		{ id: "p1-spine", label: "Blue spine/head", player: 1, joints: [Core, Neck, Head] }
	];

	var el = {};
	var scene;
	var engine;
	var camera;
	var updatePlayers;
	var dbPositions = [];
	var basePosition;
	var currentPosition;
	var autoSolving = false;
	var autoStepBudget = 0;
	var solverStep = 0;
	var maxMatrixStep = 0.008;
	var maxJointStep = 0.018;
	var maxDragStep = 0.035;
	var maxLengthProjectionStep = 0.022;
	var maxContactProjectionStep = 0.026;
	var maxUnpinnedJointFrameStep = 0.055;
	var maxBendProjectionStep = 0.03;
	var minAcceptedClearance = 0.035;
	var contactProof = emptyContactProof();
	var proofMeshes = [];
	var handleMeshes = [];
	var drag = null;

	function byId(id) { return document.getElementById(id); }
	function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }
	function fmt(x) { return Number.isFinite(x) ? x.toFixed(3) : "0.000"; }
	function sqr(x) { return x * x; }
	function dot(a, b) { return BABYLON.Vector3.Dot(a, b); }
	function cross(a, b) { return BABYLON.Vector3.Cross(a, b); }
	function dist(a, b) { return a.subtract(b).length(); }
	function clampVector(v, maxLen) {
		var len = v.length();
		return len > maxLen && len > 1e-8 ? v.scale(maxLen / len) : v;
	}

	function clonePosition(p) {
		return p.map(function (player) {
			return player.map(function (joint) { return joint.clone(); });
		});
	}

	function fromBase62(c) {
		var i = base62.indexOf(c);
		if (i < 0) throw new Error("Invalid base62 digit: " + c);
		return i;
	}

	function decodePosition(encoded) {
		var clean = encoded.replace(/\s+/g, "");
		var offset = 0;
		function g() {
			var d = fromBase62(clean[offset++]) * 62 + fromBase62(clean[offset++]);
			return d / 1000;
		}

		var p = [[], []];
		for (var player = 0; player < 2; ++player) {
			for (var joint = 0; joint < jointNames.length; ++joint) {
				p[player].push(v3(g() - 2, g(), g() - 2));
			}
		}
		return p;
	}

	function parseGrappleMap(text) {
		var lines = text.replace(/\r/g, "").split("\n");
		var records = [];
		var desc = [];
		var encoded = [];

		function flushEncoded() {
			if (encoded.length !== 4) return;
			records.push({
				description: desc.slice(),
				position: decodePosition(encoded.join(""))
			});
			desc = [];
			encoded = [];
		}

		for (var i = 0; i < lines.length; ++i) {
			var line = lines[i];
			if (/^    /.test(line)) {
				encoded.push(line);
				if (encoded.length === 4) flushEncoded();
			} else {
				if (encoded.length) {
					encoded = [];
					desc = [];
				}
				if (line.trim()) desc.push(line);
			}
		}

		return records.filter(function (r) { return r.description.length; });
	}

	function fallbackPositions() {
		var p = [[], []];
		for (var player = 0; player < 2; ++player) {
			var x = player === 0 ? -0.35 : 0.35;
			p[player][LeftToe] = v3(x - 0.18, 0, 0.55);
			p[player][RightToe] = v3(x + 0.18, 0, 0.55);
			p[player][LeftHeel] = v3(x - 0.18, 0, 0.35);
			p[player][RightHeel] = v3(x + 0.18, 0, 0.35);
			p[player][LeftAnkle] = v3(x - 0.17, 0.08, 0.34);
			p[player][RightAnkle] = v3(x + 0.17, 0.08, 0.34);
			p[player][LeftKnee] = v3(x - 0.16, 0.45, 0.16);
			p[player][RightKnee] = v3(x + 0.16, 0.45, 0.16);
			p[player][LeftHip] = v3(x - 0.13, 0.86, 0);
			p[player][RightHip] = v3(x + 0.13, 0.86, 0);
			p[player][Core] = v3(x, 1.04, 0);
			p[player][LeftShoulder] = v3(x - 0.22, 1.38, 0);
			p[player][RightShoulder] = v3(x + 0.22, 1.38, 0);
			p[player][LeftElbow] = v3(x - 0.35, 1.12, player === 0 ? -0.1 : 0.1);
			p[player][RightElbow] = v3(x + 0.35, 1.12, player === 0 ? 0.1 : -0.1);
			p[player][LeftWrist] = v3(x - 0.18, 0.95, player === 0 ? -0.35 : 0.35);
			p[player][RightWrist] = v3(x + 0.18, 0.95, player === 0 ? 0.35 : -0.35);
			p[player][LeftHand] = v3(x - 0.12, 0.92, player === 0 ? -0.43 : 0.43);
			p[player][RightHand] = v3(x + 0.12, 0.92, player === 0 ? 0.43 : -0.43);
			p[player][LeftFingers] = v3(x - 0.08, 0.9, player === 0 ? -0.5 : 0.5);
			p[player][RightFingers] = v3(x + 0.08, 0.9, player === 0 ? 0.5 : -0.5);
			p[player][Neck] = v3(x, 1.52, 0);
			p[player][Head] = v3(x, 1.68, 0);
		}
		return [{ description: ["Fallback standing clinch"], position: p }];
	}

	function chainSegments(position, chain) {
		var out = [];
		for (var i = 0; i < chain.joints.length - 1; ++i) {
			out.push({
				a: position[chain.player][chain.joints[i]],
				b: position[chain.player][chain.joints[i + 1]]
			});
		}
		return out;
	}

	function emptyContactProof() {
		return { contacts: [], rejected: [], projected: [], count: 0, minClearance: Infinity };
	}

	function segmentKey(player, from, to) {
		return player + ":" + Math.min(from, to) + "-" + Math.max(from, to);
	}

	function movingJointMap(move) {
		var map = {};
		move.forEach(function (item) { map[item.player + ":" + item.joint] = true; });
		return map;
	}

	function allMovableJoints(pinned) {
		var pins = pinnedJointMap(pinned);
		var out = [];
		for (var player = 0; player < 2; ++player) {
			for (var joint = 0; joint < jointNames.length; ++joint) {
				if (pins[player + ":" + joint]) continue;
				out.push({ player: player, joint: joint });
			}
		}
		return out;
	}

	function drawableBodySegments(position) {
		var out = [];
		for (var player = 0; player < 2; ++player) {
			for (var s = 0; s < segments.length; ++s) {
				if (!segments[s][3]) continue;
				var from = segments[s][0][0];
				var to = segments[s][0][1];
				out.push({
					player: player,
					from: from,
					to: to,
					a: position[player][from],
					b: position[player][to],
					radius: segments[s][2],
					key: segmentKey(player, from, to)
				});
			}
		}
		return out;
	}

	function selectedChainBodySegments(position, chain) {
		var out = [];
		for (var i = 0; i < chain.joints.length - 1; ++i) {
			var from = chain.joints[i];
			var to = chain.joints[i + 1];
			out.push({
				player: chain.player,
				from: from,
				to: to,
				a: position[chain.player][from],
				b: position[chain.player][to],
				radius: Math.max(joints[from][0], joints[to][0], 0.035),
				key: segmentKey(chain.player, from, to)
			});
		}
		return out;
	}

	function visibleSegmentList() {
		var out = [];
		for (var player = 0; player < 2; ++player) {
			for (var s = 0; s < segments.length; ++s) {
				if (!segments[s][3]) continue;
				out.push({ player: player, from: segments[s][0][0], to: segments[s][0][1] });
			}
		}
		return out;
	}

	function structuralSegmentList() {
		var out = [];
		for (var player = 0; player < 2; ++player) {
			for (var s = 0; s < segments.length; ++s) {
				out.push({ player: player, from: segments[s][0][0], to: segments[s][0][1] });
			}
		}
		return out;
	}

	function segmentLengthsFrom(position) {
		return structuralSegmentList().map(function (seg) {
			return {
				player: seg.player,
				from: seg.from,
				to: seg.to,
				length: dist(position[seg.player][seg.from], position[seg.player][seg.to])
			};
		});
	}

	function pinnedJointMap(items) {
		var map = {};
		(items || []).forEach(function (item) { map[item.player + ":" + item.joint] = true; });
		return map;
	}

	function projectBodyLengths(position, referenceLengths, pinned) {
		var pins = pinnedJointMap(pinned);
		for (var pass = 0; pass < 8; ++pass) {
			for (var i = 0; i < referenceLengths.length; ++i) {
				var seg = referenceLengths[i];
				var a = position[seg.player][seg.from];
				var b = position[seg.player][seg.to];
				var delta = b.subtract(a);
				var len = delta.length();
				if (len < 1e-6) continue;
				var correction = clampVector(delta.scale((len - seg.length) / len), maxLengthProjectionStep);
				var aKey = seg.player + ":" + seg.from;
				var bKey = seg.player + ":" + seg.to;
				var aPinned = pins[aKey];
				var bPinned = pins[bKey];
				if (aPinned && bPinned) continue;
				if (aPinned) {
					position[seg.player][seg.to] = b.subtract(correction);
				} else if (bPinned) {
					position[seg.player][seg.from] = a.add(correction);
				} else {
					position[seg.player][seg.from] = a.add(correction.scale(0.5));
					position[seg.player][seg.to] = b.subtract(correction.scale(0.5));
				}
			}
		}
	}

	function twoBoneDefs() {
		return [
			{ player: 0, root: LeftHip, mid: LeftKnee, tip: LeftAnkle },
			{ player: 0, root: RightHip, mid: RightKnee, tip: RightAnkle },
			{ player: 1, root: LeftHip, mid: LeftKnee, tip: LeftAnkle },
			{ player: 1, root: RightHip, mid: RightKnee, tip: RightAnkle },
			{ player: 0, root: LeftShoulder, mid: LeftElbow, tip: LeftWrist },
			{ player: 0, root: RightShoulder, mid: RightElbow, tip: RightWrist },
			{ player: 1, root: LeftShoulder, mid: LeftElbow, tip: LeftWrist },
			{ player: 1, root: RightShoulder, mid: RightElbow, tip: RightWrist }
		];
	}

	function projectBendPlanes(position, reference, pinned) {
		var pins = pinnedJointMap(pinned);
		twoBoneDefs().forEach(function (limb) {
			if (pins[limb.player + ":" + limb.root] || pins[limb.player + ":" + limb.mid] || pins[limb.player + ":" + limb.tip]) return;
			var p = position[limb.player];
			var r = reference[limb.player];
			var root = p[limb.root];
			var tip = p[limb.tip];
			var currentMid = p[limb.mid];
			var rootTip = tip.subtract(root);
			var d = rootTip.length();
			var l1 = dist(r[limb.root], r[limb.mid]);
			var l2 = dist(r[limb.mid], r[limb.tip]);
			if (d < 1e-6 || l1 < 1e-6 || l2 < 1e-6) return;
			var axis = rootTip.scale(1 / d);
			var x = clamp((l1 * l1 - l2 * l2 + d * d) / (2 * d), 0, d);
			var h2 = Math.max(0, l1 * l1 - x * x);
			var bendRadius = Math.sqrt(h2);
			var refAxis = r[limb.tip].subtract(r[limb.root]);
			var refLen = refAxis.length();
			if (refLen < 1e-6) return;
			refAxis = refAxis.scale(1 / refLen);
			var refBend = r[limb.mid].subtract(r[limb.root]).subtract(refAxis.scale(dot(r[limb.mid].subtract(r[limb.root]), refAxis)));
			var bendDir = refBend.subtract(axis.scale(dot(refBend, axis)));
			if (bendDir.length() < 1e-6) {
				bendDir = currentMid.subtract(root).subtract(axis.scale(dot(currentMid.subtract(root), axis)));
			}
			if (bendDir.length() < 1e-6) return;
			bendDir = bendDir.normalize();
			var targetMid = root.add(axis.scale(x)).add(bendDir.scale(bendRadius));
			p[limb.mid] = currentMid.add(clampVector(targetMid.subtract(currentMid), maxBendProjectionStep));
		});
	}

	function forcePinnedPositions(position, pinned) {
		(pinned || []).forEach(function (pin) {
			if (!pin.position) return;
			position[pin.player][pin.joint] = pin.position.clone();
		});
	}

	function limitUnpinnedFrameMotion(position, previous, pinned) {
		var pins = pinnedJointMap(pinned);
		for (var player = 0; player < 2; ++player) {
			for (var joint = 0; joint < jointNames.length; ++joint) {
				if (pins[player + ":" + joint]) continue;
				var prev = previous[player][joint];
				position[player][joint] = prev.add(clampVector(position[player][joint].subtract(prev), maxUnpinnedJointFrameStep));
			}
		}
	}

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

	function projectContacts(previous, position, chains, move) {
		var proof = emptyContactProof();
		var moving = movingJointMap(move);
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
					var minClearance = minAcceptedClearance;
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

	function normalOrZero(a, b) {
		var n = cross(a, b);
		var len = n.length();
		if (len < 1e-8) return v3(0, 0, 0);
		return n.scale(1 / len);
	}

	function safeAsin(x) {
		return Math.asin(clamp(x, -1, 1));
	}

	function segmentWrithe(segA, segB) {
		var a = segA.a, b = segA.b, c = segB.a, d = segB.b;
		var rac = c.subtract(a);
		var rad = d.subtract(a);
		var rbd = d.subtract(b);
		var rbc = c.subtract(b);
		var na = normalOrZero(rac, rad);
		var nb = normalOrZero(rad, rbd);
		var nc = normalOrZero(rbd, rbc);
		var nd = normalOrZero(rbc, rac);
		var omega = safeAsin(dot(na, nb)) + safeAsin(dot(nb, nc)) + safeAsin(dot(nc, nd)) + safeAsin(dot(nd, na));
		return omega / (4 * Math.PI);
	}

	function writheMatrix(position, chainA, chainB) {
		var a = chainSegments(position, chainA);
		var b = chainSegments(position, chainB);
		var matrix = [];
		for (var i = 0; i < a.length; ++i) {
			matrix[i] = [];
			for (var j = 0; j < b.length; ++j) matrix[i][j] = segmentWrithe(a[i], b[j]);
		}
		return matrix;
	}

	function matrixSum(m) {
		return m.reduce(function (sum, row) {
			return sum + row.reduce(function (s, v) { return s + v; }, 0);
		}, 0);
	}

	function topologyCoordinates(m) {
		var rows = m.length;
		var cols = m[0].length;
		var w = matrixSum(m);
		var weighted = 0;
		var x = 0;
		var y = 0;
		var points = [];

		for (var i = 0; i < rows; ++i) {
			for (var j = 0; j < cols; ++j) {
				var weight = Math.abs(m[i][j]);
				var nx = rows === 1 ? 0 : (i / (rows - 1)) * 2 - 1;
				var ny = cols === 1 ? 0 : (j / (cols - 1)) * 2 - 1;
				weighted += weight;
				x += nx * weight;
				y += ny * weight;
				points.push([nx, ny, weight]);
			}
		}

		x = weighted ? x / weighted : 0;
		y = weighted ? y / weighted : 0;

		var xx = 0, xy = 0, yy = 0;
		points.forEach(function (p) {
			var dx = p[0] - x;
			var dy = p[1] - y;
			xx += p[2] * dx * dx;
			xy += p[2] * dx * dy;
			yy += p[2] * dy * dy;
		});

		var principal = 0.5 * Math.atan2(2 * xy, xx - yy);
		var density = clamp(principal - Math.PI / 4, -Math.PI / 4, Math.PI / 4);
		return { writhe: w, centerA: x, centerB: y, density: density };
	}

	function makeZeroMatrix(rows, cols) {
		var m = [];
		for (var i = 0; i < rows; ++i) {
			m[i] = [];
			for (var j = 0; j < cols; ++j) m[i][j] = 0;
		}
		return m;
	}

	function splat(m, x, y, value) {
		var rows = m.length;
		var cols = m[0].length;
		var x0 = Math.floor(x);
		var y0 = Math.floor(y);
		for (var dx = 0; dx <= 1; ++dx) {
			for (var dy = 0; dy <= 1; ++dy) {
				var xi = x0 + dx;
				var yj = y0 + dy;
				if (xi < 0 || xi >= rows || yj < 0 || yj >= cols) continue;
				var wx = 1 - Math.abs(x - xi);
				var wy = 1 - Math.abs(y - yj);
				m[xi][yj] += value * Math.max(0, wx) * Math.max(0, wy);
			}
		}
	}

	function desiredWritheMatrix(rows, cols, topo) {
		var out = makeZeroMatrix(rows, cols);
		var centerCol = (cols - 1) / 2;
		var phi = topo.density + Math.PI / 4;
		var cos = Math.cos(phi);
		var sin = Math.sin(phi);
		var tx = topo.centerA * (rows - 1) * 0.5;
		var ty = topo.centerB * (cols - 1) * 0.5;

		for (var i = 0; i < rows; ++i) {
			var base = cols % 2 ? [{ j: centerCol, v: 1 / rows }] : [
				{ j: Math.floor(centerCol), v: 0.5 / rows },
				{ j: Math.ceil(centerCol), v: 0.5 / rows }
			];
			base.forEach(function (entry) {
				var x = i - (rows - 1) / 2;
				var y = entry.j - (cols - 1) / 2;
				var rx = x * cos - y * sin + (rows - 1) / 2 + tx;
				var ry = x * sin + y * cos + (cols - 1) / 2 + ty;
				splat(out, rx, ry, entry.v * topo.writhe);
			});
		}
		return out;
	}

	function matrixLoss(current, desired) {
		var loss = 0;
		for (var i = 0; i < current.length; ++i) {
			for (var j = 0; j < current[i].length; ++j) {
				loss += sqr(current[i][j] - desired[i][j]);
			}
		}
		return loss;
	}

	function matrixVector(m) {
		var out = [];
		for (var i = 0; i < m.length; ++i) {
			for (var j = 0; j < m[i].length; ++j) out.push(m[i][j]);
		}
		return out;
	}

	function steppedMatrix(current, target, maxStep) {
		var out = [];
		for (var i = 0; i < current.length; ++i) {
			out[i] = [];
			for (var j = 0; j < current[i].length; ++j) {
				out[i][j] = current[i][j] + clamp(target[i][j] - current[i][j], -maxStep, maxStep);
			}
		}
		return out;
	}

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

	function chainLengths(position, chain) {
		var lengths = [];
		for (var i = 0; i < chain.joints.length - 1; ++i) {
			lengths.push(dist(position[chain.player][chain.joints[i]], position[chain.player][chain.joints[i + 1]]));
		}
		return lengths;
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
				var iterationProof = projectContacts(beforeProjection, p, projectedChains, move);
				mergeProofs(aggregateProof, iterationProof);
			}
			restoreSelectedChainLengths(p, chainA, chainB, lenA, lenB, options);
			var remainingClearance = projectedChains.length ? measureMinClearance(p, projectedChains) : 0;
			if (remainingClearance < -0.001) {
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
		chains.forEach(function (chain) {
			chain.joints.forEach(function (joint) {
				var key = chain.player + ":" + joint;
				if (pin && key === pin.player + ":" + pin.joint) return;
				if (seen[key]) return;
				seen[key] = true;
				out.push({ player: chain.player, joint: joint });
			});
		});
		return out;
	}

	function relaxContacts(position, chains, lengths, pinned, passes) {
		var proof = emptyContactProof();
		var move = dragMoveList(chains, null);
		for (var pass = 0; pass < passes; ++pass) {
			var before = clonePosition(position);
			var passProof = projectContacts(before, position, chains, move);
			mergeProofs(proof, passProof);
			projectBodyLengths(position, lengths, pinned || []);
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
		var origin = currentPosition[player][joint].clone();
		var normal = camera.position.subtract(origin).normalize();
		drag = {
			player: player,
			joint: joint,
			target: origin.clone(),
			offset: v3(0, 0, 0),
			plane: BABYLON.Plane.FromPositionAndNormal(origin, normal),
			topologyMatrix: writheMatrix(currentPosition, chainA, chainB),
			referencePosition: clonePosition(currentPosition),
			lengths: segmentLengthsFrom(currentPosition)
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

	function dragStep() {
		if (!drag || !currentPosition) return;
		var chainA = selectedChain(el.chainA);
		var chainB = selectedChain(el.chainB);
		var previous = clonePosition(currentPosition);
		var p = clonePosition(currentPosition);
		var grabbed = p[drag.player][drag.joint];
		var delta = drag.target.subtract(grabbed);
		var deltaLen = delta.length();
		if (deltaLen > maxDragStep) delta = delta.scale(maxDragStep / deltaLen);
		var draggedPosition = grabbed.add(delta);
		draggedPosition.y = Math.max(0.02, draggedPosition.y);
		p[drag.player][drag.joint] = draggedPosition;

		var pin = { player: drag.player, joint: drag.joint, position: draggedPosition };
		var pins = [pin];
		projectBodyLengths(p, drag.lengths, [pin]);
		forcePinnedPositions(p, pins);
		projectBendPlanes(p, drag.referencePosition, [pin]);
		forcePinnedPositions(p, pins);
		relaxAllContacts(p, drag.lengths, pins, 2);
		forcePinnedPositions(p, pins);
		projectBendPlanes(p, drag.referencePosition, [pin]);
		forcePinnedPositions(p, pins);
		var current = writheMatrix(p, chainA, chainB);
		var desired = steppedMatrix(current, drag.topologyMatrix, maxMatrixStep);
		var solved = solveToward(p, chainA, chainB, desired, {
			affectA: true,
			affectB: true,
			pinned: [pin],
			iterations: 1,
			maxDelta: 0.012
		});

		p = solved.position;
		projectBodyLengths(p, drag.lengths, [pin]);
		forcePinnedPositions(p, pins);
		projectBendPlanes(p, drag.referencePosition, [pin]);
		forcePinnedPositions(p, pins);
		var projectionProof = relaxAllContacts(p, drag.lengths, pins, 5);
		forcePinnedPositions(p, pins);
		projectBendPlanes(p, drag.referencePosition, [pin]);
		forcePinnedPositions(p, pins);
		projectBodyLengths(p, drag.lengths, [pin]);
		forcePinnedPositions(p, pins);
		limitUnpinnedFrameMotion(p, previous, pins);
		forcePinnedPositions(p, pins);
		mergeProofs(solved.proof, projectionProof);

		var nextClearance = measureAllMinClearance(p);
		solved.proof.minClearance = Math.min(solved.proof.minClearance, nextClearance);

		currentPosition = p;
		contactProof = solved.proof || emptyContactProof();
		updatePlayers(currentPosition);
		updateHandles(currentPosition);
		renderContactProof(contactProof);
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

	function selectedChain(select) {
		return chainDefs.find(function (chain) { return chain.id === select.value; });
	}

	function chainContaining(player, joint) {
		return chainDefs.find(function (chain) {
			return chain.player === player && chain.joints.indexOf(joint) !== -1;
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
		var chainA = selectedChain(el.chainA);
		var chainB = selectedChain(el.chainB);
		var current = writheMatrix(currentPosition, chainA, chainB);
		var topo = topologyCoordinates(current);
		var target = readTarget();
		var desired = drag ? drag.topologyMatrix : desiredWritheMatrix(current.length, current[0].length, target);
		var desiredTopo = topologyCoordinates(desired);
		var loss = matrixLoss(current, desired);
		var acceptedClearance = drag ? measureAllMinClearance(currentPosition) : measureMinClearance(currentPosition, [chainA, chainB]);
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
			"Drag mode: clicked joint is a hard real-time control input; the solver preserves the topology matrix captured at pointer-down.",
			"Slider mode: vertical unit matrix, density rotation, center translation, scaled by target writhe.",
			"Generalized-coordinate solve: damped linearized QP/least-squares solve on ||J*dq-(Td-T)||^2 with per-frame target clamping and bone-length projection.",
			"Character control projection: every drag frame pins the grabbed joint, restores all GrappleMap body segment lengths, preserves knee/elbow bend planes, then projects every non-adjacent limb capsule pair along the nearest valid normal.",
			"Step bounds: each frame can change a writhe-matrix cell by at most " + fmt(maxMatrixStep) + ", each joint axis by at most " + fmt(maxJointStep) + " meters, each unpinned joint by at most " + fmt(maxUnpinnedJointFrameStep) + " meters, and each collision/length projection is capped.",
			"Visual proof: orange lines are rejected attempted vectors; green lines are the projected kosher vectors that replace them.",
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
			"centerA", "centerB", "frames", "writheOut", "densityOut", "centerAOut",
			"centerBOut", "framesOut", "resetBtn",
			"currentMatrix", "desiredMatrix", "proofLog", "currentWrithe", "targetWrithe",
			"residual", "solverStep", "contactProjectionCount", "minClearance", "dragStatus"
		].forEach(function (id) { el[id] = byId(id); });

		["writhe", "density", "centerA", "centerB", "frames"].forEach(function (id) {
			if (!el[id]) return;
			el[id].addEventListener("input", function () {
				queueAutoSolve(true);
			});
		});
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
}());
