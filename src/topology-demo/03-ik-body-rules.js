// IK body rules: anchored chain solving, foot triangle constraints, bone lengths, pins, and bend planes.
// Keep this file focused: future agents should change this module for this system only.

function solveTrianglePoint(a, b, la, lb, reference) {
	var axis = b.subtract(a);
	var d = axis.length();
	if (d < 1e-6) return reference.clone();
	axis = axis.scale(1 / d);
	var x = clamp((la * la - lb * lb + d * d) / (2 * d), 0, d);
	var h = Math.sqrt(Math.max(0, la * la - x * x));
	var refOffset = reference.subtract(a);
	var bend = refOffset.subtract(axis.scale(dot(refOffset, axis)));
	if (bend.length() < 1e-6) bend = cross(axis, v3(0, 1, 0));
	if (bend.length() < 1e-6) bend = cross(axis, v3(1, 0, 0));
	bend = bend.normalize();
	return a.add(axis.scale(x)).add(bend.scale(h));
}

function projectFootTriangle(position, chain, footLengths, draggedJoint, reference) {
	var side = footSideForChain(chain);
	if (!side || !footLengths) return;
	var p = position[chain.player];
	var r = reference ? reference[chain.player] : p;
	if (draggedJoint === side.heel) {
		p[side.toe] = solveTrianglePoint(p[side.ankle], p[side.heel], footLengths.ankleToe, footLengths.toeHeel, r[side.toe]);
	} else {
		p[side.heel] = solveTrianglePoint(p[side.ankle], p[side.toe], footLengths.ankleHeel, footLengths.toeHeel, r[side.heel]);
	}
}

function chainLengths(position, chain) {
	var lengths = [];
	for (var i = 0; i < chain.joints.length - 1; ++i) {
		lengths.push(dist(position[chain.player][chain.joints[i]], position[chain.player][chain.joints[i + 1]]));
	}
	return lengths;
}

function projectAnchoredChainToTarget(position, chain, lengths, targetJoint, target) {
	var player = position[chain.player];
	var targetIndex = chainJointIndex(chain, targetJoint);
	if (targetIndex <= 0) return;
	var rootJoint = chain.joints[0];
	var root = player[rootJoint].clone();
	var targetClamped = target.clone();
	targetClamped.y = Math.max(0.02, targetClamped.y);
	var reach = 0;
	for (var i = 0; i < targetIndex; ++i) reach += lengths[i];
	var rootToTarget = targetClamped.subtract(root);
	var d = rootToTarget.length();

	if (d >= reach && d > 1e-6) {
		var straight = rootToTarget.scale(1 / d);
		player[rootJoint] = root.clone();
		for (var s = 1; s <= targetIndex; ++s) {
			player[chain.joints[s]] = player[chain.joints[s - 1]].add(straight.scale(lengths[s - 1]));
		}
	} else {
		player[targetJoint] = targetClamped.clone();
		for (var iter = 0; iter < 5; ++iter) {
			player[targetJoint] = targetClamped.clone();
			for (var back = targetIndex - 1; back >= 0; --back) {
				var towardBack = player[chain.joints[back]].subtract(player[chain.joints[back + 1]]);
				var backLen = towardBack.length();
				if (backLen > 1e-6) player[chain.joints[back]] = player[chain.joints[back + 1]].add(towardBack.scale(lengths[back] / backLen));
			}
			player[rootJoint] = root.clone();
			for (var fwd = 1; fwd <= targetIndex; ++fwd) {
				var towardFwd = player[chain.joints[fwd]].subtract(player[chain.joints[fwd - 1]]);
				var fwdLen = towardFwd.length();
				if (fwdLen > 1e-6) player[chain.joints[fwd]] = player[chain.joints[fwd - 1]].add(towardFwd.scale(lengths[fwd - 1] / fwdLen));
			}
		}
	}

	for (var tail = targetIndex + 1; tail < chain.joints.length; ++tail) {
		var prev = player[chain.joints[tail - 1]];
		var cur = player[chain.joints[tail]];
		var dir = cur.subtract(prev);
		var len = dir.length();
		if (len < 1e-6) dir = v3(0, -1, 0);
		else dir = dir.scale(1 / len);
		player[chain.joints[tail]] = prev.add(dir.scale(lengths[tail - 1]));
	}
}

function projectSelectedChainLengths(position, chain, lengths, targetJoint) {
	var target = position[chain.player][targetJoint].clone();
	projectAnchoredChainToTarget(position, chain, lengths, targetJoint, target);
}

function projectSelectedLimbLengths(position, dragState) {
	projectSelectedChainLengths(position, dragState.chain, dragState.chainLengths, dragState.joint);
	projectFootTriangle(position, dragState.chain, dragState.footLengths, dragState.joint, dragState.referencePosition);
}

function maxChainLengthError(position, chain, lengths, footLengths) {
	var maxError = 0;
	for (var i = 0; i < chain.joints.length - 1; ++i) {
		var a = position[chain.player][chain.joints[i]];
		var b = position[chain.player][chain.joints[i + 1]];
		maxError = Math.max(maxError, Math.abs(dist(a, b) - lengths[i]));
	}
	var side = footSideForChain(chain);
	if (side && footLengths) {
		maxError = Math.max(maxError, Math.abs(segmentLength(position, chain.player, side.ankle, side.toe) - footLengths.ankleToe));
		maxError = Math.max(maxError, Math.abs(segmentLength(position, chain.player, side.ankle, side.heel) - footLengths.ankleHeel));
		maxError = Math.max(maxError, Math.abs(segmentLength(position, chain.player, side.toe, side.heel) - footLengths.toeHeel));
	}
	return maxError;
}

function pinnedJointMap(items) {
	var map = {};
	(items || []).forEach(function (item) { map[item.player + ":" + item.joint] = true; });
	return map;
}

function projectBodyLengths(position, referenceLengths, pinned, passes) {
	var pins = pinnedJointMap(pinned);
	for (var pass = 0; pass < (passes || 8); ++pass) {
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

function maxBodyLengthError(position, referenceLengths) {
	var maxError = 0;
	for (var i = 0; i < referenceLengths.length; ++i) {
		var seg = referenceLengths[i];
		maxError = Math.max(maxError, Math.abs(segmentLength(position, seg.player, seg.from, seg.to) - seg.length));
	}
	return maxError;
}

function perpendicularUnit(axis, fallback) {
	var side = fallback.subtract(axis.scale(dot(fallback, axis)));
	if (side.length() < 1e-6) side = cross(axis, v3(0, 1, 0));
	if (side.length() < 1e-6) side = cross(axis, v3(1, 0, 0));
	return side.normalize();
}

function clampDirectionToCone(direction, axis, maxAngleRadians, fallback) {
	var dirLen = direction.length();
	var axisLen = axis.length();
	if (dirLen < 1e-6 || axisLen < 1e-6) return null;
	var dir = direction.scale(1 / dirLen);
	var coneAxis = axis.scale(1 / axisLen);
	var cosLimit = Math.cos(maxAngleRadians);
	var along = dot(dir, coneAxis);
	if (along >= cosLimit) return dir;
	var side = perpendicularUnit(coneAxis, fallback || dir);
	var projectedSide = dir.subtract(coneAxis.scale(along));
	if (projectedSide.length() >= 1e-6) side = projectedSide.normalize();
	return coneAxis.scale(cosLimit).add(side.scale(Math.sin(maxAngleRadians))).normalize();
}

function rootRangeLimitDefs(player) {
	return [
		{ player: player, root: LeftShoulder, mid: LeftElbow, downstream: [LeftWrist, LeftHand, LeftFingers], maxAngle: 2.1 },
		{ player: player, root: RightShoulder, mid: RightElbow, downstream: [RightWrist, RightHand, RightFingers], maxAngle: 2.1 },
		{ player: player, root: LeftHip, mid: LeftKnee, downstream: [LeftAnkle, LeftToe, LeftHeel], maxAngle: 1.45 },
		{ player: player, root: RightHip, mid: RightKnee, downstream: [RightAnkle, RightToe, RightHeel], maxAngle: 1.45 }
	];
}

function projectRootRangeLimit(position, reference, limit) {
	var p = position[limit.player];
	var r = reference[limit.player];
	var current = p[limit.mid].subtract(p[limit.root]);
	var referenceDirection = r[limit.mid].subtract(r[limit.root]);
	var clamped = clampDirectionToCone(current, referenceDirection, limit.maxAngle, r[limit.mid].subtract(r[limit.root]));
	if (!clamped) return;
	var currentLen = current.length();
	var nextMid = p[limit.root].add(clamped.scale(currentLen));
	var carry = nextMid.subtract(p[limit.mid]);
	if (carry.length() < 1e-6) return;
	p[limit.mid] = nextMid;
	limit.downstream.forEach(function (joint) {
		p[joint] = p[joint].add(carry);
	});
}

function projectShoulderHipRangeLimits(position, reference, player) {
	if (!reference) return;
	rootRangeLimitDefs(player).forEach(function (limit) {
		projectRootRangeLimit(position, reference, limit);
	});
}

function midpoint(a, b) {
	return a.add(b).scale(0.5);
}

function projectSegmentConeLimit(position, player, root, child, referenceRoot, referenceChild, length, maxAngle, downstream) {
	var p = position[player];
	var current = p[child].subtract(root);
	var referenceDirection = referenceChild.subtract(referenceRoot);
	var clamped = clampDirectionToCone(current, referenceDirection, maxAngle, referenceDirection);
	if (!clamped) return;
	var nextChild = root.add(clamped.scale(length));
	var carry = nextChild.subtract(p[child]);
	if (carry.length() < 1e-6) return;
	p[child] = nextChild;
	(downstream || []).forEach(function (joint) {
		p[joint] = p[joint].add(carry);
	});
}

function projectNeckRangeLimits(position, reference, player) {
	if (!reference) return;
	var p = position[player];
	var r = reference[player];
	var shoulderCenter = midpoint(p[LeftShoulder], p[RightShoulder]);
	var referenceShoulderCenter = midpoint(r[LeftShoulder], r[RightShoulder]);
	projectSegmentConeLimit(
		position,
		player,
		shoulderCenter,
		Neck,
		referenceShoulderCenter,
		r[Neck],
		dist(referenceShoulderCenter, r[Neck]),
		0.85,
		[Head]
	);
	projectSegmentConeLimit(
		position,
		player,
		p[Neck],
		Head,
		r[Neck],
		r[Head],
		dist(r[Neck], r[Head]),
		0.95,
		[]
	);
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

