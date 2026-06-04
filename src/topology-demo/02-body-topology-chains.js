// Body chain definitions, segment inventories, limb lookup, and drag-chain selection.
// Keep this file focused: future agents should change this module for this system only.

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
	var seen = {};
	function pushSegment(from, to) {
		var key = segmentKey(chain.player, from, to);
		if (seen[key]) return;
		seen[key] = true;
		out.push({
			player: chain.player,
			from: from,
			to: to,
			a: position[chain.player][from],
			b: position[chain.player][to],
			radius: Math.max(joints[from][0], joints[to][0], 0.035),
			key: key
		});
	}
	for (var i = 0; i < chain.joints.length - 1; ++i) {
		pushSegment(chain.joints[i], chain.joints[i + 1]);
	}
	var side = footSideForChain(chain);
	if (side) {
		pushSegment(side.ankle, side.toe);
		pushSegment(side.ankle, side.heel);
		pushSegment(side.toe, side.heel);
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

function chainRootPin(chain) {
	return { player: chain.player, joint: chain.joints[0], position: null };
}

function chainJointIndex(chain, joint) {
	return chain.joints.indexOf(joint);
}

function legSideForJoint(joint) {
	if ([LeftHip, LeftKnee, LeftAnkle, LeftToe, LeftHeel].indexOf(joint) !== -1) {
		return { hip: LeftHip, knee: LeftKnee, ankle: LeftAnkle, toe: LeftToe, heel: LeftHeel, label: "left" };
	}
	if ([RightHip, RightKnee, RightAnkle, RightToe, RightHeel].indexOf(joint) !== -1) {
		return { hip: RightHip, knee: RightKnee, ankle: RightAnkle, toe: RightToe, heel: RightHeel, label: "right" };
	}
	return null;
}

function legSideForChain(chain) {
	if (!chain) return null;
	if (chain.footSide) return chain.footSide;
	for (var i = 0; i < chain.joints.length; ++i) {
		var side = legSideForJoint(chain.joints[i]);
		if (side) return side;
	}
	return null;
}

function isLegChain(chain) {
	return !!legSideForChain(chain);
}

function reversedDragChain(chain) {
	var reversed = {
		id: chain.id + "-reverse",
		label: chain.label + " reverse path",
		player: chain.player,
		joints: chain.joints.slice().reverse()
	};
	var side = legSideForChain(chain);
	if (side) reversed.footSide = side;
	return reversed;
}

function rootParentJoint(joint) {
	if (joint === LeftShoulder || joint === RightShoulder || joint === LeftHip || joint === RightHip) return Core;
	return null;
}

function torsoAnchoredDragChain(chain, parent) {
	var anchored = {
		id: chain.id + "-torso-anchor",
		label: chain.label + " torso anchored path",
		player: chain.player,
		joints: [parent].concat(chain.joints),
		projectBody: true
	};
	var side = legSideForChain(chain);
	if (side) anchored.footSide = side;
	return anchored;
}

function dragChainForJoint(baseChain, player, joint) {
	if (!baseChain || baseChain.player !== player) return baseChain;
	if (chainJointIndex(baseChain, joint) === 0) {
		var parent = rootParentJoint(joint);
		return parent === null ? reversedDragChain(baseChain) : torsoAnchoredDragChain(baseChain, parent);
	}

	var side = legSideForJoint(joint);
	if (!side || joint !== side.heel) return baseChain;
	return {
		id: baseChain.id + "-heel",
		label: baseChain.label + " heel path",
		player: player,
		joints: [side.hip, side.knee, side.ankle, side.heel],
		footSide: side
	};
}

function segmentLength(position, player, a, b) {
	return dist(position[player][a], position[player][b]);
}

function footLengthsFrom(position, player, side) {
	if (!side) return null;
	return {
		ankleToe: segmentLength(position, player, side.ankle, side.toe),
		ankleHeel: segmentLength(position, player, side.ankle, side.heel),
		toeHeel: segmentLength(position, player, side.toe, side.heel)
	};
}

function footSideForChain(chain) {
	return legSideForChain(chain);
}

