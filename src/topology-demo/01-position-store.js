// GrappleMap text decoding, database parsing, and fallback pose data.
// Keep this file focused: future agents should change this module for this system only.

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

