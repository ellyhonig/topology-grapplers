// Topology coordinate math: segment writhe, writhe matrices, target matrix synthesis, and matrix stepping.
// Keep this file focused: future agents should change this module for this system only.

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

