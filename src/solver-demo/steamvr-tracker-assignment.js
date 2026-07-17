// Choose the two lowest tracked devices, then classify them along the
// headset's horizontal right axis. Samples are returned intact so callers can
// retain their WebXR registration and pose data.
export function isFootTrackerInputSource(inputSource) {
	return !!(inputSource &&
		inputSource.handedness === "none" &&
		inputSource.targetRayMode === "tracked-pointer" &&
		!inputSource.hand);
}

export function chooseFootTrackerPair(samples, headsetPosition, rightAxis) {
	if (!headsetPosition || !rightAxis || samples.length < 2) return null;
	var selected = samples.slice().sort(function (a, b) {
		return a.pose.position.y - b.pose.position.y;
	}).slice(0, 2);

	function lateralPosition(sample) {
		var position = sample.pose.position;
		return (position.x - headsetPosition.x) * rightAxis.x +
			(position.y - headsetPosition.y) * rightAxis.y +
			(position.z - headsetPosition.z) * rightAxis.z;
	}

	selected.sort(function (a, b) {
		return lateralPosition(a) - lateralPosition(b);
	});
	return { left: selected[0], right: selected[1] };
}
