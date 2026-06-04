// Shared constants, mutable app state, and tiny math/DOM helpers.
// Keep this file focused: future agents should change this module for this system only.


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
var kosherMeshes = [];
var kosherMat = null;
var kosherEdgeMat = null;
var drag = null;
var kosherPreviewSteps = 10;

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

function emptyContactProof() {
	return { contacts: [], rejected: [], projected: [], count: 0, minClearance: Infinity };
}

