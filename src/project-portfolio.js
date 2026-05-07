(function () {
	"use strict";

	var nodes = [
		{
			id: "root",
			parent: null,
			type: "Project Root",
			title: "GrappleMap Agent Workbench",
			summary: "The full project context: a GrappleMap codebase, a topology-coordinate research node, and a runnable animation synthesis demo joined into one agent-oriented workflow.",
			pieces: ["GrappleMap.txt as the canonical position graph", "Browser rendering primitives in src/gm.js", "Knowledge-base notes and proof screenshots"],
			process: ["Use the tree to move from source understanding to research translation to implementation proof.", "Keep every derived artifact traceable to the node that produced it."],
			artifacts: [{ label: "Repo README", href: "../README.md" }]
		},
		{
			id: "codebase",
			parent: "root",
			type: "Knowledge Node",
			title: "GrappleMap Codebase Map",
			summary: "A concise operating guide for how GrappleMap stores, parses, renders, and edits grappling positions and transitions.",
			pieces: ["src/persistence.cpp base62 position decoder", "src/positions.hpp skeleton and reorientation model", "src/graph.hpp graph nodes and transition edges", "src/gm.js Babylon stick-figure renderer"],
			process: ["Established that GrappleMap positions are two players with 23 joints each.", "Identified a browser-first path for using the existing visual engine without requiring the full C++/Emscripten build."],
			artifacts: [{ label: "Open node", href: "../KNOWLEDGE_BASE.md" }, { label: "Open data", href: "../GrappleMap.txt" }]
		},
		{
			id: "paper",
			parent: "root",
			type: "Research Node",
			title: "Topology Coordinates Paper",
			summary: "The Ho-Komura Eurographics 2009 method translated into implementation notes for grappling: writhe, center, density, desired writhe matrices, and constrained synthesis.",
			pieces: ["Analytical line-segment Gauss Linking Integral", "Writhe matrix Ti,j for selected chain pairs", "Topology coordinates: writhe, center, density", "Quadratic-program style generalized-coordinate update"],
			process: ["Reduced the paper into implementable browser components.", "Mapped paper examples like wrestling holds and full-nelson-style paths onto GrappleMap limb chains."],
			artifacts: [{ label: "Open research node", href: "../KNOWLEDGE_BASE_TOPOLOGY_COORDINATES.md" }]
		},
		{
			id: "demo",
			parent: "root",
			type: "Implementation Node",
			title: "Topology Synthesis Demo",
			summary: "A standalone web demo that decodes GrappleMap positions, lets the user select limb chains, adjusts topology coordinates, and synthesizes updated joint frames.",
			pieces: ["Position selector from decoded GrappleMap records", "Chain selectors for red/blue arms, legs, spine/head", "Writhe, density, center, and frame controls", "Damped least-squares/QP-like solve with bone-length projection"],
			process: ["Reused GrappleMap's visual skeleton renderer for live feedback.", "Added current and desired writhe matrix heatmaps as visible proof of the topology-coordinate implementation."],
			artifacts: [{ label: "Open demo", href: "topology-demo.html" }, { label: "Open demo source", href: "topology-demo.js" }]
		},
		{
			id: "proof",
			parent: "demo",
			type: "Proof Node",
			title: "Rendered Visual Proof",
			summary: "Screenshots generated from headless Chrome showing that the page loads real GrappleMap data, renders the grappling skeletons, and displays topology-coordinate matrices and solver metrics.",
			pieces: ["topology-demo-proof.png before automatic synthesis", "topology-demo-proof-autosynth.png after solver run", "Metrics for current writhe, target writhe, residual, and solver step"],
			process: ["Verified browser rendering instead of only checking source files.", "Captured evidence that the matrix proof panels and synthesized pose appear in the actual page."],
			artifacts: [{ label: "Initial proof", href: "../topology-demo-proof.png" }, { label: "Autosynth proof", href: "../topology-demo-proof-autosynth.png" }]
		},
		{
			id: "next",
			parent: "root",
			type: "Next Node",
			title: "Agent Extensions",
			summary: "Future work that can turn this from a demo into a reusable GrappleMap agent capability.",
			pieces: ["Batch topology descriptors for every GrappleMap position", "Transition validation for impossible threading changes", "Suggested tags from geometric entanglement features", "Exported JSON reports for downstream agents"],
			process: ["Keep derived analysis outside GrappleMap.txt at first.", "Use advisory tags and validation warnings until the geometric features are calibrated against real grappling semantics."],
			artifacts: [{ label: "Research guidance", href: "../KNOWLEDGE_BASE_TOPOLOGY_COORDINATES.md" }]
		}
	];

	var tree = document.getElementById("treeNodes");
	var nodeType = document.getElementById("nodeType");
	var nodeTitle = document.getElementById("nodeTitle");
	var nodeSummary = document.getElementById("nodeSummary");
	var nodePieces = document.getElementById("nodePieces");
	var nodeProcess = document.getElementById("nodeProcess");
	var nodeArtifacts = document.getElementById("nodeArtifacts");

	function depthOf(node) {
		var depth = 0;
		var parent = node.parent;
		while (parent) {
			++depth;
			var found = nodes.find(function (n) { return n.id === parent; });
			parent = found ? found.parent : null;
		}
		return depth;
	}

	function renderList(target, items) {
		target.innerHTML = "";
		items.forEach(function (item) {
			var li = document.createElement("li");
			li.textContent = item;
			target.appendChild(li);
		});
	}

	function selectNode(id) {
		var node = nodes.find(function (n) { return n.id === id; }) || nodes[0];
		nodeType.textContent = node.type;
		nodeTitle.textContent = node.title;
		nodeSummary.textContent = node.summary;
		renderList(nodePieces, node.pieces);
		renderList(nodeProcess, node.process);
		nodeArtifacts.innerHTML = "";
		node.artifacts.forEach(function (artifact) {
			var a = document.createElement("a");
			a.href = artifact.href;
			a.textContent = artifact.label;
			nodeArtifacts.appendChild(a);
		});
		Array.prototype.forEach.call(tree.querySelectorAll("button"), function (button) {
			button.setAttribute("aria-current", button.dataset.id === id ? "true" : "false");
		});
	}

	function renderTree() {
		nodes.forEach(function (node) {
			var button = document.createElement("button");
			button.type = "button";
			button.dataset.id = node.id;
			button.className = "depth-" + Math.min(depthOf(node), 2);
			button.innerHTML = "<span class=\"dot\"></span><span><span class=\"tree-title\"></span><span class=\"tree-meta\"></span></span>";
			button.querySelector(".tree-title").textContent = node.title;
			button.querySelector(".tree-meta").textContent = node.type;
			button.addEventListener("click", function () { selectNode(node.id); });
			tree.appendChild(button);
		});
		selectNode("root");
	}

	renderTree();
}());
