"""Flask + SocketIO server for the robot patrol simulator."""

from __future__ import annotations

import base64
import os
import sys
from dataclasses import asdict
from pathlib import Path

import io

import numpy as np
from flask import Flask, jsonify, request, send_file, send_from_directory
from PIL import Image
from flask_socketio import SocketIO, emit

from navigation.graph import NavGraph
from navigation.robot import Robot
from vision.grounding_dino import GroundingDINOProcessor

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

MESH_PATH: Path | None = None
NO_WALLS_MESH_PATH: Path | None = None
GRAPHS_DIR = Path(__file__).resolve().parent / "graphs"

# ---------------------------------------------------------------------------
# App setup
# ---------------------------------------------------------------------------

app = Flask(__name__, static_folder="static")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

nav_graph = NavGraph()
robot: Robot | None = None
vision: GroundingDINOProcessor | None = None


def _reset_robot() -> None:
    """(Re-)create the robot at the start node (or first node) of the current graph."""
    global robot
    nodes = nav_graph.nodes
    if nodes:
        start = nav_graph.start_node if nav_graph.start_node else next(iter(nodes))
        robot = Robot(nav_graph, start)
    else:
        robot = None


# ---------------------------------------------------------------------------
# HTTP routes
# ---------------------------------------------------------------------------


@app.route("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.route("/mesh.glb")
def serve_mesh():
    return send_file(str(MESH_PATH), mimetype="model/gltf-binary")


@app.route("/mesh_no_walls.glb")
def serve_mesh_no_walls():
    if NO_WALLS_MESH_PATH and NO_WALLS_MESH_PATH.exists():
        return send_file(str(NO_WALLS_MESH_PATH), mimetype="model/gltf-binary")
    return "No walls mesh not available", 404


# ── Graph CRUD ────────────────────────────────────────────────────────────


@app.route("/api/graph", methods=["GET"])
def get_graph():
    data = nav_graph.to_dict()
    data["start_node"] = nav_graph.start_node
    return jsonify(data)


@app.route("/api/graph", methods=["DELETE"])
def clear_graph():
    global nav_graph, robot
    nav_graph = NavGraph()
    robot = None
    return jsonify({"ok": True})


@app.route("/api/graph", methods=["PUT"])
def put_graph():
    global nav_graph
    data = request.get_json(force=True)
    nav_graph = NavGraph.from_dict(data)
    _reset_robot()
    return jsonify({"ok": True})


@app.route("/api/graph/node", methods=["POST"])
def add_node():
    data = request.get_json(force=True)
    node_id = data["id"]
    position = tuple(data["position"])
    nav_graph.add_node(node_id, position)
    return jsonify({"ok": True})


@app.route("/api/graph/node/<node_id>", methods=["PUT"])
def update_node(node_id: str):
    data = request.get_json(force=True)
    try:
        node = nav_graph.get_node(node_id)
    except KeyError as exc:
        return jsonify({"error": str(exc)}), 404
    nav_graph.add_node(node_id, tuple(data["position"]))
    return jsonify({"ok": True})


@app.route("/api/graph/node/<node_id>", methods=["DELETE"])
def delete_node(node_id: str):
    try:
        nav_graph.remove_node(node_id)
    except KeyError as exc:
        return jsonify({"error": str(exc)}), 404
    # If the robot was on the deleted node, reset it.
    if robot is not None and robot.current_node == node_id:
        _reset_robot()
    return jsonify({"ok": True})


@app.route("/api/graph/start_node", methods=["PUT"])
def set_start_node():
    data = request.get_json(force=True)
    node_id = data.get("node_id")
    if node_id and node_id not in nav_graph.nodes:
        return jsonify({"error": f"Node '{node_id}' not found"}), 404
    nav_graph.start_node = node_id
    _reset_robot()
    return jsonify({"ok": True})


@app.route("/api/graph/edge", methods=["POST"])
def add_edge():
    data = request.get_json(force=True)
    try:
        nav_graph.add_edge(data["from"], data["to"])
    except KeyError as exc:
        return jsonify({"error": str(exc)}), 404
    return jsonify({"ok": True})


# ── Graph persistence ────────────────────────────────────────────────────


@app.route("/api/graph/save", methods=["POST"])
def save_graph():
    data = request.get_json(force=True)
    name = data.get("name", "default")
    path = GRAPHS_DIR / f"{name}.json"
    nav_graph.save(path)
    return jsonify({"ok": True, "path": str(path)})


@app.route("/api/graph/load/<name>", methods=["POST"])
def load_graph(name: str):
    global nav_graph
    path = GRAPHS_DIR / f"{name}.json"
    if not path.exists():
        return jsonify({"error": f"Graph '{name}' not found"}), 404
    nav_graph = NavGraph.load(path)
    _reset_robot()
    return jsonify({"ok": True})


@app.route("/api/graphs", methods=["GET"])
def list_graphs():
    names = sorted(p.stem for p in GRAPHS_DIR.glob("*.json"))
    return jsonify(names)


@app.route("/api/graph/delete/<name>", methods=["DELETE"])
def delete_graph(name: str):
    global nav_graph, robot
    path = GRAPHS_DIR / f"{name}.json"
    if not path.exists():
        return jsonify({"error": f"Graph '{name}' not found"}), 404
    path.unlink()
    # Clear in-memory graph too
    nav_graph = NavGraph()
    robot = None
    return jsonify({"ok": True})


# ── Robot ─────────────────────────────────────────────────────────────────


@app.route("/api/robot", methods=["GET"])
def get_robot():
    if robot is None:
        return jsonify({"error": "No robot (graph has no nodes)"}), 400
    return jsonify(
        {
            "position": robot.position,
            "yaw": robot.yaw,
            "pitch": robot.pitch,
            "current_node": robot.current_node,
            "idle": robot.is_idle(),
        }
    )


# ---------------------------------------------------------------------------
# WebSocket events
# ---------------------------------------------------------------------------


@socketio.on("frame")
def handle_frame(data):
    """Receive a base64 JPEG frame, run YOLO, return detections."""
    if vision is None:
        emit("result", {"error": "Vision processor not loaded"})
        return

    image_data = data["image"]
    if "," in image_data:
        image_data = image_data.split(",", 1)[1]
    raw = base64.b64decode(image_data)
    img = Image.open(io.BytesIO(raw)).convert("RGB")
    frame = np.array(img)

    result = vision.process(frame)

    detections = [asdict(d) for d in result.detections]
    annotated_b64 = base64.b64encode(result.annotated_frame).decode("utf-8")

    emit(
        "result",
        {
            "detections": detections,
            "annotated_frame": f"data:image/jpeg;base64,{annotated_b64}",
        },
    )


@socketio.on("robot_command")
def handle_robot_command(data):
    """Set a target node for the robot and return the planned path."""
    if robot is None:
        emit("robot_path", {"error": "No robot (graph has no nodes)"})
        return

    target = data.get("target")
    if target is None:
        emit("robot_path", {"error": "Missing 'target' field"})
        return

    # Client can tell us where the robot currently is
    from_node = data.get("from_node")
    if from_node and from_node in nav_graph.nodes:
        robot.current_node = from_node

    path = robot.set_target(target)
    if path is None:
        emit("robot_path", {"error": f"No path to node '{target}'"})
    else:
        emit("robot_path", {"path": path})


# ---------------------------------------------------------------------------
# Startup
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python server.py <mesh_path> [no_walls_mesh_path]", file=sys.stderr)
        sys.exit(1)

    MESH_PATH = Path(sys.argv[1]).resolve()
    if not MESH_PATH.exists():
        print(f"Mesh file not found: {MESH_PATH}", file=sys.stderr)
        sys.exit(1)

    if len(sys.argv) >= 3:
        NO_WALLS_MESH_PATH = Path(sys.argv[2]).resolve()
        if not NO_WALLS_MESH_PATH.exists():
            print(f"No-walls mesh not found: {NO_WALLS_MESH_PATH}", file=sys.stderr)
            NO_WALLS_MESH_PATH = None

    GRAPHS_DIR.mkdir(exist_ok=True)

    print("Loading Grounding DINO model...")
    vision = GroundingDINOProcessor(
        model_id="IDEA-Research/grounding-dino-tiny",
        text_prompt="a person. a chair. a door. a table. a window.",
        confidence=0.3,
        text_threshold=0.25,
    )
    print("Grounding DINO model loaded.")

    print(f"Serving mesh from: {MESH_PATH}")
    print("Open http://localhost:5000 in your browser")
    socketio.run(app, host="0.0.0.0", port=5000, allow_unsafe_werkzeug=True)
