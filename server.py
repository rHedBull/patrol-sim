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
from vision.yolo import YOLOProcessor

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

MESH_PATH: Path | None = None
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
vision: YOLOProcessor | None = None


def _reset_robot() -> None:
    """(Re-)create the robot at the first node of the current graph."""
    global robot
    nodes = nav_graph.nodes
    if nodes:
        first_node = next(iter(nodes))
        robot = Robot(nav_graph, first_node)
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


# ── Graph CRUD ────────────────────────────────────────────────────────────


@app.route("/api/graph", methods=["GET"])
def get_graph():
    return jsonify(nav_graph.to_dict())


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
        print("Usage: python server.py <mesh_path>", file=sys.stderr)
        sys.exit(1)

    MESH_PATH = Path(sys.argv[1]).resolve()
    if not MESH_PATH.exists():
        print(f"Mesh file not found: {MESH_PATH}", file=sys.stderr)
        sys.exit(1)

    GRAPHS_DIR.mkdir(exist_ok=True)

    print("Loading YOLO model...")
    vision = YOLOProcessor(model_name="yolov8n.pt", confidence=0.25)
    print("YOLO model loaded.")

    print(f"Serving mesh from: {MESH_PATH}")
    print("Open http://localhost:5000 in your browser")
    socketio.run(app, host="0.0.0.0", port=5000, allow_unsafe_werkzeug=True)
