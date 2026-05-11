"""Flask + SocketIO server for the robot patrol simulator."""

from __future__ import annotations

import base64
import json
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
GRAPHS_ROOT = Path(__file__).resolve().parent / "graphs"
GRAPHS_DIR: Path = GRAPHS_ROOT  # set per-mesh in __main__
RENDERS_ROOT = Path(__file__).resolve().parent / "renders"
RENDERS_DIR: Path = RENDERS_ROOT  # set per-scene in __main__

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
# Orbit definition tied to the *currently loaded* graph. Persisted as a
# sidecar (`<name>.orbit.json`) next to the graph json so non-orbit
# graphs stay zero-overhead. Schema:
#   {tip_node_id: str, radius: float, height: float,
#    samples: int, start_theta_deg: float}
current_orbit: dict | None = None


def _orbit_path(name: str) -> Path:
    return GRAPHS_DIR / f"{name}.orbit.json"


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


def _send_glb(path: Path):
    # conditional=True -> ETag/304; max_age caches across reloads.
    resp = send_file(
        str(path),
        mimetype="model/gltf-binary",
        conditional=True,
        max_age=3600,
    )
    resp.headers["Cache-Control"] = "public, max-age=3600"
    return resp


@app.route("/mesh.glb")
def serve_mesh():
    assert MESH_PATH is not None
    return _send_glb(MESH_PATH)


@app.route("/mesh_no_walls.glb")
def serve_mesh_no_walls():
    if NO_WALLS_MESH_PATH and NO_WALLS_MESH_PATH.exists():
        return _send_glb(NO_WALLS_MESH_PATH)
    return "No walls mesh not available", 404


# ── Graph CRUD ────────────────────────────────────────────────────────────


@app.route("/api/graph", methods=["GET"])
def get_graph():
    data = nav_graph.to_dict()
    data["start_node"] = nav_graph.start_node
    data["orbit"] = current_orbit
    return jsonify(data)


@app.route("/api/graph", methods=["DELETE"])
def clear_graph():
    global nav_graph, robot, current_orbit
    nav_graph = NavGraph()
    robot = None
    current_orbit = None
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


@app.route("/api/graph/edge", methods=["DELETE"])
def delete_edge():
    data = request.get_json(force=True)
    nav_graph.remove_edge(data["from"], data["to"])
    return jsonify({"ok": True})


# ── Graph persistence ────────────────────────────────────────────────────


@app.route("/api/graph/save", methods=["POST"])
def save_graph():
    global current_orbit
    data = request.get_json(force=True)
    name = data.get("name", "default")
    path = GRAPHS_DIR / f"{name}.json"
    nav_graph.save(path)
    # Orbit sidecar: present in body → write + remember. Explicit null or
    # missing → drop the sidecar so a saved waypoint graph doesn't keep
    # stale orbit metadata around.
    orbit = data.get("orbit", "missing")
    sidecar = _orbit_path(name)
    if orbit and orbit != "missing":
        sidecar.write_text(json.dumps(orbit, indent=2))
        current_orbit = orbit
    else:
        if sidecar.exists():
            sidecar.unlink()
        current_orbit = None
    return jsonify({"ok": True, "path": str(path)})


@app.route("/api/graph/load/<name>", methods=["POST"])
def load_graph(name: str):
    global nav_graph, current_orbit
    path = GRAPHS_DIR / f"{name}.json"
    if not path.exists():
        return jsonify({"error": f"Graph '{name}' not found"}), 404
    nav_graph = NavGraph.load(path)
    sidecar = _orbit_path(name)
    if sidecar.exists():
        try:
            current_orbit = json.loads(sidecar.read_text())
        except Exception as e:  # noqa: BLE001 — bad sidecar shouldn't kill load
            print(f"warn: failed to read orbit sidecar {sidecar}: {e}")
            current_orbit = None
    else:
        current_orbit = None
    _reset_robot()
    return jsonify({"ok": True})


@app.route("/api/graphs", methods=["GET"])
def list_graphs():
    names = sorted(p.stem for p in GRAPHS_DIR.glob("*.json"))
    return jsonify(names)


@app.route("/api/scene_info", methods=["GET"])
def get_scene_info():
    """Return the active scene name plus a Potree pointcloud URL if one exists.

    Pointclouds are looked up at ``static/pointclouds/<scene>/metadata.json``
    (the output of PotreeConverter v2). When absent, ``pointcloud_url`` is None
    and the right-panel Potree view shows a placeholder.
    """
    scene_name = GRAPHS_DIR.name
    static_dir = Path(__file__).resolve().parent / "static"
    pc_meta = static_dir / "pointclouds" / scene_name / "metadata.json"
    pointcloud_url = (
        f"/static/pointclouds/{scene_name}/metadata.json" if pc_meta.exists() else None
    )
    return jsonify({"scene": scene_name, "pointcloud_url": pointcloud_url})


@app.route("/api/orientation", methods=["GET"])
def get_orientation():
    path = GRAPHS_DIR / "_orientation.json"
    if path.exists():
        return jsonify(json.loads(path.read_text()))
    return jsonify(None)


@app.route("/api/orientation", methods=["PUT"])
def put_orientation():
    data = request.get_json(force=True)
    path = GRAPHS_DIR / "_orientation.json"
    path.write_text(json.dumps(data))
    return jsonify({"ok": True})


@app.route("/api/plan/path", methods=["POST"])
def plan_path():
    """Resolve a list of waypoints into the full chained A* polyline.

    Returns ``{path: [{id, position:[x,y,z]}, ...]}`` with consecutive legs
    glued (the shared join node is not duplicated). Used by the headless and
    interactive renderers to compute arc-length samples without driving the
    robot.
    """
    data = request.get_json(force=True)
    waypoints = data.get("waypoints") or []
    if len(waypoints) < 2:
        return jsonify({"error": "Need at least 2 waypoints"}), 400

    full: list[str] = []
    for a, b in zip(waypoints, waypoints[1:]):
        leg = nav_graph.find_path(a, b)
        if leg is None:
            return jsonify({"error": f"No path from '{a}' to '{b}'"}), 400
        if full and full[-1] == leg[0]:
            full.extend(leg[1:])
        else:
            full.extend(leg)

    nodes = nav_graph.nodes
    return jsonify({"path": [{"id": nid, "position": nodes[nid]} for nid in full]})


@app.route("/api/render_frame", methods=["POST"])
def save_render_frame():
    """Persist a single rendered frame coming from the in-page renderer.

    Body: ``{name, index, png_b64, pose}``. Writes
    ``renders/<scene>/<name>/frame_NNNN.png`` and appends to
    ``manifest.json`` in the same directory.
    """
    data = request.get_json(force=True)
    name = (data.get("name") or "").strip()
    if not name or "/" in name or ".." in name:
        return jsonify({"error": "Invalid render name"}), 400
    index = int(data.get("index", 0))
    png_b64 = data.get("png_b64") or ""
    if "," in png_b64:
        png_b64 = png_b64.split(",", 1)[1]
    if not png_b64:
        return jsonify({"error": "Missing png_b64"}), 400

    out_dir = RENDERS_DIR / name
    out_dir.mkdir(parents=True, exist_ok=True)

    frame_path = out_dir / f"frame_{index:04d}.png"
    frame_path.write_bytes(base64.b64decode(png_b64))

    manifest_path = out_dir / "manifest.json"
    manifest = {"name": name, "scene": RENDERS_DIR.name, "frames": []}
    if manifest_path.exists():
        try:
            manifest = json.loads(manifest_path.read_text())
        except Exception:
            pass
    pose = data.get("pose") or {}
    entry = {"index": index, "file": frame_path.name, **pose}
    # Replace by index if it already exists
    manifest["frames"] = [f for f in manifest.get("frames", []) if f.get("index") != index]
    manifest["frames"].append(entry)
    manifest["frames"].sort(key=lambda f: f["index"])
    manifest_path.write_text(json.dumps(manifest, indent=2))

    return jsonify({"ok": True, "path": str(frame_path)})


@app.route("/api/graph/delete/<name>", methods=["DELETE"])
def delete_graph(name: str):
    global nav_graph, robot, current_orbit
    path = GRAPHS_DIR / f"{name}.json"
    if not path.exists():
        return jsonify({"error": f"Graph '{name}' not found"}), 404
    path.unlink()
    sidecar = _orbit_path(name)
    if sidecar.exists():
        sidecar.unlink()
    # Clear in-memory graph too
    nav_graph = NavGraph()
    robot = None
    current_orbit = None
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
    image_data = data["image"]
    if "," in image_data:
        image_data = image_data.split(",", 1)[1]

    if vision is None:
        # Passthrough: echo the raw frame back with no detections.
        emit("result", {"detections": [], "annotated_frame": f"data:image/jpeg;base64,{image_data}"})
        return

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


@socketio.on("plan_command")
def handle_plan_command(data):
    """Drive the robot through an ordered list of waypoints.

    Runs A* between consecutive waypoints, concatenates the legs (without
    duplicating the shared join node), and emits the full path back over the
    same `robot_path` channel that interactive mode uses.
    """
    if robot is None:
        emit("robot_path", {"error": "No robot (graph has no nodes)"})
        return

    waypoints = data.get("waypoints") or []
    if len(waypoints) < 2:
        emit("robot_path", {"error": "Plan needs at least 2 waypoints"})
        return

    full: list[str] = []
    for a, b in zip(waypoints, waypoints[1:]):
        leg = nav_graph.find_path(a, b)
        if leg is None:
            emit("robot_path", {"error": f"No path from '{a}' to '{b}'"})
            return
        if full and full[-1] == leg[0]:
            full.extend(leg[1:])
        else:
            full.extend(leg)

    # Sync robot to plan start so movement state is consistent.
    robot.current_node = full[0]
    robot._path = list(full)
    robot._path_index = 1
    emit("robot_path", {"path": full})


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

def _maybe_optimize(path: Path, *, threshold_mb: float = 50.0) -> Path:
    """For large GLBs, return a cached Meshopt-compressed sibling.

    Skips optimization if the file is already small or the path itself looks
    optimized. The optimized cache lives next to the source as
    ``<stem>.optimized.glb`` so it persists across runs.
    """
    if path.stem.endswith(".optimized") or path.name.endswith(".optimized.glb"):
        return path
    size_mb = path.stat().st_size / (1024 * 1024)
    if size_mb < threshold_mb:
        return path

    from scripts.optimize_mesh import default_output, optimize  # local import: optional dep on npx

    cached = default_output(path)
    print(f"[server] mesh is {size_mb:.0f} MB -> producing/loading optimized cache: {cached}")
    try:
        return optimize(path, cached)
    except Exception as exc:  # pragma: no cover
        print(f"[server] optimization failed ({exc}); serving raw mesh", file=sys.stderr)
        return path


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    if not args:
        print("Usage: python server.py [--raw] <mesh_path> [no_walls_mesh_path]", file=sys.stderr)
        sys.exit(1)

    raw_mode = "--raw" in flags

    raw_mesh_path = Path(args[0]).resolve()
    if not raw_mesh_path.exists():
        print(f"Mesh file not found: {raw_mesh_path}", file=sys.stderr)
        sys.exit(1)
    MESH_PATH = raw_mesh_path if raw_mode else _maybe_optimize(raw_mesh_path)

    if len(args) >= 2:
        raw_no_walls = Path(args[1]).resolve()
        if not raw_no_walls.exists():
            print(f"No-walls mesh not found: {raw_no_walls}", file=sys.stderr)
            NO_WALLS_MESH_PATH = None
        else:
            NO_WALLS_MESH_PATH = raw_no_walls if raw_mode else _maybe_optimize(raw_no_walls)

    # Namespace graphs per mesh so each scene has its own collection.
    # Use the raw (pre-optimization) path so the cached `.optimized` suffix
    # doesn't fragment graph storage.
    scene_name = raw_mesh_path.stem
    if scene_name in {"mesh", "scene"} and raw_mesh_path.parent.name == "source":
        scene_name = raw_mesh_path.parent.parent.name
    elif scene_name in {"mesh", "scene"}:
        scene_name = raw_mesh_path.parent.name
    GRAPHS_DIR = GRAPHS_ROOT / scene_name
    GRAPHS_DIR.mkdir(parents=True, exist_ok=True)
    RENDERS_DIR = RENDERS_ROOT / scene_name
    RENDERS_DIR.mkdir(parents=True, exist_ok=True)
    print(f"Graphs dir: {GRAPHS_DIR}")
    print(f"Renders dir: {RENDERS_DIR}")

    # Vision disabled — frames will be passed through without detection.
    vision = None
    print("Vision processor disabled.")

    print(f"Serving mesh from: {MESH_PATH}")
    print("Open http://localhost:5000 in your browser")
    socketio.run(app, host="0.0.0.0", port=5000, allow_unsafe_werkzeug=True)
