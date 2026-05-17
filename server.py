"""Flask + SocketIO server for the robot patrol simulator."""

from __future__ import annotations

import base64
import io
import json
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np
from flask import Flask, abort, jsonify, request, send_file, send_from_directory
from PIL import Image
from flask_socketio import SocketIO, emit

from navigation.graph import NavGraph
from navigation.robot import Robot
from vision.grounding_dino import GroundingDINOProcessor

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

DEFAULT_SCENES_ROOT = Path("/home/hendrik/coding/engine/data/lidar/annotated")
SCENES_ROOT: Path = DEFAULT_SCENES_ROOT
GRAPHS_ROOT = Path(__file__).resolve().parent / "graphs"
# Fall-back when a scene has no in-tree `potree/` dir. Lets the old
# `static/pointclouds/<scene>/` clouds keep working for scenes we haven't
# rebuilt yet.
_LEGACY_PCD_DIR = Path(__file__).resolve().parent / "static" / "pointclouds"

# Per-scene optimized-mesh cache: scene_name -> served Path. Avoids re-running
# gltfpack on every `/mesh.glb` request.
_OPTIMIZED_CACHE: dict[str, Path] = {}


def _scene_dir(scene: str) -> Path:
    """Resolve `<SCENES_ROOT>/<scene>/`, rejecting traversal attempts."""
    if not scene or "/" in scene or ".." in scene:
        raise ValueError(f"invalid scene name: {scene!r}")
    return SCENES_ROOT / scene


def _list_scenes() -> list[str]:
    """Scenes are subdirs with a `source/mesh.glb` inside."""
    if not SCENES_ROOT.is_dir():
        return []
    return sorted(
        p.name for p in SCENES_ROOT.iterdir()
        if p.is_dir() and (p / "source" / "mesh.glb").exists()
    )


def _scene_from_request() -> str:
    """Pull `?scene=` from the current Flask request.

    Falls back to the most-recently-used scene (via `.last_used` mtime), then
    to the first scene alphabetically. Raises 400 if none exist.
    """
    name = request.args.get("scene", "").strip()
    if name:
        return name
    scenes = _list_scenes()
    if not scenes:
        abort(400, "no scenes available under " + str(SCENES_ROOT))
    # most recent last_used wins
    def _mtime(s: str) -> float:
        p = _scene_dir(s) / ".last_used"
        return p.stat().st_mtime if p.exists() else -1.0
    return max(scenes, key=_mtime)


def _touch_last_used(scene: str) -> None:
    p = _scene_dir(scene) / ".last_used"
    try:
        p.touch()
    except Exception:
        pass


def _scene_mesh_served_path(scene: str) -> Path:
    """Return the path to serve for `/mesh.glb`, optimizing on first hit."""
    cached = _OPTIMIZED_CACHE.get(scene)
    if cached is not None and cached.exists():
        return cached
    raw = _scene_dir(scene) / "source" / "mesh.glb"
    served = _maybe_optimize(raw) if raw.exists() else raw
    _OPTIMIZED_CACHE[scene] = served
    return served


def _scene_renders_dir(scene: str) -> Path:
    d = _scene_dir(scene) / "renders"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _scene_graphs_dir(scene: str) -> Path:
    """Graphs stay under walker/graphs/<scene>/ per design."""
    d = GRAPHS_ROOT / scene
    d.mkdir(parents=True, exist_ok=True)
    return d

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
    return _scene_graphs_dir(_scene_from_request()) / f"{name}.orbit.json"


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
    scene = _scene_from_request()
    served = _scene_mesh_served_path(scene)
    if not served.exists():
        return f"mesh.glb missing for scene {scene!r}", 404
    return _send_glb(served)


@app.route("/api/scenes", methods=["GET"])
def list_scenes():
    """List all scenes that have a `source/mesh.glb`.

    Each entry: {name, n_triangles, build_date, has_potree, last_used}.
    `last_used` is a unix ts (mtime of `.last_used` touchfile) or null.
    """
    out = []
    for name in _list_scenes():
        d = _scene_dir(name)
        meta_p = d / "source" / "mesh.meta.json"
        tris = build_date = None
        if meta_p.exists():
            try:
                meta = json.loads(meta_p.read_text())
                tris = int(meta.get("n_triangles") or 0) or None
                build_date = meta.get("build_date")  # may be absent
            except Exception:
                pass
        has_potree = (d / "potree" / "metadata.json").exists() or (
            _LEGACY_PCD_DIR / name / "metadata.json"
        ).exists()
        lu_p = d / ".last_used"
        last_used = lu_p.stat().st_mtime if lu_p.exists() else None
        out.append({
            "name": name,
            "n_triangles": tris,
            "build_date": build_date,
            "has_potree": has_potree,
            "last_used": last_used,
        })
    return jsonify(out)


@app.route("/scenes/<scene>/potree/<path:rest>")
def serve_scene_potree(scene: str, rest: str):
    """Serve files under `<scenes-root>/<scene>/potree/`.

    Returns 404 if the scene has no `potree/` dir. The frontend falls back to
    `/static/pointclouds/<scene>/...` for legacy clouds.
    """
    try:
        base = _scene_dir(scene) / "potree"
    except ValueError:
        return "invalid scene", 400
    if not base.is_dir():
        return "no potree for this scene", 404
    return send_from_directory(str(base), rest)


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
        nav_graph.get_node(node_id)
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
    path = _scene_graphs_dir(_scene_from_request()) / f"{name}.json"
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
    path = _scene_graphs_dir(_scene_from_request()) / f"{name}.json"
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
    names = sorted(p.stem for p in _scene_graphs_dir(_scene_from_request()).glob("*.json"))
    return jsonify(names)


@app.route("/api/scene_info", methods=["GET"])
def get_scene_info():
    """Return the active scene name plus a Potree pointcloud URL if one exists.

    Looks for the cloud at `<scene>/potree/metadata.json` first; falls back to
    the legacy `static/pointclouds/<scene>/metadata.json` if that's missing.
    Returns `null` for `pointcloud_url` when neither is present; the right-panel
    Potree view shows a placeholder in that case.

    Side effect: touches `<scene>/.last_used` so the dropdown can highlight the
    most-recent scene next time.
    """
    scene = _scene_from_request()
    _touch_last_used(scene)
    in_tree = _scene_dir(scene) / "potree" / "metadata.json"
    legacy = _LEGACY_PCD_DIR / scene / "metadata.json"
    if in_tree.exists():
        url = f"/scenes/{scene}/potree/metadata.json"
    elif legacy.exists():
        url = f"/static/pointclouds/{scene}/metadata.json"
    else:
        url = None
    return jsonify({"scene": scene, "pointcloud_url": url})


@app.route("/api/orientation", methods=["GET"])
def get_orientation():
    path = _scene_graphs_dir(_scene_from_request()) / "_orientation.json"
    if path.exists():
        return jsonify(json.loads(path.read_text()))
    return jsonify(None)


@app.route("/api/orientation", methods=["PUT"])
def put_orientation():
    data = request.get_json(force=True)
    path = _scene_graphs_dir(_scene_from_request()) / "_orientation.json"
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

    Body: ``{name, index, png_b64, pose}``. Writes to
    ``<scenes-root>/<scene>/renders/<name>/frame_NNNN.png`` and appends to
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

    scene = _scene_from_request()
    out_dir = _scene_renders_dir(scene) / name
    out_dir.mkdir(parents=True, exist_ok=True)

    frame_path = out_dir / f"frame_{index:04d}.png"
    frame_path.write_bytes(base64.b64decode(png_b64))

    manifest_path = out_dir / "manifest.json"
    manifest = {"name": name, "scene": scene, "frames": []}
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
    path = _scene_graphs_dir(_scene_from_request()) / f"{name}.json"
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
    import argparse

    parser = argparse.ArgumentParser(
        description="Robot patrol simulator server. Browses scenes under SCENES_ROOT.",
    )
    parser.add_argument(
        "--scenes-root",
        default=str(DEFAULT_SCENES_ROOT),
        help=f"directory containing scene subdirs with source/mesh.glb (default: {DEFAULT_SCENES_ROOT})",
    )
    parser.add_argument("--port", type=int, default=5050)
    parser.add_argument("--host", default="0.0.0.0")
    cli = parser.parse_args()

    SCENES_ROOT = Path(cli.scenes_root).resolve()
    if not SCENES_ROOT.is_dir():
        print(f"scenes-root not a directory: {SCENES_ROOT}", file=sys.stderr)
        sys.exit(1)

    scenes = _list_scenes()
    print(f"Scenes root: {SCENES_ROOT}")
    print(f"Scenes available ({len(scenes)}): {', '.join(scenes) if scenes else '<none>'}")

    # Vision disabled — frames will be passed through without detection.
    vision = None
    print("Vision processor disabled.")

    print(f"Open http://localhost:{cli.port} in your browser")
    socketio.run(app, host=cli.host, port=cli.port, allow_unsafe_werkzeug=True)
