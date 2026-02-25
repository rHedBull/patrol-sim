# Robot Patrol Simulator — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a robot patrol simulator that navigates a 3D scene via a graph, streams its camera POV through a pluggable vision pipeline (YOLO), and displays results in a split-view UI.

**Architecture:** New project `robot-patrol-sim/` at `../robot-patrol-sim`. Flask+SocketIO backend serves the mesh and handles WebSocket frame streaming. Three.js frontend renders split view: orbit scene overview (left) + robot camera with YOLO overlays (right). Navigation graph with A* pathfinding drives robot movement.

**Tech Stack:** Python (Flask, flask-socketio, ultralytics, numpy, pillow), Three.js (CDN), WebSocket (socket.io)

**Reference project:** `../indoor-walk` — reuse patterns for mesh loading, Catmull-Rom splines, scene setup, click-to-place editing.

---

### Task 1: Project scaffold and dependencies

**Files:**
- Create: `../robot-patrol-sim/pyproject.toml`
- Create: `../robot-patrol-sim/vision/__init__.py`
- Create: `../robot-patrol-sim/navigation/__init__.py`
- Create: `../robot-patrol-sim/graphs/.gitkeep`

**Step 1: Create project directory and pyproject.toml**

```toml
[project]
name = "robot-patrol-sim"
version = "0.1.0"
description = "Robot patrol simulator with pluggable vision pipeline"
requires-python = ">=3.12"
dependencies = [
    "flask>=3.1.2",
    "flask-socketio>=5.3.0",
    "ultralytics>=8.0.0",
    "numpy>=1.26.0",
    "pillow>=10.0.0",
]
```

**Step 2: Create empty package dirs**

```bash
mkdir -p ../robot-patrol-sim/{vision,navigation,static,graphs,docs/plans}
touch ../robot-patrol-sim/vision/__init__.py
touch ../robot-patrol-sim/navigation/__init__.py
touch ../robot-patrol-sim/graphs/.gitkeep
```

**Step 3: Initialize git and install deps**

```bash
cd ../robot-patrol-sim
git init
uv sync  # or pip install -e .
```

**Step 4: Commit**

```bash
git add -A
git commit -m "chore: scaffold project with dependencies"
```

---

### Task 2: Navigation graph data model and A* pathfinding

**Files:**
- Create: `../robot-patrol-sim/navigation/graph.py`
- Create: `../robot-patrol-sim/tests/test_graph.py`

**Step 1: Write the failing tests**

```python
# tests/test_graph.py
import json
import pytest
from navigation.graph import NavGraph

def test_add_node():
    g = NavGraph()
    g.add_node("n1", [1.0, 0.0, 2.0])
    assert g.get_node("n1") == {"id": "n1", "position": [1.0, 0.0, 2.0]}

def test_add_edge():
    g = NavGraph()
    g.add_node("n1", [0, 0, 0])
    g.add_node("n2", [1, 0, 0])
    g.add_edge("n1", "n2")
    assert "n2" in g.neighbors("n1")
    assert "n1" in g.neighbors("n2")  # bidirectional

def test_remove_node_removes_edges():
    g = NavGraph()
    g.add_node("n1", [0, 0, 0])
    g.add_node("n2", [1, 0, 0])
    g.add_edge("n1", "n2")
    g.remove_node("n1")
    assert g.get_node("n1") is None
    assert "n1" not in g.neighbors("n2")

def test_astar_shortest_path():
    g = NavGraph()
    g.add_node("a", [0, 0, 0])
    g.add_node("b", [1, 0, 0])
    g.add_node("c", [2, 0, 0])
    g.add_node("d", [1, 0, 1])  # detour
    g.add_edge("a", "b")
    g.add_edge("b", "c")
    g.add_edge("a", "d")
    g.add_edge("d", "c")
    path = g.find_path("a", "c")
    assert path == ["a", "b", "c"]  # straight line is shorter

def test_astar_no_path():
    g = NavGraph()
    g.add_node("a", [0, 0, 0])
    g.add_node("b", [1, 0, 0])
    # no edge
    assert g.find_path("a", "b") is None

def test_serialize_roundtrip():
    g = NavGraph()
    g.add_node("n1", [0, 0, 0])
    g.add_node("n2", [1, 0, 0])
    g.add_edge("n1", "n2")
    data = g.to_dict()
    g2 = NavGraph.from_dict(data)
    assert g2.get_node("n1") == g.get_node("n1")
    assert "n2" in g2.neighbors("n1")
```

**Step 2: Run tests to verify they fail**

```bash
cd ../robot-patrol-sim
python -m pytest tests/test_graph.py -v
```

Expected: FAIL — `ModuleNotFoundError: No module named 'navigation.graph'`

**Step 3: Implement NavGraph**

```python
# navigation/graph.py
"""Navigation graph with A* pathfinding."""

from __future__ import annotations
import heapq
import json
import math
from pathlib import Path


class NavGraph:
    def __init__(self):
        self.nodes: dict[str, list[float]] = {}  # id -> [x, y, z]
        self.edges: dict[str, set[str]] = {}  # id -> set of neighbor ids

    def add_node(self, node_id: str, position: list[float]):
        self.nodes[node_id] = position
        if node_id not in self.edges:
            self.edges[node_id] = set()

    def remove_node(self, node_id: str):
        if node_id in self.nodes:
            del self.nodes[node_id]
        # Remove from all neighbor lists
        neighbors = self.edges.pop(node_id, set())
        for n in neighbors:
            self.edges.get(n, set()).discard(node_id)

    def get_node(self, node_id: str) -> dict | None:
        if node_id not in self.nodes:
            return None
        return {"id": node_id, "position": self.nodes[node_id]}

    def add_edge(self, from_id: str, to_id: str):
        self.edges.setdefault(from_id, set()).add(to_id)
        self.edges.setdefault(to_id, set()).add(from_id)

    def remove_edge(self, from_id: str, to_id: str):
        self.edges.get(from_id, set()).discard(to_id)
        self.edges.get(to_id, set()).discard(from_id)

    def neighbors(self, node_id: str) -> set[str]:
        return self.edges.get(node_id, set())

    def _dist(self, a: str, b: str) -> float:
        pa, pb = self.nodes[a], self.nodes[b]
        return math.sqrt(sum((pa[i] - pb[i]) ** 2 for i in range(3)))

    def find_path(self, start: str, goal: str) -> list[str] | None:
        """A* shortest path. Returns list of node IDs or None."""
        if start not in self.nodes or goal not in self.nodes:
            return None

        open_set = [(0.0, start)]
        came_from: dict[str, str] = {}
        g_score: dict[str, float] = {start: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return list(reversed(path))

            for neighbor in self.neighbors(current):
                tentative = g_score[current] + self._dist(current, neighbor)
                if tentative < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f = tentative + self._dist(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        return None

    def to_dict(self) -> dict:
        return {
            "nodes": [{"id": k, "position": v} for k, v in self.nodes.items()],
            "edges": [{"from": a, "to": b} for a in self.edges for b in self.edges[a] if a < b],
        }

    @classmethod
    def from_dict(cls, data: dict) -> NavGraph:
        g = cls()
        for node in data.get("nodes", []):
            g.add_node(node["id"], node["position"])
        for edge in data.get("edges", []):
            g.add_edge(edge["from"], edge["to"])
        return g

    def save(self, path: str | Path):
        Path(path).write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def load(cls, path: str | Path) -> NavGraph:
        return cls.from_dict(json.loads(Path(path).read_text()))
```

**Step 4: Run tests to verify they pass**

```bash
python -m pytest tests/test_graph.py -v
```

Expected: All 6 PASS

**Step 5: Commit**

```bash
git add navigation/graph.py tests/test_graph.py
git commit -m "feat: navigation graph with A* pathfinding"
```

---

### Task 3: Robot state and movement

**Files:**
- Create: `../robot-patrol-sim/navigation/robot.py`
- Create: `../robot-patrol-sim/tests/test_robot.py`

**Step 1: Write the failing tests**

```python
# tests/test_robot.py
from navigation.graph import NavGraph
from navigation.robot import Robot

def test_robot_initial_state():
    g = NavGraph()
    g.add_node("n1", [0, 0, 0])
    robot = Robot(g, start_node="n1")
    assert robot.position == [0, 0, 0]
    assert robot.current_node == "n1"
    assert robot.is_idle()

def test_robot_set_target():
    g = NavGraph()
    g.add_node("a", [0, 0, 0])
    g.add_node("b", [3, 0, 0])
    g.add_edge("a", "b")
    robot = Robot(g, start_node="a")
    path = robot.set_target("b")
    assert path == ["a", "b"]
    assert not robot.is_idle()

def test_robot_step_moves_toward_target():
    g = NavGraph()
    g.add_node("a", [0, 0, 0])
    g.add_node("b", [3, 0, 0])
    g.add_edge("a", "b")
    robot = Robot(g, start_node="a", speed=1.0)
    robot.set_target("b")
    robot.step(1.0)  # 1 second at speed 1.0 = move 1 unit
    assert robot.position[0] > 0.0
    assert robot.position[0] < 3.0  # not arrived yet

def test_robot_arrives_at_target():
    g = NavGraph()
    g.add_node("a", [0, 0, 0])
    g.add_node("b", [1, 0, 0])
    g.add_edge("a", "b")
    robot = Robot(g, start_node="a", speed=2.0)
    robot.set_target("b")
    robot.step(1.0)  # 1s at 2.0 = 2 units, enough to reach b at distance 1
    assert robot.current_node == "b"
    assert robot.is_idle()
```

**Step 2: Run tests to verify they fail**

```bash
python -m pytest tests/test_robot.py -v
```

**Step 3: Implement Robot**

```python
# navigation/robot.py
"""Robot state and movement along navigation graph."""

from __future__ import annotations
import math
from navigation.graph import NavGraph


class Robot:
    def __init__(self, graph: NavGraph, start_node: str, speed: float = 2.0):
        self.graph = graph
        self.current_node = start_node
        self.position = list(graph.nodes[start_node])
        self.speed = speed
        self.yaw = 0.0
        self.pitch = 0.0

        self._path: list[str] = []  # remaining node IDs to visit
        self._target_pos: list[float] | None = None

    def is_idle(self) -> bool:
        return len(self._path) == 0 and self._target_pos is None

    def set_target(self, target_node: str) -> list[str] | None:
        """Set destination. Returns planned path or None if unreachable."""
        path = self.graph.find_path(self.current_node, target_node)
        if path is None:
            return None
        self._path = path[1:]  # skip current node
        if self._path:
            self._target_pos = list(self.graph.nodes[self._path[0]])
        return path

    def step(self, dt: float):
        """Advance robot by dt seconds. Call each frame."""
        if self._target_pos is None:
            if self._path:
                self._target_pos = list(self.graph.nodes[self._path[0]])
            else:
                return

        dx = self._target_pos[0] - self.position[0]
        dy = self._target_pos[1] - self.position[1]
        dz = self._target_pos[2] - self.position[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        move = self.speed * dt

        if move >= dist:
            # Arrived at next node
            self.position = list(self._target_pos)
            self.current_node = self._path.pop(0)
            self._target_pos = None

            # Continue to next node if path remains
            remaining = move - dist
            if self._path and remaining > 0:
                self._target_pos = list(self.graph.nodes[self._path[0]])
                self.step(remaining / self.speed)
        else:
            # Move toward target
            ratio = move / dist
            self.position[0] += dx * ratio
            self.position[1] += dy * ratio
            self.position[2] += dz * ratio

        # Update yaw to face movement direction
        if dist > 0.001:
            self.yaw = math.atan2(-dx, -dz)
```

**Step 4: Run tests**

```bash
python -m pytest tests/test_robot.py -v
```

Expected: All 4 PASS

**Step 5: Commit**

```bash
git add navigation/robot.py tests/test_robot.py
git commit -m "feat: robot state and movement along graph"
```

---

### Task 4: Vision pipeline interface + YOLO processor

**Files:**
- Create: `../robot-patrol-sim/vision/base.py`
- Create: `../robot-patrol-sim/vision/yolo.py`
- Create: `../robot-patrol-sim/tests/test_vision.py`

**Step 1: Write the failing tests**

```python
# tests/test_vision.py
import numpy as np
from vision.base import VisionProcessor, ProcessorResult, Detection

def test_processor_result_structure():
    det = Detection(label="person", confidence=0.95, bbox=[10, 20, 100, 200])
    result = ProcessorResult(detections=[det], annotated_frame=b"fake_png")
    assert result.detections[0].label == "person"
    assert result.annotated_frame == b"fake_png"

def test_base_processor_raises():
    proc = VisionProcessor()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    try:
        proc.process(frame)
        assert False, "Should raise NotImplementedError"
    except NotImplementedError:
        pass
```

**Step 2: Run tests to verify they fail**

```bash
python -m pytest tests/test_vision.py -v
```

**Step 3: Implement base classes**

```python
# vision/base.py
"""Pluggable vision pipeline interface."""

from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np


@dataclass
class Detection:
    label: str
    confidence: float
    bbox: list[float]  # [x1, y1, x2, y2]


@dataclass
class ProcessorResult:
    detections: list[Detection] = field(default_factory=list)
    annotated_frame: bytes = b""


class VisionProcessor:
    """Base class for vision processors. Subclass and override process()."""

    def process(self, frame: np.ndarray) -> ProcessorResult:
        raise NotImplementedError
```

**Step 4: Run tests**

```bash
python -m pytest tests/test_vision.py -v
```

Expected: All 2 PASS

**Step 5: Implement YOLO processor**

```python
# vision/yolo.py
"""YOLO vision processor using ultralytics."""

from __future__ import annotations
import io
import numpy as np
from PIL import Image
from vision.base import VisionProcessor, ProcessorResult, Detection


class YOLOProcessor(VisionProcessor):
    def __init__(self, model_name: str = "yolov8n.pt", confidence: float = 0.25):
        from ultralytics import YOLO
        self.model = YOLO(model_name)
        self.confidence = confidence

    def process(self, frame: np.ndarray) -> ProcessorResult:
        results = self.model(frame, conf=self.confidence, verbose=False)
        result = results[0]

        detections = []
        for box in result.boxes:
            detections.append(Detection(
                label=result.names[int(box.cls[0])],
                confidence=float(box.conf[0]),
                bbox=box.xyxy[0].tolist(),
            ))

        # Get annotated frame as JPEG bytes
        annotated = result.plot()
        img = Image.fromarray(annotated)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=80)
        annotated_bytes = buf.getvalue()

        return ProcessorResult(detections=detections, annotated_frame=annotated_bytes)
```

**Step 6: Commit**

```bash
git add vision/base.py vision/yolo.py tests/test_vision.py
git commit -m "feat: pluggable vision pipeline with YOLO processor"
```

---

### Task 5: Flask + SocketIO server

**Files:**
- Create: `../robot-patrol-sim/server.py`

**Step 1: Implement server**

Reuse patterns from `../indoor-walk/server.py` for mesh serving and graph CRUD. Add SocketIO for frame streaming.

```python
# server.py
"""Flask + SocketIO server for robot patrol simulator."""

import base64
import io
import json
import sys
from pathlib import Path

import numpy as np
from flask import Flask, jsonify, request, send_file, send_from_directory
from flask_socketio import SocketIO, emit
from PIL import Image

from navigation.graph import NavGraph
from navigation.robot import Robot
from vision.base import VisionProcessor
from vision.yolo import YOLOProcessor

app = Flask(__name__, static_folder="static")
app.config["SECRET_KEY"] = "robot-patrol-sim"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

MESH_PATH: str | None = None
GRAPHS_DIR = Path("graphs")

# Initialized on startup
vision_processor: VisionProcessor | None = None
nav_graph: NavGraph | None = None
robot: Robot | None = None


@app.route("/")
def index():
    return send_from_directory("static", "index.html")


@app.route("/mesh.glb")
def mesh():
    if MESH_PATH and Path(MESH_PATH).exists():
        return send_file(MESH_PATH, mimetype="model/gltf-binary")
    return "Mesh not found", 404


# --- Graph Management API ---

@app.route("/api/graph", methods=["GET"])
def get_graph():
    if nav_graph is None:
        return jsonify({"nodes": [], "edges": []})
    return jsonify(nav_graph.to_dict())


@app.route("/api/graph", methods=["PUT"])
def update_graph():
    global nav_graph, robot
    data = request.get_json()
    nav_graph = NavGraph.from_dict(data)
    # Reset robot to first node if graph changed
    if nav_graph.nodes:
        first_node = next(iter(nav_graph.nodes))
        robot = Robot(nav_graph, start_node=first_node)
    return jsonify(nav_graph.to_dict())


@app.route("/api/graph/save", methods=["POST"])
def save_graph():
    data = request.get_json()
    name = data.get("name", "graph")
    GRAPHS_DIR.mkdir(exist_ok=True)
    if nav_graph:
        nav_graph.save(GRAPHS_DIR / f"{name}.json")
    return jsonify({"status": "saved"})


@app.route("/api/graph/load/<name>", methods=["POST"])
def load_graph(name):
    global nav_graph, robot
    path = GRAPHS_DIR / f"{name}.json"
    if not path.exists():
        return jsonify({"error": "Not found"}), 404
    nav_graph = NavGraph.load(path)
    if nav_graph.nodes:
        first_node = next(iter(nav_graph.nodes))
        robot = Robot(nav_graph, start_node=first_node)
    return jsonify(nav_graph.to_dict())


@app.route("/api/graphs", methods=["GET"])
def list_graphs():
    GRAPHS_DIR.mkdir(exist_ok=True)
    return jsonify([f.stem for f in sorted(GRAPHS_DIR.glob("*.json"))])


# --- Robot Control API ---

@app.route("/api/robot", methods=["GET"])
def get_robot_state():
    if robot is None:
        return jsonify({"error": "No robot"}), 404
    return jsonify({
        "position": robot.position,
        "yaw": robot.yaw,
        "pitch": robot.pitch,
        "current_node": robot.current_node,
        "idle": robot.is_idle(),
    })


# --- WebSocket Events ---

@socketio.on("frame")
def handle_frame(data):
    """Receive robot camera frame, run through vision pipeline, return results."""
    if vision_processor is None:
        emit("result", {"error": "No vision processor"})
        return

    # Decode base64 JPEG
    image_data = data.get("image", "")
    if "," in image_data:
        image_data = image_data.split(",", 1)[1]

    img_bytes = base64.b64decode(image_data)
    img = Image.open(io.BytesIO(img_bytes)).convert("RGB")
    frame = np.array(img)

    # Process through vision pipeline
    result = vision_processor.process(frame)

    # Send back results
    annotated_b64 = base64.b64encode(result.annotated_frame).decode("utf-8")
    emit("result", {
        "detections": [
            {"label": d.label, "confidence": d.confidence, "bbox": d.bbox}
            for d in result.detections
        ],
        "annotated_frame": f"data:image/jpeg;base64,{annotated_b64}",
    })


@socketio.on("robot_command")
def handle_robot_command(data):
    """Set robot target node."""
    if robot is None or nav_graph is None:
        emit("robot_path", {"error": "No robot or graph"})
        return

    target = data.get("target")
    path = robot.set_target(target)
    emit("robot_path", {"path": path})


def main():
    global MESH_PATH, vision_processor, nav_graph, robot

    if len(sys.argv) < 2:
        print("Usage: python server.py <mesh_path>", file=sys.stderr)
        sys.exit(1)

    MESH_PATH = sys.argv[1]
    if not Path(MESH_PATH).exists():
        print(f"Mesh not found: {MESH_PATH}", file=sys.stderr)
        sys.exit(1)

    # Initialize vision pipeline
    print("Loading YOLO model...")
    vision_processor = YOLOProcessor()

    print(f"Loading mesh: {MESH_PATH}")
    print("Open http://localhost:5000 in your browser")
    socketio.run(app, host="0.0.0.0", port=5000, debug=False)


if __name__ == "__main__":
    main()
```

**Step 2: Quick smoke test**

```bash
python server.py /path/to/mesh.glb
# Should print "Loading YOLO model..." and start server
# Ctrl+C to stop
```

**Step 3: Commit**

```bash
git add server.py
git commit -m "feat: Flask + SocketIO server with vision pipeline"
```

---

### Task 6: Frontend — split view with 3D scene and robot camera

**Files:**
- Create: `../robot-patrol-sim/static/index.html`

This is the largest task. The HTML file contains the full frontend. Reuse Three.js patterns from `../indoor-walk/static/index.html`.

**Step 1: Implement the frontend**

Key sections (in order within the file):

1. **HTML structure:** Split view layout — left canvas (3D orbit scene), right canvas/img (robot camera feed), bottom control bar
2. **Three.js scene setup:** Two renderers — orbit scene (OrbitControls) and robot camera (PerspectiveCamera following robot)
3. **Navigation graph rendering:** Lines between connected nodes, clickable node spheres
4. **Graph editor:** Click-to-place nodes, click two nodes to connect, delete nodes/edges
5. **Robot marker:** Visible mesh (box or arrow) in orbit view showing robot position/heading
6. **Robot camera:** Second Three.js camera rendered to a separate canvas, captures toDataURL as JPEG
7. **WebSocket client:** socket.io — sends robot camera frames at throttled rate, receives annotated frames + detections, displays annotated frame in right panel
8. **Controls:** Start/stop robot, speed slider, click node to set target, FPS display

The frontend is large but well-structured. Key patterns from indoor-walk to reuse:
- Scene/camera/renderer setup (lines 726-749 of indoor-walk)
- Catmull-Rom spline function (lines 1500-1537)
- Waypoint mesh creation and click-to-place

**Important implementation details:**
- Two Three.js renderers sharing the same scene but different cameras
- Orbit camera uses `OrbitControls` from Three.js addons
- Robot camera is a `PerspectiveCamera` positioned at robot location, facing robot yaw/pitch
- Robot movement is driven client-side: browser requests path via WebSocket, then interpolates position locally using `requestAnimationFrame`
- Frame capture: `robotRenderer.domElement.toDataURL('image/jpeg', 0.8)` → send via socket.io
- Throttle: only send a frame every N ms (configurable, start at 100ms = 10 FPS)
- Annotated frame: display as `<img>` element, update src to received base64

**Step 2: Verify it loads**

```bash
python server.py /path/to/mesh.glb
# Open http://localhost:5000
# Should see split view with 3D scene on left, empty robot camera on right
```

**Step 3: Commit**

```bash
git add static/index.html
git commit -m "feat: split-view frontend with graph editor and robot camera"
```

---

### Task 7: Wire up WebSocket frame streaming end-to-end

**Files:**
- Modify: `../robot-patrol-sim/static/index.html` (WebSocket section)
- Modify: `../robot-patrol-sim/server.py` (if adjustments needed)

**Step 1: Add socket.io client CDN to index.html**

```html
<script src="https://cdn.socket.io/4.7.4/socket.io.min.js"></script>
```

**Step 2: Implement frame streaming loop in JS**

```javascript
const socket = io();
let frameSendInterval = 100; // ms between frames sent to server
let lastFrameSent = 0;

function sendFrame() {
    const now = Date.now();
    if (now - lastFrameSent < frameSendInterval) return;
    lastFrameSent = now;

    const dataUrl = robotRenderer.domElement.toDataURL('image/jpeg', 0.8);
    socket.emit('frame', { image: dataUrl });
}

socket.on('result', (data) => {
    // Update robot camera view with annotated frame
    robotCameraImg.src = data.annotated_frame;

    // Update detection count / labels in UI
    updateDetectionPanel(data.detections);
});
```

Call `sendFrame()` in the animation loop when robot is active.

**Step 3: Test end-to-end**

```bash
python server.py /path/to/mesh.glb
# Open browser, place nodes, connect them, start robot
# Right panel should show annotated frames with YOLO detections
```

**Step 4: Commit**

```bash
git add static/index.html server.py
git commit -m "feat: wire up WebSocket frame streaming with YOLO results"
```

---

### Task 8: Robot target routing via click

**Files:**
- Modify: `../robot-patrol-sim/static/index.html` (click handler + robot_command)

**Step 1: Add click-to-target on graph nodes**

In the orbit view, when a node sphere is clicked:
1. Send `robot_command` with the target node ID via WebSocket
2. Server computes A* path, returns it
3. Client interpolates robot along the path using Catmull-Rom splines
4. Robot marker moves in orbit view, robot camera tracks position

```javascript
// On node click in orbit view
socket.emit('robot_command', { target: clickedNodeId });

socket.on('robot_path', (data) => {
    if (data.path) {
        startRobotMovement(data.path);
    }
});
```

**Step 2: Implement client-side robot interpolation**

Use the path of node IDs → look up positions → Catmull-Rom spline → animate robot position/camera per frame.

**Step 3: Test**

- Place nodes, connect them, click a distant node
- Robot should pathfind and move smoothly
- Robot camera should update and stream frames to YOLO

**Step 4: Commit**

```bash
git add static/index.html
git commit -m "feat: click-to-target robot routing with A* pathfinding"
```

---

### Task 9: Polish and integration testing

**Files:**
- Modify: `../robot-patrol-sim/static/index.html` (UI polish)
- Modify: `../robot-patrol-sim/server.py` (if needed)

**Step 1: UI improvements**
- FPS counter showing detection pipeline throughput
- Detection panel listing current detections with labels/confidence
- Speed slider for robot movement
- Graph save/load buttons

**Step 2: Test full flow**

1. Start server with mesh
2. Place 5-10 nodes forming a grid
3. Connect edges
4. Save graph
5. Click a target node
6. Robot navigates, camera streams, YOLO detections appear in right panel
7. Click another target — robot reroutes

**Step 3: Commit**

```bash
git add -A
git commit -m "feat: polish UI with FPS counter, detection panel, save/load"
```

---

## Task Summary

| # | Task | Dependencies |
|---|------|-------------|
| 1 | Project scaffold | None |
| 2 | NavGraph + A* | 1 |
| 3 | Robot state/movement | 2 |
| 4 | Vision pipeline + YOLO | 1 |
| 5 | Flask + SocketIO server | 2, 3, 4 |
| 6 | Frontend split view | 5 |
| 7 | WebSocket streaming | 5, 6 |
| 8 | Click-to-target routing | 6, 7 |
| 9 | Polish + integration | 8 |

Tasks 2-4 can be done in parallel. Tasks 6-9 are sequential.
