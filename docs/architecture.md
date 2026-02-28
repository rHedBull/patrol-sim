# Architecture

## System overview

The simulator has three layers: a Flask+SocketIO backend, a Three.js frontend, and a pluggable vision pipeline. The frontend renders two 3D views (orbit + robot POV), sends robot camera frames to the backend via WebSocket, and receives YOLO-annotated results back.

```
Browser (Three.js)                         Flask + SocketIO (Python)
┌─────────────────────────┐                ┌──────────────────────────┐
│  Left: 3D orbit view    │                │  Static file server      │
│  - nav graph overlay    │                │  - mesh.glb              │
│  - robot marker         │                │  - mesh_no_walls.glb     │
│  - graph editor         │                │  - index.html            │
│                         │                │                          │
│  Right: Robot camera    │   WebSocket    │  Graph CRUD API (REST)   │
│  - first-person POV     │ ◄──────────► │  - /api/graph/*          │
│  - YOLO overlay toggle  │               │  - /api/graphs           │
│                         │  "frame" ──►  │                          │
│  Controls bar           │  "result" ◄── │  Vision pipeline         │
│  - Start/Stop live mode │  "robot_cmd"► │  - YOLOv8n (ultralytics) │
│  - Speed slider         │  "robot_path"◄│                          │
└─────────────────────────┘                └──────────────────────────┘
```

**Key architectural decision:** Movement is computed entirely client-side. The server only handles pathfinding (A*) and vision processing. The client requests a path via WebSocket, then animates the robot along it using frame-delta linear interpolation.

## Data flow

### Frame processing (live mode)

1. Client renders robot camera scene to a canvas (640×480, walls always visible, graph/robot marker hidden)
2. Canvas is captured as JPEG, base64-encoded, sent via `frame` WebSocket event (~10 FPS)
3. Server decodes the image, runs `YOLOProcessor.process()` on the frame
4. Server returns annotated frame (JPEG with bounding boxes) + structured detections via `result` event
5. Client displays annotated frame in YOLO view and updates detection list

### Robot navigation (live mode)

1. User clicks a waypoint node in the orbit view
2. Client emits `robot_command` with target node ID (and current node)
3. Server runs `NavGraph.find_path()` (A*) from current to target node
4. Server returns the path (list of node IDs) via `robot_path` event
5. Client resolves node IDs to 3D positions and animates the robot linearly between them
6. If the user clicks a new target mid-movement, the client queues it and re-paths after finishing the current edge segment

### Graph editing

All graph mutations go through REST endpoints (`POST /api/graph/node`, etc.) and the client updates its local state on success. Graphs are persisted as JSON files in the `graphs/` directory via `POST /api/graph/save`.

## Modules

### `navigation/graph.py` — NavGraph

Bidirectional weighted graph with A* shortest-path search.

- **Nodes**: ID + 3D position `(x, y, z)`
- **Edges**: bidirectional, implicit weight = Euclidean distance
- **A***: standard implementation with closed set, f-score priority queue, counter for stable ordering
- **Serialization**: JSON with `nodes` array and `edges` array. `from_dict()` handles both current list format and a legacy dict format for backward compatibility

The graph also tracks an optional `start_node` — the designated robot spawn point.

### `navigation/robot.py` — Robot

State machine for moving along a NavGraph path.

- Holds current position (interpolated), yaw, pitch, current node, speed
- `set_target(node_id)` → calls `graph.find_path()`, stores path
- `step(dt)` → advances position along path segments by `speed * dt`
- Handles multi-segment traversal in a single step if speed is high enough

**Note:** The server-side Robot is used only for pathfinding dispatch in the `robot_command` handler. Actual movement animation runs client-side in JavaScript.

### `vision/base.py` — VisionProcessor

Abstract base class for the pluggable vision pipeline:

```python
class VisionProcessor:
    def process(self, frame: np.ndarray) -> ProcessorResult: ...

@dataclass
class ProcessorResult:
    detections: list[Detection]    # label, confidence, bbox [x1,y1,x2,y2]
    annotated_frame: bytes         # JPEG with drawn overlays

@dataclass
class Detection:
    label: str
    confidence: float
    bbox: list[float]              # [x1, y1, x2, y2]
```

New processors can be added by subclassing `VisionProcessor` and implementing `process()`.

### `vision/yolo.py` — YOLOProcessor

YOLO implementation using `ultralytics`. Loads `yolov8n.pt` (YOLOv8 nano) at startup. Runs inference on each frame, extracts bounding boxes, and returns an annotated JPEG.

- Default confidence threshold: 0.25
- Model file: `yolov8n.pt` (downloaded automatically by ultralytics on first run)

### `server.py` — Flask + SocketIO server

- Serves static files (HTML, GLB meshes)
- REST API for graph CRUD and persistence (see [API Reference](api.md))
- WebSocket handlers for frame processing and robot commands
- Loads YOLO model at startup
- Threading async mode (`async_mode="threading"`)

### `static/index.html` — Frontend

Single-page Three.js application with:

- **Orbit view** (left panel): 3D scene with OrbitControls, nav graph overlay (spheres + tubes), robot marker, wall toggle between full and no-walls meshes
- **Robot camera** (right panel): first-person camera view, toggleable between raw 3D and YOLO-annotated overlay
- **Graph editor**: double-click to place nodes, drag to move, connect mode for edges, right-click to set start, Delete to remove (see [Graph Editor](graph-editor.md) for full controls)
- **Live mode**: click nodes to navigate, robot moves with linear interpolation, pending re-target support

### `scripts/split_mesh.py` — Mesh splitter

Utility to split a GLB mesh into "structural" (walls/ceiling) and "equipment" (floor + machinery) using a labeled PLY point cloud. Uses a KD-tree nearest-neighbor lookup to classify mesh faces by proximity to keep-points.

## Storage

### Navigation graphs (`graphs/*.json`)

```json
{
  "nodes": [
    {"id": "node_1772031419593_jn4i", "position": [2.82, -4.49, 5.87]}
  ],
  "edges": [
    {"from": "node_1772031419593_jn4i", "to": "node_1772031421483_0w45"}
  ],
  "start_node": "node_1772031419593_jn4i"
}
```

- Node IDs are generated client-side: `node_<timestamp>_<random4>`
- Edges are stored once per pair (the `a < b` canonical form)
- `start_node` is optional — if set, the robot spawns there on load
- Files are read/written by `NavGraph.save()` / `NavGraph.load()`

## Design decisions

- **Client-side movement**: Keeps the server stateless w.r.t. animation ticks. The server is only needed for pathfinding and vision — no tick loop required.
- **Same Three.js scene for both views**: The robot camera and orbit view share one `THREE.Scene`. Objects are temporarily hidden (robot marker, graph overlay) before rendering the robot POV to keep the feed clean.
- **Wall toggle**: Two separate meshes (full and no-walls) are loaded. The orbit view switches visibility between them. The robot camera always renders the full mesh so YOLO sees realistic frames.
- **Bidirectional edges only**: The graph enforces bidirectional edges — every `add_edge(a, b)` also adds `b → a`. Simplifies editing and pathfinding.
