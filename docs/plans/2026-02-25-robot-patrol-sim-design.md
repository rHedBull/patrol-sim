# Robot Patrol Simulator — Design

**Date:** 2026-02-25
**Status:** Approved
**Type:** New project (separate from indoor-walk, reuses code patterns)

## Goal

Demo/proof-of-concept: simulate a robot patrolling an industrial 3D scene, capture its camera POV, run frames through a pluggable vision pipeline (YOLO first), and display results in a split-view UI.

## Architecture: WebSocket Streaming

Three.js renders both views in the browser. Robot POV frames are sent via WebSocket to a Flask backend, processed through the vision pipeline, and results streamed back for overlay display.

```
Browser (Three.js)                    Flask + SocketIO
├─ 3D Scene Overview (orbit)          ├─ Serves mesh + static files
├─ Robot Camera View                  ├─ WebSocket frame receiver
│  └─ sends JPEG frames ──ws──────►  ├─ Vision pipeline (pluggable)
│  └─ receives annotated  ◄──ws────  │  └─ YOLO (GPU-accelerated)
└─ Controls                           └─ Nav graph management API
```

## Navigation Graph

Position-only graph, separate from look direction.

```json
{
  "nodes": [
    {"id": "n1", "position": [x, y, z]},
    {"id": "n2", "position": [x, y, z]}
  ],
  "edges": [
    {"from": "n1", "to": "n2"}
  ]
}
```

- Nodes placed manually in the 3D scene (click-to-place, like indoor-walk waypoints)
- Edges created by clicking two nodes to connect them
- Robot uses A* pathfinding on the graph to route between targets
- Smooth movement via Catmull-Rom spline interpolation along edges
- Look direction is independent — robot can look anywhere while moving

## Vision Pipeline

Pluggable processor interface:

```python
class VisionProcessor:
    def process(self, frame: np.ndarray) -> ProcessorResult: ...

class ProcessorResult:
    detections: list[Detection]  # bboxes, labels, confidence
    annotated_frame: bytes       # frame with overlays drawn
```

- YOLO is the first implementation (ultralytics, GPU)
- Architecture supports swapping/chaining processors
- Frame throttle: start ~10 FPS, tunable

## WebSocket Protocol

- `frame` (client → server): base64 JPEG of robot camera view
- `result` (server → client): JSON detections + base64 annotated frame
- `robot_command` (client → server): target node for robot routing
- `robot_path` (server → client): computed path through graph

## UI Layout

```
┌─────────────────────────────────────────────────────────┐
│  Robot Patrol Simulator                                 │
├───────────────────────────┬─────────────────────────────┤
│   3D Scene Overview       │   Robot Camera Feed         │
│   - Orbit/pan controls    │   - Annotated frames        │
│   - Nav graph visible     │   - Bounding boxes + labels │
│   - Robot marker moving   │   - Confidence scores       │
│   - Click node = target   │                             │
├───────────────────────────┴─────────────────────────────┤
│  [Start/Stop] [Speed: ___] [Target: ___]       FPS: xx │
└─────────────────────────────────────────────────────────┘
```

## Project Structure

```
robot-patrol-sim/
├── server.py              # Flask + SocketIO server
├── pyproject.toml         # dependencies
├── vision/
│   ├── __init__.py
│   ├── base.py            # VisionProcessor interface
│   └── yolo.py            # YOLO implementation
├── navigation/
│   ├── __init__.py
│   ├── graph.py           # NavGraph, A* pathfinding
│   └── robot.py           # Robot state + movement
├── static/
│   └── index.html         # Three.js split-view frontend
└── graphs/                # Saved nav graph JSON files
```

## Dependencies

**Backend:** flask, flask-socketio, ultralytics, numpy, pillow
**Frontend:** Three.js (CDN)

## Reused from indoor-walk

- GLB mesh loading + Three.js scene setup
- Catmull-Rom spline interpolation
- Click-to-place node editing pattern
- Flask mesh serving endpoint
