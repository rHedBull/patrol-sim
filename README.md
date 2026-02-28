# Robot Patrol Simulator

Simulate a robot patrolling a 3D industrial scene — navigate via an editable waypoint graph, capture the robot's camera POV, and run YOLO object detection on the feed in real time.

## Setup

Requires Python 3.12+ and [uv](https://docs.astral.sh/uv/).

```bash
uv sync
```

You need a GLB mesh file of your scene. Optionally, provide a second "no-walls" mesh for better orbit-view visibility:

```bash
# Split a mesh into structural (walls) and equipment using a labeled point cloud
uv run python scripts/split_mesh.py <source.glb> <labeled.ply> <output_dir>
```

This produces `structural.glb` (walls/ceiling) and `equipment.glb` (floor + equipment). Use `equipment.glb` as the no-walls mesh.

## Usage

Start the server:

```bash
uv run python server.py <mesh.glb> [no_walls_mesh.glb]
```

Open http://localhost:5000. The UI has two panels:

- **Left** — 3D orbit view with the navigation graph editor
- **Right** — Robot camera feed with YOLO detection overlay

Workflow:

1. Click **Edit Graph** to enter edit mode
2. Double-click the floor to place waypoint nodes
3. Click **Connect Nodes**, then click two nodes to create an edge
4. Right-click a node (or select + **Set Start**) to set the robot's spawn point
5. **Save Graph** to persist (stored as JSON in `graphs/`)
6. Click **Start** to enter live mode — click any node to send the robot there
7. Toggle **3D View** / **YOLO View** on the right panel to see raw camera or annotated detections

## Project structure

```
server.py            Flask + SocketIO server, REST API, WebSocket handlers
navigation/          NavGraph (A* pathfinding) and Robot movement
vision/              Pluggable vision pipeline (YOLO processor)
static/index.html    Three.js split-view frontend
scripts/             Mesh processing utilities
graphs/              Saved navigation graph JSON files
```

## Docs

- [Architecture](docs/architecture.md) — system overview, data flow, module design
- [API Reference](docs/api.md) — REST endpoints and WebSocket events
- [Graph Editor](docs/graph-editor.md) — frontend editing controls and UX

## License

Unlicensed / private project.
