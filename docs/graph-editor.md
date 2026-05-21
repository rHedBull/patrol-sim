# Graph Editor

The left panel provides a visual graph editor for creating and editing navigation waypoint graphs on top of the 3D mesh. See [Architecture](architecture.md#modules) for how the graph and robot modules work internally, and [API Reference](api.md#graph-crud) for the underlying REST endpoints.

## Modes

The UI has three modes:

| Mode | Entered via | Description |
|------|-------------|-------------|
| **Navigate** | Default / exit edit | Orbit view only. No graph editing. |
| **Edit** | Click **Edit Graph** | Place, move, delete nodes and edges. Robot is hidden. |
| **Live** | Click **Start** | Robot is active. Click nodes to navigate. Exits edit mode. |

## Edit mode controls

### Adding nodes

Double-click on the mesh floor to place a new waypoint node. The node is created at the clicked 3D position, snapped to the mesh surface (falls back to an invisible floor plane if the mesh raycast misses).

Nodes appear as green spheres on top of vertical posts.

### Moving nodes

Click and drag a node to reposition it. The node follows the mouse along the mesh/floor surface. Release to confirm — the new position is synced to the server.

A drag threshold of 5px prevents accidental moves during clicks.

### Deleting nodes

Select a node (click it), then press **Delete** or **Backspace**. All connected edges are removed automatically.

### Connecting nodes

1. Click **Connect Nodes** (or enter edit mode first)
2. Click the first node — it highlights yellow
3. Click the second node — an edge is created between them

Edges are always bidirectional. Duplicate edges are ignored.

Edges render as grey tubes at node height. In **Connect Nodes** mode, clicking an edge deletes it. In normal edit mode, clicking an edge **selects** it and opens the Edge Panel below the toolbar.

### Edge Panel (per-edge render options)

Selecting an edge in edit mode opens the Edge Panel:

- **Render this edge** — when unticked, the route still traverses this edge but no frames are captured along it. Skipped edges render as **dashed grey lines** instead of solid tubes for an unambiguous "will not render" signal.
- **Capture forward frame** — when unticked (and "Render this edge" stays on), samples on this edge skip the `frame_NNNN.png` forward capture but still produce any roll-view frames attached to the edge. Use it when you only want side views on a given segment.
- **Views** — up to 3 extra captures per arc-length sample on the edge. Each view is a single `roll_deg` (`0..359`) — a rotation around the edge axis that sets where the camera looks:
  - `0°` = straight up (world-up projected onto the perpendicular plane)
  - `90°` = right of traversal
  - `180°` = down
  - `270°` = left of traversal
  Edges that have views render as cyan tubes. Each view also adds a small cyan **cone glyph** at the edge midpoint pointing in the view's look direction.
- **Delete** — removes the edge.

#### Editing view angles

While an edge is selected, its view cones are draggable gizmos. Click and drag a cone tip to rotate the view around the edge axis; the `roll` slider and number field in the Edge Panel update live. You can also type or scrub directly in the panel — the cone glyph mirrors the change in 3D.

View frames are saved as `frame_NNNN__R<roll>.png` (three-digit zero-padded roll, e.g. `frame_NNNN__R090.png`), one extra image per view per sample, alongside the forward `frame_NNNN.png`. `roll_deg` is stored in canonical edge orientation (`from < to` lexicographically); when traversal is reversed, the look direction stays the same in world space, so no run-time mirror is needed.

Edge metadata persists with **Save Graph**.

### Setting the start node

The start node (blue) is where the robot spawns when entering live mode or loading a graph.

Two ways to set it:
- **Right-click** a node in edit mode
- **Select** a node, then click **Set Start**

### Node visuals

| Color | Meaning |
|-------|---------|
| Green | Default node |
| Blue | Start node |
| Yellow | Selected node |
| Light green | Hovered node |
| Red | Current navigation target (live mode) |

## Graph persistence

| Button | Action |
|--------|--------|
| **Save Graph** | Prompts for a name, saves to `graphs/<name>.json` |
| **Load Graph** | Dropdown of saved graphs — select to load |
| **New Graph** | Clears the current graph (with confirmation) |
| **Delete Graph** | Prompts for a name, permanently deletes the file |

Graphs auto-load `default` on startup if it exists.

## Wall toggle

Click **Hide Walls** to switch between the full mesh and the no-walls (equipment-only) mesh in the orbit view. This makes it easier to see the floor and place nodes in enclosed spaces.

The robot camera always renders the full mesh regardless of this toggle, so YOLO sees the complete scene.

## Live mode

Click **Start** to enter live mode:

1. Robot spawns at the start node (or first node)
2. Click any node in the orbit view to send the robot there
3. The robot plans a path (A*) and moves along it
4. The active path highlights in blue
5. If you click a new target while the robot is moving, it finishes the current edge, then re-paths to the new target
6. Frame streaming begins — robot camera frames are sent to YOLO at ~10 FPS
7. Toggle **3D View** / **YOLO View** on the right panel to see raw or annotated frames
8. Click **Stop** to exit live mode

### Speed control

The **Speed** slider (0.1–5.0) controls robot movement speed in units/second. Default is 1.0.

### FPS counter

The bottom-right FPS counter shows the YOLO detection pipeline throughput (results received per second), not the rendering frame rate.
