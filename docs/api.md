# API Reference

Server endpoints for the Flask + SocketIO backend. See [Architecture](architecture.md) for how these fit into the overall system.

## REST Endpoints

All REST endpoints return JSON. Graph mutation endpoints return `{"ok": true}` on success.

### Mesh serving

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/mesh.glb` | Full scene mesh (walls + equipment) |
| `GET` | `/mesh_no_walls.glb` | Equipment-only mesh (404 if not provided) |

### Graph CRUD

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/api/graph` | Current in-memory graph (nodes, edges, start_node) |
| `PUT` | `/api/graph` | Replace entire graph |
| `DELETE` | `/api/graph` | Clear graph and robot |

#### Nodes

| Method | Path | Description |
|--------|------|-------------|
| `POST` | `/api/graph/node` | Add a node |
| `PUT` | `/api/graph/node/<node_id>` | Update node position |
| `DELETE` | `/api/graph/node/<node_id>` | Remove node and its edges |

**Add node** — `POST /api/graph/node`

```json
{"id": "node_123_abc", "position": [1.5, -4.0, 3.2]}
```

**Update node** — `PUT /api/graph/node/<node_id>`

```json
{"position": [2.0, -4.0, 3.5]}
```

#### Edges

| Method | Path | Description |
|--------|------|-------------|
| `POST` | `/api/graph/edge` | Add bidirectional edge between two nodes |

```json
{"from": "node_123_abc", "to": "node_456_def"}
```

Returns 404 if either node doesn't exist.

#### Start node

| Method | Path | Description |
|--------|------|-------------|
| `PUT` | `/api/graph/start_node` | Set the robot spawn point |

```json
{"node_id": "node_123_abc"}
```

Pass `{"node_id": null}` to clear the start node. Resets the robot to the new start.

### Graph persistence

Graphs are saved as JSON files in the `graphs/` directory.

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/api/graphs` | List saved graph names (array of strings) |
| `POST` | `/api/graph/save` | Save current graph to disk |
| `POST` | `/api/graph/load/<name>` | Load a saved graph (replaces in-memory graph) |
| `DELETE` | `/api/graph/delete/<name>` | Delete a saved graph file |

**Save** — `POST /api/graph/save`

```json
{"name": "my-patrol-route"}
```

Saves to `graphs/my-patrol-route.json`.

### Robot

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/api/robot` | Current robot state |

Response:

```json
{
  "position": [2.5, -4.0, 3.0],
  "yaw": 1.57,
  "pitch": 0.0,
  "current_node": "node_123_abc",
  "idle": true
}
```

Returns 400 if no graph is loaded (no nodes).

## WebSocket Events

Connection uses [Socket.IO](https://socket.io/) on the default namespace.

### `frame` (client → server)

Send a robot camera frame for YOLO processing.

```json
{"image": "data:image/jpeg;base64,/9j/4AAQ..."}
```

The `image` field is a base64 JPEG, with or without the `data:` URI prefix.

### `result` (server → client)

YOLO processing result.

```json
{
  "detections": [
    {"label": "person", "confidence": 0.87, "bbox": [120.5, 45.2, 310.8, 400.1]}
  ],
  "annotated_frame": "data:image/jpeg;base64,/9j/4AAQ..."
}
```

- `detections` — array of detected objects with bounding boxes `[x1, y1, x2, y2]`
- `annotated_frame` — JPEG with bounding boxes drawn by YOLO, base64-encoded with data URI prefix

### `robot_command` (client → server)

Request a path for the robot.

```json
{"target": "node_456_def", "from_node": "node_123_abc"}
```

- `target` — destination node ID
- `from_node` — (optional) override the robot's current node. Used when the client tracks position more accurately than the server.

### `robot_path` (server → client)

Computed path response.

```json
{"path": ["node_123_abc", "node_789_ghi", "node_456_def"]}
```

Returns `{"error": "..."}` if no path exists or no robot is active.
