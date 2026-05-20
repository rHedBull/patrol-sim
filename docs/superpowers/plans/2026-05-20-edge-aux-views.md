# Per-edge aux views + render skip — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let users mark per-edge auxiliary view angles (up to 3 side-looking captures per arc-length sample) and a render-skip flag on the navigation graph, so route rendering produces forward + side frames where defined and skips frames entirely on connectivity-only edges.

**Architecture:** Extend `NavGraph` with backward-compatible edge metadata (`render: bool`, `views: [{side, tilt}]`). The frontend `arcLengthSample`/`renderPlanFrames` loop maps each sample to its source edge via per-segment node ids returned by `/api/plan/path`, then either skips the sample (render=false) or captures forward + one frame per view (with side mirrored on reverse traversal). Server `/api/render_frame` gains an optional `view` field; manifest dedupe key becomes `(index, view)` with in-place migration of legacy entries.

**Tech Stack:** Python 3.12 (NavGraph, Flask server, pytest), Three.js + Potree (frontend), vanilla JS in `static/index.html`.

**Spec:** `docs/superpowers/specs/2026-05-20-edge-aux-views-design.md`

---

## File map

| File | Role | Touch |
|---|---|---|
| `navigation/graph.py` | `EdgeMeta`, `View` dataclasses; `_edge_meta` storage; mirror helper; validation; round-trip | Modify |
| `tests/test_graph.py` | Tests for the new edge metadata API | Modify |
| `server.py` | `/api/render_frame` accepts `view`, dedupe by `(index, view)`, migrate legacy manifest entries | Modify |
| `tests/test_server.py` | New file: tests for `/api/render_frame` view handling and migration | Create |
| `static/index.html` | Edge state shape, selection UX + Edge Panel, edge styling, per-segment render loop changes | Modify |

Frontend changes are intentionally scoped to `static/index.html` rather than introducing a new module — the file's structure is monolithic by design today, and splitting it is out of scope.

---

## Conventions used in this plan

- **TDD for Python:** test first, watch it fail, implement, watch it pass, commit.
- **Frontend changes:** no JS test harness exists in this repo, so frontend tasks end with a `browser-verification` checkpoint instead of a unit test. The `browser-verification` skill must be invoked at those points.
- **DOM building:** never use `innerHTML` with interpolated values (a `PreToolUse` security hook blocks it). Build elements with `document.createElement` and assign `textContent` / properties directly.
- **Commit cadence:** one commit per task (after all steps pass). Commit messages follow the existing pattern (`feat(...)`, `feat(graph):`, `feat(render):`, etc.).
- **Commands assume cwd = worktree root** `/home/hendrik/coding/engine/tools/walker/robot-patrol-sim/.claude/worktrees/edge-aux-views/`.

---

## Task 1: `EdgeMeta` / `View` data model + validation

**Files:**
- Modify: `navigation/graph.py`
- Test: `tests/test_graph.py`

- [ ] **Step 1: Write failing tests for the new dataclasses and validation**

Append to `tests/test_graph.py`:

```python
from navigation.graph import EdgeMeta, View


class TestEdgeMeta:
    def test_default_render_is_true_and_views_empty(self):
        meta = EdgeMeta()
        assert meta.render is True
        assert meta.views == []

    def test_view_accepts_left_right_and_tilt_in_range(self):
        v1 = View(side="left", tilt=10)
        v2 = View(side="right", tilt=-45.0)
        assert v1.side == "left" and v1.tilt == 10
        assert v2.side == "right" and v2.tilt == -45.0

    def test_view_rejects_bad_side(self):
        with pytest.raises(ValueError):
            View(side="up", tilt=0)

    def test_view_rejects_tilt_out_of_range(self):
        with pytest.raises(ValueError):
            View(side="left", tilt=91)
        with pytest.raises(ValueError):
            View(side="right", tilt=-90.0001)


class TestEdgeMetaApi:
    def _g(self):
        g = NavGraph()
        g.add_node("A", (0, 0, 0))
        g.add_node("B", (1, 0, 0))
        g.add_edge("A", "B")
        return g

    def test_get_edge_meta_returns_default_when_unset(self):
        g = self._g()
        meta = g.get_edge_meta("A", "B")
        assert meta.render is True
        assert meta.views == []

    def test_set_edge_render_round_trip(self):
        g = self._g()
        g.set_edge_render("A", "B", False)
        assert g.get_edge_meta("A", "B").render is False
        # Symmetric: same answer regardless of order
        assert g.get_edge_meta("B", "A").render is False

    def test_set_edge_views_rejects_more_than_three(self):
        g = self._g()
        with pytest.raises(ValueError):
            g.set_edge_views("A", "B", [
                View("left", 0), View("right", 0),
                View("left", 30), View("right", 30),
            ])

    def test_set_edge_views_rejects_duplicate_canonical_key(self):
        g = self._g()
        # round(0.4) == round(-0.4) == 0 -> both canonicalize to (left, 0)
        with pytest.raises(ValueError):
            g.set_edge_views("A", "B", [View("left", 0.4), View("left", -0.4)])

    def test_set_edge_views_on_missing_edge_raises(self):
        g = self._g()
        with pytest.raises(KeyError):
            g.set_edge_views("A", "C", [View("left", 0)])

    def test_views_canonical_key_helper(self):
        from navigation.graph import view_canonical_key
        assert view_canonical_key(View("left", 0.4)) == ("left", 0)
        assert view_canonical_key(View("left", -0.4)) == ("left", 0)
        assert view_canonical_key(View("right", -45)) == ("right", -45)
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
uv run pytest tests/test_graph.py::TestEdgeMeta tests/test_graph.py::TestEdgeMetaApi -v
```
Expected: ImportError for `EdgeMeta`, `View`, `view_canonical_key`.

- [ ] **Step 3: Implement `EdgeMeta`, `View`, validation, and graph API**

Edit `navigation/graph.py`.

Add to imports (top of file):

```python
from typing import Literal
```

After the existing `Node` dataclass add:

```python
Side = Literal["left", "right"]


@dataclass(frozen=True)
class View:
    side: Side
    tilt: float  # degrees; 0 = horizontal, +up, -down

    def __post_init__(self) -> None:
        if self.side not in ("left", "right"):
            raise ValueError(f"View.side must be 'left' or 'right', got {self.side!r}")
        if not (-90.0 <= float(self.tilt) <= 90.0):
            raise ValueError(f"View.tilt must be in [-90, 90], got {self.tilt}")


def view_canonical_key(v: View) -> tuple[Side, int]:
    """Stable (side, signed_int_tilt) key used for dedupe + filenames.
    `-0` is normalized to `0` by int rounding."""
    return (v.side, int(round(v.tilt)) or 0)


@dataclass
class EdgeMeta:
    render: bool = True
    views: list[View] = field(default_factory=list)
```

Add to `NavGraph.__init__`:

```python
self._edge_meta: dict[frozenset[str], EdgeMeta] = {}
```

Add helpers after the existing edge methods:

```python
@staticmethod
def _edge_key(a: str, b: str) -> frozenset[str]:
    return frozenset({a, b})

def _require_edge(self, a: str, b: str) -> None:
    if b not in self._edges.get(a, set()):
        raise KeyError(f"Edge ({a!r}, {b!r}) not found")

def get_edge_meta(self, a: str, b: str) -> EdgeMeta:
    self._require_edge(a, b)
    meta = self._edge_meta.get(self._edge_key(a, b))
    # Fresh default keeps callers from mutating shared state.
    return meta if meta is not None else EdgeMeta()

def set_edge_render(self, a: str, b: str, render: bool) -> None:
    self._require_edge(a, b)
    key = self._edge_key(a, b)
    meta = self._edge_meta.get(key) or EdgeMeta()
    self._edge_meta[key] = EdgeMeta(render=bool(render), views=list(meta.views))

def set_edge_views(self, a: str, b: str, views: list[View]) -> None:
    self._require_edge(a, b)
    if len(views) > 3:
        raise ValueError(f"At most 3 views per edge, got {len(views)}")
    seen: set[tuple[Side, int]] = set()
    for v in views:
        k = view_canonical_key(v)
        if k in seen:
            raise ValueError(f"Duplicate canonical view key {k} in views list")
        seen.add(k)
    key = self._edge_key(a, b)
    meta = self._edge_meta.get(key) or EdgeMeta()
    self._edge_meta[key] = EdgeMeta(render=meta.render, views=list(views))
```

Update `remove_edge` to clear stale metadata:

```python
def remove_edge(self, from_id: str, to_id: str) -> None:
    self._edges.get(from_id, set()).discard(to_id)
    self._edges.get(to_id, set()).discard(from_id)
    self._edge_meta.pop(self._edge_key(from_id, to_id), None)
```

In `remove_node`, update the neighbor loop so metadata is cleared alongside edges:

```python
for neighbor in list(self._edges.get(id, [])):
    self._edges[neighbor].discard(id)
    self._edge_meta.pop(self._edge_key(id, neighbor), None)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
uv run pytest tests/test_graph.py -v
```
Expected: all green (round-trip is added in Task 2).

- [ ] **Step 5: Commit**

```bash
git add navigation/graph.py tests/test_graph.py
git commit -m "feat(graph): edge metadata model (render flag + views) with validation"
```

---

## Task 2: JSON round-trip for edge metadata

**Files:**
- Modify: `navigation/graph.py` (`to_dict` / `from_dict`)
- Test: `tests/test_graph.py`

- [ ] **Step 1: Write failing round-trip tests**

Append:

```python
class TestEdgeMetaRoundTrip:
    def _round_trip(self, g: NavGraph) -> NavGraph:
        return NavGraph.from_dict(g.to_dict())

    def test_default_meta_not_emitted_in_json(self):
        g = NavGraph()
        g.add_node("A", (0, 0, 0))
        g.add_node("B", (1, 0, 0))
        g.add_edge("A", "B")
        d = g.to_dict()
        edge = d["edges"][0]
        assert "render" not in edge
        assert "views" not in edge

    def test_render_flag_round_trip(self):
        g = NavGraph()
        g.add_node("A", (0, 0, 0))
        g.add_node("B", (1, 0, 0))
        g.add_edge("A", "B")
        g.set_edge_render("A", "B", False)
        g2 = self._round_trip(g)
        assert g2.get_edge_meta("A", "B").render is False

    def test_views_round_trip(self):
        g = NavGraph()
        g.add_node("A", (0, 0, 0))
        g.add_node("B", (1, 0, 0))
        g.add_edge("A", "B")
        g.set_edge_views("A", "B", [View("left", 10), View("right", -45)])
        g2 = self._round_trip(g)
        views = g2.get_edge_meta("A", "B").views
        assert [(v.side, v.tilt) for v in views] == [("left", 10), ("right", -45)]

    def test_from_dict_validates_bad_meta(self):
        bad = {
            "nodes": [{"id": "A", "position": [0, 0, 0]}, {"id": "B", "position": [1, 0, 0]}],
            "edges": [{"from": "A", "to": "B", "views": [{"side": "up", "tilt": 0}]}],
        }
        with pytest.raises(ValueError):
            NavGraph.from_dict(bad)

    def test_from_dict_legacy_loads_with_defaults(self):
        legacy = {
            "nodes": [{"id": "A", "position": [0, 0, 0]}, {"id": "B", "position": [1, 0, 0]}],
            "edges": [{"from": "A", "to": "B"}],
        }
        g = NavGraph.from_dict(legacy)
        meta = g.get_edge_meta("A", "B")
        assert meta.render is True and meta.views == []
```

- [ ] **Step 2: Run to verify they fail**

```bash
uv run pytest tests/test_graph.py::TestEdgeMetaRoundTrip -v
```

- [ ] **Step 3: Implement round-trip**

In `NavGraph.to_dict`, replace the inline edge-list comprehension with a helper call:

```python
"edges": [
    self._edge_to_dict(a, b)
    for a in self._edges
    for b in self._edges[a]
    if a < b
],
```

Add the helper alongside the other methods:

```python
def _edge_to_dict(self, a: str, b: str) -> dict[str, Any]:
    entry: dict[str, Any] = {"from": a, "to": b}
    meta = self._edge_meta.get(self._edge_key(a, b))
    if meta is None:
        return entry
    if meta.render is False:
        entry["render"] = False
    if meta.views:
        entry["views"] = [{"side": v.side, "tilt": v.tilt} for v in meta.views]
    return entry
```

In `NavGraph.from_dict`, replace the existing edge loop:

```python
for edge in data.get("edges", []):
    if isinstance(edge, dict):
        a, b = edge["from"], edge["to"]
        graph.add_edge(a, b)
        if "render" in edge:
            graph.set_edge_render(a, b, bool(edge["render"]))
        if "views" in edge:
            views = [View(side=v["side"], tilt=float(v["tilt"])) for v in edge["views"]]
            graph.set_edge_views(a, b, views)  # raises on dup / >3
    else:
        graph.add_edge(edge[0], edge[1])
```

- [ ] **Step 4: Run all graph tests**

```bash
uv run pytest tests/test_graph.py -v
```
Expected: all green.

- [ ] **Step 5: Commit**

```bash
git add navigation/graph.py tests/test_graph.py
git commit -m "feat(graph): round-trip edge metadata in JSON with backward-compatible defaults"
```

---

## Task 3: Direction-mirror helper

**Files:**
- Modify: `navigation/graph.py`
- Test: `tests/test_graph.py`

- [ ] **Step 1: Write failing tests**

```python
class TestDirectionMirror:
    def test_views_in_direction_forward_returns_as_stored(self):
        from navigation.graph import views_in_traversal_direction
        views = [View("left", 10), View("right", -20)]
        out = views_in_traversal_direction(views, reversed_=False)
        assert [(v.side, v.tilt) for v in out] == [("left", 10), ("right", -20)]

    def test_views_in_direction_reverse_mirrors_side_keeps_tilt(self):
        from navigation.graph import views_in_traversal_direction
        views = [View("left", 10), View("right", -20)]
        out = views_in_traversal_direction(views, reversed_=True)
        assert [(v.side, v.tilt) for v in out] == [("right", 10), ("left", -20)]
```

- [ ] **Step 2: Run to verify fail**

```bash
uv run pytest tests/test_graph.py::TestDirectionMirror -v
```

- [ ] **Step 3: Implement**

Append to `navigation/graph.py`:

```python
def views_in_traversal_direction(views: list[View], *, reversed_: bool) -> list[View]:
    """Mirror `side` when traversing the edge against its canonical (a < b) order.
    Tilt is preserved (gravity is direction-independent)."""
    if not reversed_:
        return list(views)
    flip = {"left": "right", "right": "left"}
    return [View(side=flip[v.side], tilt=v.tilt) for v in views]
```

- [ ] **Step 4: Run all graph tests**

```bash
uv run pytest tests/test_graph.py -v
```

- [ ] **Step 5: Commit**

```bash
git add navigation/graph.py tests/test_graph.py
git commit -m "feat(graph): direction-mirror helper for traversal-aware views"
```

---

## Task 4: Server `/api/render_frame` accepts `view` and migrates legacy manifests

**Files:**
- Modify: `server.py` (around the existing `save_render_frame` at line ~441)
- Create: `tests/test_server.py`

- [ ] **Step 1: Inspect how the scenes root is configured**

```bash
grep -n "SCENES_ROOT\|scenes_root\|--scenes-root\|_scene_dir\b" server.py
```

If the scenes root is only set via CLI arg, **first** add an `os.environ.get("SCENES_ROOT")` fallback so the test fixture can isolate writes without forking a subprocess. Keep the change minimal — one extra line in the resolution logic. Document the env var name in a comment.

- [ ] **Step 2: Write failing server tests**

Create `tests/test_server.py`:

```python
"""Tests for the Flask render-frame endpoint."""

from __future__ import annotations

import json

import pytest


# 1x1 PNG base64; content is irrelevant.
PNG_1x1 = (
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8z8BQDwAEhQGAhKmM"
    "IQAAAABJRU5ErkJggg=="
)


@pytest.fixture()
def client(tmp_path, monkeypatch):
    monkeypatch.setenv("SCENES_ROOT", str(tmp_path))
    import importlib
    import server as srv
    importlib.reload(srv)
    srv.app.config["TESTING"] = True
    scene_dir = tmp_path / "scene1"
    (scene_dir / "graphs").mkdir(parents=True)
    (scene_dir / "renders").mkdir(parents=True)
    with srv.app.test_client() as c:
        yield c, scene_dir


def _post_frame(client, *, scene, name, index, view=None, pose=None):
    body = {
        "name": name,
        "index": index,
        "png_b64": PNG_1x1,
        "pose": pose or {"position": [0, 0, 0], "yaw": 0.0},
    }
    if view is not None:
        body["view"] = view
    return client.post(
        f"/api/render_frame?scene={scene}",
        data=json.dumps(body),
        content_type="application/json",
    )


class TestRenderFrameViews:
    def test_default_view_is_forward(self, client):
        c, scene_dir = client
        r = _post_frame(c, scene="scene1", name="run1", index=0)
        assert r.status_code == 200
        manifest = json.loads((scene_dir / "renders" / "run1" / "manifest.json").read_text())
        assert manifest["frames"][0]["view"] == "forward"
        assert manifest["frames"][0]["file"] == "frame_0000.png"

    def test_view_suffix_creates_distinct_filename(self, client):
        c, scene_dir = client
        _post_frame(c, scene="scene1", name="run1", index=0)
        r = _post_frame(c, scene="scene1", name="run1", index=0, view="L+10")
        assert r.status_code == 200
        out = scene_dir / "renders" / "run1"
        assert (out / "frame_0000.png").exists()
        assert (out / "frame_0000__L+10.png").exists()
        manifest = json.loads((out / "manifest.json").read_text())
        views = sorted(f["view"] for f in manifest["frames"])
        assert views == ["L+10", "forward"]

    def test_dedupe_replaces_same_index_and_view(self, client):
        c, scene_dir = client
        _post_frame(c, scene="scene1", name="run1", index=0, view="L+10")
        _post_frame(c, scene="scene1", name="run1", index=0, view="L+10")
        manifest = json.loads((scene_dir / "renders" / "run1" / "manifest.json").read_text())
        assert sum(1 for f in manifest["frames"] if f["view"] == "L+10") == 1

    def test_legacy_manifest_entries_get_forward_view(self, client):
        c, scene_dir = client
        out = scene_dir / "renders" / "run1"
        out.mkdir()
        (out / "manifest.json").write_text(json.dumps({
            "name": "run1", "scene": "scene1",
            "frames": [{"index": 0, "file": "frame_0000.png", "position": [0, 0, 0]}],
        }))
        _post_frame(c, scene="scene1", name="run1", index=1, view="forward")
        manifest = json.loads((out / "manifest.json").read_text())
        legacy = next(f for f in manifest["frames"] if f["index"] == 0)
        assert legacy["view"] == "forward"
```

- [ ] **Step 3: Run to verify failure**

```bash
uv run pytest tests/test_server.py -v
```

- [ ] **Step 4: Update `save_render_frame` in `server.py`**

Replace the function body (currently `server.py:441-482`):

```python
@app.route("/api/render_frame", methods=["POST"])
def save_render_frame():
    """Persist a single rendered frame coming from the in-page renderer.

    Body: ``{name, index, view?, png_b64, pose}``. ``view`` defaults to
    ``"forward"``; non-forward views are written to
    ``frame_NNNN__<view>.png``. Dedupe key is ``(index, view)``; legacy
    entries without ``view`` are normalized to ``forward`` in-place.
    """
    data = request.get_json(force=True)
    name = (data.get("name") or "").strip()
    if not name or "/" in name or ".." in name:
        return jsonify({"error": "Invalid render name"}), 400
    index = int(data.get("index", 0))
    view = (data.get("view") or "forward").strip()
    if "/" in view or ".." in view or "\\" in view:
        return jsonify({"error": "Invalid view name"}), 400
    png_b64 = data.get("png_b64") or ""
    if "," in png_b64:
        png_b64 = png_b64.split(",", 1)[1]
    if not png_b64:
        return jsonify({"error": "Missing png_b64"}), 400

    scene = _scene_from_request()
    out_dir = _scene_renders_dir(scene) / name
    out_dir.mkdir(parents=True, exist_ok=True)

    suffix = "" if view == "forward" else f"__{view}"
    frame_path = out_dir / f"frame_{index:04d}{suffix}.png"
    frame_path.write_bytes(base64.b64decode(png_b64))

    manifest_path = out_dir / "manifest.json"
    manifest = {"name": name, "scene": scene, "frames": []}
    if manifest_path.exists():
        try:
            manifest = json.loads(manifest_path.read_text())
        except Exception:
            pass

    # Legacy entries become {view: "forward"} so the new dedupe key works.
    for f in manifest.get("frames", []):
        f.setdefault("view", "forward")

    pose = data.get("pose") or {}
    entry = {"index": index, "view": view, "file": frame_path.name, **pose}
    manifest["frames"] = [
        f for f in manifest.get("frames", [])
        if not (f.get("index") == index and f.get("view") == view)
    ]
    manifest["frames"].append(entry)
    manifest["frames"].sort(key=lambda f: (f["index"], f.get("view", "forward")))
    manifest_path.write_text(json.dumps(manifest, indent=2))

    return jsonify({"ok": True, "path": str(frame_path)})
```

- [ ] **Step 5: Run server + full suite**

```bash
uv run pytest tests/test_server.py -v
uv run pytest -v
```

- [ ] **Step 6: Commit**

```bash
git add server.py tests/test_server.py
git commit -m "feat(render): /api/render_frame accepts view field; dedupe by (index, view)"
```

---

## Task 5: Frontend — extend `graphEdges` shape and load/save preserving metadata

**Files:**
- Modify: `static/index.html` (the edge state and the load/save paths)

- [ ] **Step 1: Identify the touch sites**

```bash
grep -n "graphEdges = data.edges\|graphEdges.push\|graphEdges.filter\|edges\":\|edges:" static/index.html
```

Confirm the load site (~L2649), the various `graphEdges.push(...)` sites (~L1985–L2045), and the save path. Note their line numbers so the next step is precise.

- [ ] **Step 2: Extend the edge object shape**

Around L621, update the state comment:
```js
let graphEdges = [];       // [{from, to, render?: bool, views?: [{side, tilt}]}]
```

In the load path (`graphEdges = data.edges || []`), replace with a normalizer so the rest of the frontend can assume the fields exist:

```js
graphEdges = (data.edges || []).map(e => ({
    from: e.from,
    to: e.to,
    render: e.render !== false,                       // default true
    views: Array.isArray(e.views) ? e.views.slice() : [],
}));
```

Where new edges are constructed (search for `graphEdges.push`), ensure metadata defaults are set:

```js
graphEdges.push({ from, to, render: true, views: [] });
```

(Existing push sites currently push the bare `edgeData` object — update them to include `render: true, views: []`.)

In the save path (find the body sent to the graph save endpoint — it currently sends `graphEdges` directly). Replace with a serializer:

```js
function serializeEdges() {
    return graphEdges.map(e => {
        const out = { from: e.from, to: e.to };
        if (e.render === false) out.render = false;
        if (e.views && e.views.length > 0) {
            out.views = e.views.map(v => ({ side: v.side, tilt: v.tilt }));
        }
        return out;
    });
}
```

Then use `edges: serializeEdges()` wherever the save payload is built.

- [ ] **Step 3: Browser-verify load/save is non-destructive**

Use the `browser-verification` skill:
1. Start the server (`uv run python server.py <existing-mesh.glb>`).
2. Open the UI, load a legacy graph (no metadata).
3. Save it under a new name.
4. `diff <(jq -S . graphs/<scene>/<orig>.json) <(jq -S . graphs/<scene>/<new>.json)` — nodes/edges identical; no spurious `render: true` or empty `views: []` keys.

- [ ] **Step 4: Commit**

```bash
git add static/index.html
git commit -m "feat(graph-ui): preserve render/views metadata across edge load/save"
```

---

## Task 6: Frontend — edge visual styling (views tint, skip dashed)

**Files:**
- Modify: `static/index.html` (renderGraph at ~L1069, edge material declarations at ~L1108)

- [ ] **Step 1: Add new edge materials**

After the existing `edgeMaterial` line (~L1108):

```js
const edgeMaterialDefault = edgeMaterial; // keep original reference
const edgeMaterialWithViews = new THREE.MeshPhongMaterial({ color: 0x55ddee, transparent: true, opacity: 0.85 });
const edgeMaterialSkipped = new THREE.MeshPhongMaterial({ color: 0x666666, transparent: true, opacity: 0.35 });
const edgeMaterialSelected = new THREE.MeshPhongMaterial({ color: 0xffaa00, transparent: true, opacity: 0.95 });
```

- [ ] **Step 2: Add a canonical key helper next to graph state**

Near the other top-of-file helpers (after `groundKey` at ~L1131):

```js
function _edgeKey(e) { return e.from < e.to ? `${e.from}|${e.to}` : `${e.to}|${e.from}`; }
```

And the selection state slot (~L621 with the other `let` declarations):

```js
let selectedEdgeKey = null;
```

- [ ] **Step 3: Pick material per edge inside `renderGraph`**

In the existing `for (const edge of graphEdges) { ... }` loop, replace the single-material assignment with:

```js
let mat;
if (selectedEdgeKey && _edgeKey(edge) === selectedEdgeKey) {
    mat = edgeMaterialSelected;
} else if (edge.render === false) {
    mat = edgeMaterialSkipped;
} else if (edge.views && edge.views.length > 0) {
    mat = edgeMaterialWithViews;
} else {
    mat = edgeMaterialDefault;
}
const cyl = new THREE.Mesh(cylGeom, mat);
```

- [ ] **Step 4: Browser-verify**

Use `browser-verification`:
1. Load a graph; all edges should render as before.
2. In devtools: `graphEdges[0].render = false; renderGraph();` — edge becomes faded grey.
3. `graphEdges[1].views = [{side:'left',tilt:0}]; renderGraph();` — edge becomes cyan.
4. `selectedEdgeKey = _edgeKey(graphEdges[2]); renderGraph();` — that edge highlights orange.
5. Reset (`selectedEdgeKey = null; graphEdges[0].render = true; graphEdges[1].views = []; renderGraph();`) — back to default.

- [ ] **Step 5: Commit**

```bash
git add static/index.html
git commit -m "feat(graph-ui): edge styling for views (cyan) and skipped (faded)"
```

---

## Task 7: Frontend — Edge Panel UI (selection, render toggle, views editor)

**Files:**
- Modify: `static/index.html` (controls block, edit handlers)

- [ ] **Step 1: Add the Edge Panel DOM**

Locate the graph-edit controls (search for the "Connect Nodes" button). Add a sibling container, initially hidden:

```html
<div id="edge-panel" class="rl" style="display:none; margin-top:8px; padding:6px; border:1px solid #444;">
    <div id="edge-panel-header" style="display:flex; gap:6px; align-items:center;">
        <strong>Edge:</strong>
        <span id="edge-panel-label">-</span>
        <button id="edge-delete" type="button">Delete</button>
    </div>
    <label style="display:block; margin-top:4px;">
        <input type="checkbox" id="edge-render" checked> Render this edge
    </label>
    <div id="edge-views" style="margin-top:4px;"></div>
    <button id="edge-add-view" type="button">+ Add view</button>
</div>
```

- [ ] **Step 2: Implement the panel logic (no `innerHTML` — DOM building only)**

Add a JS block after the existing graph-edit helpers:

```js
const edgePanel = document.getElementById('edge-panel');
const edgePanelLabel = document.getElementById('edge-panel-label');
const edgeRenderCheckbox = document.getElementById('edge-render');
const edgeViewsContainer = document.getElementById('edge-views');
const edgeDeleteBtn = document.getElementById('edge-delete');
const edgeAddViewBtn = document.getElementById('edge-add-view');

function _findEdgeByKey(key) {
    return graphEdges.find(e => _edgeKey(e) === key) || null;
}

function _shortId(id) { return id.slice(-4); }

function openEdgePanel(from, to) {
    const e = graphEdges.find(x =>
        (x.from === from && x.to === to) || (x.from === to && x.to === from)
    );
    if (!e) return;
    selectedEdgeKey = _edgeKey(e);
    edgePanelLabel.textContent = `${_shortId(e.from)} <-> ${_shortId(e.to)}`;
    edgeRenderCheckbox.checked = e.render !== false;
    renderEdgeViewsList(e);
    edgePanel.style.display = '';
    renderGraph();
}

function closeEdgePanel() {
    selectedEdgeKey = null;
    edgePanel.style.display = 'none';
    renderGraph();
}

function _makeViewRow(view, i) {
    const row = document.createElement('div');
    row.style.cssText = 'display:flex; gap:4px; align-items:center; margin-top:2px;';

    const sel = document.createElement('select');
    sel.className = 'ev-side';
    sel.dataset.i = String(i);
    for (const opt of ['left', 'right']) {
        const o = document.createElement('option');
        o.value = opt;
        o.textContent = opt.charAt(0).toUpperCase() + opt.slice(1);
        if (view.side === opt) o.selected = true;
        sel.appendChild(o);
    }

    const tilt = document.createElement('input');
    tilt.type = 'number';
    tilt.className = 'ev-tilt';
    tilt.dataset.i = String(i);
    tilt.value = String(view.tilt);
    tilt.min = '-90'; tilt.max = '90'; tilt.step = '5';
    tilt.style.width = '64px';

    const unit = document.createElement('span');
    unit.textContent = '°';

    const del = document.createElement('button');
    del.type = 'button';
    del.className = 'ev-del';
    del.dataset.i = String(i);
    del.textContent = '×';

    row.appendChild(sel);
    row.appendChild(tilt);
    row.appendChild(unit);
    row.appendChild(del);
    return row;
}

function renderEdgeViewsList(edge) {
    edgeViewsContainer.replaceChildren();
    (edge.views || []).forEach((v, i) => {
        edgeViewsContainer.appendChild(_makeViewRow(v, i));
    });
    edgeAddViewBtn.disabled = (edge.views || []).length >= 3;
}

edgeRenderCheckbox.addEventListener('change', () => {
    const e = _findEdgeByKey(selectedEdgeKey); if (!e) return;
    e.render = edgeRenderCheckbox.checked;
    renderGraph();
});

edgeDeleteBtn.addEventListener('click', () => {
    const e = _findEdgeByKey(selectedEdgeKey); if (!e) return;
    graphEdges = graphEdges.filter(x => x !== e);
    closeEdgePanel();
});

edgeAddViewBtn.addEventListener('click', () => {
    const e = _findEdgeByKey(selectedEdgeKey); if (!e) return;
    if ((e.views || []).length >= 3) return;
    e.views = (e.views || []).concat([{ side: 'left', tilt: 0 }]);
    renderEdgeViewsList(e);
    renderGraph();
});

edgeViewsContainer.addEventListener('change', (ev) => {
    const e = _findEdgeByKey(selectedEdgeKey); if (!e) return;
    const t = ev.target;
    const i = parseInt(t.dataset.i, 10);
    if (Number.isNaN(i) || !e.views[i]) return;
    if (t.classList.contains('ev-side')) {
        e.views[i].side = t.value;
    } else if (t.classList.contains('ev-tilt')) {
        let n = parseFloat(t.value);
        if (!Number.isFinite(n)) n = 0;
        n = Math.max(-90, Math.min(90, n));
        e.views[i].tilt = n;
        t.value = String(n);
    }
    renderGraph();
});

edgeViewsContainer.addEventListener('click', (ev) => {
    if (!ev.target.classList.contains('ev-del')) return;
    const e = _findEdgeByKey(selectedEdgeKey); if (!e) return;
    const i = parseInt(ev.target.dataset.i, 10);
    e.views.splice(i, 1);
    renderEdgeViewsList(e);
    renderGraph();
});
```

- [ ] **Step 3: Wire selection on edge click; drop direct click-to-delete**

Find the edit-mode click handler around `getEdgeAtMouse` (~L1831). The code currently filters `graphEdges` to delete the hit edge. Replace that branch with selection:

```js
if (editMode && !connectMode) {
    const edge = getEdgeAtMouse(e);
    if (edge) {
        openEdgePanel(edge.from, edge.to);
        return;
    }
    // Clicked empty (no node, no edge) - deselect.
    if (!getNodeAtMouse(e)) closeEdgePanel();
}
```

Remove or guard any other code path that deletes edges on click. The node-removal-cascade filter (which clears edges of a deleted *node*) stays untouched.

- [ ] **Step 4: Browser-verify the panel end-to-end**

Use `browser-verification`:
1. Enter edit mode, click an edge — panel opens, header shows the short ids.
2. Untick "Render this edge" — edge styles change to faded.
3. Add a view — row appears; toggle side and edit tilt; edge gains cyan tint.
4. Click Delete — edge disappears and panel closes.
5. Click another edge — panel re-opens with that edge's state preserved.
6. Click empty space — panel closes; edges restore default styling.

- [ ] **Step 5: Commit**

```bash
git add static/index.html
git commit -m "feat(graph-ui): edge selection panel with render toggle and views editor"
```

---

## Task 8: Frontend — `arcLengthSample` yields segment index; renderPlanFrames threads node ids

**Files:**
- Modify: `static/index.html` (`arcLengthSample` ~L2244, `renderPlanFrames` ~L2296)

- [ ] **Step 1: Extend `arcLengthSample` output**

Where the loop does `samples.push({ pos, yaw, arcLen: s });`, change to:

```js
samples.push({ pos, yaw, arcLen: s, seg });
```

`seg` is already tracked by the existing `while (seg < cum.length - 2 && cum[seg+1] < s) seg++;` line.

- [ ] **Step 2: Capture node ids in `renderPlanFrames`**

Where `positions` is built from the path response, also build `pathIds`:

```js
const positions = path.map(n => ({ x: n.position[0], y: n.position[1], z: n.position[2] }));
const pathIds = path.map(n => n.id);
```

- [ ] **Step 3: Snapshot edge metadata at render start**

Just before the `for (let i = 0; i < samples.length; i++)` loop:

```js
const edgeMetaByKey = new Map();
for (const e of graphEdges) {
    edgeMetaByKey.set(_edgeKey(e), {
        render: e.render !== false,
        views: (e.views || []).slice(),
        // Canonical from/to (a < b) so we can detect reversed traversal.
        canonical: e.from < e.to ? [e.from, e.to] : [e.to, e.from],
    });
}
```

- [ ] **Step 4: Browser-verify samples carry `seg`**

Use `browser-verification`:
1. Open the UI on any graph.
2. In devtools console: `arcLengthSample([{x:0,y:0,z:0},{x:1,y:0,z:0},{x:2,y:0,z:0}], 0.5)`.
3. Confirm every returned sample has a numeric `seg` (0 for first segment, 1 for second).

- [ ] **Step 5: Commit**

```bash
git add static/index.html
git commit -m "feat(render): expose segment index from arcLengthSample; thread node ids through render"
```

---

## Task 9: Frontend — per-segment skip + multi-view capture

**Files:**
- Modify: `static/index.html` (`renderPlanFrames` loop body ~L2350, `refreshPlanUI`)

- [ ] **Step 1: Replace the per-sample body with per-edge logic**

Inside the `for (let i = 0; i < samples.length; i++)` block, replace the existing single-capture body with:

```js
const s = samples[i];
const a = pathIds[s.seg];
const b = pathIds[s.seg + 1];
const metaKey = a < b ? `${a}|${b}` : `${b}|${a}`;
const meta = edgeMetaByKey.get(metaKey)
          || { render: true, views: [], canonical: [a, b] };

if (!meta.render) {
    progress.textContent = `sample ${i+1}/${samples.length}: skipped (edge ${a.slice(-4)}<->${b.slice(-4)})`;
    continue;
}

const reversedTraversal = a !== meta.canonical[0];
const mirroredViews = meta.views.map(v => ({
    side: reversedTraversal ? (v.side === 'left' ? 'right' : 'left') : v.side,
    tilt: v.tilt,
}));

async function captureAndPost({ yaw, pitch, viewCode, sideOpt, tiltOpt }) {
    robotPosition.set(s.pos.x, s.pos.y, s.pos.z);
    robotYaw = yaw;
    robotPitch = pitch;
    updateRobotMarker();
    updateRobotCamera();
    await new Promise(r => setTimeout(r, 80));

    const cap = await captureFromPotree({
        maxMs: preset.maxMs,
        stableFrames: preset.stableFrames,
        parentTimeoutMs: preset.maxMs + 8000,
    });
    const pose = {
        position: [s.pos.x, s.pos.y, s.pos.z],
        yaw, pitch,
        arc_length: s.arcLen,
        quality,
    };
    if (sideOpt) { pose.side = sideOpt; pose.tilt = tiltOpt; }
    const r = await fetch('/api/render_frame', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ name: runName, index: i, view: viewCode, png_b64: cap.dataUrl, pose }),
    });
    if (!r.ok) {
        const err = await r.json().catch(() => ({}));
        throw new Error(err.error || r.status);
    }
    return cap.stats;
}

let captureStats = null;
try {
    captureStats = await captureAndPost({ yaw: s.yaw, pitch: 0, viewCode: 'forward' });
} catch (e) {
    console.error('forward capture failed at sample', i, e);
    progress.textContent = `frame ${i+1}: capture failed (${e.message})`;
    continue;
}

function _viewCode(side, tilt) {
    const t = Math.round(tilt);
    const sign = t < 0 ? '-' : '+';
    return `${side === 'left' ? 'L' : 'R'}${sign}${Math.abs(t)}`;
}
for (const v of mirroredViews) {
    const yawOffset = (v.side === 'left' ? -Math.PI/2 : +Math.PI/2);
    const tiltRad = v.tilt * Math.PI / 180;
    const code = _viewCode(v.side, v.tilt);
    try {
        await captureAndPost({
            yaw: s.yaw + yawOffset,
            pitch: tiltRad,
            viewCode: code,
            sideOpt: v.side, tiltOpt: v.tilt,
        });
    } catch (e) {
        console.error('view capture failed at sample', i, code, e);
        // Continue — partial capture is preferable to halting the run.
    }
}

robotPitch = 0;

const t = captureStats
    ? `${captureStats.elapsedMs}ms ${(captureStats.lru/1e6).toFixed(1)}M pts`
    : '';
progress.textContent = `frame ${i+1}/${samples.length}  s=${s.arcLen.toFixed(1)}m  +${mirroredViews.length} views  ${t}`;
```

Delete the previous forward-only capture-and-POST block this replaces.

- [ ] **Step 2: Gate the Edge Panel while a render is active**

Extend `refreshPlanUI()` (existing helper) so it disables the panel inputs and the Save button when `planActive`:

```js
const editingDisabled = planActive;
edgePanel.querySelectorAll('input, button, select').forEach(el => { el.disabled = editingDisabled; });
const saveBtn = document.getElementById('save-graph');
if (saveBtn) saveBtn.disabled = editingDisabled;
```

(Find the actual id of the "Save Graph" button by grepping `Save Graph` in `static/index.html`; substitute that id if different.)

- [ ] **Step 3: Browser-verify full capture flow**

Use `browser-verification`:
1. Create a graph with nodes A, B, C; edge A↔B has two views (`left 10`, `right -30`); edge B↔C has `render=false`.
2. Start at A, plan to C, click Render.
3. In `renders/<run>/` confirm:
   - `frame_NNNN.png`, `frame_NNNN__L+10.png`, `frame_NNNN__R-30.png` for samples on segment 0.
   - No files for samples on segment 1.
4. `manifest.json`: forward entries have `view: "forward"`; view entries have `view: "L+10"` / `"R-30"` and `side`/`tilt` fields.
5. Reverse: start at C, plan to A. Confirm `side` in the saved poses for edge A↔B is mirrored (`right 10`, `left -30`).

- [ ] **Step 4: Commit**

```bash
git add static/index.html
git commit -m "feat(render): per-edge view capture and segment skip during route render"
```

---

## Task 10: Final cleanup + smoke test

**Files:**
- All

- [ ] **Step 1: Full test suite**

```bash
uv run pytest -v
```
Expected: all green.

- [ ] **Step 2: Legacy graph smoke test**

Load a representative legacy graph, save-as-new, diff:

```bash
diff <(jq -S . graphs/<scene>/<orig>.json) <(jq -S . graphs/<scene>/<new>.json)
```
Expected: identical nodes/edges/start_node.

- [ ] **Step 3: Grep downstream manifest consumers for contiguous-index assumptions**

```bash
grep -rn "frame_\|manifest" scripts/ vision/ 2>/dev/null
```
For each match that iterates by index (rather than reading `manifest["frames"]` directly), flag it in the commit message — do **not** silently work around it.

- [ ] **Step 4: Docs touch-up**

If `docs/graph-editor.md` references edge-click-to-delete, update one paragraph to describe the selection panel instead. Keep it minimal.

- [ ] **Step 5: Final commit**

```bash
git add -A
git commit -m "docs(graph-editor): document edge panel + per-edge view angles"
```

---

## Risks recap (from spec)

- **Manifest sparseness:** Task 10 Step 3 grep covers this. Any contiguous-index consumer gets flagged, not silently patched.
- **Mid-render edits:** Task 9 Step 2 (UI gate) + Task 8 Step 3 (snapshot at render start) cover this.
- **Edge selection vs node hit:** Task 7 Step 3 calls `getNodeAtMouse` after `getEdgeAtMouse` to deselect only on truly-empty clicks. Verified manually in Task 7 Step 4.
