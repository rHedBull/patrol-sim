"""Characterization tests for `server.py`.

These tests pin current observable behavior of the surfaces that the
in-flight refactor plan will touch:

- `/api/plan/path` — chained-A* leg concatenation with shared-join dedup
  (slated to share a helper with the `plan_command` WS handler)
- `/api/graph/save` + `/api/graph/load/<name>` — orbit sidecar lifecycle
- `/api/render_frame` — manifest replace-by-index behavior
- WS `robot_command` / `plan_command` — happy path + error responses
- PUT `/api/graph/node/<id>` — 404 vs. update

They go through the Flask + SocketIO test clients (not direct function
calls) so the routing layer and the global-state interactions are also
exercised. The fixture lives in conftest.py and isolates SCENES_ROOT,
GRAPHS_ROOT, and the in-memory globals per test.
"""

from __future__ import annotations

import base64
import json
from pathlib import Path

import pytest


# ── Helpers ──────────────────────────────────────────────────────────────────


def _seed_line_graph(srv) -> None:
    """Populate `server.nav_graph` with A--B--C and a robot at A.

    Uses the in-memory NavGraph directly rather than going through the API so
    the test stays focused on whatever endpoint it's exercising.
    """
    from navigation.graph import NavGraph
    from navigation.robot import Robot

    g = NavGraph()
    g.add_node("A", (0.0, 0.0, 0.0))
    g.add_node("B", (10.0, 0.0, 0.0))
    g.add_node("C", (20.0, 0.0, 0.0))
    g.add_edge("A", "B")
    g.add_edge("B", "C")
    srv.nav_graph = g
    srv.robot = Robot(g, "A")


def _seed_disconnected_graph(srv) -> None:
    """A and B are nodes with no edge between them."""
    from navigation.graph import NavGraph

    g = NavGraph()
    g.add_node("A", (0.0, 0.0, 0.0))
    g.add_node("B", (10.0, 0.0, 0.0))
    srv.nav_graph = g


def _qs(scene: str) -> str:
    return f"?scene={scene}"


# ── /api/plan/path ───────────────────────────────────────────────────────────


class TestPlanPath:
    def test_happy_path_chains_legs_without_duplicating_join(self, server_state):
        s = server_state
        _seed_line_graph(s.server)
        resp = s.client.post(
            "/api/plan/path" + _qs(s.scene),
            json={"waypoints": ["A", "C"]},
        )
        assert resp.status_code == 200
        body = resp.get_json()
        ids = [n["id"] for n in body["path"]]
        assert ids == ["A", "B", "C"]
        # Positions echoed from NavGraph.
        positions = [n["position"] for n in body["path"]]
        assert positions == [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 0.0, 0.0]]

    def test_multi_waypoint_dedups_shared_join_node(self, server_state):
        s = server_state
        _seed_line_graph(s.server)
        resp = s.client.post(
            "/api/plan/path" + _qs(s.scene),
            json={"waypoints": ["A", "B", "C"]},
        )
        assert resp.status_code == 200
        ids = [n["id"] for n in resp.get_json()["path"]]
        # B appears once even though it's the end of leg-1 and start of leg-2.
        assert ids == ["A", "B", "C"]

    def test_under_two_waypoints_returns_400(self, server_state):
        s = server_state
        _seed_line_graph(s.server)
        for wps in ([], ["A"]):
            resp = s.client.post("/api/plan/path" + _qs(s.scene), json={"waypoints": wps})
            assert resp.status_code == 400
            assert "2 waypoints" in resp.get_json()["error"]

    def test_unreachable_leg_returns_400(self, server_state):
        s = server_state
        _seed_disconnected_graph(s.server)
        resp = s.client.post(
            "/api/plan/path" + _qs(s.scene),
            json={"waypoints": ["A", "B"]},
        )
        assert resp.status_code == 400
        assert "No path" in resp.get_json()["error"]

    def test_consecutive_duplicate_waypoints_are_tolerated(self, server_state):
        """`find_path(X, X)` returns `[X]`, and the dedup branch collapses it
        into the chain — so `[A, A, B]` resolves to `[A, B]`. Pin this so
        the future shared helper doesn't accidentally break the dedup.
        """
        s = server_state
        _seed_line_graph(s.server)
        resp = s.client.post(
            "/api/plan/path" + _qs(s.scene),
            json={"waypoints": ["A", "A", "B"]},
        )
        assert resp.status_code == 200
        ids = [n["id"] for n in resp.get_json()["path"]]
        assert ids == ["A", "B"]


# ── /api/graph/save + /api/graph/load orbit sidecar ──────────────────────────


class TestGraphPersistenceOrbitSidecar:
    def _save(self, s, name="g1", orbit=None):
        body = {"name": name}
        if orbit is not None:
            body["orbit"] = orbit
        return s.client.post("/api/graph/save" + _qs(s.scene), json=body)

    def test_save_without_orbit_writes_no_sidecar(self, server_state):
        s = server_state
        _seed_line_graph(s.server)
        resp = self._save(s)
        assert resp.status_code == 200
        graph_dir = s.graphs_root / s.scene
        assert (graph_dir / "g1.json").exists()
        assert not (graph_dir / "g1.orbit.json").exists()

    def test_save_with_orbit_writes_sidecar_and_load_restores_it(self, server_state):
        s = server_state
        _seed_line_graph(s.server)
        orbit = {
            "tip_node_id": "B",
            "radius": 2.5,
            "height": 1.0,
            "samples": 16,
            "start_theta_deg": 45.0,
        }
        assert self._save(s, orbit=orbit).status_code == 200
        sidecar = s.graphs_root / s.scene / "g1.orbit.json"
        assert sidecar.exists()
        assert json.loads(sidecar.read_text()) == orbit

        # Load and confirm current_orbit + GET /api/graph reflects it.
        assert s.client.post("/api/graph/load/g1" + _qs(s.scene)).status_code == 200
        body = s.client.get("/api/graph" + _qs(s.scene)).get_json()
        assert body["orbit"] == orbit

    def test_second_save_without_orbit_drops_stale_sidecar(self, server_state):
        """Pin the explicit cleanup in `save_graph` (server.py:331-333): if
        a graph was previously saved with an orbit, re-saving without one
        must delete the sidecar so stale metadata doesn't leak.
        """
        s = server_state
        _seed_line_graph(s.server)
        assert self._save(s, orbit={"tip_node_id": "B", "radius": 1, "height": 1,
                                    "samples": 8, "start_theta_deg": 0}).status_code == 200
        sidecar = s.graphs_root / s.scene / "g1.orbit.json"
        assert sidecar.exists()

        assert self._save(s).status_code == 200  # no orbit this time
        assert not sidecar.exists()

    def test_load_missing_graph_returns_404(self, server_state):
        s = server_state
        # No graph saved.
        resp = s.client.post("/api/graph/load/nope" + _qs(s.scene))
        assert resp.status_code == 404


# ── /api/render_frame manifest ───────────────────────────────────────────────


def _png_b64() -> str:
    """1x1 black PNG."""
    raw = (
        b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01"
        b"\x08\x06\x00\x00\x00\x1f\x15\xc4\x89\x00\x00\x00\rIDATx\x9cc\xfa\xcf"
        b"\x00\x00\x00\x03\x00\x01\xe5'\xde\xfc\x00\x00\x00\x00IEND\xaeB`\x82"
    )
    return base64.b64encode(raw).decode("ascii")


class TestRenderFrame:
    def test_append_then_save_at_new_index_grows_manifest(self, server_state):
        s = server_state
        png = _png_b64()
        r1 = s.client.post("/api/render_frame" + _qs(s.scene), json={
            "name": "run1", "index": 0, "png_b64": png, "pose": {"yaw": 0.0},
        })
        r2 = s.client.post("/api/render_frame" + _qs(s.scene), json={
            "name": "run1", "index": 1, "png_b64": png, "pose": {"yaw": 1.0},
        })
        assert r1.status_code == 200 and r2.status_code == 200
        manifest = json.loads(
            (s.scenes_root / s.scene / "renders" / "run1" / "manifest.json").read_text()
        )
        assert [f["index"] for f in manifest["frames"]] == [0, 1]
        assert manifest["frames"][0]["yaw"] == 0.0
        assert manifest["frames"][1]["yaw"] == 1.0

    def test_save_same_index_replaces_entry(self, server_state):
        """Pin server.py:468 — re-saving frame N replaces the existing entry
        (not appended a duplicate). The future shared frame-loop helper relies
        on this for retries.
        """
        s = server_state
        png = _png_b64()
        s.client.post("/api/render_frame" + _qs(s.scene), json={
            "name": "run1", "index": 0, "png_b64": png, "pose": {"yaw": 0.0},
        })
        s.client.post("/api/render_frame" + _qs(s.scene), json={
            "name": "run1", "index": 0, "png_b64": png, "pose": {"yaw": 9.9},
        })
        manifest = json.loads(
            (s.scenes_root / s.scene / "renders" / "run1" / "manifest.json").read_text()
        )
        assert len(manifest["frames"]) == 1
        assert manifest["frames"][0]["yaw"] == 9.9

    @pytest.mark.parametrize("bad_name", ["", "with/slash", "..", "  "])
    def test_invalid_name_returns_400(self, server_state, bad_name):
        s = server_state
        resp = s.client.post("/api/render_frame" + _qs(s.scene), json={
            "name": bad_name, "index": 0, "png_b64": _png_b64(),
        })
        assert resp.status_code == 400

    def test_missing_png_returns_400(self, server_state):
        s = server_state
        resp = s.client.post("/api/render_frame" + _qs(s.scene), json={
            "name": "run1", "index": 0, "png_b64": "",
        })
        assert resp.status_code == 400

    def test_data_url_prefix_is_stripped(self, server_state):
        """Pin server.py:455-456 — clients may post `data:image/png;base64,XYZ`
        and the server must strip the prefix before decoding.
        """
        s = server_state
        resp = s.client.post("/api/render_frame" + _qs(s.scene), json={
            "name": "run1", "index": 0,
            "png_b64": "data:image/png;base64," + _png_b64(),
        })
        assert resp.status_code == 200
        # File exists and is the decoded PNG (8 byte signature).
        png_file = s.scenes_root / s.scene / "renders" / "run1" / "frame_0000.png"
        assert png_file.exists()
        assert png_file.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


class TestRenderFrameViews:
    """Per-edge auxiliary view captures (frame_NNNN__<view>.png)."""

    def test_default_view_is_forward(self, server_state):
        s = server_state
        resp = s.client.post("/api/render_frame" + _qs(s.scene), json={
            "name": "run1", "index": 0, "png_b64": _png_b64(), "pose": {"yaw": 0.0},
        })
        assert resp.status_code == 200
        manifest = json.loads(
            (s.scenes_root / s.scene / "renders" / "run1" / "manifest.json").read_text()
        )
        assert manifest["frames"][0]["view"] == "forward"
        assert manifest["frames"][0]["file"] == "frame_0000.png"

    def test_view_suffix_creates_distinct_filename(self, server_state):
        s = server_state
        s.client.post("/api/render_frame" + _qs(s.scene), json={
            "name": "run1", "index": 0, "png_b64": _png_b64(), "pose": {"yaw": 0.0},
        })
        r = s.client.post("/api/render_frame" + _qs(s.scene), json={
            "name": "run1", "index": 0, "view": "L+10",
            "png_b64": _png_b64(), "pose": {"yaw": 0.0, "side": "left", "tilt": 10},
        })
        assert r.status_code == 200
        out = s.scenes_root / s.scene / "renders" / "run1"
        assert (out / "frame_0000.png").exists()
        assert (out / "frame_0000__L+10.png").exists()
        manifest = json.loads((out / "manifest.json").read_text())
        views = sorted(f["view"] for f in manifest["frames"])
        assert views == ["L+10", "forward"]

    def test_dedupe_replaces_same_index_and_view(self, server_state):
        s = server_state
        for _ in range(2):
            s.client.post("/api/render_frame" + _qs(s.scene), json={
                "name": "run1", "index": 0, "view": "L+10",
                "png_b64": _png_b64(), "pose": {"yaw": 0.0},
            })
        manifest = json.loads(
            (s.scenes_root / s.scene / "renders" / "run1" / "manifest.json").read_text()
        )
        assert sum(1 for f in manifest["frames"] if f["view"] == "L+10") == 1

    def test_legacy_manifest_entries_get_forward_view(self, server_state):
        s = server_state
        out = s.scenes_root / s.scene / "renders" / "run1"
        out.mkdir(parents=True)
        (out / "manifest.json").write_text(json.dumps({
            "name": "run1", "scene": s.scene,
            "frames": [{"index": 0, "file": "frame_0000.png", "position": [0, 0, 0]}],
        }))
        s.client.post("/api/render_frame" + _qs(s.scene), json={
            "name": "run1", "index": 1, "view": "forward",
            "png_b64": _png_b64(), "pose": {"yaw": 0.0},
        })
        manifest = json.loads((out / "manifest.json").read_text())
        legacy = next(f for f in manifest["frames"] if f["index"] == 0)
        assert legacy["view"] == "forward"


# ── /api/graph/node/<id> PUT ─────────────────────────────────────────────────


class TestUpdateNode:
    def test_update_missing_node_returns_404(self, server_state):
        s = server_state
        resp = s.client.put(
            "/api/graph/node/missing" + _qs(s.scene),
            json={"position": [1.0, 2.0, 3.0]},
        )
        assert resp.status_code == 404

    def test_update_existing_node_overwrites_position(self, server_state):
        s = server_state
        _seed_line_graph(s.server)
        resp = s.client.put(
            "/api/graph/node/A" + _qs(s.scene),
            json={"position": [9.0, 9.0, 9.0]},
        )
        assert resp.status_code == 200
        body = s.client.get("/api/graph" + _qs(s.scene)).get_json()
        positions = {n["id"]: n["position"] for n in body["nodes"]}
        assert positions["A"] == [9.0, 9.0, 9.0]


# ── WebSocket: robot_command + plan_command ──────────────────────────────────


def _ws(srv_module, scene):
    """Open a SocketIO test client with the scene already in the query string.

    `_scene_from_request()` reads `request.args`, which for SocketIO comes
    from the URL the client connected with.
    """
    return srv_module.socketio.test_client(
        srv_module.app, query_string=f"scene={scene}"
    )


class TestRobotCommand:
    def test_valid_target_emits_path(self, server_state):
        s = server_state
        _seed_line_graph(s.server)
        ws = _ws(s.server, s.scene)
        ws.emit("robot_command", {"target": "C"})
        events = [e for e in ws.get_received() if e["name"] == "robot_path"]
        assert len(events) == 1
        assert events[0]["args"][0]["path"] == ["A", "B", "C"]

    def test_missing_target_emits_error(self, server_state):
        s = server_state
        _seed_line_graph(s.server)
        ws = _ws(s.server, s.scene)
        ws.emit("robot_command", {})
        events = [e for e in ws.get_received() if e["name"] == "robot_path"]
        assert events[0]["args"][0]["error"] == "Missing 'target' field"

    def test_no_robot_emits_error(self, server_state):
        s = server_state
        # No nav_graph nodes → robot stays None.
        ws = _ws(s.server, s.scene)
        ws.emit("robot_command", {"target": "X"})
        events = [e for e in ws.get_received() if e["name"] == "robot_path"]
        assert "No robot" in events[0]["args"][0]["error"]


class TestPlanCommand:
    def test_valid_waypoints_emit_chained_path_and_inject_into_robot(self, server_state):
        s = server_state
        _seed_line_graph(s.server)
        ws = _ws(s.server, s.scene)
        ws.emit("plan_command", {"waypoints": ["A", "C"]})
        events = [e for e in ws.get_received() if e["name"] == "robot_path"]
        assert events[0]["args"][0]["path"] == ["A", "B", "C"]

        # The handler injects the path into the robot (server.py:574-577).
        robot = s.server.robot
        assert robot.current_node == "A"
        assert robot._path == ["A", "B", "C"]
        assert robot._path_index == 1
        assert not robot.is_idle()

    def test_under_two_waypoints_emits_error(self, server_state):
        s = server_state
        _seed_line_graph(s.server)
        ws = _ws(s.server, s.scene)
        ws.emit("plan_command", {"waypoints": ["A"]})
        events = [e for e in ws.get_received() if e["name"] == "robot_path"]
        assert "at least 2" in events[0]["args"][0]["error"]

    def test_unreachable_leg_emits_error(self, server_state):
        s = server_state
        _seed_disconnected_graph(s.server)
        # A robot needs at least one node to exist; use _reset_robot path.
        from navigation.robot import Robot
        s.server.robot = Robot(s.server.nav_graph, "A")

        ws = _ws(s.server, s.scene)
        ws.emit("plan_command", {"waypoints": ["A", "B"]})
        events = [e for e in ws.get_received() if e["name"] == "robot_path"]
        assert "No path" in events[0]["args"][0]["error"]

    def test_no_robot_emits_error(self, server_state):
        s = server_state
        ws = _ws(s.server, s.scene)
        ws.emit("plan_command", {"waypoints": ["A", "B"]})
        events = [e for e in ws.get_received() if e["name"] == "robot_path"]
        assert "No robot" in events[0]["args"][0]["error"]
