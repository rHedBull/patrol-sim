"""Tests for the Flask render-frame endpoint."""

from __future__ import annotations

import json

import pytest


PNG_1x1 = (
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8z8BQDwAEhQGAhKmM"
    "IQAAAABJRU5ErkJggg=="
)


@pytest.fixture()
def client(tmp_path, monkeypatch):
    import server as srv
    renders_dir = tmp_path / "renders" / "scene1"
    renders_dir.mkdir(parents=True)
    monkeypatch.setattr(srv, "RENDERS_DIR", renders_dir)
    srv.app.config["TESTING"] = True
    with srv.app.test_client() as c:
        yield c, renders_dir


def _post_frame(client, *, name, index, view=None, pose=None):
    body = {
        "name": name,
        "index": index,
        "png_b64": PNG_1x1,
        "pose": pose or {"position": [0, 0, 0], "yaw": 0.0},
    }
    if view is not None:
        body["view"] = view
    return client.post(
        "/api/render_frame",
        data=json.dumps(body),
        content_type="application/json",
    )


class TestRenderFrameViews:
    def test_default_view_is_forward(self, client):
        c, renders_dir = client
        r = _post_frame(c, name="run1", index=0)
        assert r.status_code == 200
        manifest = json.loads((renders_dir / "run1" / "manifest.json").read_text())
        assert manifest["frames"][0]["view"] == "forward"
        assert manifest["frames"][0]["file"] == "frame_0000.png"

    def test_view_suffix_creates_distinct_filename(self, client):
        c, renders_dir = client
        _post_frame(c, name="run1", index=0)
        r = _post_frame(c, name="run1", index=0, view="L+10")
        assert r.status_code == 200
        out = renders_dir / "run1"
        assert (out / "frame_0000.png").exists()
        assert (out / "frame_0000__L+10.png").exists()
        manifest = json.loads((out / "manifest.json").read_text())
        views = sorted(f["view"] for f in manifest["frames"])
        assert views == ["L+10", "forward"]

    def test_dedupe_replaces_same_index_and_view(self, client):
        c, renders_dir = client
        _post_frame(c, name="run1", index=0, view="L+10")
        _post_frame(c, name="run1", index=0, view="L+10")
        manifest = json.loads((renders_dir / "run1" / "manifest.json").read_text())
        assert sum(1 for f in manifest["frames"] if f["view"] == "L+10") == 1

    def test_legacy_manifest_entries_get_forward_view(self, client):
        c, renders_dir = client
        out = renders_dir / "run1"
        out.mkdir()
        (out / "manifest.json").write_text(json.dumps({
            "name": "run1", "scene": "scene1",
            "frames": [{"index": 0, "file": "frame_0000.png", "position": [0, 0, 0]}],
        }))
        _post_frame(c, name="run1", index=1, view="forward")
        manifest = json.loads((out / "manifest.json").read_text())
        legacy = next(f for f in manifest["frames"] if f["index"] == 0)
        assert legacy["view"] == "forward"
