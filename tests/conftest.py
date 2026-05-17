"""Shared test setup.

Two jobs:

1. Stub `torch` and `transformers` in `sys.modules` *before* anything imports
   `server.py`. The server transitively imports `vision.grounding_dino`, which
   pulls torch + transformers at module load. Tests don't instantiate any
   model, so stub modules are enough to let the import succeed without a
   multi-GB CUDA download.

2. Provide a `server_state` fixture that:
   - Points `SCENES_ROOT` at a tmp scene with `source/mesh.glb` (empty file
     is fine — `_list_scenes()` only checks existence).
   - Points `GRAPHS_ROOT` at a tmp dir.
   - Resets `nav_graph`, `robot`, `current_orbit`, and the optimized-mesh
     cache between tests so global state doesn't leak.
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

import pytest


# ── sys.modules stubs (must run before `server` import) ──────────────────────

def _install_stub(name: str, attrs: dict | None = None) -> types.ModuleType:
    mod = sys.modules.get(name)
    if mod is None:
        mod = types.ModuleType(name)
        sys.modules[name] = mod
    for k, v in (attrs or {}).items():
        setattr(mod, k, v)
    return mod


_install_stub("torch")
_install_stub(
    "transformers",
    {
        "AutoProcessor": type("AutoProcessor", (), {}),
        "AutoModelForZeroShotObjectDetection": type(
            "AutoModelForZeroShotObjectDetection", (), {}
        ),
    },
)


# ── Server state fixture ─────────────────────────────────────────────────────


@pytest.fixture
def server_state(tmp_path, monkeypatch):
    """Isolate server module-level state in a tmp filesystem.

    Yields a small namespace with `server`, `scene` name, `scenes_root`,
    `graphs_root`, and a Flask `client`. SocketIO test client is created
    on demand inside tests that need it.
    """
    # Late import: relies on the sys.modules stubs above.
    import server  # noqa: E402
    from navigation.graph import NavGraph

    scenes_root = tmp_path / "scenes"
    graphs_root = tmp_path / "graphs"
    scene = "test_scene"
    (scenes_root / scene / "source").mkdir(parents=True)
    (scenes_root / scene / "source" / "mesh.glb").write_bytes(b"")
    graphs_root.mkdir()

    monkeypatch.setattr(server, "SCENES_ROOT", scenes_root)
    monkeypatch.setattr(server, "GRAPHS_ROOT", graphs_root)
    monkeypatch.setattr(server, "nav_graph", NavGraph())
    monkeypatch.setattr(server, "robot", None)
    monkeypatch.setattr(server, "current_orbit", None)
    monkeypatch.setattr(server, "_OPTIMIZED_CACHE", {})
    monkeypatch.setattr(server, "vision", None)

    server.app.config["TESTING"] = True
    client = server.app.test_client()

    yield types.SimpleNamespace(
        server=server,
        scene=scene,
        scenes_root=scenes_root,
        graphs_root=graphs_root,
        client=client,
    )
