"""Navigation graph with A* pathfinding."""

from __future__ import annotations

import heapq
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class Node:
    id: str
    position: tuple[float, float, float]


@dataclass(frozen=True)
class View:
    """A per-edge aux view defined as a rotation around the edge axis.

    `roll_deg` is the angle (in degrees, [0, 360)) of the camera look
    direction on the plane perpendicular to the edge's traversal direction.
    0° points along world-up projected onto that plane; 90° points "right"
    of traversal; 180° points down; 270° points "left"."""

    roll_deg: float

    def __post_init__(self) -> None:
        if not (0.0 <= float(self.roll_deg) < 360.0):
            raise ValueError(f"View.roll_deg must be in [0, 360), got {self.roll_deg}")


def view_canonical_key(v: View) -> int:
    """Stable integer roll key used for dedupe + filenames."""
    return int(round(v.roll_deg)) % 360


def views_in_traversal_direction(views: list[View], *, reversed_: bool) -> list[View]:
    """Mirror roll across the world-up axis when traversing the edge against
    its canonical (a < b) order. Reflection sends roll → (360 - roll) % 360:
    `roll=90` (right of forward travel) becomes `roll=270` (left of reversed
    travel); `roll=0` (up) and `roll=180` (down) are invariant."""
    if not reversed_:
        return list(views)
    return [View(roll_deg=(360.0 - v.roll_deg) % 360.0) for v in views]


@dataclass
class EdgeMeta:
    render: bool = True
    views: list[View] = field(default_factory=list)


class NavGraph:
    """Bidirectional navigation graph supporting A* shortest-path search."""

    def __init__(self) -> None:
        self._nodes: dict[str, Node] = {}
        self._edges: dict[str, set[str]] = {}
        self._edge_meta: dict[frozenset[str], EdgeMeta] = {}
        self.start_node: str | None = None

    @property
    def nodes(self) -> dict[str, list[float]]:
        """Public mapping of node ID to position list, for use by Robot etc."""
        return {nid: list(node.position) for nid, node in self._nodes.items()}

    # ── Node operations ──────────────────────────────────────────────

    def add_node(self, id: str, position: tuple[float, float, float]) -> None:
        self._nodes[id] = Node(id=id, position=position)
        self._edges.setdefault(id, set())

    def remove_node(self, id: str) -> None:
        if id not in self._nodes:
            raise KeyError(f"Node '{id}' not found")
        # Remove all edges referencing this node
        for neighbor in list(self._edges.get(id, [])):
            self._edges[neighbor].discard(id)
            self._edge_meta.pop(self._edge_key(id, neighbor), None)
        del self._edges[id]
        del self._nodes[id]
        if self.start_node == id:
            self.start_node = None

    def get_node(self, id: str) -> Node:
        if id not in self._nodes:
            raise KeyError(f"Node '{id}' not found")
        return self._nodes[id]

    # ── Edge operations ──────────────────────────────────────────────

    def add_edge(self, from_id: str, to_id: str) -> None:
        if from_id not in self._nodes:
            raise KeyError(f"Node '{from_id}' not found")
        if to_id not in self._nodes:
            raise KeyError(f"Node '{to_id}' not found")
        self._edges[from_id].add(to_id)
        self._edges[to_id].add(from_id)

    def remove_edge(self, from_id: str, to_id: str) -> None:
        self._edges.get(from_id, set()).discard(to_id)
        self._edges.get(to_id, set()).discard(from_id)
        self._edge_meta.pop(self._edge_key(from_id, to_id), None)

    def neighbors(self, id: str) -> set[str]:
        if id not in self._nodes:
            raise KeyError(f"Node '{id}' not found")
        return set(self._edges.get(id, set()))

    # ── Edge metadata ────────────────────────────────────────────────

    @staticmethod
    def _edge_key(a: str, b: str) -> frozenset[str]:
        return frozenset({a, b})

    def _require_edge(self, a: str, b: str) -> None:
        if b not in self._edges.get(a, set()):
            raise KeyError(f"Edge ({a!r}, {b!r}) not found")

    def get_edge_meta(self, a: str, b: str) -> EdgeMeta:
        self._require_edge(a, b)
        meta = self._edge_meta.get(self._edge_key(a, b))
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
        seen: set[int] = set()
        for v in views:
            k = view_canonical_key(v)
            if k in seen:
                raise ValueError(f"Duplicate canonical view key {k} in views list")
            seen.add(k)
        key = self._edge_key(a, b)
        meta = self._edge_meta.get(key) or EdgeMeta()
        self._edge_meta[key] = EdgeMeta(render=meta.render, views=list(views))

    # ── A* pathfinding ───────────────────────────────────────────────

    @staticmethod
    def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))

    def find_path(self, start: str, goal: str) -> list[str] | None:
        if start not in self._nodes or goal not in self._nodes:
            return None

        start_pos = self._nodes[start].position
        goal_pos = self._nodes[goal].position

        # Priority queue: (f_score, counter, node_id)
        counter = 0
        open_set: list[tuple[float, int, str]] = []
        heapq.heappush(open_set, (self._distance(start_pos, goal_pos), counter, start))

        came_from: dict[str, str] = {}
        g_score: dict[str, float] = {start: 0.0}

        closed: set[str] = set()

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            if current in closed:
                continue
            closed.add(current)

            for neighbor in self._edges.get(current, set()):
                if neighbor in closed:
                    continue
                tentative_g = g_score[current] + self._distance(
                    self._nodes[current].position,
                    self._nodes[neighbor].position,
                )
                if tentative_g < g_score.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._distance(
                        self._nodes[neighbor].position, goal_pos
                    )
                    counter += 1
                    heapq.heappush(open_set, (f, counter, neighbor))

        return None

    # ── Serialization ────────────────────────────────────────────────

    def _edge_to_dict(self, a: str, b: str) -> dict[str, Any]:
        entry: dict[str, Any] = {"from": a, "to": b}
        meta = self._edge_meta.get(self._edge_key(a, b))
        if meta is None:
            return entry
        if meta.render is False:
            entry["render"] = False
        if meta.views:
            entry["views"] = [{"roll_deg": v.roll_deg} for v in meta.views]
        return entry

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {
            "nodes": [
                {"id": nid, "position": list(node.position)}
                for nid, node in self._nodes.items()
            ],
            "edges": [
                self._edge_to_dict(a, b)
                for a in self._edges
                for b in self._edges[a]
                if a < b  # store each bidirectional edge once
            ],
        }
        if self.start_node is not None:
            d["start_node"] = self.start_node
        return d

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> NavGraph:
        graph = cls()
        nodes = data.get("nodes", [])
        if isinstance(nodes, list):
            for node in nodes:
                graph.add_node(node["id"], tuple(node["position"]))
        else:
            # Legacy dict format
            for nid, pos in nodes.items():
                graph.add_node(nid, tuple(pos))
        for edge in data.get("edges", []):
            if isinstance(edge, dict):
                a, b = edge["from"], edge["to"]
                graph.add_edge(a, b)
                if "render" in edge:
                    graph.set_edge_render(a, b, bool(edge["render"]))
                if "views" in edge:
                    views = [View(roll_deg=float(v["roll_deg"])) for v in edge["views"]]
                    graph.set_edge_views(a, b, views)
            else:
                graph.add_edge(edge[0], edge[1])
        start = data.get("start_node")
        if start and start in graph._nodes:
            graph.start_node = start
        return graph

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def load(cls, path: str | Path) -> NavGraph:
        data = json.loads(Path(path).read_text())
        return cls.from_dict(data)
