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


class NavGraph:
    """Bidirectional navigation graph supporting A* shortest-path search."""

    def __init__(self) -> None:
        self._nodes: dict[str, Node] = {}
        self._edges: dict[str, set[str]] = {}

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
        del self._edges[id]
        del self._nodes[id]

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

    def neighbors(self, id: str) -> set[str]:
        if id not in self._nodes:
            raise KeyError(f"Node '{id}' not found")
        return set(self._edges.get(id, set()))

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

    def to_dict(self) -> dict[str, Any]:
        return {
            "nodes": [
                {"id": nid, "position": list(node.position)}
                for nid, node in self._nodes.items()
            ],
            "edges": [
                {"from": a, "to": b}
                for a in self._edges
                for b in self._edges[a]
                if a < b  # store each bidirectional edge once
            ],
        }

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
                graph.add_edge(edge["from"], edge["to"])
            else:
                graph.add_edge(edge[0], edge[1])
        return graph

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def load(cls, path: str | Path) -> NavGraph:
        data = json.loads(Path(path).read_text())
        return cls.from_dict(data)
