"""Tests for NavGraph."""

import json
import tempfile
from pathlib import Path

import pytest

from navigation.graph import NavGraph


class TestAddNode:
    def test_add_node(self):
        g = NavGraph()
        g.add_node("A", (1.0, 2.0, 3.0))
        node = g.get_node("A")
        assert node.id == "A"
        assert node.position == (1.0, 2.0, 3.0)

    def test_get_missing_node_raises(self):
        g = NavGraph()
        with pytest.raises(KeyError):
            g.get_node("X")


class TestAddEdge:
    def test_add_edge_bidirectional(self):
        g = NavGraph()
        g.add_node("A", (0, 0, 0))
        g.add_node("B", (1, 0, 0))
        g.add_edge("A", "B")
        assert "B" in g.neighbors("A")
        assert "A" in g.neighbors("B")


class TestRemoveNodeRemovesEdges:
    def test_remove_node_removes_edges(self):
        g = NavGraph()
        g.add_node("A", (0, 0, 0))
        g.add_node("B", (1, 0, 0))
        g.add_node("C", (2, 0, 0))
        g.add_edge("A", "B")
        g.add_edge("B", "C")
        g.remove_node("B")
        assert "B" not in g.neighbors("A")
        assert "B" not in g.neighbors("C")
        with pytest.raises(KeyError):
            g.get_node("B")


class TestAStarShortestPath:
    def test_astar_shortest_path(self):
        """Build a diamond graph where the direct route is shorter.

        A --10-- B --10-- D
        A --1--- C --1--- D

        A* must pick A -> C -> D (cost 2) over A -> B -> D (cost 20).
        """
        g = NavGraph()
        g.add_node("A", (0, 0, 0))
        g.add_node("B", (10, 10, 0))  # far detour
        g.add_node("C", (1, 0, 0))    # close shortcut
        g.add_node("D", (2, 0, 0))
        g.add_edge("A", "B")
        g.add_edge("B", "D")
        g.add_edge("A", "C")
        g.add_edge("C", "D")
        path = g.find_path("A", "D")
        assert path == ["A", "C", "D"]


class TestAStarNoPath:
    def test_astar_no_path(self):
        g = NavGraph()
        g.add_node("A", (0, 0, 0))
        g.add_node("B", (1, 0, 0))
        # No edge between A and B
        assert g.find_path("A", "B") is None


class TestSerializeRoundtrip:
    def test_serialize_roundtrip(self):
        g = NavGraph()
        g.add_node("A", (0, 0, 0))
        g.add_node("B", (1, 0, 0))
        g.add_node("C", (2, 0, 0))
        g.add_edge("A", "B")
        g.add_edge("B", "C")

        # dict roundtrip
        d = g.to_dict()
        g2 = NavGraph.from_dict(d)
        assert g2.get_node("A").position == (0, 0, 0)
        assert "B" in g2.neighbors("A")
        assert "C" in g2.neighbors("B")

        # file roundtrip
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            tmp = Path(f.name)
        g.save(tmp)
        g3 = NavGraph.load(tmp)
        assert g3.get_node("B").position == (1, 0, 0)
        assert g3.find_path("A", "C") == ["A", "B", "C"]
        tmp.unlink()
