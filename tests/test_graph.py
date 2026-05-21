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


from navigation.graph import EdgeMeta, View


class TestEdgeMeta:
    def test_default_render_is_true_and_views_empty(self):
        meta = EdgeMeta()
        assert meta.render is True
        assert meta.render_forward is True
        assert meta.views == []

    def test_view_accepts_roll_in_range(self):
        v1 = View(roll_deg=0)
        v2 = View(roll_deg=359.5)
        assert v1.roll_deg == 0
        assert v2.roll_deg == 359.5

    def test_view_rejects_roll_out_of_range(self):
        with pytest.raises(ValueError):
            View(roll_deg=-0.1)
        with pytest.raises(ValueError):
            View(roll_deg=360.0)


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
        assert g.get_edge_meta("B", "A").render is False  # symmetric

    def test_set_edge_render_forward_round_trip(self):
        g = self._g()
        g.set_edge_render_forward("A", "B", False)
        m = g.get_edge_meta("A", "B")
        assert m.render is True              # global render still on
        assert m.render_forward is False
        assert g.get_edge_meta("B", "A").render_forward is False  # symmetric

    def test_set_edge_render_forward_preserves_other_fields(self):
        g = self._g()
        g.set_edge_views("A", "B", [View(roll_deg=90)])
        g.set_edge_render_forward("A", "B", False)
        m = g.get_edge_meta("A", "B")
        assert m.render_forward is False
        assert [v.roll_deg for v in m.views] == [90]

    def test_set_edge_views_rejects_more_than_three(self):
        g = self._g()
        with pytest.raises(ValueError):
            g.set_edge_views("A", "B", [
                View(roll_deg=0), View(roll_deg=90),
                View(roll_deg=180), View(roll_deg=270),
            ])

    def test_set_edge_views_rejects_duplicate_canonical_key(self):
        g = self._g()
        with pytest.raises(ValueError):
            g.set_edge_views("A", "B", [View(roll_deg=90.4), View(roll_deg=89.6)])

    def test_set_edge_views_on_missing_edge_raises(self):
        g = self._g()
        with pytest.raises(KeyError):
            g.set_edge_views("A", "C", [View(roll_deg=0)])

    def test_views_canonical_key_helper(self):
        from navigation.graph import view_canonical_key
        assert view_canonical_key(View(roll_deg=89.6)) == 90
        assert view_canonical_key(View(roll_deg=90.4)) == 90
        assert view_canonical_key(View(roll_deg=359.6)) == 0  # wraps


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
        assert "render_forward" not in edge
        assert "views" not in edge

    def test_render_forward_round_trip(self):
        g = NavGraph()
        g.add_node("A", (0, 0, 0))
        g.add_node("B", (1, 0, 0))
        g.add_edge("A", "B")
        g.set_edge_render_forward("A", "B", False)
        d = g.to_dict()
        assert d["edges"][0]["render_forward"] is False
        g2 = NavGraph.from_dict(d)
        assert g2.get_edge_meta("A", "B").render_forward is False

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
        g.set_edge_views("A", "B", [View(roll_deg=10), View(roll_deg=270)])
        g2 = self._round_trip(g)
        views = g2.get_edge_meta("A", "B").views
        assert [v.roll_deg for v in views] == [10, 270]

    def test_from_dict_validates_bad_meta(self):
        bad = {
            "nodes": [{"id": "A", "position": [0, 0, 0]}, {"id": "B", "position": [1, 0, 0]}],
            "edges": [{"from": "A", "to": "B", "views": [{"roll_deg": 400}]}],
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


class TestDirectionMirror:
    def test_views_in_direction_forward_returns_as_stored(self):
        from navigation.graph import views_in_traversal_direction
        views = [View(roll_deg=90), View(roll_deg=45)]
        out = views_in_traversal_direction(views, reversed_=False)
        assert [v.roll_deg for v in out] == [90, 45]

    def test_views_in_direction_reverse_mirrors_roll(self):
        from navigation.graph import views_in_traversal_direction
        # 90 (right) → 270 (left); 45 → 315; 0 and 180 invariant
        views = [View(roll_deg=90), View(roll_deg=45), View(roll_deg=0), View(roll_deg=180)]
        out = views_in_traversal_direction(views, reversed_=True)
        assert [v.roll_deg for v in out] == [270, 315, 0, 180]
