"""Tests for the Robot class."""

import math

import pytest

from navigation.graph import NavGraph
from navigation.robot import Robot


def _make_line_graph() -> NavGraph:
    """Create a simple 3-node line graph: A -- B -- C.

    A is at origin, B is 10 units along +X, C is 20 units along +X.
    """
    g = NavGraph()
    g.add_node("A", [0.0, 0.0, 0.0])
    g.add_node("B", [10.0, 0.0, 0.0])
    g.add_node("C", [20.0, 0.0, 0.0])
    g.add_edge("A", "B")
    g.add_edge("B", "C")
    return g


def test_robot_initial_state():
    g = _make_line_graph()
    robot = Robot(g, "A", speed=2.0)

    assert robot.current_node == "A"
    assert robot.position == [0.0, 0.0, 0.0]
    assert robot.speed == 2.0
    assert robot.yaw == 0.0
    assert robot.pitch == 0.0
    assert robot.is_idle()


def test_robot_set_target():
    g = _make_line_graph()
    robot = Robot(g, "A")

    path = robot.set_target("C")

    assert path is not None
    assert path[0] == "A"
    assert path[-1] == "C"
    assert not robot.is_idle()


def test_robot_step_moves_toward_target():
    g = _make_line_graph()
    robot = Robot(g, "A", speed=5.0)
    robot.set_target("B")

    # Step 1 second at speed 5 => should move 5 units toward B (at x=10).
    robot.step(1.0)

    assert not robot.is_idle()
    assert robot.position[0] == pytest.approx(5.0, abs=1e-6)
    assert robot.position[1] == pytest.approx(0.0, abs=1e-6)
    assert robot.position[2] == pytest.approx(0.0, abs=1e-6)


def test_robot_arrives_at_target():
    g = _make_line_graph()
    # Speed high enough to reach C (20 units) in one 1-second step.
    robot = Robot(g, "A", speed=100.0)
    robot.set_target("C")

    robot.step(1.0)

    assert robot.is_idle()
    assert robot.current_node == "C"
    assert robot.position == [20.0, 0.0, 0.0]

