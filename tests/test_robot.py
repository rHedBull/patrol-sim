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


# ── Characterization tests (pin behavior before refactoring) ─────────────────


def test_robot_step_crosses_node_boundary():
    """When `step()` consumes more than one segment in a single dt, it must
    advance `current_node` as it passes through interior nodes and continue
    with the remainder distance. Pins behavior of the inner-loop traversal
    used by `Robot.step` (navigation/robot.py:49-72).
    """
    g = _make_line_graph()
    # speed=15 over dt=1 → step distance 15 units. A→B is 10, so we should
    # arrive at B and continue 5 more units toward C.
    robot = Robot(g, "A", speed=15.0)
    robot.set_target("C")

    robot.step(1.0)

    assert not robot.is_idle()
    assert robot.current_node == "B", "must update current_node when passing through"
    assert robot.position[0] == pytest.approx(15.0, abs=1e-6)
    assert robot.position[1] == pytest.approx(0.0, abs=1e-6)
    assert robot.position[2] == pytest.approx(0.0, abs=1e-6)


def test_robot_accepts_external_path_injection():
    """Pin the pattern used by `server.handle_plan_command` (server.py:574-577),
    which sets `current_node`, `_path`, and `_path_index = 1` directly to
    drive the robot through a server-computed multi-leg plan. A future refactor
    will replace this with a public `Robot.assume_path(...)` — this test
    documents the current contract that those three attributes must stay in
    sync for `is_idle()` / `step()` to work.
    """
    g = _make_line_graph()
    robot = Robot(g, "A", speed=5.0)

    full = ["A", "B", "C"]
    robot.current_node = full[0]
    robot._path = list(full)
    robot._path_index = 1

    assert not robot.is_idle()
    robot.step(1.0)

    # 5 units along A→B.
    assert robot.position[0] == pytest.approx(5.0, abs=1e-6)
    assert robot.position[1] == pytest.approx(0.0, abs=1e-6)
    assert robot.current_node == "A"  # still on the A segment


def test_robot_yaw_updates_on_step():
    """Pin the yaw convention: `yaw = atan2(-dx, -dz)` for movement direction
    (navigation/robot.py:58). Frontend/orbit code assumes this orientation.
    """
    g = _make_line_graph()
    robot = Robot(g, "A", speed=1.0)
    robot.set_target("B")
    robot.step(0.5)
    # Movement is along +X, so dx>0, dz=0 → atan2(-dx, 0) = -pi/2.
    assert robot.yaw == pytest.approx(-math.pi / 2, abs=1e-6)

