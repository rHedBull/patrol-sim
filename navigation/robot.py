"""Robot that navigates a NavGraph using pathfinding."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from navigation.graph import NavGraph


class Robot:
    """A robot that moves along a NavGraph between waypoint nodes."""

    def __init__(self, graph: NavGraph, start_node: str, speed: float = 2.0) -> None:
        self.graph = graph
        self.current_node = start_node
        self.position: list[float] = list(graph.nodes[start_node])
        self.speed = speed
        self.yaw: float = 0.0
        self.pitch: float = 0.0
        self._path: list[str] = []
        self._path_index: int = 0

    def is_idle(self) -> bool:
        """Return True if the robot has no active path to follow."""
        return self._path_index >= len(self._path)

    def set_target(self, target_node: str) -> list[str] | None:
        """Plan a path from current_node to target_node.

        Returns the path (list of node ids) or None if no path exists.
        """
        path = self.graph.find_path(self.current_node, target_node)
        if path is None:
            return None
        self._path = path
        # Start moving toward the second node (index 1); index 0 is current.
        self._path_index = 1
        return list(path)

    def assume_path(self, path: list[str]) -> None:
        """Adopt an externally-computed path and start driving it.

        Used when a caller (e.g. the multi-leg `plan_command` WS handler)
        has already chained A* legs together and just wants the robot to
        follow the result. `current_node` snaps to `path[0]` and movement
        starts at `path[1]`. Position is NOT teleported — `step()` will
        animate from wherever the robot currently is toward `path[1]`.
        """
        if not path:
            raise ValueError("assume_path requires a non-empty path")
        self.current_node = path[0]
        self._path = list(path)
        self._path_index = 1

    def step(self, dt: float) -> None:
        """Advance the robot along its path by *dt* seconds."""
        if self.is_idle():
            return

        remaining_dist = self.speed * dt

        while remaining_dist > 0.0 and not self.is_idle():
            target_pos = self.graph.nodes[self._path[self._path_index]]
            dx = target_pos[0] - self.position[0]
            dy = target_pos[1] - self.position[1]
            dz = target_pos[2] - self.position[2]
            seg_dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            # Update yaw toward movement direction.
            if seg_dist > 1e-9:
                self.yaw = math.atan2(-dx, -dz)

            if seg_dist <= remaining_dist:
                # Arrive at the next node.
                self.position = list(target_pos)
                self.current_node = self._path[self._path_index]
                remaining_dist -= seg_dist
                self._path_index += 1
            else:
                # Move partially along the segment.
                frac = remaining_dist / seg_dist
                self.position[0] += dx * frac
                self.position[1] += dy * frac
                self.position[2] += dz * frac
                remaining_dist = 0.0
