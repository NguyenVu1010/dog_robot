"""Rolling per-leg foot-tip trail for RViz visualization.

Each leg maintains a deque of recent foot positions (in base_link frame).
A helper builds a visualization_msgs/Marker LINE_STRIP from the trail.

Pure Python — no ROS state. The ROS node creates one FootTrail per leg,
appends a 3-vector each tick, and calls build_marker(...) to package the
buffer into a Marker for publishing.
"""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass
from typing import Deque, Tuple

import numpy as np


# Color per leg (r, g, b) in [0,1]. Standard quadruped convention.
LEG_COLORS: dict = {
    "FL": (1.0, 0.0, 0.0),    # red
    "FR": (0.0, 1.0, 0.0),    # green
    "BL": (0.0, 0.5, 1.0),    # blue
    "BR": (1.0, 1.0, 0.0),    # yellow
}


@dataclass
class FootTrail:
    name: str
    color: Tuple[float, float, float]
    max_points: int

    def __post_init__(self):
        if self.max_points <= 0:
            raise ValueError(f"max_points must be > 0, got {self.max_points}")
        self._buf: Deque[np.ndarray] = deque(maxlen=self.max_points)

    def append(self, point_xyz) -> None:
        p = np.asarray(point_xyz, dtype=float).reshape(3)
        self._buf.append(p)

    def points(self) -> list:
        return list(self._buf)

    def __len__(self) -> int:
        return len(self._buf)

    def clear(self) -> None:
        self._buf.clear()


def build_marker(trail: FootTrail,
                 frame_id: str,
                 marker_id: int,
                 stamp,
                 line_width: float = 0.005):
    """Package the trail into a visualization_msgs/Marker LINE_STRIP.

    `stamp` is a builtin_interfaces/Time (e.g. from
    `node.get_clock().now().to_msg()`). Returns a Marker ready to publish.
    """
    # Imported lazily so the module stays importable without ROS installed
    # (unit tests use only FootTrail; only build_marker needs the ROS msg).
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point

    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = "foot_trail"
    m.id = int(marker_id)
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0     # identity orientation
    m.scale.x = float(line_width)
    r, g, b = trail.color
    m.color.r = float(r)
    m.color.g = float(g)
    m.color.b = float(b)
    m.color.a = 1.0
    m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                for p in trail.points()]
    return m
