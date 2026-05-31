"""Body-level command state: cmd_vel + gait phase clock.

Plain Python (no ROS). The ROS node feeds Twist values to `on_cmd_vel` and
ticks `tick(dt)` on its timer; LegDriver pulls `body_vel_xy()` and
`phase(leg_name)` each tick. Trot phase pattern: FL/BR together, FR/BL
together 180 deg out of phase.
"""
from __future__ import annotations
from typing import Tuple


class BodyCommander:
    # Trot diagonals: FL & BR move together; FR & BL are pi out of phase.
    PHASE_OFFSETS = {"FL": 0.0, "BR": 0.0, "FR": 0.5, "BL": 0.5}

    def __init__(self, step_freq: float = 1.5):
        self.step_freq = float(step_freq)
        self._t = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

    def on_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        self._vx = float(linear_x)
        self._vy = float(linear_y)
        self._wz = float(angular_z)

    def tick(self, dt: float) -> None:
        self._t += float(dt)

    def phase(self, leg_name: str) -> float:
        offset = self.PHASE_OFFSETS[leg_name]
        return (self._t * self.step_freq + offset) % 1.0

    def body_vel_xy(self) -> Tuple[float, float]:
        return (self._vx, self._vy)

    def body_yaw_rate(self) -> float:
        # Reserved for future use; current LegDriver does not consume it.
        return self._wz

    def time(self) -> float:
        return self._t
