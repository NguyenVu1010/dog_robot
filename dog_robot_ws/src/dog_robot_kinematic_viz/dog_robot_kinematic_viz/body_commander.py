"""Body-level command state: cmd_vel + gait phase clock + body-height state.

Plain Python (no ROS). The ROS node feeds Twist values to `on_cmd_vel` and
ticks `tick(dt)` on its timer; LegDriver pulls `body_vel_xy()`,
`phase(leg_name)`, and `body_z()` each tick. Trot phase pattern: FL/BR
together, FR/BL together 180 deg out of phase.

`body_z` is integrated from `linear.z` (velocity, m/s) and clamped to
[body_z_min, body_z_max]; this class is the single source of truth for the
clamp, so downstream callers may assume the value they receive is in range.
"""
from __future__ import annotations
from typing import Tuple


class BodyCommander:
    # Trot diagonals: FL & BR move together; FR & BL are pi out of phase.
    PHASE_OFFSETS = {"FL": 0.0, "BR": 0.0, "FR": 0.5, "BL": 0.5}

    def __init__(self, step_freq: float = 1.5,
                 body_z_min: float = -0.04,
                 body_z_max: float = +0.04):
        self.step_freq = float(step_freq)
        self.body_z_min = float(body_z_min)
        self.body_z_max = float(body_z_max)
        self._t = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._wz = 0.0
        self._z = 0.0

    def on_cmd_vel(self, linear_x: float, linear_y: float,
                   linear_z: float, angular_z: float) -> None:
        self._vx = float(linear_x)
        self._vy = float(linear_y)
        self._vz = float(linear_z)
        self._wz = float(angular_z)

    def tick(self, dt: float) -> None:
        dt = float(dt)
        self._t += dt
        new_z = self._z + self._vz * dt
        if new_z > self.body_z_max:
            new_z = self.body_z_max
        elif new_z < self.body_z_min:
            new_z = self.body_z_min
        self._z = new_z

    def phase(self, leg_name: str) -> float:
        offset = self.PHASE_OFFSETS[leg_name]
        return (self._t * self.step_freq + offset) % 1.0

    def body_vel_xy(self) -> Tuple[float, float]:
        return (self._vx, self._vy)

    def body_yaw_rate(self) -> float:
        # Reserved for future use; current LegDriver does not consume it.
        return self._wz

    def body_z(self) -> float:
        return self._z

    def time(self) -> float:
        return self._t
