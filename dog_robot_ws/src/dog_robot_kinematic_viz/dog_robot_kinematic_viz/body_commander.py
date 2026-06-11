"""Body-level command state: cmd_vel + gait phase clock + body-height + pitch.

Plain Python (no ROS). The ROS node feeds Twist values to `on_cmd_vel` and
ticks `tick(dt)` on its timer; LegDriver pulls `body_vel_xy()`,
`phase(leg_name)`, `body_z()`, and `pitch_amount()` each tick. Trot phase
pattern: FL/BR together, FR/BL together 180 deg out of phase.

`body_z` is integrated from `linear.z` (velocity, m/s) and clamped to
[body_z_min, body_z_max]; `pitch_amount` is integrated from `angular.y`
(velocity, m/s) and clamped to [pitch_min, pitch_max]. This class is the
single source of truth for both clamps.
"""
from __future__ import annotations
from typing import Tuple


class BodyCommander:
    # Trot diagonals: FL & BR move together; FR & BL are pi out of phase.
    PHASE_OFFSETS = {"FL": 0.0, "BR": 0.0, "FR": 0.5, "BL": 0.5}

    def __init__(self, step_freq: float = 1.5,
                 body_z_min: float = -0.03,
                 body_z_max: float = +0.03,
                 pitch_min: float = -0.05,
                 pitch_max: float = +0.05):
        self.step_freq = float(step_freq)
        self.body_z_min = float(body_z_min)
        self.body_z_max = float(body_z_max)
        self.pitch_min = float(pitch_min)
        self.pitch_max = float(pitch_max)
        self._t = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vz = 0.0
        self._wy = 0.0
        self._wz = 0.0
        self._z = 0.0
        self._pitch = 0.0

    def on_cmd_vel(self, linear_x: float, linear_y: float,
                   linear_z: float, angular_y: float,
                   angular_z: float) -> None:
        self._vx = float(linear_x)
        self._vy = float(linear_y)
        self._vz = float(linear_z)
        self._wy = float(angular_y)
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
        new_pitch = self._pitch + self._wy * dt
        if new_pitch > self.pitch_max:
            new_pitch = self.pitch_max
        elif new_pitch < self.pitch_min:
            new_pitch = self.pitch_min
        self._pitch = new_pitch

    def phase(self, leg_name: str) -> float:
        offset = self.PHASE_OFFSETS[leg_name]
        return (self._t * self.step_freq + offset) % 1.0

    def body_vel_xy(self) -> Tuple[float, float]:
        return (self._vx, self._vy)

    def body_yaw_rate(self) -> float:
        return self._wz

    def body_z(self) -> float:
        return self._z

    def pitch_amount(self) -> float:
        return self._pitch

    def time(self) -> float:
        return self._t
