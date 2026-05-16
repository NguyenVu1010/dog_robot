"""Foot trajectory: linear stance + Bezier-4 swing.

Position returned is an offset (dx, dy, dz) from nominal foot position
in body frame. Caller adds to nominal_foot to get target.
"""
import numpy as np


class FootPlanner:
    def __init__(self, cycle_time=0.4, duty_factor=0.5, step_height=0.05,
                 max_stride=0.10):
        self.T = cycle_time
        self.beta = duty_factor
        self.H = step_height
        self.max_stride = max_stride

    def _stride(self, vel):
        """Stride vector in body-XY plane (lift only Z)."""
        vx, vy, _ = vel
        stride_x = np.clip(vx * self.T * self.beta, -self.max_stride, self.max_stride)
        stride_y = np.clip(vy * self.T * self.beta, -self.max_stride, self.max_stride)
        return stride_x, stride_y

    def foot_position(self, phase, vel):
        """Return (x, y, z) offset from nominal foot position.

        phase ∈ [0, 1). vel = (vx, vy, vz_unused) in body frame.
        """
        stride_x, stride_y = self._stride(vel)

        if phase < self.beta:
            # STANCE: linear from +stride/2 to -stride/2
            t = phase / self.beta   # 0..1
            return (stride_x * (0.5 - t), stride_y * (0.5 - t), 0.0)
        else:
            # SWING: Bezier-4 with 5 control points
            t = (phase - self.beta) / (1 - self.beta)   # 0..1
            # Control points (x, y, z):
            P0 = np.array([-stride_x/2, -stride_y/2, 0.0])
            P1 = np.array([-stride_x/2, -stride_y/2, self.H])
            P2 = np.array([0.0,         0.0,         self.H])
            P3 = np.array([+stride_x/2, +stride_y/2, self.H])
            P4 = np.array([+stride_x/2, +stride_y/2, 0.0])
            b = ( ((1-t)**4)*P0
                + 4*((1-t)**3)*t*P1
                + 6*((1-t)**2)*(t**2)*P2
                + 4*(1-t)*(t**3)*P3
                + (t**4)*P4 )
            return tuple(b)
