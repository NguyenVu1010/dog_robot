"""Body pose controller — port of CHAMP BodyController.

Given a requested body pose (translation x,y,z and orientation roll,pitch,yaw),
produce per-leg foot positions in the BODY-AT-HIP frame (axes parallel to
body frame, origin at the leg's hip joint). Foot starts at zero_stance and is
translated/rotated opposite to the requested body motion (legs stay grounded
while the body moves). The final transformToHip subtracts the hip position
vector, so the output frame origin is at the hip joint.
"""
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from dog_robot_kinematics.kinematics_dh import DHParams
from dog_robot_kinematics.leg_config import LegConfig
from dog_robot_control.gait.gait_config import GaitConfig, zero_stance


@dataclass(frozen=True)
class BodyPose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


def _Rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def _Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def _Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])


class BodyController:
    def __init__(self, legs: Sequence[LegConfig], dh: DHParams,
                 gait: GaitConfig) -> None:
        self.legs = list(legs)
        self.dh = dh
        self.gait = gait
        self._zero_stance = [zero_stance(L, dh, gait) for L in self.legs]

    def pose_command(self, req: BodyPose) -> List[np.ndarray]:
        """Return list of 4 foot positions in body-at-hip frame."""
        out: List[np.ndarray] = []
        for i, L in enumerate(self.legs):
            z0 = self._zero_stance[i]
            tx = -req.x
            ty = -req.y
            tz_raw = -(z0[2] + req.z)
            max_tz = -z0[2] * 0.65
            tz = max(0.0, min(tz_raw, max_tz))

            foot = z0 + np.array([tx, ty, tz])
            R = _Rz(-req.yaw) @ _Ry(-req.pitch) @ _Rx(-req.roll)
            foot = R @ foot

            hip = np.array(L.base_to_hip_xyz)
            foot = foot - hip
            out.append(foot)
        return out
