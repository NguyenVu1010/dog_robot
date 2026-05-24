"""End-to-end gait pipeline test using a mocked clock.

Drives BodyController + LegController for 3 strides at vx=0.1 and verifies:
- ik_leg succeeds for every leg every tick (foot stays in reach).
- joint angles stay within URDF limits.
- trot timing: leg 0 and 3 phase signals match within 1 tick.
"""
import math
import numpy as np
import pytest

from dog_robot_kinematics.kinematics_dh import DHParams, ik_leg
from dog_robot_kinematics.leg_config import LEGS
from dog_robot_control.gait.gait_config import GaitConfig
from dog_robot_control.gait.body_controller import BodyController, BodyPose
from dog_robot_control.gait.leg_controller import LegController, Velocity


DH = DHParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043)
GC = GaitConfig(nominal_height=0.15, stance_duration=0.30, swing_height=0.03,
                stance_depth=0.001, max_linear_velocity_x=0.15,
                max_linear_velocity_y=0.08, max_angular_velocity_z=0.50)


JOINT_LIMITS = {
    "hip":   (-0.785,  0.785),
    "thigh": (-1.571,  1.571),
    "knee":  (0.0,     2.617),
}


def _Rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def _Ry(a): c, s = math.cos(a), math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def _Rz(a): c, s = math.cos(a), math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def _body_to_hip(foot_body_at_hip, leg):
    r, p, y = leg.base_to_hip_rpy
    R_bh = _Rz(y) @ _Ry(p) @ _Rx(r)
    return R_bh.T @ foot_body_at_hip


def test_walking_pipeline_no_ik_failures():
    bc = BodyController(LEGS, DH, GC)
    lc = LegController(LEGS, DH, GC)
    dt = 0.02
    n_ticks = int((GC.stance_duration + 0.25) * 3 / dt)  # ~3 strides
    failures = []
    leg0_track = []
    leg3_track = []
    for k in range(n_ticks):
        t = k * dt
        feet = bc.pose_command(BodyPose(0, 0, GC.nominal_height, 0, 0, 0))
        feet = lc.velocity_command(feet, Velocity(0.1, 0, 0), t)
        for i, L in enumerate(LEGS):
            foot_h = _body_to_hip(feet[i], L)
            try:
                q = ik_leg(DH, foot_h, knee_direction=+1)
                if not (JOINT_LIMITS["hip"][0]   <= q[0] <= JOINT_LIMITS["hip"][1]):
                    failures.append((k, L.name, "hip", q[0]))
                if not (JOINT_LIMITS["thigh"][0] <= q[1] <= JOINT_LIMITS["thigh"][1]):
                    failures.append((k, L.name, "thigh", q[1]))
                if not (JOINT_LIMITS["knee"][0]  <= q[2] <= JOINT_LIMITS["knee"][1]):
                    failures.append((k, L.name, "knee", q[2]))
            except ValueError as e:
                failures.append((k, L.name, "ik_value", str(e)))
        leg0_track.append(lc.phase_generator.stance_phase_signal[0])
        leg3_track.append(lc.phase_generator.stance_phase_signal[3])
    assert not failures, f"IK or limit failures: {failures[:5]}"
    diffs = [abs(a - b) for a, b in zip(leg0_track, leg3_track)]
    assert max(diffs) < 1e-9, f"trot in-phase broken: max diff {max(diffs)}"
