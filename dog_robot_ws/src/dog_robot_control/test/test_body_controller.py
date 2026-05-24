import math
import numpy as np
import pytest

from dog_robot_kinematics.kinematics_dh import DHParams
from dog_robot_kinematics.leg_config import LEGS
from dog_robot_control.gait.gait_config import GaitConfig
from dog_robot_control.gait.body_controller import BodyController, BodyPose


DH = DHParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043)
GC = GaitConfig(0.15, 0.30, 0.03, 0.001, 0.15, 0.08, 0.50)


def test_pose_at_nominal_centers_foot_below_hip():
    """In body-at-hip frame: foot directly below hip on ground when body at
    nominal_height. Z = -(nominal_height + hip.z)."""
    bc = BodyController(LEGS, DH, GC)
    foot = bc.pose_command(BodyPose(0, 0, GC.nominal_height, 0, 0, 0))
    for i in range(4):
        expected_z = -(GC.nominal_height + LEGS[i].base_to_hip_xyz[2])
        assert foot[i][0] == pytest.approx(0.0, abs=1e-6)
        assert foot[i][1] == pytest.approx(0.0, abs=1e-6)
        assert foot[i][2] == pytest.approx(expected_z, abs=1e-6)


def test_pose_z_lowers_body_raises_foot():
    """Body below nominal: foot pulls up toward body (less negative z)."""
    bc = BodyController(LEGS, DH, GC)
    foot = bc.pose_command(BodyPose(0, 0, GC.nominal_height - 0.05, 0, 0, 0))
    for i in range(4):
        expected_z = -(GC.nominal_height - 0.05 + LEGS[i].base_to_hip_xyz[2])
        assert foot[i][2] == pytest.approx(expected_z, abs=1e-6)


def test_pose_x_translates_foot_opposite():
    """Body moves +x -> foot moves -x relative to hip."""
    bc = BodyController(LEGS, DH, GC)
    foot = bc.pose_command(BodyPose(0.04, 0, GC.nominal_height, 0, 0, 0))
    for i in range(4):
        assert foot[i][0] == pytest.approx(-0.04, abs=1e-6)


def test_pose_yaw_runs_without_nan():
    bc = BodyController(LEGS, DH, GC)
    foot = bc.pose_command(BodyPose(0, 0, GC.nominal_height, 0, 0, 0.2))
    for i in range(4):
        assert not np.any(np.isnan(foot[i]))
