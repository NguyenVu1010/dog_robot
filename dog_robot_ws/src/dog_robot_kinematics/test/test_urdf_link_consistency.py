"""Verify the FK chain built from dog_robot.urdf.xacro joint origins matches
kinematics_link.fk_leg for all four legs. Both derive from the same
config/link_params.yaml + urdf_joints.yaml, so they must agree exactly.
"""
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from dog_robot_kinematics.kinematics_link import load_link_params, fk_leg

DESC = (Path(__file__).resolve().parents[2] / "dog_robot_description")
URDF_XACRO = DESC / "urdf" / "dog_robot.urdf.xacro"
LINK_CFG = DESC / "config" / "link_params.yaml"

pytestmark = pytest.mark.skipif(
    shutil.which("xacro") is None and shutil.which("ros2") is None,
    reason="xacro not available")


def _expand_urdf() -> str:
    if shutil.which("xacro"):
        cmd = ["xacro", str(URDF_XACRO)]
    else:
        cmd = ["ros2", "run", "xacro", "xacro", str(URDF_XACRO)]
    return subprocess.check_output(cmd, text=True)


def _rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _T(R, t=np.zeros(3)):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def _Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _joint_origin(root, name):
    # ros2_control adds <joint name=...> elements without an <origin>; skip those.
    for j in root.iter("joint"):
        if j.get("name") == name:
            o = j.find("origin")
            if o is None:
                continue
            xyz = np.array([float(v) for v in o.get("xyz").split()])
            rpy = np.array([float(v) for v in o.get("rpy").split()])
            return xyz, rpy
    raise AssertionError(f"kinematic joint {name!r} with <origin> not found")


@pytest.mark.parametrize("leg", ["FL", "FR", "BL", "BR"])
def test_urdf_fk_matches_kinematics_link(leg):
    root = ET.fromstring(_expand_urdf())
    p = load_link_params(LINK_CFG, leg)

    xyz_h, rpy_h = _joint_origin(root, f"{leg}_hip_yaw")
    xyz_t, rpy_t = _joint_origin(root, f"{leg}_thigh_pitch")
    xyz_k, rpy_k = _joint_origin(root, f"{leg}_knee_pitch")
    xyz_f, rpy_f = _joint_origin(root, f"{leg}_foot_fixed")

    T_base_hip = _T(_rpy_matrix(rpy_h), xyz_h)

    rng = np.random.default_rng(7)
    for _ in range(10):
        q = (float(rng.uniform(-0.3, 0.3)),
             float(rng.uniform(0.2, 1.0)),
             float(rng.uniform(-1.2, -0.2)))
        # URDF chain from base
        T = (T_base_hip @ _T(_Rz(q[0]))
             @ _T(_rpy_matrix(rpy_t), xyz_t) @ _T(_Rz(q[1]))
             @ _T(_rpy_matrix(rpy_k), xyz_k) @ _T(_Rz(q[2]))
             @ _T(_rpy_matrix(rpy_f), xyz_f))
        foot_base_urdf = T[:3, 3]

        # kinematics_link FK gives foot in the hip-yaw frame; lift to base.
        foot_hip = fk_leg(p, q)
        foot_base_kin = (T_base_hip @ np.array([*foot_hip, 1.0]))[:3]

        np.testing.assert_allclose(foot_base_urdf, foot_base_kin, atol=1e-6)
