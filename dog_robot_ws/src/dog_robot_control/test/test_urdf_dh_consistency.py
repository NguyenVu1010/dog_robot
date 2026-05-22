"""Verify FK chain built from dog_robot.urdf.xacro matches kinematics_dh.fk_leg
for all 4 legs at random joint configurations.
"""
import os
import subprocess
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from dog_robot_control.kinematics_dh import DHParams, fk_leg
from dog_robot_control.leg_config import LEGS

DH = DHParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043)
TOL = 1e-4   # meters


def _xacro_to_urdf(xacro_path: str) -> str:
    out = subprocess.check_output([
        "xacro", xacro_path, "controllers_yaml_path:=/tmp/dummy.yaml",
    ])
    return out.decode("utf-8")


def _Rx(a): c,s=math.cos(a),math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def _Ry(a): c,s=math.cos(a),math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def _Rz(a): c,s=math.cos(a),math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def _T(R, t):
    M = np.eye(4); M[:3,:3]=R; M[:3,3]=t; return M

def _urdf_origin(elem):
    o = elem.find("origin")
    xyz = (0.0,0.0,0.0); rpy = (0.0,0.0,0.0)
    if o is not None:
        if o.get("xyz"): xyz = tuple(float(v) for v in o.get("xyz").split())
        if o.get("rpy"): rpy = tuple(float(v) for v in o.get("rpy").split())
    r,p,y = rpy
    return _T(_Rz(y) @ _Ry(p) @ _Rx(r), np.array(xyz))


def _joint_axis(elem):
    a = elem.find("axis")
    return tuple(float(v) for v in a.get("xyz").split())


def _axis_angle(axis, theta):
    ax = np.array(axis) / np.linalg.norm(axis)
    K = np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
    return np.eye(3) + math.sin(theta)*K + (1-math.cos(theta))*K@K


def _urdf_fk_foot(urdf_root, leg, theta):
    """Walk URDF joint chain from base_link to {leg}_foot_link."""
    joints = {j.get("name"): j for j in urdf_root.findall("joint")}
    chain = [(f"{leg}_hip_yaw",     theta[0]),
             (f"{leg}_thigh_pitch", theta[1]),
             (f"{leg}_knee_pitch",  theta[2]),
             (f"{leg}_foot_fixed",  0.0)]
    T = np.eye(4)
    for jname, q in chain:
        j = joints[jname]
        T_origin = _urdf_origin(j)
        if j.get("type") == "fixed":
            T = T @ T_origin
        else:
            axis = _joint_axis(j)
            R = _axis_angle(axis, q)
            T_q = _T(R, np.zeros(3))
            T = T @ T_origin @ T_q
    return T


def _load_urdf_root():
    xacro = os.path.expanduser(
        "~/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro")
    urdf_str = _xacro_to_urdf(xacro)
    return ET.fromstring(urdf_str)


def test_urdf_fk_matches_dh_fk_at_zero():
    root = _load_urdf_root()
    for L in LEGS:
        T_foot = _urdf_fk_foot(root, L.name, (0.0, 0.0, 0.0))
        foot_world = T_foot[:3, 3]
        r,p,y = L.base_to_hip_rpy
        R_bh = _Rz(y) @ _Ry(p) @ _Rx(r)
        T_bh = _T(R_bh, np.array(L.base_to_hip_xyz))
        foot_hip = (np.linalg.inv(T_bh) @ np.append(foot_world, 1.0))[:3]
        foot_dh = fk_leg(DH, (0.0, 0.0, 0.0))
        assert np.allclose(foot_hip, foot_dh, atol=TOL), (L.name, foot_hip, foot_dh)


def test_urdf_fk_matches_dh_fk_random():
    root = _load_urdf_root()
    rng = np.random.default_rng(seed=0)
    for L in LEGS:
        for _ in range(10):
            theta = (rng.uniform(-0.5, 0.5),
                     rng.uniform(-0.5, 0.5),
                     rng.uniform( 0.3, 1.5))
            T_foot = _urdf_fk_foot(root, L.name, theta)
            foot_world = T_foot[:3, 3]
            r,p,y = L.base_to_hip_rpy
            T_bh = _T(_Rz(y) @ _Ry(p) @ _Rx(r), np.array(L.base_to_hip_xyz))
            foot_hip = (np.linalg.inv(T_bh) @ np.append(foot_world, 1.0))[:3]
            foot_dh = fk_leg(DH, theta)
            assert np.allclose(foot_hip, foot_dh, atol=TOL), (L.name, theta, foot_hip, foot_dh)
