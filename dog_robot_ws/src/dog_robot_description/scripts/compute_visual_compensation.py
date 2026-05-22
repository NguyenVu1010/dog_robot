#!/usr/bin/env python3
"""Compute mesh visual origin (xyz, rpy) so STL meshes (authored in the OLD URDF
link frames) render in the same world location under the NEW DH-aligned link
frames.

For each link, compensation = inv(T_new_link_in_parent) * T_old_link_in_parent
where both Ts are evaluated with all joint angles at 0.

Prints, per leg, a block of 4 lines suitable to paste into dog_robot.urdf.xacro
as <xacro:leg ... mesh_*_xyz=... mesh_*_rpy=... .../>
"""
import math

import numpy as np


def Rx(a): c,s=math.cos(a),math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def Ry(a): c,s=math.cos(a),math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def Rz(a): c,s=math.cos(a),math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])


def H(R, t):
    M = np.eye(4)
    M[:3,:3] = R
    M[:3, 3] = t
    return M


def rpy(R):
    sy = -R[2,0]
    if abs(sy) < 0.9999999:
        p = math.asin(sy)
        r = math.atan2(R[2,1], R[2,2])
        y = math.atan2(R[1,0], R[0,0])
    else:
        p = math.copysign(math.pi/2, sy)
        r = 0.0
        y = math.atan2(-R[0,1], R[1,1])
    return (r, p, y)


def urdf_origin(xyz, rpy_tuple):
    """URDF origin T = T_xyz * R_rpy."""
    r,p,y = rpy_tuple
    R = Rz(y) @ Ry(p) @ Rx(r)
    return H(R, np.array(xyz))


# Old URDF (committed state, CHAMP-IK surgery applied) – per-leg parameters.
OLD = {
    "FL": dict(hip_xyz=( 0.07480, 0.04000, 0.03510), hip_rpy=(0,0,0),
               thigh_xyz=( 0.02520, 0.02536,-0.01317), thigh_rpy=(0,0.94261,0),
               knee_xyz=(0.0,0.04102,-0.10984),       knee_rpy=(0,-1.93175,0),
               foot_xyz=(0.0,-0.01922,-0.06773),      foot_rpy=(0,0,0)),
    "FR": dict(hip_xyz=( 0.07480,-0.04000, 0.03510), hip_rpy=(0,0,0),
               thigh_xyz=( 0.02520,-0.02570,-0.01250), thigh_rpy=(0,0.93698,0),
               knee_xyz=(0.0,-0.04270,-0.10920),      knee_rpy=(0,-1.91411,0),
               foot_xyz=(0.0,0.01826,-0.06802),       foot_rpy=(0,0,0)),
    "BL": dict(hip_xyz=(-0.07480, 0.04000, 0.03510), hip_rpy=(0,0,0),
               thigh_xyz=(-0.02520, 0.02536,-0.01318), thigh_rpy=(0,0.86151,0),
               knee_xyz=(0.0,0.04082,-0.10992),       knee_rpy=(0,-1.96488,0),
               foot_xyz=(0.0,-0.01906,-0.06742),      foot_rpy=(0,0,0)),
    "BR": dict(hip_xyz=(-0.07480,-0.04000, 0.03510), hip_rpy=(0,0,0),
               thigh_xyz=(-0.02520,-0.02570,-0.01250), thigh_rpy=(0,0.86324,0),
               knee_xyz=(0.0,-0.04270,-0.10920),      knee_rpy=(0,-1.95214,0),
               foot_xyz=(0.0,0.01842,-0.06838),       foot_rpy=(0,0,0)),
}

PI_2 = math.pi / 2
NEW_HIP_RPY = {"FL": (0, PI_2, 0), "FR": (0, PI_2, math.pi),
               "BL": (0, PI_2, 0), "BR": (0, PI_2, math.pi)}
L_HH, L_TH, L_SH = 0.02553, 0.11725, 0.07043


def main():
    for leg in ("FL", "FR", "BL", "BR"):
        O = OLD[leg]
        T_old_hip   = urdf_origin(O["hip_xyz"],   O["hip_rpy"])
        T_old_thigh = urdf_origin(O["thigh_xyz"], O["thigh_rpy"])
        T_old_shank = urdf_origin(O["knee_xyz"],  O["knee_rpy"])
        T_old_foot  = urdf_origin(O["foot_xyz"],  O["foot_rpy"])

        T_new_hip   = urdf_origin(O["hip_xyz"], NEW_HIP_RPY[leg])
        T_new_thigh = urdf_origin((L_HH, 0, 0), (-PI_2, 0, 0))
        T_new_shank = urdf_origin((L_TH, 0, 0), (0, 0, 0))
        T_new_foot  = urdf_origin((L_SH, 0, 0), (0, 0, 0))

        comp_hip   = np.linalg.inv(T_new_hip)   @ T_old_hip
        comp_thigh = np.linalg.inv(T_new_thigh) @ T_old_thigh
        comp_shank = np.linalg.inv(T_new_shank) @ T_old_shank
        comp_foot  = np.linalg.inv(T_new_foot)  @ T_old_foot

        def fmt(T, name):
            xyz = tuple(round(v, 5) for v in T[:3, 3])
            r,p,y = rpy(T[:3, :3])
            return (f'  mesh_{name}_xyz="{xyz[0]} {xyz[1]} {xyz[2]}" '
                    f'mesh_{name}_rpy="{r:.5f} {p:.5f} {y:.5f}"')

        print(f"<!-- {leg} -->")
        print(fmt(comp_hip,   "hip"))
        print(fmt(comp_thigh, "thigh"))
        print(fmt(comp_shank, "shank"))
        print(fmt(comp_foot,  "foot"))
        print()


if __name__ == "__main__":
    main()
