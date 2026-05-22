#!/usr/bin/env python3
"""Compute symmetric DH link lengths from existing per-leg CAD values in dog_robot.urdf.xacro.

L_hh = mean |thigh_xyz Y component| (hip-to-thigh common normal)
L_th = mean magnitude(knee_xyz in original frame) (thigh length)
L_sh = mean magnitude(foot_xyz in original frame) (shank length)
"""
import math

# Values copied from current dog_robot.urdf.xacro (committed state).
LEGS = {
    "FL": dict(thigh_xyz=( 0.02520,  0.02536, -0.01317),
               knee_xyz =( 0.0,      0.04102, -0.10984),
               foot_xyz =( 0.0,     -0.01922, -0.06773)),
    "FR": dict(thigh_xyz=( 0.02520, -0.02570, -0.01250),
               knee_xyz =( 0.0,     -0.04270, -0.10920),
               foot_xyz =( 0.0,      0.01826, -0.06802)),
    "BL": dict(thigh_xyz=(-0.02520,  0.02536, -0.01318),
               knee_xyz =( 0.0,      0.04082, -0.10992),
               foot_xyz =( 0.0,     -0.01906, -0.06742)),
    "BR": dict(thigh_xyz=(-0.02520, -0.02570, -0.01250),
               knee_xyz =( 0.0,     -0.04270, -0.10920),
               foot_xyz =( 0.0,      0.01842, -0.06838)),
}

def mag(v):
    return math.sqrt(sum(x * x for x in v))

L_hh = sum(abs(L["thigh_xyz"][1]) for L in LEGS.values()) / 4
L_th = sum(mag(L["knee_xyz"]) for L in LEGS.values()) / 4
L_sh = sum(mag(L["foot_xyz"]) for L in LEGS.values()) / 4

print(f"L_hh = {L_hh:.5f}  # m, hip-to-thigh common normal")
print(f"L_th = {L_th:.5f}  # m, thigh length")
print(f"L_sh = {L_sh:.5f}  # m, shank length")
