"""Geometric constants and joint limits for the dog robot (meters, REP-103)."""

# Leg dimensions (m) — from TestIK/4leg.py, scaled mm → m
L1 = 0.0125
L2 = 0.04895
L3 = 0.109202
L4 = 0.115

# Body dimensions (m)
BODY_LENGTH = 0.200  # X dim: front-back hip distance
BODY_WIDTH  = 0.080  # Y dim: left-right hip distance
BODY_HEIGHT = 0.140  # foot below body in standing pose

# Naming
LEG_NAMES = ["FL", "FR", "BL", "BR"]
JOINT_SUFFIXES = ["hip_yaw", "thigh_pitch", "knee_pitch"]
JOINT_NAMES = [f"{leg}_{j}" for leg in LEG_NAMES for j in JOINT_SUFFIXES]

# Joint limits (rad) — per spec D7
JOINT_LIMITS = {}
for _leg in LEG_NAMES:
    JOINT_LIMITS[f"{_leg}_hip_yaw"]     = {"lower": -0.785, "upper":  0.785}
    JOINT_LIMITS[f"{_leg}_thigh_pitch"] = {"lower": -1.571, "upper":  1.571}
    JOINT_LIMITS[f"{_leg}_knee_pitch"]  = {"lower":  0.0,   "upper":  2.617}

# Hip joint positions in base_link frame (URDF, REP-103)
HIP_POSITIONS = {
    "FL": (+BODY_LENGTH/2, +BODY_WIDTH/2, 0.0),
    "FR": (+BODY_LENGTH/2, -BODY_WIDTH/2, 0.0),
    "BL": (-BODY_LENGTH/2, +BODY_WIDTH/2, 0.0),
    "BR": (-BODY_LENGTH/2, -BODY_WIDTH/2, 0.0),
}

# Nominal foot position offset from hip in body frame when joints=0 (m)
# Foot hangs L3+L4 below + L2 outward laterally
NOMINAL_FOOT_OFFSET_FROM_HIP = {
    "FL": (0.0, +L2,  -(L3 + L4)),
    "FR": (0.0, -L2,  -(L3 + L4)),
    "BL": (0.0, +L2,  -(L3 + L4)),
    "BR": (0.0, -L2,  -(L3 + L4)),
}
