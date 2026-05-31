"""Pure-NumPy kinematics for the dog_robot quadruped (no ROS deps).

Joint-attached frame kinematics derived from the URDF / FreeCAD geometry —
see docs/superpowers/specs/2026-05-26-joint-frame-export-design.md.
"""
from dog_robot_kinematics.kinematics_link import (
    LinkParams,
    load_link_params,
    fk_leg,
    ik_leg,
)

__all__ = [
    "LinkParams",
    "load_link_params",
    "fk_leg",
    "ik_leg",
]
