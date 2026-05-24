"""DH kinematics for the dog_robot quadruped (pure NumPy, no ROS deps)."""
from dog_robot_kinematics.kinematics_dh import DHParams, fk_leg, ik_leg, mdh_transform
from dog_robot_kinematics.leg_config import LegConfig, LEGS

__all__ = [
    "DHParams",
    "fk_leg",
    "ik_leg",
    "mdh_transform",
    "LegConfig",
    "LEGS",
]
