"""Per-leg base->hip transform loaded from dog_robot_description/urdf_joints.yaml.

`dog_robot_kinematics.leg_config.LEGS` was removed in the cleanup commit
because its simplified rpy = (0, pi/2, 0) does not match the joint-frame URDF
(which has a per-leg yaw splay). The new LegGeom carries the exact transform
the URDF uses.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple, Union

import numpy as np


LEG_NAMES: Tuple[str, ...] = ("FL", "FR", "BL", "BR")


@dataclass(frozen=True)
class LegGeom:
    name: str
    base_to_hip_xyz: np.ndarray            # (3,)
    base_to_hip_rpy: Tuple[float, float, float]
    R_base_to_hip: np.ndarray              # (3,3) = Rz . Ry . Rx


def _rpy_to_matrix(rpy: Tuple[float, float, float]) -> np.ndarray:
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def load_leg_geoms(yaml_path: Union[str, Path]) -> Dict[str, LegGeom]:
    """Load all 4 leg base->hip transforms from urdf_joints.yaml."""
    import yaml
    cfg = yaml.safe_load(Path(yaml_path).read_text())
    per_leg = cfg["per_leg"]
    out: Dict[str, LegGeom] = {}
    for name in LEG_NAMES:
        if name not in per_leg:
            raise KeyError(f"per_leg.{name} missing in {yaml_path}")
        blk = per_leg[name]
        rpy = tuple(float(v) for v in blk["base_to_hip_rpy"])
        xyz = np.asarray(blk["base_to_hip_xyz"], dtype=float)
        out[name] = LegGeom(name=name, base_to_hip_xyz=xyz,
                            base_to_hip_rpy=rpy,
                            R_base_to_hip=_rpy_to_matrix(rpy))
    return out
