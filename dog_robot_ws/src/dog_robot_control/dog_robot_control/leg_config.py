"""Per-leg configuration: base→hip rigid transform + mirror sign for IK."""
from dataclasses import dataclass
from typing import Tuple
import math


@dataclass(frozen=True)
class LegConfig:
    name: str                              # "FL" | "FR" | "BL" | "BR"
    base_to_hip_xyz: Tuple[float, float, float]
    base_to_hip_rpy: Tuple[float, float, float]
    mirror: int                            # +1 left, -1 right


_PI_2 = math.pi / 2
_PI = math.pi

LEGS: Tuple[LegConfig, ...] = (
    LegConfig("FL", ( 0.07480,  0.04000, 0.03510), (0.0, _PI_2, 0.0), +1),
    LegConfig("FR", ( 0.07480, -0.04000, 0.03510), (0.0, _PI_2, _PI), -1),
    LegConfig("BL", (-0.07480,  0.04000, 0.03510), (0.0, _PI_2, 0.0), +1),
    LegConfig("BR", (-0.07480, -0.04000, 0.03510), (0.0, _PI_2, _PI), -1),
)


def get_leg(name: str) -> LegConfig:
    for L in LEGS:
        if L.name == name:
            return L
    raise KeyError(name)
