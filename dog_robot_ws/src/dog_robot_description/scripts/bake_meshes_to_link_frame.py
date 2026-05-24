#!/usr/bin/env python3
"""Bake the visual <origin xyz rpy/> transform into each leg STL's vertex
coordinates so the URDF can use identity visual origins.

Input  : meshes/visual/<link>.stl  (vertices in the historical STL frame)
Output : meshes/visual_dh/<link>.stl (vertices in the new DH link frame)

For every link we read the mesh visual origin currently written in
dog_robot.urdf.xacro and apply it to the STL vertices once:

    v_new = R @ v_old + t

After this, the URDF's <visual><origin xyz="0 0 0" rpy="0 0 0"/> places the
mesh correctly without any per-link compensation math.
"""
import math
import os
from pathlib import Path

import numpy as np
from stl import mesh as stlmesh


def Rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def Ry(a): c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def Rz(a): c, s = math.cos(a), math.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_R(rpy):
    r, p, y = rpy
    return Rz(y) @ Ry(p) @ Rx(r)


# Visual origins read verbatim from dog_robot.urdf.xacro (state fd3a8db).
# Per-leg entries: {link_name: (xyz_tuple, rpy_tuple)}.
LEG_PARAMS = {
    "FL": {
        "hip":   ((0.0,      0.0,     0.0),     ( 0.00000, -1.57080,  0.00000)),
        "thigh": ((-0.01236, -0.0252, 0.02536), ( 1.57080,  0.00000, -1.57080)),
        "shank": ((-0.06506, 0.06367, 0.06638), ( 1.57080,  0.00000, -1.57080)),
        "foot":  ((-0.09828, 0.00708, 0.04716), ( 1.57080,  0.00000, -1.57080)),
    },
    "FR": {
        "hip":   ((0.0,      0.0,     0.0),     ( 0.00000,  1.57080, -3.14159)),
        "thigh": ((-0.01303, 0.0252,  0.0257),  (-1.57080,  0.00000,  1.57080)),
        "shank": ((-0.06561, -0.06279, 0.0684), (-1.57080,  0.00000,  1.57080)),
        "foot":  ((-0.09799, -0.00641, 0.05014), (-1.57080, 0.00000,  1.57080)),
    },
    "BL": {
        "hip":   ((0.0,      0.0,     0.0),     ( 0.00000, -1.57080,  0.00000)),
        "thigh": ((-0.01235, 0.0252,  0.02536), ( 1.57080,  0.00000, -1.57080)),
        "shank": ((-0.05801, 0.10861, 0.06618), ( 1.57080,  0.00000, -1.57080)),
        "foot":  ((-0.09806, 0.04842, 0.04712), ( 1.57080,  0.00000, -1.57080)),
    },
    "BR": {
        "hip":   ((0.0,      0.0,     0.0),     ( 0.00000,  1.57080, -3.14159)),
        "thigh": ((-0.01303, -0.0252, 0.0257),  (-1.57080,  0.00000,  1.57080)),
        "shank": ((-0.0593,  -0.10819, 0.0684), (-1.57080,  0.00000,  1.57080)),
        "foot":  ((-0.09804, -0.04759, 0.04998), (-1.57080, 0.00000,  1.57080)),
    },
}

LINK_SUFFIX = {"hip": "hip_link", "thigh": "thigh_link", "shank": "shank_link", "foot": "foot_link"}


def bake(stl_in: Path, stl_out: Path, xyz, rpy):
    m = stlmesh.Mesh.from_file(str(stl_in))
    R = rpy_to_R(rpy)
    t = np.array(xyz)

    # mesh.vectors shape (N, 3, 3): N triangles, 3 vertices, 3 coords.
    # Apply v_new = R @ v_old + t, vectorized.
    flat = m.vectors.reshape(-1, 3)             # (N*3, 3)
    flat = flat @ R.T + t                       # (N*3, 3); v @ R.T == R @ v for row vectors
    m.vectors = flat.reshape(-1, 3, 3)

    # Recompute face normals after the transform.
    m.update_normals()

    stl_out.parent.mkdir(parents=True, exist_ok=True)
    m.save(str(stl_out))


def main():
    here = Path(__file__).resolve()
    pkg = here.parents[1]                       # .../dog_robot_description
    src_dir = pkg / "meshes" / "visual"
    dst_dir = pkg / "meshes" / "visual_dh"

    count = 0
    for leg, links in LEG_PARAMS.items():
        for link_key, (xyz, rpy) in links.items():
            link_name = f"{leg}_{LINK_SUFFIX[link_key]}"
            stl_in = src_dir / f"{link_name}.stl"
            stl_out = dst_dir / f"{link_name}.stl"
            if not stl_in.is_file():
                print(f"SKIP (missing): {stl_in}")
                continue
            bake(stl_in, stl_out, xyz, rpy)
            print(f"baked: {link_name}")
            count += 1

    # base_link: no compensation needed (already in base frame).
    base_in = src_dir / "base_link.stl"
    base_out = dst_dir / "base_link.stl"
    if base_in.is_file():
        bake(base_in, base_out, (0, 0, 0), (0, 0, 0))
        print("baked: base_link (identity)")
        count += 1

    print(f"\n{count} STL(s) baked -> {dst_dir}")


if __name__ == "__main__":
    main()
