#!/usr/bin/env python3
"""Re-export dog_robot link STLs from FreeCAD, aligned to DH link frames.

PRECONDITIONS (must be set up by the user):
  1. FreeCAD is running.
  2. Document `RobotDog` has `robotdogassem.STEP` imported.
  3. FreeCAD MCP server is listening on port 9875.

Run from a normal shell:
    python3 dog_robot_ws/src/dog_robot_description/scripts/export_dh_links_from_freecad.py

For each link in dh_link_placements.yaml:
  1. Find the solids belonging to that link (cluster classification
     identical to /home/nguyenvd/workspace/dog_robot/scripts/compute_joints.py).
  2. Compose them into a Part.Compound.
  3. Apply the INVERSE of the link's DH Placement to the compound — moves
     geometry into the DH link frame.
  4. Tessellate at 0.05 mm tolerance.
  5. Write meshes/visual_dh/<link>.stl.

Does not modify the FreeCAD document.
"""
from __future__ import annotations

import socket
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
PLACEMENTS = PKG / "config" / "dh_link_placements.yaml"
OUT_DIR = PKG / "meshes" / "visual_dh"


FREECAD_SCRIPT = '''
import FreeCAD, Part, Mesh
from FreeCAD import Vector, Rotation, Placement

OUT_DIR = {out_dir!r}
PLACEMENTS = {placements!r}

import yaml
with open(PLACEMENTS) as f:
    cfg = yaml.safe_load(f)

doc = FreeCAD.getDocument("RobotDog")

# Cluster classification by bounding-box centroid (CAD mm).
# Identical to /home/nguyenvd/workspace/dog_robot/scripts/compute_joints.py.
def classify(cx, cy, cz):
    if cz > 50.0:
        return "base_link"
    leg_x = "F" if cx < 100 else "B"
    leg_z = "L" if cz > -40 else "R"
    leg = leg_x + leg_z
    # link kind by Y position (CAD): hip near Y~10, thigh Y~-5 to -50, shank Y~-70 to -130, foot Y<-125.
    # Use Y bucket for now; sufficient since solids are well-separated per link.
    if cy > -20:
        return f"{{leg}}_hip_link"
    elif cy > -55:
        return f"{{leg}}_thigh_link"
    elif cy > -100:
        return f"{{leg}}_shank_link"
    else:
        return f"{{leg}}_foot_link"

# Build solid -> link mapping by bbox centroid classification.
LINK_MAP = {{}}                                  # solid_obj -> link_name
for obj in doc.Objects:
    if not hasattr(obj, "Shape") or obj.Shape.ShapeType != "Solid":
        continue
    bb = obj.Shape.BoundBox
    cx = (bb.XMin + bb.XMax) / 2
    cy = (bb.YMin + bb.YMax) / 2
    cz = (bb.ZMin + bb.ZMax) / 2
    LINK_MAP[obj] = classify(cx, cy, cz)

# Per link: build compound, apply inverse Placement, tessellate, save.
import os
os.makedirs(OUT_DIR, exist_ok=True)

for link_name, info in cfg["links"].items():
    pos = Vector(*info["position_cad_mm"])
    qx, qy, qz, qw = info["quat_xyzw"]
    rot = Rotation(qx, qy, qz, qw)
    plc = Placement(pos, rot)
    inv = plc.inverse()

    solids = [s for s, n in LINK_MAP.items() if n == link_name]
    if not solids:
        print(f"WARN: no solids for {{link_name}} — skipping")
        continue
    shapes = [s.Shape.copy() for s in solids]
    comp = Part.makeCompound(shapes)
    comp.Placement = inv.multiply(comp.Placement)

    mesh = Mesh.Mesh()
    mesh.addFacets(comp.tessellate(0.05))
    out = OUT_DIR + "/" + link_name + ".stl"
    mesh.write(out)
    print(f"wrote {{out}} ({{mesh.CountFacets}} tris)")
'''.format(out_dir=str(OUT_DIR), placements=str(PLACEMENTS))


def send_to_freecad(code: str, host: str = "localhost", port: int = 9875) -> str:
    """Send Python code to FreeCAD MCP server via raw socket."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(60.0)
    s.connect((host, port))
    s.sendall((code + "\n__END__\n").encode())
    out = b""
    while True:
        chunk = s.recv(8192)
        if not chunk:
            break
        out += chunk
        if b"__END__" in chunk:
            break
    s.close()
    return out.decode(errors="replace")


def main() -> None:
    if not PLACEMENTS.is_file():
        sys.exit(f"missing {PLACEMENTS} — run derive_dh_frames.py first")
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    try:
        out = send_to_freecad(FREECAD_SCRIPT)
        print(out)
    except ConnectionRefusedError:
        sys.exit("FreeCAD MCP not reachable on localhost:9875 — "
                  "open FreeCAD, load robotdogassem.STEP into doc RobotDog, "
                  "start MCP server, then re-run.")


if __name__ == "__main__":
    main()
