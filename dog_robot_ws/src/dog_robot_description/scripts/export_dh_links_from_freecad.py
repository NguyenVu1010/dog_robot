#!/usr/bin/env python3
"""Re-export dog_robot link STLs from FreeCAD into their joint-attached frames.

Convention (see specs/2026-05-26-joint-frame-export-design.md):
  Each link's vertices end up in its joint-attached frame — origin at the
  parent joint center, Z along the parent joint axis, X toward the child
  joint center. The URDF then uses identity <visual> origins.

PRECONDITIONS (set up by the user):
  1. FreeCAD is running.
  2. Document `RobotDog` has `robotdogassem.STEP` imported.
  3. FreeCAD MCP server is listening on port 9875.

Run from a normal shell:
    python3 dog_robot_ws/src/dog_robot_description/scripts/export_dh_links_from_freecad.py

Solid grouping is label-aware (part1=hip, part2=thigh, part3=shank,
demchan=foot, body keywords=base_link), ported from
/home/nguyenvd/workspace/dog_robot/scripts/export_links_from_freecad.py —
NOT the fragile bbox-centroid bucketing used previously. The head
sub-assembly gets its CAD-local pre-rotation + retranslation so it attaches
at the body neck.

Per-link transform applied to each CAD vertex v (mm):
    p_urdf = cad_to_urdf_point(v)            # meters, URDF axes, body origin
    p_link = R_link^T @ (p_urdf - O_link)    # meters, joint-attached frame
where O_link / R_link come from derive_joint_frames.link_frames_urdf().

Does not modify the FreeCAD document.
"""
from __future__ import annotations

import socket
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
SCRIPTS = PKG / "scripts"
OUT_DIR = PKG / "meshes" / "visual_dh"


FREECAD_SCRIPT = '''
import sys, os, math
sys.path.insert(0, {scripts!r})

import FreeCAD, Mesh
import numpy as np
import derive_joint_frames as djf

OUT_DIR = {out_dir!r}
os.makedirs(OUT_DIR, exist_ok=True)

doc = FreeCAD.getDocument("RobotDog")

# --- label-aware solid grouping (ported from export_links_from_freecad.py) ---
def corner(cx, cz):
    return ("F" if cx < 100 else "B") + ("L" if cz > -40 else "R")

LINKS = ["base_link"]
for c in ("FL", "FR", "BL", "BR"):
    LINKS += [f"{{c}}_hip_link", f"{{c}}_thigh_link",
              f"{{c}}_shank_link", f"{{c}}_foot_link"]

clusters = {{name: [] for name in LINKS}}
unclassified = []
for o in doc.Objects:
    if not (hasattr(o, "Shape") and o.Shape and o.Shape.ShapeType == "Solid"):
        continue
    name = o.Label.lower()
    bb = o.Shape.BoundBox
    cx, cy, cz = bb.Center.x, bb.Center.y, bb.Center.z
    if ("screw" in name or "passivated" in name
            or "______" in o.Label or "------" in o.Label):
        continue
    if any(k in name for k in ["dethan", "thanhngang", "opthan", "opkhop",
                                "noi2op", "headcam", "duoi", "rpi lcd", "3.5inch"]):
        clusters["base_link"].append(o)
        continue
    if "demchan" in name:
        clusters[f"{{corner(cx, cz)}}_foot_link"].append(o)
        continue
    base = name.split(".")[0]
    if base.startswith("part1"):
        clusters[f"{{corner(cx, cz)}}_hip_link"].append(o)
    elif base.startswith("part2"):
        clusters[f"{{corner(cx, cz)}}_thigh_link"].append(o)
    elif base.startswith("part3"):
        clusters[f"{{corner(cx, cz)}}_shank_link"].append(o)
    else:
        unclassified.append(o.Label)

# --- head sub-assembly CAD-frame fix (ported verbatim) ---
HEAD_KEYWORDS = ("headcam", "lcd", "3.5inch")
HEAD_ROTATION_DEG = -90      # around CAD-Y
HEAD_TARGET_CENTER = (-53.628, 5.776, -34.631)  # CAD frame

def is_head(o):
    n = o.Label.lower()
    return any(k in n for k in HEAD_KEYWORDS)

def head_group_center(solids):
    h = [s for s in solids if is_head(s)]
    if not h:
        return None
    cs = [s.Shape.BoundBox.Center for s in h]
    return FreeCAD.Vector(sum(c.x for c in cs) / len(cs),
                          sum(c.y for c in cs) / len(cs),
                          sum(c.z for c in cs) / len(cs))

# --- frame data from derive_joint_frames (URDF meters) ---
frames = djf.link_frames_urdf()

def cad_vertex_to_link(vx, vy, vz, O_link, R_link_T):
    p = djf.cad_to_urdf_point((vx, vy, vz))   # meters, URDF, body origin
    p = p - O_link
    return R_link_T @ p

results = []
for link_name, solids in clusters.items():
    if not solids:
        results.append(f"WARN {{link_name}}: no solids")
        continue
    O_link = frames[link_name]["O"]
    R_link_T = frames[link_name]["R"].T

    hc = head_group_center(solids)
    ha = math.radians(HEAD_ROTATION_DEG)
    cos_h, sin_h = math.cos(ha), math.sin(ha)
    if hc is not None:
        htx = HEAD_TARGET_CENTER[0] - hc.x
        hty = HEAD_TARGET_CENTER[1] - hc.y
        htz = HEAD_TARGET_CENTER[2] - hc.z

    merged = Mesh.Mesh()
    for o in solids:
        try:
            verts, facets = o.Shape.tessellate(0.5)
        except Exception as e:
            results.append(f"WARN tess {{o.Label}}: {{e}}")
            continue
        o_is_head = hc is not None and is_head(o)
        tris = []
        for tri in facets:
            pts = []
            for idx in tri:
                v = verts[idx]
                if o_is_head:
                    dx = v.x - hc.x
                    dz = v.z - hc.z
                    cvx = hc.x + cos_h * dx + sin_h * dz + htx
                    cvy = v.y + hty
                    cvz = hc.z - sin_h * dx + cos_h * dz + htz
                else:
                    cvx, cvy, cvz = v.x, v.y, v.z
                pl = cad_vertex_to_link(cvx, cvy, cvz, O_link, R_link_T)
                pts.append((float(pl[0]), float(pl[1]), float(pl[2])))
            tris.append(tuple(pts))
        m = Mesh.Mesh()
        m.addFacets(tris)
        merged.addMesh(m)
    out = OUT_DIR + "/" + link_name + ".stl"
    merged.write(out)
    results.append(f"{{link_name}}: {{merged.CountFacets}} tris")

if unclassified:
    results.append(f"UNCLASSIFIED {{len(unclassified)}}: {{unclassified[:6]}}")
print("\\n".join(results))
print("links written:", sum(1 for r in results if " tris" in r))
'''.format(out_dir=str(OUT_DIR), scripts=str(SCRIPTS))


def send_to_freecad(code: str, host: str = "localhost", port: int = 9875) -> str:
    """Send Python code to FreeCAD MCP server via raw socket."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(120.0)
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
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    try:
        print(send_to_freecad(FREECAD_SCRIPT))
    except ConnectionRefusedError:
        sys.exit("FreeCAD MCP not reachable on localhost:9875 — "
                 "open FreeCAD, load robotdogassem.STEP into doc RobotDog, "
                 "start MCP server, then re-run.")


if __name__ == "__main__":
    main()
