"""Probe FreeCAD selection to get target CAD position for head.

Usage in FreeCAD MCP / Python console:
    1. Click on a vertex/edge/face on the body where head should attach.
    2. Run:
        exec(open("/home/nguyenvd/workspace/dog_robot/scripts/probe_head_target.py").read())
        probe_head_target()
    3. Returned tuple → paste into HEAD_TARGET_CENTER in export_links_from_freecad.py
"""

import FreeCAD
import FreeCADGui as Gui


def probe_head_target():
    sel = Gui.Selection.getSelectionEx()
    if not sel:
        print("No selection. Click on a vertex/edge/face first.")
        return None

    s = sel[0]
    pts = []
    for sub in s.SubObjects:
        # Try Point (vertex) first
        if hasattr(sub, "Point"):
            pts.append(sub.Point)
        elif hasattr(sub, "CenterOfMass"):
            pts.append(sub.CenterOfMass)
        elif hasattr(sub, "BoundBox"):
            bb = sub.BoundBox
            pts.append(FreeCAD.Vector(bb.Center.x, bb.Center.y, bb.Center.z))

    if not pts:
        # Fallback: use whole object bbox center
        if hasattr(s.Object, "Shape"):
            bb = s.Object.Shape.BoundBox
            pts.append(FreeCAD.Vector(bb.Center.x, bb.Center.y, bb.Center.z))

    if not pts:
        print("Could not extract a position from selection.")
        return None

    cx = sum(p.x for p in pts) / len(pts)
    cy = sum(p.y for p in pts) / len(pts)
    cz = sum(p.z for p in pts) / len(pts)
    # Compute URDF position for reference
    BODY = (100.0, -22.6, -40.0)
    urdf_x = BODY[0] - cx
    urdf_y = cz - BODY[2]
    urdf_z = cy - BODY[1]
    print(f"Selected: {s.ObjectName} / {s.SubElementNames}")
    print(f"CAD target: ({cx:.3f}, {cy:.3f}, {cz:.3f})")
    print(f"  → URDF: x={urdf_x:.1f} (fwd) y={urdf_y:.1f} (left) z={urdf_z:.1f} (up)")
    print(f"\nPaste into export_links_from_freecad.py:")
    print(f"HEAD_TARGET_CENTER = ({cx:.3f}, {cy:.3f}, {cz:.3f})")
    return (cx, cy, cz)


print("Loaded. Click a vertex/edge/face, then call probe_head_target()")
