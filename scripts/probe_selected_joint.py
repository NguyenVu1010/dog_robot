"""Probe FreeCAD selection: extract circle center + axis from selected edge(s).

Run via FreeCAD MCP execute_code() after user selects an edge that's a circle/arc
(e.g., the shaft hole on a part). Writes result to /tmp/freecad_joints.txt for
the host to read.

Usage in FreeCAD Python console / MCP:
    exec(open("/home/nguyenvd/workspace/dog_robot/scripts/probe_selected_joint.py").read())
    probe_selection(joint_name="BL_knee_pitch")
"""

import FreeCADGui as Gui
import FreeCAD
import Part

OUT_FILE = "/tmp/freecad_joints.txt"


def _edge_circle_info(edge):
    """If edge is a circle/arc, return (center, axis, radius). Else None."""
    curve = edge.Curve
    if isinstance(curve, Part.Circle):
        c = curve.Center
        a = curve.Axis
        return (c.x, c.y, c.z), (a.x, a.y, a.z), curve.Radius
    return None


def probe_selection(joint_name=""):
    sel = Gui.Selection.getSelectionEx()
    if not sel:
        print("No selection.")
        return None

    lines_out = []
    for s in sel:
        for sub_name, sub_obj in zip(s.SubElementNames, s.SubObjects):
            if not hasattr(sub_obj, "Curve"):
                print(f"  [skip] {sub_name}: not an edge")
                continue
            info = _edge_circle_info(sub_obj)
            if info is None:
                print(f"  [skip] {sub_name}: not a circle (curve={type(sub_obj.Curve).__name__})")
                continue
            (cx, cy, cz), (ax, ay, az), r = info
            tag = joint_name or f"{s.ObjectName}/{sub_name}"
            line = (f"{tag}: center=({cx:.3f}, {cy:.3f}, {cz:.3f}), "
                    f"axis=({ax:.4f}, {ay:.4f}, {az:.4f}), r={r:.3f}")
            print("  " + line)
            lines_out.append(line)

    if lines_out:
        with open(OUT_FILE, "a") as f:
            f.write("\n".join(lines_out) + "\n")
        print(f"  Appended {len(lines_out)} line(s) to {OUT_FILE}")
    return lines_out


def reset_log():
    open(OUT_FILE, "w").close()
    print(f"Cleared {OUT_FILE}")


# Convenience: if executed via exec(open(...).read()), don't auto-run probe.
# User explicitly calls probe_selection() or reset_log().
print("Helpers loaded: probe_selection(joint_name='...'), reset_log()")
