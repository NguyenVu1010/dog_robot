"""Auto-find knee_pitch joint center on all 4 part3 (shank) solids.

Reference: Part__Feature041 / Edge396 (user click) on BL part3.
Logic:
  1. Read reference Edge396 of Part__Feature041 → (center, radius, axis).
  2. For ALL solids whose label contains 'part3' (incl. part3_mirror),
     find circular edges matching the reference radius;
     pick the one at the TOP (max CAD Y) within each leg's bbox cluster.
  3. Classify FL/FR/BL/BR by bbox center.

Usage in FreeCAD MCP / Python console (NO selection needed):
    exec(open("/home/nguyenvd/workspace/dog_robot/scripts/find_part3_joints.py").read())
    find_part3_knees()
"""

import FreeCAD
import Part

OUT_FILE = "/tmp/freecad_part3_joints.txt"
RADIUS_TOL = 0.3  # mm
REF_NAME = "Part__Feature041"
REF_EDGE_INDEX = 396  # 1-based as in FreeCAD GUI


def _leg_tag(bbox_center):
    """Return FL/FR/BL/BR based on bounding-box center in CAD frame."""
    cx, _, cz = bbox_center.x, bbox_center.y, bbox_center.z
    fb = "F" if cx < 100 else "B"
    lr = "L" if cz > -40 else "R"
    return f"{fb}{lr}"


def _get_reference_circle(doc):
    """Find object by Name (Part__FeatureNNN), return Edge<N>'s Part.Circle."""
    ref = None
    for o in doc.Objects:
        if o.Name == REF_NAME:
            ref = o
            break
    if ref is None:
        raise RuntimeError(f"Object Name '{REF_NAME}' not found")
    if not (hasattr(ref, "Shape") and ref.Shape):
        raise RuntimeError(f"{ref.Name} has no Shape")
    edges = ref.Shape.Edges
    if len(edges) < REF_EDGE_INDEX:
        raise RuntimeError(
            f"{ref.Name} has {len(edges)} edges, need Edge{REF_EDGE_INDEX}")
    edge = edges[REF_EDGE_INDEX - 1]
    if not isinstance(edge.Curve, Part.Circle):
        raise RuntimeError(
            f"Edge{REF_EDGE_INDEX} of {ref.Name} is not a circle "
            f"({type(edge.Curve).__name__})")
    return edge.Curve, ref


def find_part3_knees():
    doc = FreeCAD.ActiveDocument
    if doc is None:
        raise RuntimeError("No active document")

    ref_curve, ref_obj = _get_reference_circle(doc)
    ref_r = ref_curve.Radius
    rc = ref_curve.Center
    ra = ref_curve.Axis
    print(f"Reference: {ref_obj.Name} (label='{ref_obj.Label}') Edge{REF_EDGE_INDEX}")
    print(f"  center=({rc.x:.3f}, {rc.y:.3f}, {rc.z:.3f})")
    print(f"  axis=({ra.x:.4f}, {ra.y:.4f}, {ra.z:.4f})")
    print(f"  radius={ref_r:.3f}")
    ref_leg = _leg_tag(ref_obj.Shape.BoundBox.Center)
    print(f"  ref bbox classified as {ref_leg}")
    print()

    ref_nedges = len(ref_obj.Shape.Edges)
    # Only consider solids with SAME edge count as reference — these are
    # the identical part3-main pieces (one per leg).
    main_solids = [o for o in doc.Objects
                   if hasattr(o, "Shape") and o.Shape
                   and o.Shape.ShapeType == "Solid"
                   and "part3" in o.Label.lower()
                   and len(o.Shape.Edges) == ref_nedges]
    print(f"Found {len(main_solids)} part3-main solids ({ref_nedges} edges):")
    for s in main_solids:
        bb = s.Shape.BoundBox.Center
        print(f"  {s.Name} '{s.Label}': bbox=({bb.x:.1f},{bb.y:.1f},{bb.z:.1f}) "
              f"→ {_leg_tag(bb)}")
    print()

    # Strategy: try Edge<REF_EDGE_INDEX> on each main solid first
    # (same topology assumption). If not a circle / radius mismatch, fall back to
    # nearest circle by relative position within solid's bbox.
    ref_bb = ref_obj.Shape.BoundBox.Center
    ref_rel = FreeCAD.Vector(rc.x - ref_bb.x, rc.y - ref_bb.y, rc.z - ref_bb.z)
    print(f"Reference relative offset from bbox center: "
          f"({ref_rel.x:.3f}, {ref_rel.y:.3f}, {ref_rel.z:.3f})")
    print()

    results = {}  # leg → (center, axis, radius, solid_label, source)
    for solid in main_solids:
        leg = _leg_tag(solid.Shape.BoundBox.Center)
        # 1) try same edge index
        edge_at_idx = solid.Shape.Edges[REF_EDGE_INDEX - 1]
        if isinstance(edge_at_idx.Curve, Part.Circle) and \
                abs(edge_at_idx.Curve.Radius - ref_r) < RADIUS_TOL:
            c = edge_at_idx.Curve.Center
            results[leg] = (c, edge_at_idx.Curve.Axis,
                            edge_at_idx.Curve.Radius, solid.Label,
                            f"Edge{REF_EDGE_INDEX} (same index)")
            continue

        # 2) fallback: find circle closest to expected relative position
        bb = solid.Shape.BoundBox.Center
        # Mirror parts have z flipped relative to body center (z ≈ -40 mid)
        z_sign = 1 if leg.endswith("L") else -1
        expected = FreeCAD.Vector(bb.x + ref_rel.x,
                                  bb.y + ref_rel.y,
                                  bb.z + ref_rel.z * z_sign)
        best = None
        best_dist = float("inf")
        for edge in solid.Shape.Edges:
            if isinstance(edge.Curve, Part.Circle):
                if abs(edge.Curve.Radius - ref_r) < RADIUS_TOL:
                    c = edge.Curve.Center
                    d = (c - expected).Length
                    if d < best_dist:
                        best_dist = d
                        best = edge.Curve
        if best is not None:
            results[leg] = (best.Center, best.Axis, best.Radius,
                            solid.Label, f"closest to expected, d={best_dist:.2f}")

    print("KNEE JOINTS (paste into CAD_JOINT_POSITIONS):")
    lines = []
    for leg in ("FL", "FR", "BL", "BR"):
        if leg not in results:
            line = f'    "{leg}_knee_pitch":  None,  # NOT FOUND'
            print(line)
            lines.append(line)
            continue
        c, a, r, lbl, src = results[leg]
        py = f'    "{leg}_knee_pitch":  ({c.x:.1f}, {c.y:.1f}, {c.z:.1f}),'
        info = (f"  {leg}_knee_pitch: center=({c.x:.3f}, {c.y:.3f}, {c.z:.3f}) "
                f"axis=({a.x:.3f},{a.y:.3f},{a.z:.3f}) r={r:.2f} from '{lbl}' [{src}]")
        print(info)
        lines.append(py)

    with open(OUT_FILE, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\nWritten to {OUT_FILE}")
    return results


print("Loaded. Call find_part3_knees() — no selection needed.")
