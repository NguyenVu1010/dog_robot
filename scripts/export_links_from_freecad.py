"""
Export 17 links (visual + collision) from FreeCAD assembly into ROS2 meshes/ dir.

Run via FreeCAD MCP execute_code() or paste into FreeCAD Python console after
importing step/robotdogassem.STEP into doc "RobotDog".

Output:
  dog_robot_ws/src/dog_robot_description/meshes/visual/<link>.stl   (17)
  dog_robot_ws/src/dog_robot_description/meshes/collision/<link>.stl (17 convex hull)
"""

import os
import FreeCAD
import Part
import Mesh

PROJECT_ROOT = "/home/nguyenvd/workspace/dog_robot"
MESH_OUT_VIS = f"{PROJECT_ROOT}/dog_robot_ws/src/dog_robot_description/meshes/visual"
MESH_OUT_COL = f"{PROJECT_ROOT}/dog_robot_ws/src/dog_robot_description/meshes/collision"

# Geometric constants (mm, IK frame in CAD)
L1 = 12.5
L2 = 48.95
L3 = 109.202
L4 = 115.0
L = 200.0
W = 80.0

# Body center in CAD frame (from inspection: dethan.step BoundBox center)
BODY_CENTER_CAD = FreeCAD.Vector(100.0, 0.0, -40.0)

# Hip offsets in IK frame: X=forward(length), Y=up, Z=lateral(left+)
HIP_OFFSETS_IK = {
    "FL": FreeCAD.Vector(+L/2, 0.0, +W/2),
    "FR": FreeCAD.Vector(+L/2, 0.0, -W/2),
    "BL": FreeCAD.Vector(-L/2, 0.0, +W/2),
    "BR": FreeCAD.Vector(-L/2, 0.0, -W/2),
}


def thigh_offset_from_hip_ik(side):
    """L2 outward laterally in IK Z direction."""
    sign = +1 if side == "L" else -1
    return FreeCAD.Vector(0, 0, sign * L2)


KNEE_OFFSET_FROM_THIGH_IK = FreeCAD.Vector(0, -L3, 0)
FOOT_OFFSET_FROM_SHANK_IK = FreeCAD.Vector(0, -L4, 0)


def _classify_solids(doc):
    """Return dict {link_name: [solid_obj, ...]}."""
    clusters = {name: [] for name in [
        "base_link",
        "FL_hip_link", "FL_thigh_link", "FL_shank_link", "FL_foot_link",
        "FR_hip_link", "FR_thigh_link", "FR_shank_link", "FR_foot_link",
        "BL_hip_link", "BL_thigh_link", "BL_shank_link", "BL_foot_link",
        "BR_hip_link", "BR_thigh_link", "BR_shank_link", "BR_foot_link",
    ]}
    unclassified = []

    for o in doc.Objects:
        if not (hasattr(o, "Shape") and o.Shape and o.Shape.ShapeType == "Solid"):
            continue
        name = o.Label.lower()
        bb = o.Shape.BoundBox
        cx, cy, cz = bb.Center.x, bb.Center.y, bb.Center.z

        # BODY parts
        if any(k in name for k in [
            "dethan", "thanhngang", "opthan", "opkhop", "noi2op",
            "head", "duoi", "rpi lcd", "3.5inch",
        ]):
            clusters["base_link"].append(o)
            continue

        # FEET (đệm chân)
        if "demchan" in name:
            corner = ("F" if cx < 100 else "B") + ("L" if cz > -40 else "R")
            clusters[f"{corner}_foot_link"].append(o)
            continue

        # Skip screws / unnamed
        if ("screw" in name or "passivated" in name
                or "______" in o.Label or "------" in o.Label):
            continue

        # LEG parts: part1=hip, part2=thigh, part3=shank
        corner = ("F" if cx < 100 else "B") + ("L" if cz > -40 else "R")
        base = name.split(".")[0]
        if base.startswith("part1"):
            clusters[f"{corner}_hip_link"].append(o)
        elif base.startswith("part2"):
            clusters[f"{corner}_thigh_link"].append(o)
        elif base.startswith("part3"):
            clusters[f"{corner}_shank_link"].append(o)
        else:
            unclassified.append((o.Label, cx, cy, cz))

    if unclassified:
        print(f"  [WARN] {len(unclassified)} unclassified solids:")
        for label, cx, cy, cz in unclassified[:5]:
            print(f"    {label} at ({cx:.0f},{cy:.0f},{cz:.0f})")
    return clusters


def _link_origin_in_world(link_name):
    """Joint origin in CAD/IK frame for the link."""
    if link_name == "base_link":
        return BODY_CENTER_CAD

    parts = link_name.split("_")
    corner, seg = parts[0], parts[1]
    side = corner[1]
    hip_world = BODY_CENTER_CAD + HIP_OFFSETS_IK[corner]
    if seg == "hip":
        return hip_world
    thigh_world = hip_world + thigh_offset_from_hip_ik(side)
    if seg == "thigh":
        return thigh_world
    knee_world = thigh_world + KNEE_OFFSET_FROM_THIGH_IK
    if seg == "shank":
        return knee_world
    foot_world = knee_world + FOOT_OFFSET_FROM_SHANK_IK
    if seg == "foot":
        return foot_world
    raise ValueError(f"Unknown link: {link_name}")


def _link_transform(origin_world):
    """Return placement matrix: translate so origin_world→0, then swap Y↔Z."""
    matrix = FreeCAD.Matrix()
    matrix.A11 = 1; matrix.A12 = 0; matrix.A13 = 0; matrix.A14 = -origin_world.x
    matrix.A21 = 0; matrix.A22 = 0; matrix.A23 = 1; matrix.A24 = -origin_world.z
    matrix.A31 = 0; matrix.A32 = 1; matrix.A33 = 0; matrix.A34 = -origin_world.y
    matrix.A41 = 0; matrix.A42 = 0; matrix.A43 = 0; matrix.A44 = 1
    return matrix


def _solids_to_mesh(solids, origin_world, deviation=0.5):
    """Tessellate each solid separately, transform to link-local frame, merge meshes.

    Faster and more robust than Part.fuse() on many solids.
    """
    matrix = _link_transform(origin_world)
    scale = FreeCAD.Matrix()
    scale.scale(0.001, 0.001, 0.001)

    merged = Mesh.Mesh()
    for o in solids:
        try:
            shape = o.Shape.copy()
            shape = shape.transformGeometry(matrix)
            mesh = Mesh.Mesh()
            mesh.addFacets(shape.tessellate(deviation))
            mesh.transform(scale)
            merged.addMesh(mesh)
        except Exception as e:
            print(f"  [WARN] tess failed for {o.Label}: {e}")
    return merged


def _convex_hull_mesh(merged_mesh):
    """Compute convex hull of merged Mesh using vertex set."""
    try:
        from scipy.spatial import ConvexHull
        import numpy as np
        pts = np.array([(p.x, p.y, p.z) for p in merged_mesh.Points])
        if len(pts) < 4:
            return merged_mesh
        hull = ConvexHull(pts)
        hull_mesh = Mesh.Mesh()
        facets = []
        for simplex in hull.simplices:
            tri = [tuple(pts[i]) for i in simplex]
            facets.append(tri)
        hull_mesh.addFacets(facets)
        return hull_mesh
    except ImportError:
        # Fallback: just decimate the merged mesh
        return merged_mesh
    except Exception as e:
        print(f"  [WARN] convex hull failed: {e}")
        return merged_mesh


def _export_link(link_name, solids):
    if not solids:
        print(f"  [SKIP] {link_name} (no solids)")
        return
    origin = _link_origin_in_world(link_name)

    # Visual
    mesh_vis = _solids_to_mesh(solids, origin, deviation=0.5)
    out_vis = f"{MESH_OUT_VIS}/{link_name}.stl"
    mesh_vis.write(out_vis)

    # Collision (convex hull of merged mesh)
    mesh_col_full = _solids_to_mesh(solids, origin, deviation=1.5)
    mesh_col = _convex_hull_mesh(mesh_col_full)
    out_col = f"{MESH_OUT_COL}/{link_name}.stl"
    mesh_col.write(out_col)

    print(f"  OK {link_name} ({len(solids)} solids, {mesh_vis.CountFacets} tris vis, {mesh_col.CountFacets} tris col)")


def export_all():
    os.makedirs(MESH_OUT_VIS, exist_ok=True)
    os.makedirs(MESH_OUT_COL, exist_ok=True)
    doc = FreeCAD.getDocument("RobotDog")
    clusters = _classify_solids(doc)

    print(f"Classified into {len(clusters)} clusters:")
    for name, solids in clusters.items():
        print(f"  {name}: {len(solids)} solids")
    print()

    for link_name, solids in clusters.items():
        _export_link(link_name, solids)


if __name__ == "__main__":
    export_all()
