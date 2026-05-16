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

# Body center in CAD frame (dethan.step BoundBox center)
BODY_CENTER_CAD = FreeCAD.Vector(100.0, -22.6, -40.0)

# CAD-measured joint axis positions (mm) — derived from cluster centroid midpoints.
# See scripts/compute_joints.py for derivation.
CAD_JOINT_POSITIONS = {
    "FL_hip_yaw":     (25.2,   12.5,    0.0),
    "FL_thigh_pitch": (0.0,    -0.7,   25.4),
    "FL_knee_pitch":  (88.9,  -65.2,   66.4),
    "FL_foot_fixed":  (32.3, -102.4,   47.2),
    "FR_hip_yaw":     (25.2,   12.5,  -80.0),
    "FR_thigh_pitch": (0.0,     0.0, -105.7),
    "FR_knee_pitch":  (88.0,  -64.7, -148.4),
    "FR_foot_fixed":  (31.6, -102.7, -130.1),
    "BL_hip_yaw":     (174.8,  12.5,    0.0),
    "BL_thigh_pitch": (200.0,  -0.7,   25.4),
    "BL_knee_pitch":  (283.4, -72.3,   66.2),
    "BL_foot_fixed":  (223.2,-102.6,   47.1),
    "BR_hip_yaw":     (174.8,  12.5,  -80.0),
    "BR_thigh_pitch": (200.0,   0.0, -105.7),
    "BR_knee_pitch":  (283.0, -71.0, -148.4),
    "BR_foot_fixed":  (222.4,-102.7, -130.0),
}


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

        # Skip screws / unnamed (must come BEFORE body classification, else
        # screws labeled "...Phillips Head..." fall into base_link via "head" match)
        if ("screw" in name or "passivated" in name
                or "______" in o.Label or "------" in o.Label):
            continue

        # BODY parts (use "headcam" not "head" to avoid screw label false-match)
        if any(k in name for k in [
            "dethan", "thanhngang", "opthan", "opkhop", "noi2op",
            "headcam", "duoi", "rpi lcd", "3.5inch",
        ]):
            clusters["base_link"].append(o)
            continue

        # FEET (đệm chân)
        if "demchan" in name:
            corner = ("F" if cx < 100 else "B") + ("L" if cz > -40 else "R")
            clusters[f"{corner}_foot_link"].append(o)
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
    """Joint origin in CAD frame for the link — at the joint that connects it to parent.

    Uses CAD-measured joint positions (see CAD_JOINT_POSITIONS).
    """
    if link_name == "base_link":
        return BODY_CENTER_CAD

    # Map link name to the joint whose origin this link's mesh should center around
    # e.g., FL_hip_link is the child of FL_hip_yaw joint → mesh origin at FL_hip_yaw axis
    parts = link_name.split("_")
    corner, seg = parts[0], parts[1]
    joint_key_map = {
        "hip":   f"{corner}_hip_yaw",
        "thigh": f"{corner}_thigh_pitch",
        "shank": f"{corner}_knee_pitch",
        "foot":  f"{corner}_foot_fixed",
    }
    pos = CAD_JOINT_POSITIONS[joint_key_map[seg]]
    return FreeCAD.Vector(*pos)


def _link_transform(origin_world):
    """CAD → URDF transform: flip X, swap Y↔Z, translate origin_world → (0,0,0).

    Mapping per axis:
      urdf_x = -(cad_x - origin.x) = origin.x - cad_x   (CAD X is reversed)
      urdf_y = cad_z - origin.z                          (CAD Z = URDF Y left)
      urdf_z = cad_y - origin.y                          (CAD Y = URDF Z up)
    """
    matrix = FreeCAD.Matrix()
    matrix.A11 = -1; matrix.A12 = 0; matrix.A13 = 0; matrix.A14 = +origin_world.x
    matrix.A21 =  0; matrix.A22 = 0; matrix.A23 = 1; matrix.A24 = -origin_world.z
    matrix.A31 =  0; matrix.A32 = 1; matrix.A33 = 0; matrix.A34 = -origin_world.y
    matrix.A41 =  0; matrix.A42 = 0; matrix.A43 = 0; matrix.A44 = 1
    return matrix


# Head sub-assembly has its own local frame in the SolidWorks part — appears
# rotated 90° when placed in body assembly, AND offset laterally to one side.
# Apply pre-rotation around CAD-Y (vertical), then translate to a target
# CAD-frame position before CAD→URDF transform. Adjust HEAD_ROTATION_DEG (±90)
# and HEAD_TARGET_CENTER if alignment still off.
HEAD_KEYWORDS = ("headcam", "lcd", "3.5inch")
HEAD_ROTATION_DEG = -90  # rotate around CAD-Y; flip sign if direction wrong
HEAD_TARGET_CENTER = (-53.628, 5.776, -34.631)  # CAD frame target for head center
# Computed from 2 face mate constraints (see scripts/probe_head_target.py):
#   Coincident: head_F087.Face106 ≡ body_F093.Face26 (opthan1.step001)
#   Parallel:   head_F087.Face124 || body_F093.Face18
# Rotation -90° around CAD-Y satisfies both mates' normal alignment.
# Translation derived from rotated face center → body face center.
# → URDF: (+153.6, +5.4, +28.4) head exactly attached at body neck face


def _is_head_solid(o):
    name = o.Label.lower()
    return any(k in name for k in HEAD_KEYWORDS)


def _head_group_center(solids):
    head = [s for s in solids if _is_head_solid(s)]
    if not head:
        return None
    bbs = [s.Shape.BoundBox.Center for s in head]
    return FreeCAD.Vector(
        sum(b.x for b in bbs) / len(bbs),
        sum(b.y for b in bbs) / len(bbs),
        sum(b.z for b in bbs) / len(bbs),
    )


def _solids_to_mesh(solids, origin_world, deviation=0.5):
    """Tessellate each solid in world frame, transform to link-local, merge.

    Note: o.Shape.tessellate() returns vertices in WORLD frame (placement already
    applied by FreeCAD). So we just transform with the CAD→URDF matrix.
    """
    import math
    matrix = _link_transform(origin_world)
    scale = FreeCAD.Matrix()
    scale.scale(0.001, 0.001, 0.001)

    head_center = _head_group_center(solids)
    ha = math.radians(HEAD_ROTATION_DEG)
    cos_h, sin_h = math.cos(ha), math.sin(ha)
    if head_center is not None:
        head_tx = HEAD_TARGET_CENTER[0] - head_center.x
        head_ty = HEAD_TARGET_CENTER[1] - head_center.y
        head_tz = HEAD_TARGET_CENTER[2] - head_center.z
    else:
        head_tx = head_ty = head_tz = 0.0

    merged = Mesh.Mesh()
    for o in solids:
        is_head = head_center is not None and _is_head_solid(o)
        try:
            verts, facets = o.Shape.tessellate(deviation)
            mesh = Mesh.Mesh()
            # Build triangle list from world-frame vertices, transformed by matrix
            tris = []
            for tri in facets:
                pts = []
                for idx in tri:
                    v = verts[idx]
                    # Pre-rotate head solids around head_center (CAD-Y axis),
                    # then translate head so its center moves to HEAD_TARGET_CENTER.
                    if is_head:
                        dx = v.x - head_center.x
                        dz = v.z - head_center.z
                        vx = head_center.x + cos_h * dx + sin_h * dz + head_tx
                        vy = v.y + head_ty
                        vz = head_center.z - sin_h * dx + cos_h * dz + head_tz
                    else:
                        vx, vy, vz = v.x, v.y, v.z
                    # Apply matrix to world-frame vertex
                    nx = matrix.A11*vx + matrix.A12*vy + matrix.A13*vz + matrix.A14
                    ny = matrix.A21*vx + matrix.A22*vy + matrix.A23*vz + matrix.A24
                    nz = matrix.A31*vx + matrix.A32*vy + matrix.A33*vz + matrix.A34
                    pts.append((nx, ny, nz))
                tris.append(tuple(pts))
            mesh.addFacets(tris)
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
