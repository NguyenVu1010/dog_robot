"""Compute joint positions for all 4 legs from CAD cluster data."""

# All cluster centroids in CAD frame (mm), from FreeCAD inspection output
# Format: name → [(x, y, z), ...] of constituent solid bbox centers

CLUSTERS = {
    "base_link": [
        (100.0, -22.6, -40.0), (100.0, 0.0, -93.5), (100.0, 0.0, 13.5),
        (21.2, 0.0, -40.0), (-20.6, 0.0, -40.0), (0.5, 32.3, -40.0), (0.5, -32.3, -40.0),
        (178.8, 0.0, -40.0), (220.7, 0.0, -40.0), (199.5, 32.3, -40.0), (199.5, -32.3, -40.0),
        (38.0, 65.6, 98.9), (38.0, 117.0, 103.9), (41.4, 65.6, 122.9),
        (40.4, 66.1, 124.2), (59.0, 42.1, 117.3), (234.4, 0.1, -40.0),
        (65.0, 19.4, -40.0), (65.0, -19.3, -40.0), (164.4, 19.4, -40.0), (164.4, -19.3, -40.0),
    ],
    # FL leg (X<100, Z>-40)
    "FL_hip_link":   [(15.8,12.5,-0.0),(-0.0,-0.6,21.4),(-16.7,12.5,-0.0),(4.0,0.0,-0.3),(-12.9,0.0,-0.3),(-1.0,-0.0,-0.3)],
    "FL_thigh_link": [(82.2,-44.3,28.1),(12.0,-5.6,45.1),(33.0,-21.3,46.4),(-0.0,-0.7,28.3),(31.7,-12.8,49.0),(82.2,-45.3,67.6)],
    "FL_shank_link": [(41.1,-92.3,47.7),(88.9,-64.3,32.3),(77.8,-71.7,49.0),(88.9,-65.2,64.7)],
    "FL_foot_link":  [(-9.6,-131.5,45.9)],

    # FR leg (X>100, Z>-40): cluster center X~200, Z~0-67 (per earlier output where I named it FR
    # but X>100 is "B" in classifier. Reality: leg with X>100 and Z>-40 is what I called leg_FR earlier
    # but classifier produces "BL" (X>=100 → B, Z>-40 → L). Confusion. Let me match classifier names:
    # corner = ("F" if cx < 100 else "B") + ("L" if cz > -40 else "R")
    # So X>=100, Z>-40 → "BL". That's BACK-LEFT in classifier name.
    # But visually it's at "back" because IK X is reversed (CAD X>100 = IK X<0 = back in IK frame)

    # X>=100, Z>-40 (classifier "BL"):
    "BL_hip_link":   [(195.8,12.5,-0.0),(200.0,-0.6,21.4),(216.7,12.5,-0.0),(212.9,0.0,-0.3),(196.0,0.0,-0.3),(201.0,-0.0,-0.3)],
    "BL_thigh_link": [(231.8,-25.7,46.3),(211.7,-6.6,45.0),(277.9,-50.7,28.0),(200.0,-0.7,28.3),(230.7,-15.3,48.9),(277.9,-51.7,67.5)],
    "BL_shank_link": [(232.8,-93.8,47.6),(283.4,-71.3,32.1),(271.8,-77.4,48.8),(283.4,-72.2,64.5)],
    "BL_foot_link":  [(178.6,-126.6,46.0)],

    # X<100, Z<-40 (classifier "FR"):
    "FR_hip_link":   [(15.8,12.5,-80.0),(-0.0,0.0,-101.7),(-16.7,12.5,-80.0),(-12.9,0.0,-80.0),(-1.0,0.0,-80.0),(4.0,0.0,-80.0)],
    "FR_thigh_link": [(32.8,-21.1,-127.6),(12.0,-4.6,-125.4),(81.5,-44.6,-110.2),(-0.0,0.0,-108.6),(31.5,-11.8,-129.7),(81.5,-44.6,-149.2)],
    "FR_shank_link": [(40.3,-92.7,-130.1),(88.0,-64.7,-114.3),(77.0,-71.7,-131.1),(88.0,-64.7,-146.8)],
    "FR_foot_link":  [(-10.1,-132.0,-129.7)],

    # X>=100, Z<-40 (classifier "BR"):
    "BR_hip_link":   [(184.2,12.5,-80.0),(200.0,0.0,-101.7),(216.6,12.5,-80.0),(212.9,0.0,-80.0),(201.0,0.0,-80.0),(196.0,0.0,-80.0)],
    "BR_thigh_link": [(232.6,-93.6,-130.1),(211.7,-5.5,-125.4),(277.5,-50.4,-110.1),(200.0,0.0,-108.6),(230.6,-14.1,-129.6),(277.5,-50.4,-149.1),(271.4,-76.7,-131.1)],
    "BR_shank_link": [(283.0,-71.0,-114.3),(283.0,-71.0,-146.7),(232.6,-93.6,-130.1)],
    "BR_foot_link":  [(178.6,-126.8,-129.6)],
}


def avg(pts):
    n = len(pts)
    return (sum(p[0] for p in pts)/n, sum(p[1] for p in pts)/n, sum(p[2] for p in pts)/n)

def mid(a, b):
    return ((a[0]+b[0])/2, (a[1]+b[1])/2, (a[2]+b[2])/2)


# Compute cluster centroids
centroids = {name: avg(pts) for name, pts in CLUSTERS.items()}

# Body center: use dethan center (the main body solid)
BODY_CENTER = (100.0, -22.6, -40.0)

# Joint axis centers measured from CAD circular edges via
# scripts/find_part3_joints.py (knee) and direct MCP queries (hip, thigh).
# Replaces cluster-centroid approximation that was off by 20-30mm.
MEASURED_HIP = {
    "FL": (25.200,  12.500,    0.000),
    "FR": (25.200,  12.500,  -80.000),
    "BL": (174.800, 12.500,    0.000),
    "BR": (174.800, 12.500,  -80.000),
}
MEASURED_THIGH = {
    "FL": (0.000,   -0.671,   25.362),
    "FR": (0.000,    0.000, -105.700),
    "BL": (200.000, -0.675,   25.361),
    "BR": (200.000,  0.000, -105.700),
}
MEASURED_KNEE = {
    "FL": (88.875,  -65.224,   66.379),
    "FR": (87.991,  -64.673, -148.400),
    "BL": (283.410, -72.261,   66.183),
    "BR": (282.987, -70.980, -148.400),
}

# Joint positions per leg
# Per classifier name: FL = front-near-left, BL = back-near-left, FR = front-far-right, BR = back-far-right
# But we want URDF names that match the actual position semantics.
# The classifier uses "F" for X<100 and "B" for X>=100. In URDF after X-flip, X<100 in CAD = front (positive X URDF).
# So classifier name maps directly to URDF name (F=front, B=back, L=left, R=right) — BUT we need to check
# which Z side is "left" in REP-103. REP-103 +Y = left, so we need +Y_URDF = +CAD_Z. Right is -Y_URDF = -CAD_Z.
# Classifier "L" = CAD_Z > -40, which in URDF Y = CAD_Z - body_z = positive when CAD_Z > -40. ✓ Yes "L" = left in URDF.

# So mapping classifier name → URDF name is identity. FL → FL etc.

print("\n# Joint positions in CAD frame (mm) — for export script")
print("CAD_JOINT_POSITIONS = {")
for leg in ["FL", "FR", "BL", "BR"]:
    hip = centroids[f"{leg}_hip_link"]
    thigh = centroids[f"{leg}_thigh_link"]
    shank = centroids[f"{leg}_shank_link"]
    foot = centroids[f"{leg}_foot_link"]
    j_hip = MEASURED_HIP[leg]
    j_thigh = MEASURED_THIGH[leg]
    j_knee = MEASURED_KNEE[leg]
    j_foot = mid(shank, foot)
    print(f'  "{leg}_hip_yaw":     ({j_hip[0]:.1f}, {j_hip[1]:.1f}, {j_hip[2]:.1f}),')
    print(f'  "{leg}_thigh_pitch": ({j_thigh[0]:.1f}, {j_thigh[1]:.1f}, {j_thigh[2]:.1f}),')
    print(f'  "{leg}_knee_pitch":  ({j_knee[0]:.1f}, {j_knee[1]:.1f}, {j_knee[2]:.1f}),')
    print(f'  "{leg}_foot_fixed":  ({j_foot[0]:.1f}, {j_foot[1]:.1f}, {j_foot[2]:.1f}),')
print("}")

# Transform CAD → URDF: X flip, Y↔Z swap, translate
def to_urdf(p, origin):
    return (origin[0] - p[0], p[2] - origin[2], p[1] - origin[1])

# Compute URDF joint origins (each relative to its parent joint)
print("\n# URDF joint origins (m, relative to parent) — for leg.xacro")
print("URDF_JOINT_ORIGINS = {")
for leg in ["FL", "FR", "BL", "BR"]:
    hip = centroids[f"{leg}_hip_link"]
    thigh = centroids[f"{leg}_thigh_link"]
    shank = centroids[f"{leg}_shank_link"]
    foot = centroids[f"{leg}_foot_link"]
    j_hip = MEASURED_HIP[leg]
    j_thigh = MEASURED_THIGH[leg]
    j_knee = MEASURED_KNEE[leg]
    j_foot = mid(shank, foot)

    u_hip = to_urdf(j_hip, BODY_CENTER)         # base → hip
    u_thigh = to_urdf(j_thigh, j_hip)           # hip → thigh
    u_knee = to_urdf(j_knee, j_thigh)           # thigh → knee
    u_foot = to_urdf(j_foot, j_knee)            # knee → foot

    print(f'  # {leg}:')
    print(f'  "{leg}_hip_yaw":     ({u_hip[0]/1000:.5f}, {u_hip[1]/1000:.5f}, {u_hip[2]/1000:.5f}),')
    print(f'  "{leg}_thigh_pitch": ({u_thigh[0]/1000:.5f}, {u_thigh[1]/1000:.5f}, {u_thigh[2]/1000:.5f}),')
    print(f'  "{leg}_knee_pitch":  ({u_knee[0]/1000:.5f}, {u_knee[1]/1000:.5f}, {u_knee[2]/1000:.5f}),')
    print(f'  "{leg}_foot_fixed":  ({u_foot[0]/1000:.5f}, {u_foot[1]/1000:.5f}, {u_foot[2]/1000:.5f}),')
print("}")
