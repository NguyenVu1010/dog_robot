"""Derive joint-attached link frames from CAD-measured joint centers.

Convention (see specs/2026-05-26-joint-frame-export-design.md):
    Per link: origin at parent joint center; Z along parent joint axis;
    X = orthogonalised (J_child - J_parent); Y = Z x X.
    base_link and *_foot_link: URDF-standard (Z up, X forward).
"""
from __future__ import annotations
from typing import Dict, Tuple

import numpy as np


def _require_yaml():
    try:
        import yaml
        return yaml
    except ImportError as exc:
        raise SystemExit("PyYAML missing — `pip install pyyaml`") from exc


# Body center in CAD frame (mm). Same value as scripts/compute_joints.py.
BODY_CENTER_MM: Tuple[float, float, float] = (100.0, -22.6, -40.0)

# Joint axis centers in CAD frame (mm). Copied from
# dog_robot_ws/src/dog_robot_description/scripts/derive_dh_frames.py
# (which copied them from /workspace/dog_robot/scripts/compute_joints.py).
MEASURED_HIP_MM: Dict[str, Tuple[float, float, float]] = {
    "FL": (25.200, 12.500,   0.000),
    "FR": (25.200, 12.500, -80.000),
    "BL": (174.800, 12.500,   0.000),
    "BR": (174.800, 12.500, -80.000),
}
MEASURED_THIGH_MM: Dict[str, Tuple[float, float, float]] = {
    "FL": (0.000,   -0.671,   25.362),
    "FR": (0.000,    0.000, -105.700),
    "BL": (200.000, -0.675,   25.361),
    "BR": (200.000,  0.000, -105.700),
}
MEASURED_KNEE_MM: Dict[str, Tuple[float, float, float]] = {
    "FL": (88.875,  -65.224,   66.379),
    "FR": (87.991,  -64.673, -148.400),
    "BL": (283.410, -72.261,   66.183),
    "BR": (282.987, -70.980, -148.400),
}
# Foot center: midpoint of shank-cluster and foot-cluster centroids in CAD mm.
# Source: scripts/compute_joints.py output ("CAD_JOINT_POSITIONS" section,
# *_foot_fixed entries). To regenerate, run
#   python3 scripts/compute_joints.py 2>&1 | grep foot_fixed
MEASURED_FOOT_MM: Dict[str, Tuple[float, float, float]] = {
    "FL": (25.38,  -134.29,   42.874),
    "FR": (25.38,  -134.29, -122.874),
    "BL": (174.98, -134.29,   42.874),
    "BR": (174.98, -134.29, -122.874),
}


def cad_to_urdf_point(p_mm: Tuple[float, float, float],
                       origin_mm: Tuple[float, float, float] = BODY_CENTER_MM
                       ) -> np.ndarray:
    """Convert CAD point (mm) to URDF point (m)."""
    p = np.asarray(p_mm, dtype=float)
    o = np.asarray(origin_mm, dtype=float)
    return 0.001 * np.array([o[0] - p[0], p[2] - o[2], p[1] - o[1]])


def cad_to_urdf_direction(v_cad) -> np.ndarray:
    """Convert CAD direction vector to URDF (linear part of cad_to_urdf_point)."""
    v = np.asarray(v_cad, dtype=float)
    return np.array([-v[0], v[2], v[1]])


# CAD axis directions for joints derived from FreeCAD circle inspection.
# Hip is set to URDF body +X (forward) per ROS REP-103: first leg joint is the
# hip-roll / abduction joint, rotating the leg in the YZ body plane.
CAD_AXIS = {
    "thigh": np.array([0.0, 0.0, 1.0]),  # CAD Z -> URDF +Y (pitch axis)
    "knee":  np.array([0.0, 0.0, 1.0]),  # CAD Z -> URDF +Y (pitch axis)
}

# Hip rotation axis in URDF body frame (uniform across 4 legs).
HIP_AXIS_URDF = np.array([1.0, 0.0, 0.0])


def joint_axes_urdf() -> Dict[str, Dict[str, np.ndarray]]:
    """Per-leg unit joint axes in URDF frame, sign-normalised to +1 on dominant axis.

    Axes are symmetric across the four legs; the per-leg dict structure
    matches joint_centers_urdf for uniform downstream access.
    """
    out: Dict[str, Dict[str, np.ndarray]] = {}
    for leg in ("FL", "FR", "BL", "BR"):
        leg_axes: Dict[str, np.ndarray] = {"hip": HIP_AXIS_URDF.copy()}
        for jname, vc in CAD_AXIS.items():
            v = cad_to_urdf_direction(vc)
            v = v / np.linalg.norm(v)
            dom = int(np.argmax(np.abs(v)))
            if v[dom] < 0:
                v = -v
            leg_axes[jname] = v
        out[leg] = leg_axes
    return out


def joint_centers_urdf() -> Dict[str, Dict[str, np.ndarray]]:
    """Per-leg dict of joint center positions in URDF frame (m)."""
    out: Dict[str, Dict[str, np.ndarray]] = {}
    for leg in ("FL", "FR", "BL", "BR"):
        out[leg] = {
            "hip":   cad_to_urdf_point(MEASURED_HIP_MM[leg]),
            "thigh": cad_to_urdf_point(MEASURED_THIGH_MM[leg]),
            "knee":  cad_to_urdf_point(MEASURED_KNEE_MM[leg]),
            "foot":  cad_to_urdf_point(MEASURED_FOOT_MM[leg]),
        }
    return out


def _orthogonalise(target_dir: np.ndarray, z: np.ndarray,
                    name: str) -> np.ndarray:
    perp = target_dir - np.dot(target_dir, z) * z
    n = np.linalg.norm(perp)
    if n < 1e-6:
        raise ValueError(f"{name}: target direction parallel to Z; degenerate")
    return perp / n


def _frame_from_zaxis_and_target(O: np.ndarray, z_axis: np.ndarray,
                                  target: np.ndarray, name: str) -> Dict[str, np.ndarray]:
    z = z_axis / np.linalg.norm(z_axis)
    x = _orthogonalise(target - O, z, name)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    return {"O": O.copy(), "R": R}


def link_frames_urdf() -> Dict[str, Dict[str, np.ndarray]]:
    """Per-link frame {O, R} in URDF root.  17 entries: base + 4 legs * 4 links."""
    centers = joint_centers_urdf()
    axes = joint_axes_urdf()
    frames: Dict[str, Dict[str, np.ndarray]] = {
        "base_link": {"O": np.zeros(3), "R": np.eye(3)},
    }
    for leg in ("FL", "FR", "BL", "BR"):
        c = centers[leg]
        a = axes[leg]
        frames[f"{leg}_hip_link"] = _frame_from_zaxis_and_target(
            c["hip"], a["hip"], c["thigh"], f"{leg}_hip_link")
        frames[f"{leg}_thigh_link"] = _frame_from_zaxis_and_target(
            c["thigh"], a["thigh"], c["knee"], f"{leg}_thigh_link")
        frames[f"{leg}_shank_link"] = _frame_from_zaxis_and_target(
            c["knee"], a["knee"], c["foot"], f"{leg}_shank_link")
        # Foot: world-aligned at foot center.
        frames[f"{leg}_foot_link"] = {"O": c["foot"].copy(), "R": np.eye(3)}
    return frames


def _length(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(b - a))


def link_params() -> Dict[str, object]:
    """Compute mean link lengths and constant inter-link rotations across 4 legs.

    Returns a dict with:
      L_hh, L_th, L_sh  – mean link lengths (m) across the 4 legs
      per_leg            – per-leg breakdown dict[leg][L_*]
      R_const_ht         – mean rotation hip_link -> thigh_link (3x3 SO(3))
      R_const_tk         – mean rotation thigh_link -> shank_link (3x3 SO(3))
      R_const_kf         – mean rotation shank_link -> foot_link (3x3 SO(3))
    """
    centers = joint_centers_urdf()
    frames = link_frames_urdf()
    per_leg: Dict[str, Dict[str, float]] = {}
    R_hts, R_tks, R_kfs = [], [], []
    for leg in ("FL", "FR", "BL", "BR"):
        c = centers[leg]
        per_leg[leg] = {
            "L_hh": _length(c["hip"],   c["thigh"]),
            "L_th": _length(c["thigh"], c["knee"]),
            "L_sh": _length(c["knee"],  c["foot"]),
        }
        Rh = frames[f"{leg}_hip_link"]["R"]
        Rt = frames[f"{leg}_thigh_link"]["R"]
        Rs = frames[f"{leg}_shank_link"]["R"]
        Rf = frames[f"{leg}_foot_link"]["R"]
        R_hts.append(Rh.T @ Rt)
        R_tks.append(Rt.T @ Rs)
        R_kfs.append(Rs.T @ Rf)

    def mean(key: str) -> float:
        return float(np.mean([per_leg[L][key] for L in per_leg]))

    out: Dict[str, object] = {
        "L_hh": mean("L_hh"), "L_th": mean("L_th"), "L_sh": mean("L_sh"),
        "per_leg": per_leg,
        "R_const_ht": np.mean(R_hts, axis=0),
        "R_const_tk": np.mean(R_tks, axis=0),
        "R_const_kf": np.mean(R_kfs, axis=0),
    }
    # Mean of rotation matrices isn't a rotation; re-orthonormalise via SVD.
    for k in ("R_const_ht", "R_const_tk", "R_const_kf"):
        U, _, Vt = np.linalg.svd(out[k])
        R = U @ Vt
        if np.linalg.det(R) < 0:
            U[:, -1] *= -1
            R = U @ Vt
        out[k] = R
    return out


def _reorthonormalise(M: np.ndarray) -> np.ndarray:
    """Nearest proper rotation (SO(3)) to M via SVD."""
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


# (parent link suffix, child link suffix, transition key)
_TRANSITIONS = (("hip_link", "thigh_link", "hip_to_thigh"),
                ("thigh_link", "shank_link", "thigh_to_knee"),
                ("shank_link", "foot_link", "knee_to_foot"))


def leg_joint_transforms() -> Dict[str, Dict[str, Dict[str, np.ndarray]]]:
    """Per-leg fixed joint transforms in the parent frame.

    Returns dict[leg][transition] = {"xyz": (3,), "R": (3,3)} where xyz is the
    child origin in the parent frame and R = R_parent^T @ R_child (the constant
    inter-link rotation). Exact per leg — the four legs differ (front thighs
    point forward, back thighs backward; left/right mirror), so no averaging.
    The in-plane Y component of xyz is ~0 by frame construction.
    """
    frames = link_frames_urdf()
    out: Dict[str, Dict[str, Dict[str, np.ndarray]]] = {}
    for leg in ("FL", "FR", "BL", "BR"):
        out[leg] = {}
        for parent, child, key in _TRANSITIONS:
            P, C = f"{leg}_{parent}", f"{leg}_{child}"
            Op, Rp = frames[P]["O"], frames[P]["R"]
            Oc, Rc = frames[C]["O"], frames[C]["R"]
            out[leg][key] = {"xyz": Rp.T @ (Oc - Op), "R": Rp.T @ Rc}
    return out


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------

from pathlib import Path


def _rotation_to_quat_xyzw(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert 3x3 rotation matrix to (x, y, z, w) quaternion. FreeCAD convention."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


def _matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """ZYX intrinsic Euler (URDF convention: roll-pitch-yaw)."""
    sy = -R[2, 0]
    cy = float(np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    if cy > 1e-9:
        roll  = float(np.arctan2(R[2, 1], R[2, 2]))
        pitch = float(np.arctan2(sy, cy))
        yaw   = float(np.arctan2(R[1, 0], R[0, 0]))
    else:
        roll  = float(np.arctan2(-R[1, 2], R[1, 1]))
        pitch = float(np.arctan2(sy, cy))
        yaw   = 0.0
    return (roll, pitch, yaw)


def _link_placements_cad() -> Dict[str, Dict]:
    """Per-link Placement in CAD frame (mm + quat xyzw) for FreeCAD exporter."""
    cad_centers = {
        "FL": dict(hip=MEASURED_HIP_MM["FL"], thigh=MEASURED_THIGH_MM["FL"],
                    knee=MEASURED_KNEE_MM["FL"], foot=MEASURED_FOOT_MM["FL"]),
        "FR": dict(hip=MEASURED_HIP_MM["FR"], thigh=MEASURED_THIGH_MM["FR"],
                    knee=MEASURED_KNEE_MM["FR"], foot=MEASURED_FOOT_MM["FR"]),
        "BL": dict(hip=MEASURED_HIP_MM["BL"], thigh=MEASURED_THIGH_MM["BL"],
                    knee=MEASURED_KNEE_MM["BL"], foot=MEASURED_FOOT_MM["BL"]),
        "BR": dict(hip=MEASURED_HIP_MM["BR"], thigh=MEASURED_THIGH_MM["BR"],
                    knee=MEASURED_KNEE_MM["BR"], foot=MEASURED_FOOT_MM["BR"]),
    }

    def urdf_to_cad_dir(v: np.ndarray) -> np.ndarray:
        return np.array([-v[0], v[2], v[1]])

    def urdf_rotation_to_cad(R: np.ndarray) -> np.ndarray:
        return np.column_stack([urdf_to_cad_dir(R[:, i]) for i in range(3)])

    frames = link_frames_urdf()
    out: Dict[str, Dict] = {}
    out["base_link"] = {
        "position_cad_mm": list(BODY_CENTER_MM),
        "quat_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    for leg in ("FL", "FR", "BL", "BR"):
        c = cad_centers[leg]
        for link, joint in (("hip_link", "hip"), ("thigh_link", "thigh"),
                              ("shank_link", "knee"), ("foot_link", "foot")):
            name = f"{leg}_{link}"
            R_cad = urdf_rotation_to_cad(frames[name]["R"])
            out[name] = {
                "position_cad_mm": [float(v) for v in c[joint]],
                "quat_xyzw": list(_rotation_to_quat_xyzw(R_cad)),
            }
    return out


def write_outputs(out_dir: Path) -> None:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    yaml = _require_yaml()
    lp = link_params()
    frames = link_frames_urdf()

    (out_dir / "joint_frames.yaml").write_text(
        "# Generated by scripts/derive_joint_frames.py — do not edit by hand.\n"
        "# Per-link joint-attached Placement in CAD frame (mm + quat xyzw).\n"
        + yaml.safe_dump({"links": _link_placements_cad()}, sort_keys=False),
        encoding="utf-8")

    legs = leg_joint_transforms()
    lp_doc = {
        # Scalar link lengths (full 3D joint-to-joint distance), for gait geometry.
        "L_hh": float(lp["L_hh"]), "L_th": float(lp["L_th"]), "L_sh": float(lp["L_sh"]),
    }
    for leg in ("FL", "FR", "BL", "BR"):
        lp_doc[leg] = {
            key: {
                "xyz": [float(v) for v in legs[leg][key]["xyz"]],
                "rpy": list(_matrix_to_rpy(legs[leg][key]["R"])),
            }
            for key in ("hip_to_thigh", "thigh_to_knee", "knee_to_foot")
        }
    (out_dir / "link_params.yaml").write_text(
        "# Generated by scripts/derive_joint_frames.py — do not edit by hand.\n"
        "# L_* are gait-reference lengths. Per-leg (FL/FR/BL/BR) blocks hold the\n"
        "# exact per-joint (xyz, rpy) transforms in the parent frame — front and\n"
        "# back legs differ, so each leg is stored exactly (no averaging).\n"
        + yaml.safe_dump(lp_doc, sort_keys=False),
        encoding="utf-8")

    per_leg = {}
    for leg in ("FL", "FR", "BL", "BR"):
        R_hip = frames[f"{leg}_hip_link"]["R"]
        O_hip = frames[f"{leg}_hip_link"]["O"]
        per_leg[leg] = {
            "base_to_hip_xyz": [float(v) for v in O_hip],
            "base_to_hip_rpy": list(_matrix_to_rpy(R_hip)),
        }
    (out_dir / "urdf_joints.yaml").write_text(
        "# Generated by scripts/derive_joint_frames.py — do not edit by hand.\n"
        + yaml.safe_dump({"per_leg": per_leg}, sort_keys=False),
        encoding="utf-8")


def main() -> None:
    out_dir = Path(__file__).resolve().parents[1] / "config"
    write_outputs(out_dir)
    print(f"wrote {out_dir}/joint_frames.yaml")
    print(f"wrote {out_dir}/link_params.yaml")
    print(f"wrote {out_dir}/urdf_joints.yaml")


if __name__ == "__main__":
    main()
