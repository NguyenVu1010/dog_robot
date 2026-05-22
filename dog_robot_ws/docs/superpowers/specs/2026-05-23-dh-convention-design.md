# DH Convention Conversion Design

**Date:** 2026-05-23
**Status:** Approved (user fast-track)
**Scope:** Convert dog_robot to Modified DH convention (Craig). Rewrite URDF leg frames, add closed-form DH-based IK, stand-only controller. CHAMP control pipeline disabled (walking out of scope, follow-up plan).

---

## Goals

1. URDF leg frames follow Modified DH (Craig): joint axis = Z_i, X_i along common normal.
2. One symmetric DH table for all 4 legs (per-leg variation absorbed into base→hip fixed transform).
3. New Python kinematics module (`kinematics_dh.py`) with closed-form FK + IK for one 3-DOF leg.
4. New `stand_controller` ROS 2 node — accepts body pose target, ramps joints, publishes to effort controller.
5. README documents DH table and FK/IK derivation.
6. pytest verifies FK ↔ IK roundtrip; Gazebo verifies stand stays upright.

## Non-goals

- Walking gait (deferred — follow-up plan can layer trot on top of DH-IK).
- Replacing CHAMP source — CHAMP fork stays in `src/champ/` but isn't launched.
- STM32 / servo bridge.

---

## Frame Convention

### Body frame B (URDF root, unchanged)
- Origin at `base_link`, axes: X forward, Y left, Z up.

### Hip frame H_<leg> (DH frame 0 per leg)
- Origin at hip yaw joint center for that leg.
- Z_H along the hip yaw axis (= body X).
- X_H downward (= -body Z).
- Y_H by right-hand rule.
- Realized in URDF as the parent-of-hip_yaw link orientation via the base→hip fixed transform.

### DH frames 1, 2, 3 (per leg)
| Frame | After joint | Z direction | X direction |
|---|---|---|---|
| 1 | hip_yaw (θ₁) | along hip yaw axis | along common normal to thigh axis |
| 2 | thigh_pitch (θ₂) | along thigh axis | along thigh toward knee |
| 3 | knee_pitch (θ₃) | along knee axis | along shank toward foot |
| F | foot tool frame | parallel Z₃ | — |

### Mirror convention
- Left legs (FL, BL): base→hip rpy = `(0, +π/2, 0)` so Z_H = +X_body.
- Right legs (FR, BR): base→hip rpy = `(0, +π/2, π)` so Z_H = +X_body, X_H = +Z_body — places the leg geometry on the opposite side of body Y with the same DH equations.
- One DH table + one IK function works for all 4 legs.

---

## DH Table (Modified DH / Craig)

Convention: T_{i-1→i} = Rx(α_{i-1}) · Tx(a_{i-1}) · Rz(θ_i) · Tz(d_i)

| i | α_{i-1} | a_{i-1} | d_i | θ_i      | Notes |
|---|---------|---------|-----|----------|-------|
| 1 | 0       | 0       | 0   | θ_hip    | hip yaw |
| 2 | -π/2    | L_hh    | 0   | θ_thigh  | twist 90° so thigh axis ⟂ hip axis |
| 3 | 0       | L_th    | 0   | θ_knee   | knee pitch |
| F | 0       | L_sh    | 0   | 0        | foot tool frame |

**Link length constants** (extracted from CAD via `scripts/compute_joints.py`):
- `L_hh` = 0.040 m — hip-to-thigh common normal distance
- `L_th` = 0.117 m — thigh length
- `L_sh` = 0.070 m — shank length

Exact values written into `dog_robot_control/config/dh_params.yaml` after CAD recomputation. Per-leg CAD asymmetry (currently ≤2 mm + ≤5° rpy) absorbed entirely into base→hip rigid transform.

---

## URDF Rewrite

### `leg.xacro` changes

- Drop parameters: `thigh_rpy`, `knee_rpy`, `mesh_thigh_rpy`, `mesh_shank_rpy`, `mesh_foot_rpy` (CHAMP-IK surgery legacy).
- Add: `base_to_hip_xyz`, `base_to_hip_rpy`, `mirror` (left/right).
- New joint origins. URDF origin is `T_xyz · R_rpy`. For Modified DH frame transition `Rx(α_{i-1})·Tx(a_{i-1})·Rz(θ_i)·Tz(d_i)` at θ_i=0, d_i=0, the parent→child transform equals `T(a_{i-1}, 0, 0) · Rx(α_{i-1})`, so we set `xyz=(a, 0, 0)`, `rpy=(α, 0, 0)`:
  - `${prefix}_hip_yaw`: xyz=`base_to_hip_xyz`, rpy=`base_to_hip_rpy`, axis=`0 0 1`.
  - `${prefix}_thigh_pitch`: xyz=`(L_hh, 0, 0)`, rpy=`(-π/2, 0, 0)`, axis=`0 0 1`.
  - `${prefix}_knee_pitch`: xyz=`(L_th, 0, 0)`, rpy=`(0, 0, 0)`, axis=`0 0 1`.
  - `${prefix}_foot_fixed`: xyz=`(L_sh, 0, 0)`, rpy=`(0, 0, 0)`.
- Joint axis is always `0 0 1` (DH convention).

### Visual mesh compensation
Mesh STLs are authored in original CAD orientation. After re-orienting joint frames, add a rotation `<origin rpy="...">` on each `<visual>` so meshes still render correctly. Compensation rpy per link computed by composing the inverse of the cumulative frame rotation from base to that link in the original URDF.

Compute via a helper script `scripts/compute_visual_compensation.py` (numpy). Each leg gets four compensating rpy values: hip_link, thigh_link, shank_link, foot_link. Mesh translations also need offset because mesh origin in CAD assumed the old frame; the offset is the inverse of the frame translation. Helper script outputs ready-to-paste xacro snippets.

### `dog_robot.urdf.xacro` per-leg config
Replace the four `<xacro:leg>` calls with:
```xml
<xacro:leg prefix="FL" base_to_hip_xyz="0.07480 0.04000 0.03510" base_to_hip_rpy="0 1.5708 0" mirror="left" .../>
<xacro:leg prefix="FR" base_to_hip_xyz="0.07480 -0.04000 0.03510" base_to_hip_rpy="0 1.5708 3.1416" mirror="right" .../>
<xacro:leg prefix="BL" base_to_hip_xyz="-0.07480 0.04000 0.03510" base_to_hip_rpy="0 1.5708 0" mirror="left" .../>
<xacro:leg prefix="BR" base_to_hip_xyz="-0.07480 -0.04000 0.03510" base_to_hip_rpy="0 1.5708 3.1416" mirror="right" .../>
```
Per-leg CAD position differences in Z direction (current asymmetry of ≤1 mm) folded into `base_to_hip_xyz` Z.

### `ros2_control.xacro`
No structural change — still effort command interface, 12 joints. Joint names stay the same so existing controller configs continue to work.

### `gazebo.xacro`
Foot friction, self_collide settings unchanged.

---

## DH-IK Module

### File: `dog_robot_control/dog_robot_control/kinematics_dh.py`

```python
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class DHParams:
    L_hh: float
    L_th: float
    L_sh: float

def mdh_transform(alpha: float, a: float, d: float, theta: float) -> np.ndarray:
    """Modified DH (Craig) homogeneous transform i-1 -> i."""
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct,       -st,      0,     a],
        [st*ca,  ct*ca,  -sa, -d*sa],
        [st*sa,  ct*sa,   ca,  d*ca],
        [0,         0,        0,     1],
    ])

def fk_leg(dh: DHParams, theta: tuple) -> np.ndarray:
    """Foot position in hip frame H. theta = (θ_hip, θ_thigh, θ_knee). Returns (x, y, z)."""
    A1 = mdh_transform(0,      0,       0, theta[0])
    A2 = mdh_transform(-np.pi/2, dh.L_hh, 0, theta[1])
    A3 = mdh_transform(0,      dh.L_th, 0, theta[2])
    AF = mdh_transform(0,      dh.L_sh, 0, 0)
    T = A1 @ A2 @ A3 @ AF
    return T[:3, 3]

def ik_leg(dh: DHParams, foot_h: np.ndarray, knee_direction: int = +1) -> tuple:
    """Closed-form IK for foot target in hip frame H.
    Returns (θ_hip, θ_thigh, θ_knee) or raises ValueError if unreachable.
    knee_direction: +1 = knee bends forward, -1 = knee bends backward.
    """
    x, y, z = foot_h
    # Hip yaw: rotates foot around X_H (= Z of hip frame).
    # After hip_yaw, foot must lie in the plane Y_post_hip = -L_hh (offset shift).
    # Solve θ_hip from y, z, L_hh.
    r_yz = np.hypot(y, z)
    if r_yz < dh.L_hh:
        raise ValueError(f"foot too close to hip axis: r_yz={r_yz}, L_hh={dh.L_hh}")
    theta_hip = np.arctan2(-y, -z) - np.arctan2(dh.L_hh, np.sqrt(r_yz**2 - dh.L_hh**2))
    # After hip rotation, work in 2R planar (X_H, Z'_H) plane.
    z_planar = -np.sqrt(r_yz**2 - dh.L_hh**2)  # foot Z in rotated frame, negative = down
    x_planar = x
    # 2R IK: thigh (L_th) and shank (L_sh)
    d = (x_planar**2 + z_planar**2 - dh.L_th**2 - dh.L_sh**2) / (2 * dh.L_th * dh.L_sh)
    if abs(d) > 1.0:
        raise ValueError(f"foot out of reach: x={x}, z={z}, d={d}")
    theta_knee = knee_direction * np.arccos(d)
    theta_thigh = np.arctan2(x_planar, z_planar) - np.arctan2(
        dh.L_sh * np.sin(theta_knee),
        dh.L_th + dh.L_sh * np.cos(theta_knee),
    )
    return (theta_hip, theta_thigh, theta_knee)
```

### File: `dog_robot_control/dog_robot_control/leg_config.py`

```python
from dataclasses import dataclass
from .kinematics_dh import DHParams

@dataclass(frozen=True)
class LegConfig:
    name: str            # "FL" | "FR" | "BL" | "BR"
    base_to_hip_xyz: tuple
    mirror: int          # +1 left, -1 right (sign of L_hh in IK)

LEGS = (
    LegConfig("FL", ( 0.07480,  0.04000, 0.03510), +1),
    LegConfig("FR", ( 0.07480, -0.04000, 0.03510), -1),
    LegConfig("BL", (-0.07480,  0.04000, 0.03510), +1),
    LegConfig("BR", (-0.07480, -0.04000, 0.03510), -1),
)
DH = DHParams(L_hh=0.040, L_th=0.117, L_sh=0.070)  # loaded from yaml at runtime
```

### File: `dog_robot_control/config/dh_params.yaml`

```yaml
/**:
  ros__parameters:
    dh:
      L_hh: 0.040
      L_th: 0.117
      L_sh: 0.070
    legs:
      FL: { base_to_hip_xyz: [ 0.07480,  0.04000, 0.03510], mirror:  1 }
      FR: { base_to_hip_xyz: [ 0.07480, -0.04000, 0.03510], mirror: -1 }
      BL: { base_to_hip_xyz: [-0.07480,  0.04000, 0.03510], mirror:  1 }
      BR: { base_to_hip_xyz: [-0.07480, -0.04000, 0.03510], mirror: -1 }
    stand:
      default_height: 0.18
      ramp_time: 2.0
```

Exact `L_hh`, `L_th`, `L_sh` are recomputed by `scripts/compute_joints.py` (already exists for current CAD measurement workflow) and pasted in.

---

## Stand Controller Node

### File: `dog_robot_control/dog_robot_control/stand_controller.py`

ROS 2 node. Responsibilities:
- Subscribe `/stand_cmd` (geometry_msgs/Pose). Pose.position = body target in world (z = body height, x/y currently ignored). Pose.orientation = body rpy target (ignored for v1, set to 0).
- On startup: read `/joint_states`, hold current pose for 1 s, then ramp linearly to default stand pose over `ramp_time` (default 2 s).
- For each leg: compute foot target in hip frame H, run `ik_leg`, build `JointTrajectoryPoint`.
- Publish `/joint_group_effort_controller/joint_trajectory` at 50 Hz with current ramp position.
- If IK raises `ValueError`, log warn and skip that publish cycle.

### Foot target convention
- Default foot position in hip frame H = `(0, sign·L_hh, -body_height)` where sign matches mirror.
- This places foot directly below the thigh pitch axis vertically, with the natural L_hh lateral offset.

### File: `dog_robot_control/launch/stand.launch.py`

Launch order:
1. `gazebo_ros gzserver` + `gzclient` with `simple.world`.
2. Spawn robot URDF (via `xacro_clean.sh` from existing `description.launch.py`).
3. Load `joint_state_broadcaster` + `joint_group_effort_controller`.
4. Start `stand_controller` node with `dh_params.yaml`.
5. (No champ launch.)

Kill script `scripts/dog_kill_all.sh` already covers gzserver/gzclient/rviz2/joint_state_publisher; add `stand_controller` pattern.

---

## README — DH Section

Append to `dog_robot_ws/README.md` (create if missing):

````markdown
## Kinematics

The dog_robot uses Modified Denavit-Hartenberg (Craig) convention. Each leg is a 3-DOF chain (hip yaw, thigh pitch, knee pitch).

### Frames
- **Body B** — URDF root (X forward, Y left, Z up).
- **Hip H_<leg>** — fixed transform per leg; Z_H along hip yaw axis (= body X), X_H down.
- **DH frames 1-3** — at each joint, Z along joint axis.

### DH Table
| i | α_{i-1} | a_{i-1} | d_i | θ_i |
|---|---------|---------|-----|-----|
| 1 | 0       | 0       | 0   | θ_hip |
| 2 | -π/2    | L_hh = 0.040 m | 0 | θ_thigh |
| 3 | 0       | L_th = 0.117 m | 0 | θ_knee |
| F | 0       | L_sh = 0.070 m | 0 | 0 |

### Forward kinematics
```python
from dog_robot_control.kinematics_dh import DHParams, fk_leg
dh = DHParams(L_hh=0.040, L_th=0.117, L_sh=0.070)
foot_xyz = fk_leg(dh, (theta_hip, theta_thigh, theta_knee))
```

### Inverse kinematics
```python
from dog_robot_control.kinematics_dh import ik_leg
theta_hip, theta_thigh, theta_knee = ik_leg(dh, foot_xyz_in_hip_frame)
```

Closed-form, deterministic. Raises `ValueError` if target is unreachable.

### Per-leg base→hip transforms
| Leg | base→hip xyz (m) | Mirror |
|---|---|---|
| FL | ( 0.0748,  0.0400, 0.0351) | left  (+1) |
| FR | ( 0.0748, -0.0400, 0.0351) | right (-1) |
| BL | (-0.0748,  0.0400, 0.0351) | left  (+1) |
| BR | (-0.0748, -0.0400, 0.0351) | right (-1) |

Mirror: right legs negate `L_hh` in IK; URDF base→hip rpy adds π yaw.
````

---

## Testing

### `test/test_kinematics_dh.py` (pytest, in dog_robot_control)

1. **FK at zero angles** — foot at expected position from DH table.
2. **FK roundtrip** — for 50 random (θ_hip, θ_thigh, θ_knee) in joint limits, IK(FK(θ)) ≈ θ within 1e-6 rad.
3. **IK roundtrip** — for 50 random reachable foot positions, FK(IK(p)) ≈ p within 1e-6 m.
4. **IK unreachable** — foot at 5x leg length raises `ValueError`.
5. **Mirror symmetry** — IK(foot, mirror=+1) and IK(mirrored_foot, mirror=-1) give angles related by sign convention.

### `test/test_urdf_dh_consistency.py` (pytest)

Build URDF via `xacro_clean.sh`, parse with `urdf_parser_py`. For 20 random joint angle vectors:
- Compute foot pose via URDF FK chain.
- Compute foot pose via `kinematics_dh.fk_leg`.
- Assert agreement within 1e-4 m / 1e-4 rad on all 4 legs.

### Gazebo regression

Manual: launch `stand.launch.py`, robot should reach default stand height within 3 s and stay upright for ≥10 s (body roll/pitch < 0.1 rad).

---

## Migration / Cleanup

- Old `gazebo.launch.py` (CHAMP) stays in tree for reference but not invoked. Add a `# DEPRECATED` comment.
- `champ_*` packages stay in `src/champ/` (still built — disabling build requires CMake skips not worth it).
- CHAMP-IK surgery params in `dog_robot.urdf.xacro` go away entirely. Old commit `1a17849` ("fix(description): rotate leg frames so CHAMP IK matches URDF") becomes historical context only.
- `kill_all.sh`: add `stand_controller` to pkill patterns.

---

## Architecture summary

```
User cmd
   │
   ▼
/stand_cmd  ─────►  stand_controller node
                          │
                          ▼
                    kinematics_dh.ik_leg (×4)
                          │
                          ▼
              /joint_group_effort_controller/joint_trajectory
                          │
                          ▼
                  gazebo_ros2_control
                          │
                          ▼
                      Gazebo sim
```

CHAMP packages still build (unused). Walking is a follow-up plan that will sit between user cmd_vel and `stand_controller` (or replace it with a gait_controller using the same DH-IK).

---

## Open questions (resolved during brainstorming)

- ✅ DH variant: Modified DH (Craig)
- ✅ Symmetry: one symmetric table, 4 base→hip transforms
- ✅ IK module location: dog_robot_control, bypass CHAMP IK
- ✅ Approach: stand-only (walking is follow-up)
- ✅ User fast-track approval: all sections approved, auto-implement.
