# Joint-attached frame re-export — Design

**Date:** 2026-05-26
**Status:** Approved (sections 1–6), pending user spec review
**Scope:** Re-export every link STL from FreeCAD with vertices already in
its **joint-attached frame** (origin at parent joint center, Z along parent
joint axis, X pointing toward child joint center). Replace the entire DH
machinery (`kinematics_dh`, `dh_params.yaml`, derivation + tests) with a
simpler `kinematics_link` module. Eliminates the Modified-DH common-normal
math; URDF `<visual><origin>` stays identity everywhere.

**Supersedes:** [`2026-05-25-dh-mesh-export-design.md`](./2026-05-25-dh-mesh-export-design.md).
The MDH approach landed but the (α, a, d) decomposition is overkill for a
3-DOF symmetric leg — the joint-attached convention captures the same
geometry with one rotation matrix per link transition instead.

---

## 1. Goals

1. Each STL in `meshes/visual_dh/<link>.stl` has vertices in its
   joint-attached link frame — URDF `<visual><origin xyz="0 0 0" rpy="0 0 0"/>`
   on every link.
2. URDF joint origins are derived from one source: measured joint axis
   centers in CAD (`scripts/compute_joints.py`). No hand-tuned values.
3. The kinematics layer carries **only** what's geometrically needed:
   three link lengths plus three constant rotations between adjacent link
   frames. No DH α/a/d, no Craig vs. Hartenberg confusion.
4. RViz shows a coherent quadruped; Gazebo stand controller keeps the
   robot upright after the convention change.

## 2. Non-goals

- Walk gait engine logic — `walker_controller` API is preserved
  (params type swap only); walking remains deferred.
- `dog_robot_kinematic_viz` rig content (rebuilds to pick up new URDF;
  nothing else).
- Re-measuring joint axes in CAD — uses existing `compute_joints.py`
  cluster averages.
- Collision meshes — keep current `collision/` STLs until a follow-up
  task. Visual-only re-export.

---

## 3. Frame convention

Notation: all vectors in URDF root frame, units = m. `J_i` = joint i
center. `a_i` = unit vector of joint i axis.

| Link | Origin `O` | Z axis `ẑ` | X axis `x̂` (provisional) | Y axis `ŷ` |
|---|---|---|---|---|
| `base_link` | `(0,0,0)` (URDF root) | `(0,0,1)` | `(1,0,0)` | `(0,1,0)` |
| `*_hip_link` | `J_hip` | `a_hip` | `J_thigh − J_hip` | `ẑ × x̂` |
| `*_thigh_link` | `J_thigh` | `a_thigh` | `J_knee − J_thigh` | `ẑ × x̂` |
| `*_shank_link` | `J_knee` | `a_knee` | `J_foot − J_knee` | `ẑ × x̂` |
| `*_foot_link` | `J_foot` | `(0,0,1)` | `(1,0,0)` | `(0,1,0)` |

**X orthogonalisation:** `x̂ = (target_dir − (target_dir · ẑ) ẑ) /
||target_dir − (target_dir · ẑ) ẑ||`. Abort with diagnostic if the
perpendicular component has norm < 1e-6 (axes are colinear with
target direction — geometry is degenerate).

**Rotation matrix:** `R_link = [x̂ | ŷ | ẑ]` (columns) is the rotation from
the URDF root frame to the link frame.

**Link lengths** (per leg, used in IK):
- `L_hh = ||J_thigh − J_hip||` (~25 mm)
- `L_th = ||J_knee  − J_thigh||` (~117 mm)
- `L_sh = ||J_foot  − J_knee||` (~70 mm)

These are plain 3D Euclidean distances, not DH `a_i` or `d_i`.

**Constant rotations between adjacent link frames** (encapsulate the
non-identity inter-link orientation):
- `R_const_ht = R_hipᵀ · R_thigh`   (hip → thigh, at θ=0)
- `R_const_tk = R_thighᵀ · R_shank` (thigh → shank, at θ=0)
- `R_const_kf = R_shankᵀ · R_foot`  (shank → foot)

Stored as rpy in `link_params.yaml`. Replaces the role of (α_i, d_i) in MDH.

---

## 4. Architecture

```
            ┌──────────────────────────────────────────────┐
            │ scripts/compute_joints.py (existing)         │
            │ → joint axis centers (CAD mm) per leg        │
            └─────────────────┬────────────────────────────┘
                              ▼
            ┌──────────────────────────────────────────────┐
            │ scripts/derive_joint_frames.py (NEW)         │
            │  In: CAD joint centers + URDF transform      │
            │ Out: 3 YAMLs (see Components)                │
            └─────┬────────────┬───────────────┬───────────┘
                  │            │               │
                  ▼            ▼               ▼
         joint_frames.yaml  link_params.yaml  urdf_joints.yaml
         (FreeCAD input)    (kinematics)      (URDF properties)
                  │                            │
                  ▼                            ▼
   export_dh_links_from_freecad.py     dog_robot.urdf.xacro
   (existing — only rename of          (load 3 properties,
    input YAML path)                    identity visual origin)
                  │                            │
                  ▼                            ▼
         meshes/visual_dh/*.stl    robot_state_publisher
                  │                            │
                  └────────────┬───────────────┘
                               ▼
                          RViz / Gazebo
```

`walker_controller`, `stand_controller`, `leg_controller`,
`body_controller`, `gait_config` all import from `kinematics_link`
(swap-in for `kinematics_dh`).

---

## 5. Components

### 5.1 `scripts/derive_joint_frames.py` (NEW)

Input: cluster averages in `compute_joints.py` (CAD mm).

Procedure per leg:
1. Average each cluster → `J_hip`, `J_thigh`, `J_knee`, `J_foot` (CAD mm).
2. Convert CAD → URDF via the existing transform in `compute_joints.py`:
   `to_urdf(p, BODY_CENTER) = (origin[0] - p[0], p[2] - origin[2],
   p[1] - origin[1])`. Direction vectors transform with the linear part
   only.
3. For each link, build `O, ẑ, x̂, ŷ` per the table in §3.
4. `R_link = [x̂ | ŷ | ẑ]`.
5. For each parent → child joint:
   - `xyz_parent = R_parentᵀ · (J_child − O_parent)`
   - `R_rel = R_parentᵀ · R_child`
   - `rpy = euler_from_matrix(R_rel)`
6. Compute `L_hh, L_th, L_sh` and `R_const_ht, R_const_tk, R_const_kf`.

**Sanity checks (abort on failure):**
- `||R_linkᵀ · R_link − I||_F < 1e-9` (orthonormal)
- `det(R_link) > 0` (right-handed)
- 4 legs symmetric: `max |L_*_i − mean(L_*)| < 1 mm`
- URDF FK rebuild matches measured `J_*` < 0.5 mm

**Outputs:**
- `src/dog_robot_description/config/joint_frames.yaml` — per-link
  Placement (xyz mm + quat xyzw) in CAD frame, consumed by the FreeCAD
  exporter. Format identical to current `dh_link_placements.yaml` so the
  exporter needs only an input-path rename.
- `src/dog_robot_description/config/link_params.yaml` — `L_hh`, `L_th`,
  `L_sh` (mean of 4 legs) + `hip_to_thigh_rpy`, `thigh_to_knee_rpy`,
  `knee_to_foot_rpy`.
- `src/dog_robot_description/config/urdf_joints.yaml` — per-leg
  `base_to_hip_xyz` / `base_to_hip_rpy` (4 entries).

### 5.2 `scripts/export_dh_links_from_freecad.py` (existing, minimal change)

Only change: input YAML path `dh_link_placements.yaml` → `joint_frames.yaml`.
Logic unchanged — compose solids per cluster, apply inverse Placement,
tessellate at 0.05 mm, write `meshes/visual_dh/<link>.stl`. Does not modify
the FreeCAD document.

Precondition unchanged: FreeCAD running, `RobotDog` doc has
`robotdogassem.STEP`, MCP server on port 9875.

### 5.3 `kinematics_link.py` (NEW)

Replaces `kinematics_dh.py`. Same `dog_robot_kinematics` package.

```python
@dataclass(frozen=True)
class LinkParams:
    L_hh: float        # ||J_thigh − J_hip||  (m)
    L_th: float        # ||J_knee  − J_thigh|| (m)
    L_sh: float        # ||J_foot  − J_knee||  (m)
    R_const_ht: np.ndarray  # 3x3, hip→thigh constant rotation
    R_const_tk: np.ndarray  # 3x3, thigh→shank constant rotation
    R_const_kf: np.ndarray  # 3x3, shank→foot constant rotation

def fk_leg(p: LinkParams, theta: tuple[float, float, float]) -> np.ndarray:
    """Foot position in hip-yaw frame F_hip.
    theta = (q_yaw, q_thigh, q_knee).
    """

def ik_leg(p: LinkParams, foot_in_hip: np.ndarray,
           knee_branch: int = +1) -> tuple[float, float, float]:
    """Closed-form 3R: hip yaw + 2R planar (thigh + knee)."""
```

**FK chain (joint-attached, no DH transforms):**
- `T_yaw   = Rz(q_yaw)`
- `T_h→t   = Tx(L_hh) · R_const_ht`
- `T_thigh = Rz(q_thigh)`
- `T_t→k   = Tx(L_th)  · R_const_tk`
- `T_knee  = Rz(q_knee)`
- `T_k→f   = Tx(L_sh)  · R_const_kf`
- `foot_in_hip = (T_yaw · T_h→t · T_thigh · T_t→k · T_knee · T_k→f
                  · [0,0,0,1]ᵀ).xyz`

**IK closed form:**
1. Project `foot_in_hip` onto the plane perpendicular to `Z_hip`;
   `q_yaw = atan2(y, x)`. (Valid because `R_const_ht` is pure Rx/Ry
   for this robot — see §8 risk.)
2. Rotate foot into the hip plane: `foot' = Rz(−q_yaw) · foot`.
3. Subtract `L_hh` along X, transform by `R_const_htᵀ` to enter thigh
   root frame; result is `(u, v)` in the 2R plane.
4. 2R planar:
   - `c = (u² + v² − L_th² − L_sh²) / (2 L_th L_sh)`
   - `q_knee  = knee_branch · acos(c)`
   - `q_thigh = atan2(v, u) − atan2(L_sh sin q_knee,
                                    L_th + L_sh cos q_knee)`

`R_const_kf` does not enter the IK position calculation — foot origin in
shank frame is `(L_sh, 0, 0)` regardless of foot orientation. `R_const_kf`
only matters when an upstream consumer needs foot **orientation**.

**Errors:**
- `|c| > 1` → `ValueError("foot unreachable")`
- foot on hip yaw axis (`sqrt(x²+y²) < 1e-9`) → `ValueError("yaw undefined")`

### 5.4 URDF (`leg.xacro`, `dog_robot.urdf.xacro`)

`leg.xacro` macro:

```xml
<xacro:macro name="leg" params="prefix
                                base_to_hip_xyz base_to_hip_rpy
                                L_hh L_th L_sh
                                hip_to_thigh_rpy
                                thigh_to_knee_rpy:='0 0 0'
                                knee_to_foot_rpy:='0 0 0'">
```

Joint origins:
- `hip_yaw`:     `xyz="${base_to_hip_xyz}"`, `rpy="${base_to_hip_rpy}"`, `axis="0 0 1"`
- `thigh_pitch`: `xyz="${L_hh} 0 0"`,         `rpy="${hip_to_thigh_rpy}"`, `axis="0 0 1"`
- `knee_pitch`:  `xyz="${L_th} 0 0"`,         `rpy="${thigh_to_knee_rpy}"`, `axis="0 0 1"`
- `foot_fixed`:  `xyz="${L_sh} 0 0"`,         `rpy="${knee_to_foot_rpy}"`

All `<visual>`: `<origin xyz="0 0 0" rpy="0 0 0"/>`, mesh path
`meshes/visual_dh/<link>.stl`.

`dog_robot.urdf.xacro` properties (filled from `link_params.yaml` +
`urdf_joints.yaml`):

```xml
<xacro:property name="L_hh" value="<derived>"/>
<xacro:property name="L_th" value="<derived>"/>
<xacro:property name="L_sh" value="<derived>"/>
<xacro:property name="hip_to_thigh_rpy"  value="<derived>"/>
<xacro:property name="thigh_to_knee_rpy" value="<derived>"/>
<xacro:property name="knee_to_foot_rpy"  value="<derived>"/>
```

Per-leg `base_to_hip_xyz` / `rpy` read from `urdf_joints.yaml`.

---

## 6. Migration order

Each step is a separate commit, independently revertable.

1. **derive_joint_frames.py** + 3 YAML output. Self-test passes.
2. **kinematics_link.py** + `test_kinematics_link.py` (200-iter FK/IK
   roundtrip). No consumer yet.
3. **Export STL** via FreeCAD MCP. 17 STLs in `meshes/visual_dh/`.
4. **URDF update** (leg.xacro + dog_robot.urdf.xacro). `xacro` expand +
   `check_urdf` clean.
5. **Rewire controllers**: swap `from kinematics_dh import DHParams,
   ik_leg` → `from kinematics_link import LinkParams, ik_leg` in:
   `walker_controller.py`, `stand_controller.py`, `gait/leg_controller.py`,
   `gait/body_controller.py`, `gait/gait_config.py`.
6. **Update tests**: rename `test_kinematics_dh` → `test_kinematics_link`;
   drop `d_*` cases; migrate `test_walker_integration`, `test_leg_controller`,
   `test_body_controller`.
7. **Smoke test**: `colcon build && colcon test --packages-select
   dog_robot_kinematics dog_robot_control` — all green.
8. **RViz check**: `ros2 launch dog_robot_kinematic_viz kinematic.launch.py`
   — visible quadruped, no flipped links, joints animate cleanly.
9. **Gazebo stand**: `ros2 launch dog_robot_control stand.launch.py` —
   body z > 0.10 m, drift < 0.15 m, held 30 s.
10. **Cleanup commit**: delete `kinematics_dh.py`, `dh_params.yaml`,
    `derive_dh_frames.py`, `dh_link_placements.yaml`,
    `compute_visual_compensation.py`, `bake_meshes_to_link_frame.py`,
    `meshes/visual/`, `meshes/collision/`, `test_dh_derivation.py`,
    `test_urdf_dh_consistency.py`, `test_kinematics_dh.py`.

---

## 7. Tests

- `test_derive_joint_frames.py` (NEW): feed `compute_joints.py` cluster
  averages, assert 4 legs yield same `L_*` within 1 mm; assert all
  `R_link` orthonormal + right-handed.
- `test_kinematics_link.py` (NEW): 200 random foot targets in workspace,
  `fk(ik(foot)) ≈ foot` within 1e-6 m, both knee branches.
- `test_urdf_link_consistency.py` (NEW): xacro-expand `dog_robot.urdf.xacro`,
  manually chain joint origins for 40 random `(q_yaw, q_thigh, q_knee)`
  per leg, assert foot position matches `fk_leg(p, θ)` < 1e-9 m.

---

## 8. Error handling

- `derive_joint_frames.py` symmetry check fail (legs disagree > 1 mm) →
  print per-leg `L_*` and abort. Indicates measurement inconsistency in
  `compute_joints.py`.
- Degenerate X axis (target direction parallel to Z) → abort with link
  name + axes printed.
- FreeCAD MCP unreachable → exporter aborts with "open FreeCAD with port
  9875 first" (existing behaviour).
- Empty solid cluster for a link → warn + skip; summary lists missing
  links (existing).
- `kinematics_link.ik_leg`: unreachable or yaw-undefined → `ValueError`.
- xacro expand failure or `check_urdf` error → block migration; do not
  proceed to step 7+.

---

## 9. Acceptance criteria

- `colcon test --packages-select dog_robot_kinematics dog_robot_control`
  → all tests green.
- `xacro` expansion of `dog_robot.urdf.xacro` succeeds; `check_urdf`
  clean.
- RViz shows coherent quadruped (no flipped / rotated link parts) at
  θ=0 and at stand pose.
- `ros2 launch dog_robot_control stand.launch.py` — robot spawns,
  joints settle to symmetric stand within 3 s; body z > 0.10 m, drift
  < 0.15 m sustained 30 s, no parts fly apart.
- Final tree contains zero references to `kinematics_dh`, `DHParams`,
  `dh_params.yaml`, or `dh_link_placements.yaml`.

---

## 10. Risk

**Medium-low.** The convention is simpler than MDH — no common-normal
arithmetic — so derivation is straightforward.

Primary risk: the IK closed-form assumes `R_const_ht` is a pure Rx or Ry
rotation, so `q_yaw = atan2(y, x)` is exact. Current CAD measurements
indicate hip axis = URDF Z, thigh axis = URDF Y, knee axis = URDF Y →
`R_const_ht ≈ Rx(-π/2)` and the assumption holds. If `derive_joint_frames.py`
finds `R_const_ht` deviates (off-diagonal terms > 1e-3), fall back to
numerical IK via `scipy.optimize.fsolve` seeded with the closed-form
solution. Listed as a follow-up — not in initial scope.

FreeCAD export side is low risk — script reused unchanged except for
input path.

Blast radius is wide (5 controller files + 4 tests need import swaps),
but each is a mechanical rename; risk is in tests catching any
DHParams-specific assumption that doesn't carry over.

---

## 11. Open questions (resolved during brainstorming)

- ✅ Convention: joint-attached frame (origin at parent joint center,
  Z = parent joint axis, X → child joint center). Not MDH, not standard
  DH.
- ✅ `base_link`: standard URDF (geometric body center, Z up, X forward).
- ✅ `*_foot_link`: same as base — origin at foot center, Z up, X forward.
- ✅ Approach: re-derivation + re-export + delete `kinematics_dh` entirely
  (Cách 1).
- ✅ CAD source: `robotdogassem.STEP` via FreeCAD MCP (unchanged).
- ✅ Collision meshes: out of scope this iteration.
