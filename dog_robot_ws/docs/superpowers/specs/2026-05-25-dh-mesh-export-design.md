# DH-canonical mesh re-export — Design

**Date:** 2026-05-25
**Status:** Approved (sections 1–5)
**Scope:** Re-export every link STL from FreeCAD with its vertices already in
the link's Modified-DH (Craig) frame. Update the URDF and `kinematics_dh`
module to consume the clean DH parameters. Eliminates visual-compensation
math entirely.

---

## Goals

1. Each STL in `meshes/visual_dh/<link>.stl` is in its own link frame —
   URDF `<visual><origin xyz="0 0 0" rpy="0 0 0"/>` everywhere.
2. The URDF joint origins, the DH parameters, and the mesh frames are all
   derived from one source: measured joint axis centers in CAD
   (`scripts/compute_joints.py`).
3. Future link-frame changes need only re-run the derivation + export
   scripts. No hand-edited compensation values anywhere.

## Non-goals

- Walker controller logic, gait engine, control packages.
- `dog_robot_kinematic_viz` rig (rebuilds to pick up new URDF, nothing else).
- CHAMP / stand controller (still deprecated).
- Re-measuring joint axes — uses existing `compute_joints.py` measurements.

---

## Architecture

```
                  ┌────────────────────────────────────────────┐
                  │ scripts/compute_joints.py                  │
                  │ → measured joint axis centers (CAD mm)     │
                  │   HIP, THIGH, KNEE per leg                 │
                  └─────────────────┬──────────────────────────┘
                                    │
                                    ▼
        ┌──────────────────────────────────────────────────────┐
        │ scripts/derive_dh_frames.py  (NEW)                   │
        │  In: measured joint axes (CAD mm) + URDF transform   │
        │ Out: per-link DH-canonical Placement (xyz, rpy) in   │
        │      CAD frame, ready for FreeCAD; plus clean        │
        │      (a_i, d_i, alpha_i) for kinematics_dh           │
        └─────────────────┬────────────────────────────────────┘
                          │
                          ▼
        ┌──────────────────────────────────────────────────────┐
        │ scripts/export_dh_links_from_freecad.py  (NEW)       │
        │  In FreeCAD via MCP:                                 │
        │   1. for each link: compose its solids               │
        │   2. apply inverse DH Placement in memory            │
        │   3. tessellate + write meshes/visual_dh/<link>.stl  │
        │  Does not modify the FreeCAD document.               │
        └─────────────────┬────────────────────────────────────┘
                          │
                          ▼
        ┌──────────────────────────────────────────────────────┐
        │ URDF update (leg.xacro, dog_robot.urdf.xacro)        │
        │  - new MDH params: L_hh, L_th, L_sh, alpha, d        │
        │  - joint xyz = (L_i, 0, d_i), rpy = (alpha_i, 0, 0)  │
        │  - visual <origin> = identity                        │
        │  - mesh path → meshes/visual_dh/                     │
        └─────────────────┬────────────────────────────────────┘
                          │
                          ▼
        ┌──────────────────────────────────────────────────────┐
        │ kinematics_dh.py update                              │
        │  - DHParams adds d_thigh, d_knee, d_foot             │
        │  - fk_leg uses non-zero d via existing mdh_transform │
        │  - ik_leg handles d_knee Y-offset (extra ~10 LOC)    │
        │  - URDF↔kinematics_dh consistency test extended      │
        └──────────────────────────────────────────────────────┘
```

---

## Components

### `scripts/derive_dh_frames.py`

Ground-truth input — joint axis centers (CAD mm), already in `compute_joints.py`:

| Joint | Axis direction in CAD | Notes |
|-------|----------------------|-------|
| HIP   | X                    | hip yaw rotates about CAD X |
| THIGH | Z                    | pitch about CAD Z |
| KNEE  | Z                    | pitch about CAD Z (parallel to thigh) |

Procedure per leg:

1. Convert axis positions CAD → URDF via the existing transform in
   `compute_joints.py`: `to_urdf(p, BODY_CENTER) = (origin[0] - p[0],
   p[2] - origin[2], p[1] - origin[1])`. Direction vectors transform with
   the same matrix (no translation).
2. Build the MDH chain frame by frame:
   - **base → frame 1 (hip)**: Z_0 chosen along URDF Z; Z_1 along URDF X
     (hip axis). Common normal along URDF Y. Derives `alpha_0 = +π/2`,
     `a_0` (Y offset), `d_1` (X offset on Z_1).
   - **frame 1 → frame 2 (thigh)**: Z_2 along URDF Y. Common normal along
     URDF Z. Derives `alpha_1 = -π/2`, `a_1` (Z gap), `d_2` (Y offset on Z_2).
   - **frame 2 → frame 3 (knee)**: Z_3 parallel to Z_2 (both URDF Y).
     `alpha_2 = 0`, `a_2 = L_th` (between thigh and knee Z lines),
     `d_3` (knee-line Y offset).
   - **frame 3 → foot tip**: `alpha_3 = 0`, `a_3 = L_sh`, `d_4` (foot Y offset).
3. Per-link Placement for FreeCAD export: each Body's CAD-frame xyz + rpy
   that aligns it with its DH link frame (origin at common-normal foot on
   Z_i, Z_i along joint axis, X_i along common normal).

Sanity checks:

- The four legs must yield identical MDH lengths within 1 mm.
- Reconstructing each joint center via FK from the derived params must match
  the input CAD measurement within 0.5 mm.
- Script prints both checks; fails fast with diagnostic if violated.

Outputs:

```
src/dog_robot_description/config/dh_params.yaml           (NEW)
src/dog_robot_description/config/dh_link_placements.yaml  (NEW, FreeCAD input)
```

### `scripts/export_dh_links_from_freecad.py`

Runs inside FreeCAD via MCP `execute_code()` against the doc containing
`robotdogassem.STEP` (doc name `RobotDog`).

Per link (17 total = 1 base + 4 legs × 4 links):

1. Read planned Placement from `dh_link_placements.yaml`.
2. Find the link's solids by cluster classification (same logic as
   `compute_joints.py`: bbox centroid bucketing).
3. `Part.Compound` the solids.
4. Apply `inverse(Placement)` to the compound — moves geometry into the DH
   link frame.
5. Tessellate at 0.05 mm tolerance.
6. Write `meshes/visual_dh/<link>.stl`.
7. Optional: write `meshes/collision_dh/<link>.stl` as convex hull.

Does **not** modify the FreeCAD document — purely read-transform-write.

### URDF update

`leg.xacro` macro keeps the same surface but accepts new MDH offset params:

```xml
<xacro:macro name="leg" params="prefix
                                base_to_hip_xyz base_to_hip_rpy
                                L_hh L_th L_sh
                                d_thigh:='0' d_knee:='0' d_foot:='0'
                                alpha_thigh:='-1.5707963'
                                foot_sphere_xyz:='0 0 0'">
```

Joint origins:

- hip_yaw:    `xyz="${base_to_hip_xyz}"` `rpy="${base_to_hip_rpy}"` `axis="0 0 1"`
- thigh_pitch: `xyz="${L_hh} 0 ${d_thigh}"` `rpy="${alpha_thigh} 0 0"` `axis="0 0 1"`
- knee_pitch: `xyz="${L_th} 0 ${d_knee}"` `rpy="0 0 0"` `axis="0 0 1"`
- foot_fixed: `xyz="${L_sh} 0 ${d_foot}"` `rpy="0 0 0"`

All `<visual><origin xyz="0 0 0" rpy="0 0 0"/>` pointing at
`meshes/visual_dh/<link>.stl`.

`dog_robot.urdf.xacro` reads all DH params from properties at top of file
(derived values from `dh_params.yaml`):

```xml
<xacro:property name="L_hh" value="<derived>"/>
<xacro:property name="L_th" value="<derived>"/>
<xacro:property name="L_sh" value="<derived>"/>
<xacro:property name="d_thigh" value="<derived>"/>
<xacro:property name="d_knee"  value="<derived>"/>
<xacro:property name="d_foot"  value="<derived>"/>
<xacro:property name="alpha_thigh" value="-1.5707963"/>
```

Per-leg `base_to_hip_xyz` / `base_to_hip_rpy` unchanged structurally.

### `kinematics_dh.py` update

Extend `DHParams`:

```python
@dataclass(frozen=True)
class DHParams:
    L_hh: float; L_th: float; L_sh: float
    d_thigh: float = 0.0
    d_knee:  float = 0.0
    d_foot:  float = 0.0
```

- `fk_leg`: already chains `mdh_transform(alpha, a, d, theta)`. Only change
  is passing the non-zero `d_*` values.
- `ik_leg`: closed-form 2R planar + hip yaw. With `d_knee ≠ 0` the planar 2R
  sits on a plane offset by `d_knee` from the hip-yaw axis. Implementation:
  - project foot target onto the offset plane,
  - solve 2R for thigh + knee,
  - solve hip yaw from the in-plane angle plus the lateral correction from
    `d_knee`.
  About 10 extra lines.

Fallback if closed-form proves harder than expected: numerical IK via
`scipy.optimize.fsolve` seeded with the closed-form solution. Listed as
risk; not in initial scope.

### Tests (`dog_robot_kinematics/test/`)

1. `test_kinematics_dh.py` extended: 200-iter FK/IK roundtrip with non-zero
   `d_thigh`, `d_knee` across all 4 legs.
2. `test_urdf_dh_consistency.py` extended: URDF chain FK matches
   `fk_leg(dh)` for all 4 legs at 40 random configurations.
3. `test_dh_derivation.py` (NEW): feed measured CAD joint centers into
   `derive_dh_frames.py`, assert all 4 legs yield same MDH params within
   1 mm.

---

## Data flow

```
compute_joints.py (measured CAD mm)
       │
       ▼
derive_dh_frames.py
       │
       ├── dh_params.yaml  ────┐
       │                       ▼
       │              dog_robot.urdf.xacro  ◄── leg.xacro (macro)
       │                                              │
       └── dh_link_placements.yaml                    │
                       │                              │
                       ▼                              │
              export_dh_links_from_freecad.py         │
                       │                              │
                       ▼                              │
            meshes/visual_dh/*.stl  ─────────────────►┘
                                              │
                                              ▼
                                     robot_state_publisher
                                              │
                                              ▼
                                            RViz
```

`walker_controller` (unchanged) reads DH params from `dh_params.yaml`,
calls `ik_leg(dh, foot_in_hip)`, publishes `JointState` / `JointTrajectory`.

---

## Migration steps

In order; each is independently revertable in git.

1. Run `derive_dh_frames.py` → produce `dh_params.yaml`,
   `dh_link_placements.yaml`, sanity-check report.
2. User starts FreeCAD with `robotdogassem.STEP` loaded, MCP server on
   port 9875.
3. Run `export_dh_links_from_freecad.py` → write `meshes/visual_dh/*.stl`
   and optional `meshes/collision_dh/*.stl`.
4. Update `kinematics_dh.py` (`d_*` fields + `ik_leg` offset handling).
5. Update `leg.xacro` + `dog_robot.urdf.xacro` (new MDH params, visual
   path → `visual_dh/`, identity visual origin).
6. `colcon test --packages-select dog_robot_kinematics` — must pass.
7. `ros2 launch dog_robot_kinematic_viz kinematic.launch.py` — RViz check.
8. `ros2 launch dog_robot_control walk.launch.py` — Gazebo regression.
9. Delete `meshes/visual/`, `meshes/collision/`,
   `compute_visual_compensation.py`, `bake_meshes_to_link_frame.py`.

---

## Error handling

- `derive_dh_frames.py` sanity-check fail (legs disagree > 1 mm) →
  print diff, abort. Indicates measurement inconsistency in
  `compute_joints.py`.
- FreeCAD MCP unreachable → `export_dh_links_from_freecad.py` aborts
  with "open FreeCAD with port 9875 first".
- Empty solid cluster for a link → log warn, skip; summary lists
  missing links.
- Tessellation yields 0 triangles → fail (sentinel for bad Placement
  math).
- Test regression in `test_kinematics_dh` → derivation likely wrong;
  don't ship URDF until green.

---

## Acceptance criteria

- `colcon test --packages-select dog_robot_kinematics` → all tests green
  (existing 11 + new derivation + extended roundtrip).
- `xacro` expansion of `dog_robot.urdf.xacro` succeeds; `check_urdf` clean.
- RViz shows coherent quadruped (no flipped / rotated link parts) at stand
  pose and θ=0.
- `cmd_vel{linear.x:0.1}` for 5 s — legs animate in trot pattern, no IK
  errors logged.
- Gazebo `walk.launch.py` — robot spawns, joints settle to symmetric stand
  within 3 s, no parts fly apart.

---

## Risk

**Medium.** The MDH derivation math (extracting clean `(a, d, alpha)` from
measured joint axes) is the main risk. If `d_thigh` / `d_knee` are small
(< 5 mm) the geometry is near-canonical DH and IK closed-form needs only
small corrections. If they're large, the IK math gets fiddlier; fall back
plan: numerical IK via `scipy.optimize.fsolve` seeded with the
closed-form solution. Listed as a follow-up if needed.

The FreeCAD export side is low risk — the script does a transform on
already-validated geometry; failure modes are obvious (tessellation, file
write).

---

## Open questions (resolved during brainstorming)

- ✅ CAD source: FreeCAD MCP (user will reopen with `robotdogassem.STEP`).
- ✅ Link frame convention: Modified DH (Craig) — Z along joint axis,
  X along common normal.
- ✅ Re-export scope: 17 visual STLs (4 legs × 4 links + base);
  collision STLs optional.
- ✅ URDF representation: keep existing `leg.xacro` macro shape; extend
  with `d_*` and `alpha_*` params.
- ✅ Fallback for non-canonical IK: numerical IK if closed-form proves
  intractable — follow-up, not in initial scope.
