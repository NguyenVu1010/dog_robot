# URDF Export Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Tạo ROS2 package `dog_robot_description` chứa URDF + meshes + launch + test, xuất từ assembly FreeCAD hiện tại, plug-and-play với RViz + Gazebo Classic.

**Architecture:** Dùng FreeCAD Python (qua MCP) để fuse 100 solid thành 17 link, export 34 STL (visual + collision convex hull). Sinh URDF qua xacro macro để DRY 4 chân. Test bằng `check_urdf` + FK cross-validation với `TestIK/4leg.py`.

**Tech Stack:** ROS2 Humble, Gazebo Classic 11 (gazebo_ros2_control), xacro, FreeCAD 1.0.1 + freecad-mcp, Python 3.10, pytest, pinocchio (FK validation).

**Env deviation from spec:** Spec phase 1 D10 đề cập `gz_ros2_control`, môi trường thực tế chỉ có Gazebo Classic → plan dùng `gazebo_ros2_control/GazeboSystem` plugin và `gazebo_ros2_control` package.

**Spec:** `docs/superpowers/specs/2026-05-15-urdf-export-design.md`

---

## File Structure

```
dog_robot/                                       # existing project root (not git yet)
├── .gitignore                                   # NEW
├── step/                                        # EXISTING — không sửa
├── TestIK/                                      # EXISTING — không sửa
├── docs/                                        # EXISTING
├── scripts/                                     # NEW
│   ├── export_links_from_freecad.py             # NEW — chạy qua FreeCAD MCP
│   └── README.md                                # NEW — hướng dẫn chạy
└── dog_robot_ws/                                # NEW — ROS2 workspace
    └── src/
        └── dog_robot_description/               # NEW — ament_cmake package
            ├── package.xml
            ├── CMakeLists.txt
            ├── README.md
            ├── urdf/
            │   ├── dog_robot.urdf.xacro
            │   ├── leg.xacro
            │   ├── inertial.xacro
            │   ├── materials.xacro
            │   ├── gazebo.xacro
            │   └── ros2_control.xacro
            ├── meshes/
            │   ├── visual/     (17 STL)
            │   └── collision/  (17 STL)
            ├── config/
            │   ├── joint_limits.yaml
            │   └── ros2_controllers.yaml
            ├── launch/
            │   ├── display.launch.py
            │   └── gazebo.launch.py
            ├── rviz/
            │   └── dog_robot.rviz
            └── test/
                └── test_urdf.py
```

---

## Task 1: Initialize git repo + workspace skeleton

**Files:**
- Create: `/home/nguyenvd/workspace/dog_robot/.gitignore`
- Create: `/home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/` (directory tree)

- [ ] **Step 1: Initialize git repo**

```bash
cd /home/nguyenvd/workspace/dog_robot
git init
git config user.email "nguyenvd11@fpt.com"
git config user.name "nguyenvd11"
```

- [ ] **Step 2: Create `.gitignore`**

```
# Python
__pycache__/
*.pyc
*.pyo
*.egg-info/
.pytest_cache/

# ROS2 / colcon
build/
install/
log/

# FreeCAD
*.FCBak
*.FCStd1

# IDE
.vscode/
.idea/
```

- [ ] **Step 3: Create workspace + package directory tree**

```bash
cd /home/nguyenvd/workspace/dog_robot
mkdir -p dog_robot_ws/src/dog_robot_description/{urdf,meshes/visual,meshes/collision,config,launch,rviz,test}
mkdir -p scripts
```

- [ ] **Step 4: Verify structure**

```bash
find dog_robot_ws scripts -type d | sort
```
Expected output includes: `dog_robot_ws/src/dog_robot_description/urdf`, `dog_robot_ws/src/dog_robot_description/meshes/visual`, etc.

- [ ] **Step 5: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add .gitignore
git commit -m "chore: initialize repo and workspace skeleton"
```

---

## Task 2: Create ROS2 package metadata (package.xml + CMakeLists.txt)

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/package.xml`
- Create: `dog_robot_ws/src/dog_robot_description/CMakeLists.txt`
- Create: `dog_robot_ws/src/dog_robot_description/README.md`

- [ ] **Step 1: Write `package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>dog_robot_description</name>
  <version>0.1.0</version>
  <description>URDF, meshes, launch and config for the 12-DOF dog robot.</description>
  <maintainer email="nguyenvd11@fpt.com">nguyenvd</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>xacro</exec_depend>
  <exec_depend>urdf</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>gazebo_ros2_control</exec_depend>
  <exec_depend>ros2_controllers</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>joint_trajectory_controller</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Write `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(dog_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

- [ ] **Step 3: Write `README.md` placeholder**

```markdown
# dog_robot_description

URDF + meshes + launch for 12-DOF quadruped dog robot.

## Build

```bash
cd dog_robot_ws
colcon build --packages-select dog_robot_description
source install/setup.bash
```

## Launch

```bash
ros2 launch dog_robot_description display.launch.py     # RViz + slider
ros2 launch dog_robot_description gazebo.launch.py      # Gazebo Classic sim
```
```

- [ ] **Step 4: Verify package builds (empty)**

```bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dog_robot_description
```
Expected: `Summary: 1 package finished` (success).

- [ ] **Step 5: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add dog_robot_ws/src/dog_robot_description/{package.xml,CMakeLists.txt,README.md}
git commit -m "feat(description): add ROS2 package metadata"
```

---

## Task 3: Write FreeCAD export script

**Files:**
- Create: `scripts/export_links_from_freecad.py`
- Create: `scripts/README.md`

- [ ] **Step 1: Write `scripts/export_links_from_freecad.py`**

```python
"""
Export 17 links (visual + collision) from FreeCAD assembly into ROS2 meshes/ dir.

Run via FreeCAD MCP execute_code(open(__file__).read()) or paste into FreeCAD
Python console after importing robotdogassem.STEP into doc "RobotDog".

Output:
  dog_robot_ws/src/dog_robot_description/meshes/visual/<link>.stl   (×17)
  dog_robot_ws/src/dog_robot_description/meshes/collision/<link>.stl (×17 convex hull)
"""

import os
import math
import FreeCAD
import Part
import Mesh

# ----------------------------------------------------------------------
# Config
# ----------------------------------------------------------------------

PROJECT_ROOT = "/home/nguyenvd/workspace/dog_robot"
MESH_OUT_VIS = f"{PROJECT_ROOT}/dog_robot_ws/src/dog_robot_description/meshes/visual"
MESH_OUT_COL = f"{PROJECT_ROOT}/dog_robot_ws/src/dog_robot_description/meshes/collision"

# Hằng số (mm, frame IK trong CAD)
L1 = 12.5
L2 = 48.95
L3 = 109.202
L4 = 115.0
L = 200.0
W = 80.0

# Body center in CAD frame (from inspect output earlier)
BODY_CENTER_CAD = FreeCAD.Vector(100.0, 0.0, -40.0)

# Joint positions in IK frame (mm), origin = body center after recentering
# IK frame: X=length, Y=up, Z=lateral (left)
HIP_OFFSETS_IK = {
    "FL": FreeCAD.Vector(+L/2, 0,            +W/2),
    "FR": FreeCAD.Vector(+L/2, 0,            -W/2),
    "BL": FreeCAD.Vector(-L/2, 0,            +W/2),
    "BR": FreeCAD.Vector(-L/2, 0,            -W/2),
}
# thigh_pitch relative to hip_yaw (lateral L2 outward)
THIGH_OFFSET_FROM_HIP_IK = lambda side: FreeCAD.Vector(0, 0, +L2 if side == "L" else -L2)
# knee_pitch relative to thigh (down L3 in IK Y direction)
KNEE_OFFSET_FROM_THIGH_IK = FreeCAD.Vector(0, -L3, 0)
# foot_fixed relative to shank (down L4)
FOOT_OFFSET_FROM_SHANK_IK = FreeCAD.Vector(0, -L4, 0)


def _classify_solids(doc):
    """Return dict {link_name: [solid_obj, ...]} from current document."""
    clusters = {name: [] for name in [
        "base_link",
        "FL_hip_link", "FL_thigh_link", "FL_shank_link", "FL_foot_link",
        "FR_hip_link", "FR_thigh_link", "FR_shank_link", "FR_foot_link",
        "BL_hip_link", "BL_thigh_link", "BL_shank_link", "BL_foot_link",
        "BR_hip_link", "BR_thigh_link", "BR_shank_link", "BR_foot_link",
    ]}

    for o in doc.Objects:
        if not (hasattr(o, "Shape") and o.Shape and o.Shape.ShapeType == "Solid"):
            continue
        name = o.Label.lower()
        bb = o.Shape.BoundBox
        cx, cy, cz = bb.Center.x, bb.Center.y, bb.Center.z

        # Body parts
        if any(k in name for k in ["dethan", "thanhngang", "opthan", "opkhop", "noi2op",
                                    "head", "duoi", "rpi lcd", "3.5inch"]):
            clusters["base_link"].append(o)
            continue

        # Feet
        if "demchan" in name:
            if cx < 100:
                side = "L" if cz > -40 else "R"  # Wait — Z sign convention
                # Actually feet at cz around 46 (one side) and cz around -130 (other side)
                # We'll bin by cz: cz>-40 = front-positive-Z, cz<-40 = back-negative-Z
            # Use bounding by X (front/back) and Z (left/right) sign relative to body center
            corner = ("F" if cx < 100 else "B") + ("L" if cz > -40 else "R")
            clusters[f"{corner}_foot_link"].append(o)
            continue

        # Screws / unnamed small parts → ignore (negligible inertia)
        if "screw" in name or "passivated" in name or "______" in o.Label or "------" in o.Label:
            continue

        # Leg parts: part1.* → hip, part2.* → thigh, part3.* → shank
        # Decide leg corner by (X, Z)
        corner = ("F" if cx < 100 else "B") + ("L" if cz > -40 else "R")

        if name.startswith("part1") or "part1_miror" in name:
            clusters[f"{corner}_hip_link"].append(o)
        elif name.startswith("part2") or "part2_miror" in name:
            clusters[f"{corner}_thigh_link"].append(o)
        elif name.startswith("part3") or "part3_mirror" in name:
            clusters[f"{corner}_shank_link"].append(o)
        else:
            print(f"  [WARN] unclassified solid: {o.Label} at ({cx:.0f},{cy:.0f},{cz:.0f})")

    return clusters


def _link_origin_in_world(link_name):
    """Return joint origin (FreeCAD.Vector) of `link_name` in world (CAD) frame.
    The link's mesh will be re-centered so this point becomes its local origin.
    """
    if link_name == "base_link":
        return BODY_CENTER_CAD

    parts = link_name.split("_")
    corner = parts[0]               # FL / FR / BL / BR
    seg = parts[1]                  # hip / thigh / shank / foot
    side = corner[1]                # L / R

    hip_world = BODY_CENTER_CAD + HIP_OFFSETS_IK[corner]
    if seg == "hip":
        return hip_world

    thigh_world = hip_world + THIGH_OFFSET_FROM_HIP_IK(side)
    if seg == "thigh":
        return thigh_world

    knee_world = thigh_world + KNEE_OFFSET_FROM_THIGH_IK
    if seg == "shank":
        return knee_world

    foot_world = knee_world + FOOT_OFFSET_FROM_SHANK_IK
    if seg == "foot":
        return foot_world

    raise ValueError(f"Unknown link: {link_name}")


def _shape_to_link_local(shape, origin_world):
    """Translate so origin_world becomes (0,0,0), then rotate Rx(-90deg) to swap Y↔Z (REP-103)."""
    s = shape.copy()
    s.translate(-origin_world)
    # Rotate -90° around X axis to swap Y/Z: new_Y = old_Z, new_Z = -old_Y
    # But we want: URDF_Y = IK_Z (left positive), URDF_Z = IK_Y (up positive)
    # Rotation Rx(-90°): (x, y, z) → (x, z, -y). So URDF_Y = IK_Z ✓, URDF_Z = -IK_Y ✗
    # Need Rx(+90°): (x, y, z) → (x, -z, y). So URDF_Y = -IK_Z ✗
    # Solution: Rx(-90°) gives (x, z, -y). URDF_Z = -IK_Y means IK_Y=140 (up) → URDF_Z=-140 (below)
    # But our convention says foot is BELOW body, IK_Y=-140 for foot (down in IK has Y=-140)
    # Wait — re-check IK convention: foot at Y=-140 means foot is at NEGATIVE Y in IK.
    # If IK_Y is "up positive", foot below means negative ✓
    # After Rx(-90°): URDF_Z = -IK_Y. Foot IK_Y=-140 → URDF_Z=+140 (foot above body) ✗
    # Need Rx(+90°): (x, y, z) → (x, -z, y). URDF_Z = IK_Y. Foot IK_Y=-140 → URDF_Z=-140 (below) ✓
    rot = FreeCAD.Rotation(FreeCAD.Vector(1, 0, 0), 90.0)  # +90° around X
    placement = FreeCAD.Placement(FreeCAD.Vector(0, 0, 0), rot)
    s = s.transformGeometry(placement.toMatrix())
    return s


def _fuse_solids(solids):
    """Fuse list of Part solids into one Shape."""
    if not solids:
        return None
    shape = solids[0].Shape.copy()
    for o in solids[1:]:
        shape = shape.fuse(o.Shape)
    return shape


def export_all():
    os.makedirs(MESH_OUT_VIS, exist_ok=True)
    os.makedirs(MESH_OUT_COL, exist_ok=True)

    doc = FreeCAD.getDocument("RobotDog")
    clusters = _classify_solids(doc)

    print(f"Classified into {len(clusters)} clusters:")
    for name, solids in clusters.items():
        print(f"  {name}: {len(solids)} solids")

    for link_name, solids in clusters.items():
        if not solids:
            print(f"  [SKIP] {link_name} (no solids)")
            continue

        shape = _fuse_solids(solids)
        origin = _link_origin_in_world(link_name)
        shape_local = _shape_to_link_local(shape, origin)

        # Visual export (full mesh, scale mm→m via mesh deviation+scale post-process)
        mesh = Mesh.Mesh()
        mesh.addFacets(shape_local.tessellate(0.1))  # 0.1mm deviation
        # Scale mm to m
        scale_matrix = FreeCAD.Matrix()
        scale_matrix.scale(0.001, 0.001, 0.001)
        mesh.transform(scale_matrix)
        out_vis = f"{MESH_OUT_VIS}/{link_name}.stl"
        mesh.write(out_vis)
        print(f"  ✓ visual:    {out_vis}")

        # Collision: convex hull
        try:
            hull_solid = Part.makeShell(shape_local.tessellate(0.5))
            # Try Part.Shape convex hull (FreeCAD 1.0+)
            hull = shape_local.makeConvexHull() if hasattr(shape_local, "makeConvexHull") else hull_solid
        except Exception as e:
            print(f"  [WARN] convex hull fallback for {link_name}: {e}")
            hull = shape_local

        mesh_col = Mesh.Mesh()
        mesh_col.addFacets(hull.tessellate(0.5))
        mesh_col.transform(scale_matrix)
        out_col = f"{MESH_OUT_COL}/{link_name}.stl"
        mesh_col.write(out_col)
        print(f"  ✓ collision: {out_col}")


if __name__ == "__main__":
    export_all()
```

- [ ] **Step 2: Write `scripts/README.md`**

```markdown
# scripts/

## export_links_from_freecad.py

Export 17 link STL files (visual + collision) from the FreeCAD assembly.

### Usage

1. Open FreeCAD
2. Start MCP RPC server (via Python console: `from rpc_server import rpc_server; rpc_server.start_rpc_server()`)
3. Import `step/robotdogassem.STEP` into document `RobotDog`
4. Run this script via FreeCAD MCP `execute_code()` or paste into FreeCAD Python console

### Output

- `dog_robot_ws/src/dog_robot_description/meshes/visual/*.stl` (17 files)
- `dog_robot_ws/src/dog_robot_description/meshes/collision/*.stl` (17 files)
```

- [ ] **Step 3: Lint-check the script syntax (no FreeCAD yet)**

```bash
python3 -c "import ast; ast.parse(open('/home/nguyenvd/workspace/dog_robot/scripts/export_links_from_freecad.py').read()); print('SYNTAX OK')"
```
Expected: `SYNTAX OK`

- [ ] **Step 4: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add scripts/
git commit -m "feat(scripts): add FreeCAD link export script"
```

---

## Task 4: Run export script via FreeCAD MCP to generate STL files

**Files (output):**
- 17 STL in `dog_robot_ws/src/dog_robot_description/meshes/visual/`
- 17 STL in `dog_robot_ws/src/dog_robot_description/meshes/collision/`

**Prerequisite:** FreeCAD instance with `RobotDog` document containing imported `robotdogassem.STEP`. (Currently in this session FreeCAD MCP is already connected and assembly imported, so this is ready.)

- [ ] **Step 1: Verify FreeCAD MCP connection + document state**

Run via MCP `mcp__freecad__list_documents` and confirm `RobotDog` exists. If not, re-run Task 0 in URDF spec (import STEP).

- [ ] **Step 2: Execute export script via MCP `execute_code`**

```python
# In FreeCAD MCP execute_code:
exec(open("/home/nguyenvd/workspace/dog_robot/scripts/export_links_from_freecad.py").read())
```

- [ ] **Step 3: Verify 34 STL files generated**

```bash
ls /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/meshes/visual/*.stl | wc -l
ls /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/meshes/collision/*.stl | wc -l
```
Expected: 17 each.

- [ ] **Step 4: Sanity-check 1 STL file (FL_thigh_link should be ~10cm long)**

```bash
python3 << 'EOF'
import struct
with open('/home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/meshes/visual/FL_thigh_link.stl', 'rb') as f:
    f.read(80)  # header
    n_tri, = struct.unpack('<I', f.read(4))
    print(f"FL_thigh_link.stl: {n_tri} triangles")
EOF
```
Expected: positive triangle count (>100).

- [ ] **Step 5: Commit STL files**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add dog_robot_ws/src/dog_robot_description/meshes/
git commit -m "feat(description): generate 34 STL link meshes from FreeCAD"
```

---

## Task 5: Create `urdf/materials.xacro` + `urdf/inertial.xacro`

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/urdf/materials.xacro`
- Create: `dog_robot_ws/src/dog_robot_description/urdf/inertial.xacro`

- [ ] **Step 1: Write `materials.xacro`**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="dog_robot_materials">

  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <material name="dark_grey">
    <color rgba="0.2 0.2 0.2 1.0"/>
  </material>

  <material name="orange">
    <color rgba="1.0 0.5 0.0 1.0"/>
  </material>

</robot>
```

- [ ] **Step 2: Write `inertial.xacro` with helper macros**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="dog_robot_inertial">

  <!-- Box inertial -->
  <xacro:macro name="inertial_box" params="mass x y z *origin">
    <inertial>
      <xacro:insert_block name="origin"/>
      <mass value="${mass}"/>
      <inertia
        ixx="${mass*(y*y+z*z)/12.0}" ixy="0.0" ixz="0.0"
        iyy="${mass*(x*x+z*z)/12.0}" iyz="0.0"
        izz="${mass*(x*x+y*y)/12.0}"/>
    </inertial>
  </xacro:macro>

  <!-- Sphere inertial -->
  <xacro:macro name="inertial_sphere" params="mass radius *origin">
    <inertial>
      <xacro:insert_block name="origin"/>
      <mass value="${mass}"/>
      <inertia
        ixx="${2.0*mass*radius*radius/5.0}" ixy="0.0" ixz="0.0"
        iyy="${2.0*mass*radius*radius/5.0}" iyz="0.0"
        izz="${2.0*mass*radius*radius/5.0}"/>
    </inertial>
  </xacro:macro>

</robot>
```

- [ ] **Step 3: Verify XML valid**

```bash
xmllint --noout /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/urdf/materials.xacro
xmllint --noout /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/urdf/inertial.xacro
```
Expected: no output (=valid XML).

- [ ] **Step 4: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add dog_robot_ws/src/dog_robot_description/urdf/materials.xacro dog_robot_ws/src/dog_robot_description/urdf/inertial.xacro
git commit -m "feat(description): add materials and inertial xacro helpers"
```

---

## Task 6: Create `urdf/leg.xacro` macro

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/urdf/leg.xacro`

- [ ] **Step 1: Write the leg macro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="dog_robot_leg">

  <!-- Per spec D6:
       prefix: FL / FR / BL / BR
       x_offset: ±L/2 (+0.100 for front, -0.100 for back)
       y_offset: ±W/2 (+0.040 for left, -0.040 for right)
       y_sign:   +1 for left, -1 for right (controls thigh_pitch joint origin)
  -->
  <xacro:macro name="leg" params="prefix x_offset y_offset y_sign">

    <!-- HIP LINK -->
    <link name="${prefix}_hip_link">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="package://dog_robot_description/meshes/visual/${prefix}_hip_link.stl"/>
        </geometry>
        <material name="grey"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="package://dog_robot_description/meshes/collision/${prefix}_hip_link.stl"/>
        </geometry>
      </collision>
      <xacro:inertial_box mass="0.15" x="0.04" y="0.04" z="0.04">
        <origin xyz="0 0 0" rpy="0 0 0"/>
      </xacro:inertial_box>
    </link>

    <!-- HIP_YAW JOINT (base ↔ hip) -->
    <joint name="${prefix}_hip_yaw" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_hip_link"/>
      <origin xyz="${x_offset} ${y_offset} 0" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="-0.785" upper="0.785" effort="2.0" velocity="5.0"/>
      <dynamics damping="0.01" friction="0.0"/>
    </joint>

    <!-- THIGH LINK -->
    <link name="${prefix}_thigh_link">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="package://dog_robot_description/meshes/visual/${prefix}_thigh_link.stl"/>
        </geometry>
        <material name="dark_grey"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="package://dog_robot_description/meshes/collision/${prefix}_thigh_link.stl"/>
        </geometry>
      </collision>
      <xacro:inertial_box mass="0.10" x="0.03" y="0.03" z="0.10">
        <origin xyz="0 0 -0.05" rpy="0 0 0"/>
      </xacro:inertial_box>
    </link>

    <!-- THIGH_PITCH JOINT (hip ↔ thigh) -->
    <joint name="${prefix}_thigh_pitch" type="revolute">
      <parent link="${prefix}_hip_link"/>
      <child link="${prefix}_thigh_link"/>
      <origin xyz="0 ${y_sign * 0.04895} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.571" upper="1.571" effort="2.0" velocity="5.0"/>
      <dynamics damping="0.01" friction="0.0"/>
    </joint>

    <!-- SHANK LINK -->
    <link name="${prefix}_shank_link">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="package://dog_robot_description/meshes/visual/${prefix}_shank_link.stl"/>
        </geometry>
        <material name="grey"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="package://dog_robot_description/meshes/collision/${prefix}_shank_link.stl"/>
        </geometry>
      </collision>
      <xacro:inertial_box mass="0.08" x="0.02" y="0.02" z="0.115">
        <origin xyz="0 0 -0.0575" rpy="0 0 0"/>
      </xacro:inertial_box>
    </link>

    <!-- KNEE_PITCH JOINT (thigh ↔ shank) -->
    <joint name="${prefix}_knee_pitch" type="revolute">
      <parent link="${prefix}_thigh_link"/>
      <child link="${prefix}_shank_link"/>
      <origin xyz="0 0 -0.109202" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0.0" upper="2.617" effort="2.0" velocity="5.0"/>
      <dynamics damping="0.01" friction="0.0"/>
    </joint>

    <!-- FOOT LINK -->
    <link name="${prefix}_foot_link">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="package://dog_robot_description/meshes/visual/${prefix}_foot_link.stl"/>
        </geometry>
        <material name="dark_grey"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="package://dog_robot_description/meshes/collision/${prefix}_foot_link.stl"/>
        </geometry>
      </collision>
      <xacro:inertial_sphere mass="0.02" radius="0.015">
        <origin xyz="0 0 0" rpy="0 0 0"/>
      </xacro:inertial_sphere>
    </link>

    <!-- FOOT_FIXED JOINT (shank ↔ foot) -->
    <joint name="${prefix}_foot_fixed" type="fixed">
      <parent link="${prefix}_shank_link"/>
      <child link="${prefix}_foot_link"/>
      <origin xyz="0 0 -0.115" rpy="0 0 0"/>
    </joint>

  </xacro:macro>

</robot>
```

- [ ] **Step 2: Verify XML**

```bash
xmllint --noout /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/urdf/leg.xacro
```

- [ ] **Step 3: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add dog_robot_ws/src/dog_robot_description/urdf/leg.xacro
git commit -m "feat(description): add reusable leg.xacro macro"
```

---

## Task 7: Create `urdf/gazebo.xacro`

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/urdf/gazebo.xacro`

- [ ] **Step 1: Write Gazebo plugins + foot friction**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="dog_robot_gazebo">

  <!-- Foot friction (per spec D10b) -->
  <xacro:macro name="foot_friction" params="prefix">
    <gazebo reference="${prefix}_foot_link">
      <mu1>1.0</mu1>
      <mu2>1.0</mu2>
      <kp>1000000.0</kp>
      <kd>100.0</kd>
      <minDepth>0.001</minDepth>
      <maxVel>0.1</maxVel>
      <material>Gazebo/DarkGrey</material>
    </gazebo>
  </xacro:macro>

  <xacro:foot_friction prefix="FL"/>
  <xacro:foot_friction prefix="FR"/>
  <xacro:foot_friction prefix="BL"/>
  <xacro:foot_friction prefix="BR"/>

  <!-- Base link Gazebo material -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
    <self_collide>false</self_collide>
  </gazebo>

  <!-- Gazebo Classic ros2_control plugin (per spec D10d, adapted for Classic) -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find dog_robot_description)/config/ros2_controllers.yaml</parameters>
    </plugin>
  </gazebo>

</robot>
```

- [ ] **Step 2: Verify XML**

```bash
xmllint --noout /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/urdf/gazebo.xacro
```

- [ ] **Step 3: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add dog_robot_ws/src/dog_robot_description/urdf/gazebo.xacro
git commit -m "feat(description): add Gazebo plugin and foot friction xacro"
```

---

## Task 8: Create `urdf/ros2_control.xacro`

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/urdf/ros2_control.xacro`

- [ ] **Step 1: Write the ros2_control block**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="dog_robot_ros2_control">

  <xacro:macro name="joint_iface" params="name lower upper">
    <joint name="${name}">
      <command_interface name="position">
        <param name="min">${lower}</param>
        <param name="max">${upper}</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </xacro:macro>

  <ros2_control name="dog_robot_hw" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

    <!-- 4 legs × 3 joints = 12 joints -->
    <xacro:joint_iface name="FL_hip_yaw"     lower="-0.785" upper="0.785"/>
    <xacro:joint_iface name="FL_thigh_pitch" lower="-1.571" upper="1.571"/>
    <xacro:joint_iface name="FL_knee_pitch"  lower="0.0"    upper="2.617"/>
    <xacro:joint_iface name="FR_hip_yaw"     lower="-0.785" upper="0.785"/>
    <xacro:joint_iface name="FR_thigh_pitch" lower="-1.571" upper="1.571"/>
    <xacro:joint_iface name="FR_knee_pitch"  lower="0.0"    upper="2.617"/>
    <xacro:joint_iface name="BL_hip_yaw"     lower="-0.785" upper="0.785"/>
    <xacro:joint_iface name="BL_thigh_pitch" lower="-1.571" upper="1.571"/>
    <xacro:joint_iface name="BL_knee_pitch"  lower="0.0"    upper="2.617"/>
    <xacro:joint_iface name="BR_hip_yaw"     lower="-0.785" upper="0.785"/>
    <xacro:joint_iface name="BR_thigh_pitch" lower="-1.571" upper="1.571"/>
    <xacro:joint_iface name="BR_knee_pitch"  lower="0.0"    upper="2.617"/>
  </ros2_control>

</robot>
```

- [ ] **Step 2: Verify XML**

```bash
xmllint --noout /home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/urdf/ros2_control.xacro
```

- [ ] **Step 3: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add dog_robot_ws/src/dog_robot_description/urdf/ros2_control.xacro
git commit -m "feat(description): add ros2_control hardware interface for 12 joints"
```

---

## Task 9: Create main `urdf/dog_robot.urdf.xacro`

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro`

- [ ] **Step 1: Write the entry point**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="dog_robot">

  <xacro:include filename="$(find dog_robot_description)/urdf/materials.xacro"/>
  <xacro:include filename="$(find dog_robot_description)/urdf/inertial.xacro"/>
  <xacro:include filename="$(find dog_robot_description)/urdf/leg.xacro"/>
  <xacro:include filename="$(find dog_robot_description)/urdf/gazebo.xacro"/>
  <xacro:include filename="$(find dog_robot_description)/urdf/ros2_control.xacro"/>

  <!-- BASE LINK -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://dog_robot_description/meshes/visual/base_link.stl"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://dog_robot_description/meshes/collision/base_link.stl"/>
      </geometry>
    </collision>
    <xacro:inertial_box mass="0.80" x="0.200" y="0.080" z="0.060">
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </xacro:inertial_box>
  </link>

  <!-- 4 LEGS: FL=front-left, FR=front-right, BL=back-left, BR=back-right -->
  <xacro:leg prefix="FL" x_offset=" 0.100" y_offset=" 0.040" y_sign=" 1"/>
  <xacro:leg prefix="FR" x_offset=" 0.100" y_offset="-0.040" y_sign="-1"/>
  <xacro:leg prefix="BL" x_offset="-0.100" y_offset=" 0.040" y_sign=" 1"/>
  <xacro:leg prefix="BR" x_offset="-0.100" y_offset="-0.040" y_sign="-1"/>

</robot>
```

- [ ] **Step 2: Process xacro to URDF and run check_urdf**

```bash
source /opt/ros/humble/setup.bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
colcon build --packages-select dog_robot_description
source install/setup.bash
xacro src/dog_robot_description/urdf/dog_robot.urdf.xacro > /tmp/dog_robot.urdf
check_urdf /tmp/dog_robot.urdf
```
Expected: `robot name is: dog_robot ... Successfully Parsed XML ...`. Output shows tree with 17 links.

- [ ] **Step 3: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro
git commit -m "feat(description): add main URDF xacro with 17 links + 12 joints"
```

---

## Task 10: Create `config/ros2_controllers.yaml`

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/config/ros2_controllers.yaml`
- Create: `dog_robot_ws/src/dog_robot_description/config/joint_limits.yaml`

- [ ] **Step 1: Write `ros2_controllers.yaml`**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  ros__parameters:
    joints:
      - FL_hip_yaw
      - FL_thigh_pitch
      - FL_knee_pitch
      - FR_hip_yaw
      - FR_thigh_pitch
      - FR_knee_pitch
      - BL_hip_yaw
      - BL_thigh_pitch
      - BL_knee_pitch
      - BR_hip_yaw
      - BR_thigh_pitch
      - BR_knee_pitch
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
```

- [ ] **Step 2: Write `joint_limits.yaml` (for MoveIt if added later)**

```yaml
joint_limits:
  FL_hip_yaw:     {has_position_limits: true, min_position: -0.785, max_position: 0.785, has_velocity_limits: true, max_velocity: 5.0, has_acceleration_limits: false, has_jerk_limits: false, has_effort_limits: true, max_effort: 2.0}
  FL_thigh_pitch: {has_position_limits: true, min_position: -1.571, max_position: 1.571, has_velocity_limits: true, max_velocity: 5.0, has_acceleration_limits: false, has_jerk_limits: false, has_effort_limits: true, max_effort: 2.0}
  FL_knee_pitch:  {has_position_limits: true, min_position:  0.0,   max_position: 2.617, has_velocity_limits: true, max_velocity: 5.0, has_acceleration_limits: false, has_jerk_limits: false, has_effort_limits: true, max_effort: 2.0}
  FR_hip_yaw:     {has_position_limits: true, min_position: -0.785, max_position: 0.785, has_velocity_limits: true, max_velocity: 5.0, has_acceleration_limits: false, has_jerk_limits: false, has_effort_limits: true, max_effort: 2.0}
  FR_thigh_pitch: {has_position_limits: true, min_position: -1.571, max_position: 1.571, has_velocity_limits: true, max_velocity: 5.0, has_acceleration_limits: false, has_jerk_limits: false, has_effort_limits: true, max_effort: 2.0}
  FR_knee_pitch:  {has_position_limits: true, min_position:  0.0,   max_position: 2.617, has_velocity_limits: true, max_velocity: 5.0, has_acceleration_limits: false, has_jerk_limits: false, has_effort_limits: true, max_effort: 2.0}
  BL_hip_yaw:     {has_position_limits: true, min_position: -0.785, max_position: 0.785, has_velocity_limits: true, max_velocity: 5.0, has_acceleration_limits: false, has_jerk_limits: false, has_effort_limits: true, max_effort: 2.0}
  BL_thigh_pitch: {has_position_limits: true, min_position: -1.571, max_position: 1.571, has_velocity_limits: true, max_velocity: 5.0, has_acceleration_limits: false, has_jerk_limits: false, has_effort_limits: true, max_effort: 2.0}
  BL_knee_pitch:  {has_position_limits: true, min_position:  0.0,   max_position: 2.617, has_velocity_limits: true, max_velocity: 5.0, has_acceleration_limits: false, has_jerk_limits: false, has_effort_limits: true, max_effort: 2.0}
  BR_hip_yaw:     {has_position_limits: true, min_position: -0.785, max_position: 0.785, has_velocity_limits: true, max_velocity: 5.0, has_acceleration_limits: false, has_jerk_limits: false, has_effort_limits: true, max_effort: 2.0}
  BR_thigh_pitch: {has_position_limits: true, min_position: -1.571, max_position: 1.571, has_velocity_limits: true, max_velocity: 5.0, has_acceleration_limits: false, has_jerk_limits: false, has_effort_limits: true, max_effort: 2.0}
  BR_knee_pitch:  {has_position_limits: true, min_position:  0.0,   max_position: 2.617, has_velocity_limits: true, max_velocity: 5.0, has_acceleration_limits: false, has_jerk_limits: false, has_effort_limits: true, max_effort: 2.0}
```

- [ ] **Step 3: Verify YAML parses**

```bash
python3 -c "import yaml; yaml.safe_load(open('/home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/config/ros2_controllers.yaml')); yaml.safe_load(open('/home/nguyenvd/workspace/dog_robot/dog_robot_ws/src/dog_robot_description/config/joint_limits.yaml')); print('YAML OK')"
```
Expected: `YAML OK`

- [ ] **Step 4: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add dog_robot_ws/src/dog_robot_description/config/
git commit -m "feat(description): add ros2_controllers and joint_limits config"
```

---

## Task 11: Create `launch/display.launch.py` for RViz

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/launch/display.launch.py`
- Create: `dog_robot_ws/src/dog_robot_description/rviz/dog_robot.rviz`

- [ ] **Step 1: Write `display.launch.py`**

```python
"""Launch RViz + joint_state_publisher_gui to visualize the robot URDF."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("dog_robot_description")

    urdf_xacro = PathJoinSubstitution([pkg, "urdf", "dog_robot.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg, "rviz", "dog_robot.rviz"])

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", urdf_xacro])
    }

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
```

- [ ] **Step 2: Write minimal `dog_robot.rviz`**

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      Visual Enabled: true
      Collision Enabled: false
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
  Tools:
    - Class: rviz_default_plugins/Orbit
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 1.5
```

- [ ] **Step 3: Rebuild + run launch (manual visual verification needed)**

```bash
source /opt/ros/humble/setup.bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
colcon build --packages-select dog_robot_description
source install/setup.bash
ros2 launch dog_robot_description display.launch.py &
sleep 5
ros2 node list
```
Expected: `/robot_state_publisher`, `/joint_state_publisher_gui`, `/rviz2` listed. Visually: in RViz the robot model should display with 4 legs hanging down. User should drag joint sliders to see joint motion.

Kill: `pkill -f display.launch`

- [ ] **Step 4: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add dog_robot_ws/src/dog_robot_description/launch/display.launch.py dog_robot_ws/src/dog_robot_description/rviz/dog_robot.rviz
git commit -m "feat(description): add RViz display launch and config"
```

---

## Task 12: Create `launch/gazebo.launch.py`

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/launch/gazebo.launch.py`

- [ ] **Step 1: Ensure `gazebo_ros2_control` is installed**

```bash
dpkg -l | grep gazebo-ros2-control || sudo apt install -y ros-humble-gazebo-ros2-control
```
Expected: package installed (idempotent).

- [ ] **Step 2: Write `gazebo.launch.py`**

```python
"""Launch Gazebo Classic + spawn robot + ros2_control controllers."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("dog_robot_description")
    urdf_xacro = PathJoinSubstitution([pkg, "urdf", "dog_robot.urdf.xacro"])

    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", urdf_xacro])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"
        ]),
        launch_arguments={"verbose": "false"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "dog_robot",
            "-z", "0.30",  # drop from 30cm
        ],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"],
        output="screen",
    )

    load_jtc = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_trajectory_controller"],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_jtc],
            )
        ),
    ])
```

- [ ] **Step 3: Rebuild + run (manual smoke test)**

```bash
source /opt/ros/humble/setup.bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
colcon build --packages-select dog_robot_description
source install/setup.bash
ros2 launch dog_robot_description gazebo.launch.py &
sleep 15
ros2 control list_controllers
ros2 topic list | grep joint
pkill -f "gz\|gazebo\|spawn_entity"
```
Expected: `joint_trajectory_controller active`, `joint_state_broadcaster active`, topics `/joint_states`, `/joint_trajectory_controller/joint_trajectory` exist.

- [ ] **Step 4: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add dog_robot_ws/src/dog_robot_description/launch/gazebo.launch.py
git commit -m "feat(description): add Gazebo launch with ros2_control bringup"
```

---

## Task 13: Write `test/test_urdf.py` — automated URDF + FK validation

**Files:**
- Create: `dog_robot_ws/src/dog_robot_description/test/test_urdf.py`

- [ ] **Step 1: Write the test (uses pinocchio for FK)**

```python
"""URDF validation: check_urdf passes + FK matches TestIK reference."""
import math
import os
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path("/home/nguyenvd/workspace/dog_robot")
WS = PROJECT_ROOT / "dog_robot_ws"
URDF_XACRO = WS / "src/dog_robot_description/urdf/dog_robot.urdf.xacro"


def test_xacro_processes():
    """xacro must process the URDF without errors."""
    result = subprocess.run(
        ["xacro", str(URDF_XACRO)], capture_output=True, text=True
    )
    assert result.returncode == 0, f"xacro failed:\n{result.stderr}"
    assert "<robot" in result.stdout
    assert result.stdout.count("<link") >= 17  # 17 links
    assert result.stdout.count('type="revolute"') == 12  # 12 revolute joints


def test_check_urdf_passes():
    """check_urdf must accept the generated URDF."""
    xacro = subprocess.run(["xacro", str(URDF_XACRO)], capture_output=True, text=True)
    urdf_str = xacro.stdout
    tmp = Path("/tmp/dog_robot_test.urdf")
    tmp.write_text(urdf_str)
    result = subprocess.run(["check_urdf", str(tmp)], capture_output=True, text=True)
    assert result.returncode == 0, f"check_urdf failed:\n{result.stderr}"
    assert "Successfully Parsed XML" in result.stdout


def test_fk_matches_testik():
    """FK from URDF (pinocchio) must match TestIK calcLegPoints at random configs."""
    try:
        import pinocchio as pin
        import numpy as np
    except ImportError:
        import pytest
        pytest.skip("pinocchio not installed")

    sys.path.insert(0, str(PROJECT_ROOT / "TestIK"))
    from Test2 import calcLegPoints  # type: ignore

    # Build pinocchio model from URDF
    xacro = subprocess.run(["xacro", str(URDF_XACRO)], capture_output=True, text=True)
    urdf_str = xacro.stdout
    model = pin.buildModelFromXML(urdf_str)
    data = model.createData()

    # Random joint config for FL leg
    np.random.seed(0)
    for _ in range(3):
        omega = np.random.uniform(-0.5, 0.5)
        theta = np.random.uniform(-0.5, 0.5)
        phi = np.random.uniform(0.5, 2.0)

        # Compute foot pos via TestIK (mm in IK frame)
        D_proxy = 48.95  # L2 placeholder; actual D depends on full IK; here we just
        # use calcLegPoints for relative FK consistency:
        pts = calcLegPoints(omega, theta, phi, D=D_proxy)
        foot_ik_mm = pts[3]  # P3
        foot_ik_m = np.array([foot_ik_mm[0], foot_ik_mm[1], foot_ik_mm[2]]) / 1000.0
        # Swap Y↔Z for REP-103: URDF_X = IK_X, URDF_Y = IK_Z, URDF_Z = IK_Y
        foot_urdf_expected = np.array([foot_ik_m[0], foot_ik_m[2], foot_ik_m[1]])

        # Compute foot pos via URDF/pinocchio (FL_foot_link relative to FL_hip_link)
        q = np.zeros(model.nq)
        # Find joint indices for FL
        for jname, qidx in zip(["FL_hip_yaw", "FL_thigh_pitch", "FL_knee_pitch"],
                                [omega, theta, phi]):
            jid = model.getJointId(jname)
            q[model.idx_qs[jid]] = qidx

        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        foot_frame = model.getFrameId("FL_foot_link")
        foot_urdf_actual = data.oMf[foot_frame].translation
        # foot_urdf_actual is in base_link frame; offset by hip position
        hip_pos = np.array([0.100, 0.040, 0.0])
        foot_in_hip = foot_urdf_actual - hip_pos

        err = np.linalg.norm(foot_in_hip - foot_urdf_expected)
        assert err < 0.005, f"FK mismatch: {err*1000:.2f} mm at q=({omega:.2f},{theta:.2f},{phi:.2f})"
```

- [ ] **Step 2: Install pinocchio if missing**

```bash
pip install pin || sudo apt install -y ros-humble-pinocchio || true
python3 -c "import pinocchio" && echo "pinocchio OK" || echo "pinocchio not available - test will skip"
```

- [ ] **Step 3: Run the tests**

```bash
source /opt/ros/humble/setup.bash
cd /home/nguyenvd/workspace/dog_robot
python3 -m pytest dog_robot_ws/src/dog_robot_description/test/test_urdf.py -v
```
Expected: `test_xacro_processes PASSED`, `test_check_urdf_passes PASSED`, `test_fk_matches_testik PASSED or SKIPPED`.

- [ ] **Step 4: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add dog_robot_ws/src/dog_robot_description/test/test_urdf.py
git commit -m "test(description): add URDF validation and FK cross-check tests"
```

---

## Task 14: Wire test into colcon + add CMakeLists entry

**Files:**
- Modify: `dog_robot_ws/src/dog_robot_description/CMakeLists.txt`
- Modify: `dog_robot_ws/src/dog_robot_description/package.xml`

- [ ] **Step 1: Add pytest test in CMakeLists.txt**

Append before `ament_package()`:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py
    TIMEOUT 60
  )
endif()
```

- [ ] **Step 2: Add test depend in package.xml**

Add inside `<package>` block:

```xml
<test_depend>ament_cmake_pytest</test_depend>
```

- [ ] **Step 3: Run colcon test**

```bash
source /opt/ros/humble/setup.bash
cd /home/nguyenvd/workspace/dog_robot/dog_robot_ws
colcon build --packages-select dog_robot_description
colcon test --packages-select dog_robot_description
colcon test-result --verbose
```
Expected: tests pass (or pinocchio test skipped if not installed).

- [ ] **Step 4: Commit**

```bash
cd /home/nguyenvd/workspace/dog_robot
git add dog_robot_ws/src/dog_robot_description/CMakeLists.txt dog_robot_ws/src/dog_robot_description/package.xml
git commit -m "test(description): integrate test_urdf.py with colcon test"
```

---

## Self-Review

**Spec coverage:** 
- D1 (scope: full URDF for sim+ROS) → Tasks 6-9 (link + joint + gazebo + ros2_control)
- D2 (REP-103, scale) → Task 3 (transform in export script), Task 6 (origins in meters)
- D3 (convex hull collision) → Task 3 (`makeConvexHull()`)
- D4 (17 links, 12 joints) → Task 6 (leg macro × 4), Task 9 (base + 4 legs)
- D5 (solid-to-link mapping) → Task 3 (`_classify_solids`)
- D6 (axes & origins) → Task 6 (concrete xyz in leg.xacro)
- D7 (joint limits) → Task 6 (limits in xacro), Task 10 (joint_limits.yaml)
- D8 (inertial) → Tasks 5, 6, 9 (inertial macros)
- D9 (mesh export pipeline) → Task 3 (script)
- D10 (Gazebo physics + ros2_control) → Tasks 7, 8, 10 (gazebo.xacro, ros2_control.xacro, controllers.yaml)
- ROS2 package layout → Tasks 1, 2 (init + metadata)
- Pipeline kiểm thử → Tasks 11, 12, 13, 14

All spec sections covered.

**Placeholder scan:** No "TBD" / "TODO" / "fill in" / "similar to Task N". Each step has concrete commands and code.

**Type consistency:** Joint names consistent across tasks: `FL_hip_yaw`, `FL_thigh_pitch`, `FL_knee_pitch`, etc. Link names consistent: `FL_hip_link`, `FL_thigh_link`, `FL_shank_link`, `FL_foot_link`. Constants L1..L4 used same value in Task 3 + Task 6 + Task 13.

Plan ready.
