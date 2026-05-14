# Robot Dog — URDF Export Design

**Date:** 2026-05-15
**Owner:** nguyenvd11@fpt.com
**Status:** Design (pending implementation)

## Mục tiêu

Xuất file URDF chuẩn ROS2 cho robot dog 12 DOF, dùng được cho cả mô phỏng (Gazebo / Isaac Sim / PyBullet) và stack ROS2 (RViz, robot_state_publisher, MoveIt, ros2_control). Nguồn dữ liệu: 20 file STEP trong `step/` và mô hình động học có sẵn trong `TestIK/`.

## Bối cảnh

- File `robotdogassem.STEP` đã import vào FreeCAD: 236 đối tượng, **100 solid**, phân thành body + 4 chân + head/tail + 4 đệm chân.
- Code động học có sẵn trong `TestIK/`:
  - `IK_leg.py`: IK 1 chân với 3 góc `(omega, theta, phi)`
  - `4leg.py`: IK toàn thân với 4 chân `LF/RF/LB/RB`, đầy đủ `bodyIK + legIK + world_to_leg`
  - `FK_body.py`: vẽ pose body bằng matplotlib
- Hằng số đã verify trong code IK (đơn vị mm):
  - `L1 = 12.5` — offset body → hip stand-off
  - `L2 = 48.95` — offset hip_yaw axis → thigh_pitch axis
  - `L3 = 109.202` — chiều dài đùi
  - `L4 = 115.0` — chiều dài cẳng chân
  - `L = 200` — khoảng cách front-back hip
  - `W = 80` — khoảng cách left-right hip
- Right side dùng `Ix` matrix đảo dấu X — quy ước IK đã xử lý mirror trái/phải.

## Quyết định thiết kế

### D1. Phạm vi: full URDF chuẩn (sim + ROS2)

URDF có đủ `visual`, `collision`, `inertial`, `joint_limits`, `<ros2_control>` block, gazebo plugin tags. Plug-and-play với MoveIt, ros2_control, navigation stack.

### D2. Hệ trục: REP-103

URDF dùng **mét** + **REP-103** (X forward, Y left, Z up).

Mapping từ frame của code IK (X forward, Y up, Z left) sang URDF: **swap Y ↔ Z**.

```
URDF_X = IK_X   (forward)
URDF_Y = IK_Z   (left)
URDF_Z = IK_Y   (up)
```

Scale: chia 1000 (mm → m). Sau scale:
```
L1 = 0.0125 m
L2 = 0.04895 m
L3 = 0.109202 m
L4 = 0.115 m
L  = 0.200 m
W  = 0.080 m
```

### D3. Collision: convex hull

Mỗi link có collision mesh là convex hull của visual mesh. Dùng `Part.Shape.makeConvexHull()` trong FreeCAD hoặc trimesh fallback.

### D4. Kinematic structure: 17 link, 12 joint động, 4 joint fixed

```
base_link
├── FL_hip_link  ─[FL_hip_yaw, revolute, axis=X]─ thuộc FL chain
│   └── FL_thigh_link  ─[FL_thigh_pitch, revolute, axis=Y]─
│       └── FL_shank_link  ─[FL_knee_pitch, revolute, axis=Y]─
│           └── FL_foot_link  ─[FL_foot_fixed]─
├── FR_… (mirror)
├── BL_… (back-left)
└── BR_… (back-right)
```

Đặt tên joint match với code IK: `omega = hip_yaw`, `theta = thigh_pitch`, `phi = knee_pitch`.

### D5. Solid-to-link mapping

| Link | Solid trong FreeCAD |
|---|---|
| `base_link` | `dethan` + 2 `thanhngangbody` + 4 `opthan` + 4 `opkhop1.*` + 4 `noi2op*` + 3 `headcamv2` + 2 `RPi LCD` + `duoi` |
| `<leg>_hip_link` | 6 sub-feature của `part1.*` / `part1_miror.*` (cụm vỏ servo hip gắn vào body) |
| `<leg>_thigh_link` | 6 sub-feature của `part2.*` / `part2_miror.*` (đùi + vỏ servo gối) |
| `<leg>_shank_link` | 4 sub-feature của `part3.*` / `part3_mirror.*` (cẳng chân) |
| `<leg>_foot_link` | 1 `demchan-2li.*` (đệm chân) |

**Lưu ý:** Trong 100 solid có 8 part nhỏ tên `______*.STEP` (servo horn/washer) và 2 screw chưa được phân lớp. Script export phải bổ sung filter: phần nhỏ nằm gần servo nào (theo bounding-box center distance) thì gom vào link tương ứng — `*_thigh_link` cho horn ở khớp gối, `*_hip_link` cho horn ở hông. 2 screw có thể gộp vào `base_link` hoặc bỏ (không ảnh hưởng inertial đáng kể).

Phân chân theo tọa độ tâm trong frame CAD (đã xác định):
- FL: X≈0, Z≈0..67
- FR: X≈200, Z≈0..67
- BL: X≈0, Z≈-80..-149
- BR: X≈200, Z≈-80..-149

### D6. Joint axes & origins (REP-103, mét)

**Axes (đồng nhất 4 chân):**
- `*_hip_yaw`: axis = `(1, 0, 0)`
- `*_thigh_pitch`: axis = `(0, 1, 0)`
- `*_knee_pitch`: axis = `(0, 1, 0)`

Quy ước "axis-as-is" (link frame thẳng theo world khi joint=0), chuẩn ngành cho quadruped (Spot Mini, MIT Cheetah, Unitree, ANYmal). Tương thích đầy đủ MoveIt, PyBullet, Isaac, Pinocchio, Drake.

**Origins:**

| Joint | Parent | xyz (m) | rpy |
|---|---|---|---|
| `FL_hip_yaw` | base_link | `(+0.100, +0.040, 0)` | `0 0 0` |
| `FR_hip_yaw` | base_link | `(+0.100, −0.040, 0)` | `0 0 0` |
| `BL_hip_yaw` | base_link | `(−0.100, +0.040, 0)` | `0 0 0` |
| `BR_hip_yaw` | base_link | `(−0.100, −0.040, 0)` | `0 0 0` |
| `*L_thigh_pitch` | `*L_hip_link` | `(0, +0.04895, 0)` | `0 0 0` |
| `*R_thigh_pitch` | `*R_hip_link` | `(0, −0.04895, 0)` | `0 0 0` |
| `*_knee_pitch` | `*_thigh_link` | `(0, 0, −0.109202)` | `0 0 0` |
| `*_foot_fixed` | `*_shank_link` | `(0, 0, −0.115)` | `0 0 0` |

### D7. Joint limits (placeholder, servo phổ thông)

| Joint | lower (rad) | upper (rad) | effort (N·m) | velocity (rad/s) |
|---|---|---|---|---|
| `*_hip_yaw` | −0.785 (−45°) | 0.785 (+45°) | 2.0 | 5.0 |
| `*_thigh_pitch` | −1.571 (−90°) | 1.571 (+90°) | 2.0 | 5.0 |
| `*_knee_pitch` | 0 | 2.617 (+150°) | 2.0 | 5.0 |

Chỉnh lại theo datasheet servo thực tế sau khi build.

### D8. Inertial: tính từ FreeCAD

Mỗi link tính khối lượng + tensor quán tính bằng `Shape.computeInertial()` (hoặc thuộc tính `MatrixOfInertia`). Mật độ giả định **1.20 g/cm³** cho PLA in 3D. Có thể override mỗi link bằng config nếu lắp servo (khối lượng động cơ ~55g cho MG996R).

### D9. Mesh export pipeline

Trong FreeCAD MCP, mỗi link:
1. Fuse các solid cùng cluster: `Part.fuse(solids)`
2. Translate về frame link-local (origin tại joint position trong frame parent của link đó)
3. Rotate `Rx(-90°)` để remap CAD frame → REP-103 (`Placement = Rotation(Vector(1,0,0), -90)`)
4. Scale ÷1000 khi export STL (`exportStl(filename, scale=0.001)` hoặc apply scale trong shape)
5. Export visual: `Mesh.export([obj], "meshes/visual/<link>.stl")`
6. Export collision: `obj.Shape.makeConvexHull()` → export `"meshes/collision/<link>.stl"`

### D10. Gazebo physics & ros2_control (yêu cầu từ control spec D9)

Bổ sung sau khi chốt control spec để URDF sẵn sàng cho Gazebo sim + position control.

**(a) `<ros2_control>` block — trong `urdf/ros2_control.xacro`:**

```xml
<ros2_control name="dog_robot_hw" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>

  <xacro:macro name="joint_iface" params="name">
    <joint name="${name}">
      <command_interface name="position">
        <param name="min">${joint_lower}</param>
        <param name="max">${joint_upper}</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </xacro:macro>

  <!-- 12 joint: FL/FR/BL/BR × (hip_yaw, thigh_pitch, knee_pitch) -->
  <xacro:joint_iface name="FL_hip_yaw"/>
  <xacro:joint_iface name="FL_thigh_pitch"/>
  ...
</ros2_control>
```

**(b) Foot friction — trong `urdf/gazebo.xacro` cho mỗi foot link:**

```xml
<xacro:macro name="foot_friction" params="prefix">
  <gazebo reference="${prefix}_foot_link">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
    <kp>1000000.0</kp>     <!-- contact stiffness -->
    <kd>100.0</kd>          <!-- contact damping -->
    <minDepth>0.001</minDepth>
    <maxVel>0.1</maxVel>
  </gazebo>
</xacro:macro>

<xacro:foot_friction prefix="FL"/>
<xacro:foot_friction prefix="FR"/>
<xacro:foot_friction prefix="BL"/>
<xacro:foot_friction prefix="BR"/>
```

**(c) Joint dynamics — trong `urdf/leg.xacro` cho mỗi 12 joint:**

```xml
<joint name="${prefix}_${joint_name}" type="revolute">
  <axis xyz="..."/>
  <origin xyz="..." rpy="..."/>
  <parent link="..."/>
  <child link="..."/>
  <limit lower="..." upper="..." effort="2.0" velocity="5.0"/>
  <dynamics damping="0.01" friction="0.0"/>   <!-- ← bổ sung -->
</joint>
```

**(d) Gazebo plugin block — cuối `dog_robot.urdf.xacro`:**

```xml
<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find dog_robot_description)/config/ros2_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

**(e) `config/ros2_controllers.yaml` — đã định nghĩa chi tiết trong control spec D10. Tóm tắt:**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  ros__parameters:
    joints: [FL_hip_yaw, FL_thigh_pitch, FL_knee_pitch, ..., BR_knee_pitch]  # 12
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    state_publish_rate: 50.0
```

**Không bao gồm v1:** IMU, camera, lidar sensor plugin (control spec D4 confirm không cần). Có thể bổ sung sau khi spec sub-project "sensor & feedback".

## Kiến trúc ROS2 package

```
dog_robot_ws/
└── src/
    └── dog_robot_description/               # ament_cmake
        ├── package.xml
        ├── CMakeLists.txt
        ├── README.md
        ├── urdf/
        │   ├── dog_robot.urdf.xacro         # entry point
        │   ├── leg.xacro                    # macro 1 chân
        │   ├── inertial.xacro               # macro inertial helpers
        │   ├── materials.xacro              # màu visual
        │   ├── gazebo.xacro                 # gazebo plugin
        │   └── ros2_control.xacro           # <ros2_control> 12 joint
        ├── meshes/
        │   ├── visual/      (17 STL từ FreeCAD)
        │   └── collision/   (17 STL convex hull)
        ├── config/
        │   ├── joint_limits.yaml            # MoveIt
        │   └── ros2_controllers.yaml        # JointTrajectoryController
        ├── launch/
        │   ├── display.launch.py            # robot_state_publisher + RViz + JSP-GUI
        │   ├── gazebo.launch.py             # Gazebo + ros2_control
        │   └── rviz.launch.py
        ├── rviz/
        │   └── dog_robot.rviz
        └── test/
            └── test_urdf.py                 # pytest
```

**Build & run:**
```bash
cd dog_robot_ws
colcon build --packages-select dog_robot_description
source install/setup.bash
ros2 launch dog_robot_description display.launch.py     # RViz visual
ros2 launch dog_robot_description gazebo.launch.py      # Sim
```

## Scripts triển khai

Đặt trong `dog_robot/scripts/`:

1. **`export_links_from_freecad.py`** — chạy trong FreeCAD MCP qua `execute_code`:
   - Input: doc `RobotDog` (đã import sẵn)
   - Output: 34 file STL (17 visual + 17 collision) vào `meshes/visual/` và `meshes/collision/`
   - Áp dụng pipeline mesh export ở D9

2. **`generate_urdf.py`** — chạy ngoài FreeCAD:
   - Input: hằng số IK (L1..L4, L, W) hardcoded từ `TestIK/4leg.py`
   - Output: `urdf/dog_robot.urdf.xacro` + `urdf/leg.xacro`
   - Dùng template Python `string.Template` hoặc jinja2

## Pipeline kiểm thử

1. **Syntax check** — `check_urdf $(xacro dog_robot.urdf.xacro)`. Phải PASS.

2. **Visual + joint slider** — `ros2 launch dog_robot_description display.launch.py`. Kéo từng joint, quan sát:
   - hip_yaw: chân xoay ra ngoài/vào trong
   - thigh_pitch: đùi đu trước/sau
   - knee_pitch: cẳng gập về đùi
   - Không có link bay tự do

3. **FK cross-validation** (`test/test_urdf.py`):
   - Random 12 góc (omega, theta, phi)×4
   - Compute foot position bằng (a) Pinocchio FK qua URDF, (b) `TestIK/4leg.py` `solve_leg` chuyển ngược (FK = inv(IK))
   - Sai số <1mm
   - Nếu sai → joint origin/axis sai

4. **Sim drop test (PyBullet/Gazebo)** — thả robot từ 0.3m:
   - Robot không penetrate mặt đất
   - Không self-collision tại pose nominal (joints=0)
   - Inertial OK (không bị nảy điên loạn)

**Tiêu chí DONE:** cả 4 test pass + nhìn RViz thấy 4 chân thẳng đứng dưới body khi joint=0.

## Phạm vi không bao gồm (out of scope)

- Controller hoặc gait planning — chỉ xuất URDF + skeleton ros2_control config
- IMU / camera sensor plugin trong Gazebo — chừa hook trong `gazebo.xacro` để bổ sung sau
- Friction tuning chi tiết cho foot — đặt giá trị mặc định
- MoveIt config package (`dog_robot_moveit_config`) — tạo sau bằng `moveit_setup_assistant` từ URDF này
