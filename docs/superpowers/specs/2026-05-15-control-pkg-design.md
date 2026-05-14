# Robot Dog — Control Package Design

**Date:** 2026-05-15
**Owner:** nguyenvd11@fpt.com
**Status:** Design (pending implementation)
**Depends on:** `2026-05-15-urdf-export-design.md`

## Mục tiêu

Xây dựng control stack cho robot dog 12 DOF chạy trong **Gazebo simulation**, hỗ trợ:
- Đứng yên với điều khiển 6-DOF body pose
- Đi tới-lui-xoay bằng trot gait, nhận `cmd_vel` chuẩn ROS2
- Tận dụng IK code có sẵn trong `TestIK/`

Out of scope (v1): hardware deployment, RL policy, IMU closed-loop, MPC, walk 4-phase gait, sensor (camera/lidar).

## Bối cảnh

- `TestIK/` có sẵn: `bodyIK`, `legIK`, `calcLegPoints`, `world_to_leg` — math đầy đủ, nhưng là Python script tương tác (input), chưa có integration với ROS hay event loop.
- URDF spec ở phase 1 đã định nghĩa 12 joint với position interface, REP-103, mét.
- Greenfield: chưa có controller code nào.

## Quyết định thiết kế

### D1. Scope: chỉ Gazebo simulation

Không deploy hardware. Không cần hardware abstraction. Không cần real-time guarantee.

### D2. Sim engine: Gazebo (Ignition/Harmonic)

Tích hợp `gz_ros2_control` plugin. Tương thích đầy đủ với spec URDF phase 1.

### D3. Control level: position control

Mỗi joint nhận lệnh position (rad). Dùng `joint_trajectory_controller/JointTrajectoryController` — chuẩn ros2_control. Khớp tự nhiên với output của IK code.

### D4. Locomotion scope: STAND + TROT

V1 hỗ trợ 2 chế độ:
- **STAND**: 4 chân chạm đất, body pose điều khiển được qua `/body_pose_setpoint`
- **TROT**: diagonal-pair gait, nhận `cmd_vel.linear.x/y` + `angular.z`

Out of scope v1: WALK 4-phase, BOUND, GALLOP, JUMP.

### D5. Architecture: Library + thin ROS2 node

3 package độc lập:

1. **`dog_kinematics`** — pure Python library (no ROS dep). Refactor `TestIK/` thành module sạch:
   - `constants.py`: L1..L4, L, W, joint limits, nominal foot pos
   - `leg.py`: `legIK`, `legFK`, `calcLegPoints` (giữ nguyên math từ `Test2.py`)
   - `body.py`: `bodyIK`, `world_to_leg`, mirror handling
   - `solver.py`: `solve_all_legs(body_pose, foot_targets_world) → dict[joint_name, rad]`

2. **`dog_gait`** — pure Python library (no ROS dep). Depends `dog_kinematics`:
   - `state_machine.py`: 3 state OFF → STAND → TROT
   - `foot_planner.py`: Bezier swing + linear stance
   - `body_planner.py`: body height + sway control
   - `controller.py`: `tick(cmd_vel, body_pose_setpoint, dt) → 12 joint angles`

3. **`dog_robot_control`** — ament_python ROS2 package. Depends `dog_kinematics` + `dog_gait`:
   - `controller_node.py`: ROS2 node 50 Hz, wrap library, sub/pub topic
   - `teleop_keyboard.py`: tiện ích test
   - `launch/`, `config/`, `test/`

Lý do: pure-Python lib unit-test bằng `pytest` không cần ROS, tái dùng được cho RL training/notebook. Node thin (~150 LOC) dễ debug.

### D6. Gait state machine

```
        ┌──────┐  enable   ┌────────┐  cmd_vel != 0   ┌──────┐
        │ OFF  ├──────────►│ STAND  ├────────────────►│ TROT │
        └──────┘           │        │◄────────────────┤      │
            ▲              │ all 4  │  cmd_vel == 0   └──┬───┘
            │              │ on gnd │                    │
            └──────────────┴────────┴────────────────────┘
                          e-stop / shutdown
```

**TROT timing:**
- Cycle time `T = 0.4 s`
- Duty factor `β = 0.5` (0.2s stance, 0.2s swing)
- Diagonal pairs: FL+BR in-phase (φ=0), FR+BL anti-phase (φ=0.5)

### D7. Foot trajectory

**Stance phase** (φ ∈ [0, β)): foot linear từ `+stride/2` → `−stride/2` trong frame body, đáy foot bám đất.

**Swing phase** (φ ∈ [β, 1)): Bezier bậc 4 với 5 control point:
```
P0=(−stride/2, 0,         0)         liftoff
P1=(−stride/2, H_step,    0)
P2=(0,         H_step,    0)         apex
P3=(+stride/2, H_step,    0)
P4=(+stride/2, 0,         0)         touchdown
```

**Combined linear + angular `cmd_vel`:**
```
v_foot_body = cmd_vel.linear + cmd_vel.angular × r_foot_nominal
```

**Edge cases:**
- `|cmd_vel| < 0.01` → quay về STAND, reset phase
- `legIK` raise (ngoài workspace) → clip stride, log warning, không crash
- `cmd_vel` đổi đột ngột → low-pass filter `α = 0.2` trên target velocity

### D8. ROS2 interfaces

**Subscriptions:**

| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Vận tốc target |
| `/body_pose_setpoint` | `geometry_msgs/Pose` | Body pose khi STAND (optional) |
| `/joint_states` | `sensor_msgs/JointState` | Feedback monitor (không closed-loop v1) |

**Publications:**

| Topic | Type | Purpose |
|---|---|---|
| `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 12 joint command |
| `/gait_state` | `std_msgs/String` | `"OFF"`, `"STAND"`, `"TROT"` |
| `/debug/foot_targets` | `visualization_msgs/MarkerArray` | Foot target markers cho RViz |

**Services:**

| Service | Type | Purpose |
|---|---|---|
| `/enable` | `std_srvs/SetBool` | OFF↔STAND, e-stop |
| `/reset_gait` | `std_srvs/Trigger` | Reset gait phase |

**Parameters (`config/controller_params.yaml`):**

```yaml
controller_node:
  ros__parameters:
    tick_rate: 50.0
    joint_names: [FL_hip_yaw, FL_thigh_pitch, FL_knee_pitch, FR_hip_yaw, ...]   # 12 total, đúng thứ tự URDF
    gait:
      cycle_time: 0.4
      duty_factor: 0.5
      step_height: 0.05
      max_stride: 0.10
      body_height: 0.140
    cmd_vel:
      max_linear_x: 0.20
      max_linear_y: 0.10
      max_angular_z: 0.50
      lowpass_alpha: 0.2
    debug:
      publish_foot_markers: true
      log_ik_warnings: true
```

### D9. URDF spec dependencies (phase 1 cần bổ sung)

Áp dụng ngược vào URDF spec:
- ✅ `<ros2_control>` block 12 joint × (position cmd + position state) — đã có trong URDF spec D6
- ➕ **Bổ sung:** `<gazebo reference="*_foot_link">` với `mu1=mu2=1.0, kp=1e6, kd=100` (foot friction)
- ➕ **Bổ sung:** Mỗi `<joint>` có `<dynamics damping="0.01" friction="0.0"/>` (Gazebo physics stability)
- ❌ Không cần IMU/camera plugin v1

### D10. ros2_controllers.yaml (trong `dog_robot_description/config/`)

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
    joints: [FL_hip_yaw, FL_thigh_pitch, FL_knee_pitch, ...]  # 12
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    state_publish_rate: 50.0
```

## Package structure

```
dog_robot_ws/src/
├── dog_robot_description/           # phase 1 (URDF) — bổ sung D9, D10
├── dog_kinematics/                  # pure-Python (no ROS)
│   ├── pyproject.toml
│   ├── dog_kinematics/
│   │   ├── __init__.py
│   │   ├── constants.py
│   │   ├── leg.py
│   │   ├── body.py
│   │   └── solver.py
│   └── tests/
├── dog_gait/                        # pure-Python (no ROS)
│   ├── pyproject.toml
│   ├── dog_gait/
│   │   ├── __init__.py
│   │   ├── state_machine.py
│   │   ├── foot_planner.py
│   │   ├── body_planner.py
│   │   └── controller.py
│   └── tests/
└── dog_robot_control/               # ament_python ROS2
    ├── package.xml
    ├── setup.py
    ├── dog_robot_control/
    │   ├── controller_node.py
    │   └── teleop_keyboard.py
    ├── config/
    │   └── controller_params.yaml
    ├── launch/
    │   ├── controller.launch.py
    │   └── full_sim.launch.py
    └── test/
        ├── test_node_integration.py
        └── test_sim_smoke.py
```

**Install / build:**
```bash
pip install -e src/dog_kinematics src/dog_gait
colcon build --packages-select dog_robot_control
source install/setup.bash
ros2 launch dog_robot_control full_sim.launch.py
```

## Launch flow (`full_sim.launch.py`)

1. `robot_state_publisher` với URDF (xacro processed)
2. `gz_sim` spawn world + spawn robot URDF
3. `controller_manager` + spawn `JointTrajectoryController` + `JointStateBroadcaster`
4. `dog_robot_control/controller_node`
5. (optional) RViz2 + teleop_keyboard

## Testing strategy

| Mức | Test | Vị trí | Lệnh |
|---|---|---|---|
| Unit (kinematics) | Round-trip IK→FK <1e-6 rad; mirror đối xứng; edge case raise đúng | `dog_kinematics/tests/` | `pytest` |
| Unit (gait) | State machine transitions đúng; foot trajectory C¹; Bezier qua waypoint | `dog_gait/tests/` | `pytest` |
| Integration (node) | Node lên; sub/pub đúng tên topic; service `/enable` | `dog_robot_control/test/` | `colcon test` |
| Sim smoke | Gazebo + controller; `cmd_vel.x=0.1` 5s; robot tiến >0.3m, không lật | `dog_robot_control/test/` | `colcon test` |
| Sim manual | Teleop 30s không lật; cmd_vel=0 → STAND | — | `ros2 launch …` |

**Tiêu chí DONE:**
- `pytest` + `colcon test` pass 100%
- 30s trot manual: không lật, không penetrate, hướng đi khớp `cmd_vel` ±10°
- IK warning <1% tick

## Phụ thuộc lẫn nhau

```
dog_robot_control  ──depends──►  dog_gait  ──depends──►  dog_kinematics
       │                                                       │
       └────────────depends─────────────────────────────────────┘
       │
       └──runtime requires──►  dog_robot_description (URDF, mesh, ros2_controllers.yaml)
                                       │
                                       └── needs D9 bổ sung (foot friction, joint damping)
```

## Phạm vi không bao gồm (out of scope v1)

- Hardware deployment (PCA9685, RPi GPIO, servo driver)
- Real-time guarantee
- IMU closed-loop balance
- WALK 4-phase gait, BOUND, GALLOP, JUMP
- Sensor plugins (camera/lidar)
- RL training pipeline
- MPC controller
- MoveIt integration
- Multi-robot coordination

Mỗi mục trên có thể là sub-project tiếp theo, mỗi sub-project 1 spec riêng.
