# dog_robot_ws

ROS 2 workspace for a 12-DOF quadruped robot.

## Kinematics

The dog_robot uses Modified Denavit-Hartenberg (Craig) convention. Each leg is a
3-DOF chain (hip yaw, thigh pitch, knee pitch). One symmetric DH table covers
all four legs; per-leg variation lives in the base→hip fixed transform.

### Frames

- **Body B** — URDF root `base_link` (X forward, Y left, Z up).
- **Hip H_<leg>** — fixed transform per leg; Z_H along the hip yaw axis (= body
  X), X_H downward (= -body Z).
- **DH frames 1-3** — at each joint, Z along that joint's axis.

### DH Table

| i | α_{i-1} | a_{i-1}            | d_i | θ_i      |
|---|---------|---------------------|-----|----------|
| 1 | 0       | 0                  | 0   | θ_hip    |
| 2 | -π/2    | L_hh = 0.02553 m   | 0   | θ_thigh  |
| 3 | 0       | L_th = 0.11725 m   | 0   | θ_knee   |
| F | 0       | L_sh = 0.07043 m   | 0   | 0        |

Lengths come from `src/dog_robot_description/scripts/compute_dh_lengths.py`,
which averages the four legs' CAD measurements.

### Forward kinematics

```python
from dog_robot_control.kinematics_dh import DHParams, fk_leg
dh = DHParams(L_hh=0.02553, L_th=0.11725, L_sh=0.07043)
foot_xyz = fk_leg(dh, (theta_hip, theta_thigh, theta_knee))
```

### Inverse kinematics

```python
from dog_robot_control.kinematics_dh import ik_leg
theta_hip, theta_thigh, theta_knee = ik_leg(dh, foot_xyz_in_hip_frame,
                                            knee_direction=+1)
```

Closed-form 2R planar + hip yaw decomposition. Raises `ValueError` for foot
targets on the hip yaw axis or beyond reach.

### Per-leg base→hip transforms

| Leg | base→hip xyz (m)            | base→hip rpy (rad)    | Mirror |
|-----|------------------------------|------------------------|--------|
| FL  | ( 0.0748,  0.0400, 0.0351)  | (0, π/2, 0)           | +1     |
| FR  | ( 0.0748, -0.0400, 0.0351)  | (0, π/2, π)           | -1     |
| BL  | (-0.0748,  0.0400, 0.0351)  | (0, π/2, 0)           | +1     |
| BR  | (-0.0748, -0.0400, 0.0351)  | (0, π/2, π)           | -1     |

The right-side `π` yaw places right legs on the opposite side of body Y while
keeping the same DH table — IK and FK code is identical for all four legs.

### Stand controller

`ros2 launch dog_robot_control stand.launch.py` brings up Gazebo, spawns the
robot, activates `joint_state_broadcaster` + `joint_trajectory_controller`
(position command interface, open-loop mode), and starts `stand_controller`,
which ramps from the spawn pose to the default stand pose
(`default_height = 0.15 m`) using DH IK.

Publish `geometry_msgs/Pose` to `/stand_cmd` to change body height on the fly:
`Pose.position.z` = target body height in metres (range: 0.05–0.30 m).

### Verification

```bash
cd src/dog_robot_control && python3 -m pytest test/
```

Tests check FK/IK roundtrip (200 random configs) and URDF chain ↔ kinematics
module agreement on 40 random joint angle sets across all four legs.

## Walking

`walker_controller` is the production controller. It subsumes the older
`stand_controller`: when `/cmd_vel` is zero, the walker holds the stand pose;
non-zero `/cmd_vel` triggers a trot gait (Bernstein-Bezier swing + linear
stance) computed entirely in Python and converted to joint commands via the
DH-IK module.

### Launch

```bash
bash dog_robot_ws/scripts/dog_kill_all.sh
ros2 launch dog_robot_control walk.launch.py
```

Robot ramps to stand within 3 s. Then:

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}' -r 10
```

Twist field map:
- `linear.x` — forward / backward (m/s), capped at `gait.max_linear_velocity_x`
- `linear.y` — sideways (m/s)
- `angular.z` — yaw (rad/s)

Set all to zero (or stop publishing — there is a 0.5 s timeout) to stop.

### Gait config

`dog_robot_control/config/walker_params.yaml` exposes tunable gait params
(stance duration, swing height, velocity caps, knee direction, etc.). See
inline comments in the YAML.

### Architecture

`/cmd_vel` → `BodyController.pose_command` → `LegController.velocity_command`
(phase_generator + trajectory_planner per leg) → for each leg: rotate to DH
hip frame → `ik_leg` → `JointTrajectory` → joint_trajectory_controller →
Gazebo. Python module layout: `dog_robot_control/dog_robot_control/gait/`.

### Deprecated

`stand_controller` + `stand.launch.py` remain in the tree for reference but
are deprecated. Use the walker.
