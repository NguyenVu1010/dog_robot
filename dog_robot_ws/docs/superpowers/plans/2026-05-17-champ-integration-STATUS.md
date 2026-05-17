# CHAMP integration — paused 2026-05-17

Companion to `2026-05-17-champ-integration.md`. Captures where execution stopped and what remains.

## Done (Tasks 1–12)

- `feature/champ-integration` branch cut from `main`
- CHAMP vendored into `dog_robot_ws/src/champ/` (8 packages + libchamp submodule), builds clean
- `dog_kinematics`, `dog_gait`, `dog_robot_bringup` removed
- `dog_robot_control` trimmed to `teleop_keyboard` only
- `dog_robot_config` package authored: joints.yaml, links.yaml, gait.yaml, ros_control.yaml (effort interface, scaled gains), simple.world, bringup.launch.py, gazebo.launch.py
- URDF updated: ros2_control block uses effort interface, gazebo plugin reads yaml from `dog_robot_config` via `$(find ...)`
- `champ_description/launch/description.launch.py` and `champ_bringup/launch/bringup.launch.py` patched with `ParameterValue(..., value_type=str)` wrappers — required for long URDF strings on ROS 2 Humble launch
- Gazebo bringup: robot spawns successfully, `joint_group_effort_controller` and `joint_states_controller` both reach `active`

## Blocker (Task 13)

CHAMP IK output for nominal stand pose is **kinematically incompatible** with our CAD-derived URDF link frames:

- CHAMP commanded `thigh_pitch = 2.858 rad (164°)`, `knee_pitch = -0.182 rad (-10°)` to put foot at `nominal_height = 0.18 m` below base.
- Forward kinematics of our URDF at those angles puts foot at `(0.10, 0.10, +0.10)` in `base_link` frame — i.e. **above and to the side of the base**, not below.
- Robot in Gazebo therefore stays at world origin with all joints clamped to URDF limits (`thigh_pitch=2.858 > 1.571` clamp) and never moves.

### Root cause

CHAMP's IK formulas were derived for the Spot Micro convention where, at joint angle 0:

- `thigh_link` extends straight DOWN (in `-Z` of hip frame)
- `shank_link` extends along `+Z` of thigh frame after knee bend

Our URDF (auto-generated from FreeCAD CAD) has different link rest orientations:

- `thigh_link` mesh extends in `-X` direction at θ=0 (visible from STL bbox)
- The link frame at θ=0 is rotated ~90° around `+Y` axis relative to spot-micro convention

The joint AXES (`hip` around X, `thigh`/`knee` around Y) match spot-micro and CHAMP. The issue is the link FRAMES — specifically the URDF `<origin>` rotations of each joint relative to its parent.

## What's left (Tasks 13–15)

To make CHAMP actually drive our URDF, one of these must happen:

### Option A (deep): re-do the URDF link frames

Modify the FreeCAD→URDF generator (`scripts/export_links_from_freecad.py`) so each leg link's frame is aligned with CHAMP convention:

- `hip_link` frame X axis along leg axis-of-abduction
- `thigh_link` frame Z axis along leg-down direction at θ=0
- `shank_link` frame Z axis along leg-down direction at θ=0

Naive attempts (adding `rpy="0 -π/2 0"` to thigh joint origin) move things partially but don't fully solve because subsequent joint origin XYZ values were also computed in the OLD frame — re-orientation requires re-computing all four offset xyz values (hip, thigh, knee, foot) consistently in the new frame.

Estimated effort: half-day to one day for someone with CAD context.

### Option B (medium): rewrite `dog_kinematics` IK module

Drop CHAMP for controller. Re-derive the leg IK from scratch matching our URDF convention (knee axis (0,1,0), positive bend, etc.). Keep our `controller_node.py` shell (or write a new one) calling the new IK. CHAMP still useful for reference and state estimation can come from joint_state_broadcaster + plain odom.

Estimated effort: half-day to one day.

### Option C (light): CHAMP state estimation only

Use CHAMP's `state_estimation_node` + `robot_localization/ekf` for `/odom` and TF, write a minimal gait module of our own that targets our URDF FK directly (no IK).

Estimated effort: hours for basic walking, more for tuning.

### Recommended next step

**Option B** is most likely the right pivot. CHAMP's controller framework is tightly coupled to its IK convention; cleaner to write a correct IK against our actual URDF than to bend our URDF to fit CHAMP.

## Useful artifacts

- `champ_ws/` — separate workspace where CHAMP walking with spot-micro URDF was demonstrated (working baseline).
- `/tmp/dog_robot_bringup.log` (no-Gazebo bringup) and `/tmp/dog_robot_gz.log` (Gazebo bringup) show full controller activation sequence for our URDF.
- `dog_robot_config/config/gait/gait.yaml` and `ros_control.yaml` are CHAMP-style and can be reused if Option A succeeds; `joints.yaml`/`links.yaml` mapping is verified correct against our joint names.

## Reverting back to a known-good Gazebo state

`feature/control-pkg` was the previous branch which had the broken in-house controller. Until Option B or A lands, the robot in `feature/champ-integration` does not walk in Gazebo.

To resume:

```bash
git checkout feature/champ-integration
# Apply chosen option (A / B / C) following the spec
```

Or to discard and start over from a working state:

```bash
git checkout main
```
