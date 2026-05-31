"""Kinematic-only RViz rig for the 12-DOF dog robot.

Pipeline: /cmd_vel -> BodyCommander -> LegDriver(x N) -> ik_leg -> /joint_states.
Static world->base_link TF anchors the body so the legs swing in space.
See docs/superpowers/specs/2026-06-01-kinematics-only-worktree-design.md.
"""
