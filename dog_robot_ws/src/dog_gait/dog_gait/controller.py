"""Gait controller: ticks state machine + foot planner + IK → 12 joint angles."""
import math
import numpy as np
from dog_kinematics.constants import LEG_NAMES, L2, L3, L4
from dog_kinematics.body import bodyIK
from dog_kinematics.solver import solve_all_legs
from .state_machine import GaitStateMachine, State
from .foot_planner import FootPlanner

# Nominal foot position in leg-frame (IK frame convention):
#   x=0 (no fore-aft offset at stand), y=-(L3+L4) (straight down), z=L2 (abductor offset)
_NOM_LEG_FRAME = np.array([0.0, -(L3 + L4), L2, 1.0])


def _compute_nominal_feet_world(body_pose):
    """Return dict of nominal foot positions in world frame for all legs.

    Derives foot world positions from hip transforms so they lie exactly
    at the IK-frame nominal stand position (foot directly below hip).
    Both left and right legs use the same leg-frame nominal because x=0
    so the right-leg X-flip (IX) has no effect.
    """
    Tlf, Trf, Tlb, Trb, _ = bodyIK(*body_pose)
    return {
        "FL": tuple(Tlf @ _NOM_LEG_FRAME),
        "FR": tuple(Trf @ _NOM_LEG_FRAME),
        "BL": tuple(Tlb @ _NOM_LEG_FRAME),
        "BR": tuple(Trb @ _NOM_LEG_FRAME),
    }


class GaitController:
    def __init__(self, cycle_time=0.4, duty_factor=0.5, step_height=0.05,
                 max_stride=0.10):
        self.sm = GaitStateMachine()
        self.planner = FootPlanner(cycle_time, duty_factor, step_height, max_stride)
        self.cycle_time = cycle_time
        self.phase = 0.0

        # Diagonal pair phase offsets: FL+BR = 0, FR+BL = 0.5
        self.leg_phase_offset = {"FL": 0.0, "FR": 0.5, "BL": 0.5, "BR": 0.0}

    def enable(self):
        self.sm.enable()

    def disable(self):
        self.sm.disable()

    def tick(self, cmd_vel, body_pose, dt):
        """Advance one tick. Returns dict of 12 joint angles or None if OFF."""
        # Update state machine
        v_norm = math.sqrt(cmd_vel[0]**2 + cmd_vel[1]**2 + cmd_vel[2]**2)
        self.sm.update(v_norm)

        if self.sm.state == State.OFF:
            return None

        # Build foot targets
        nominal = _compute_nominal_feet_world(body_pose)
        foot_targets = {}
        if self.sm.state == State.STAND:
            foot_targets = dict(nominal)
        else:
            # TROT: advance phase + plan each foot
            self.phase = (self.phase + dt / self.cycle_time) % 1.0
            for leg in LEG_NAMES:
                phi = (self.phase + self.leg_phase_offset[leg]) % 1.0
                nom = nominal[leg]
                dx, dy, dz = self.planner.foot_position(phi, cmd_vel)
                foot_targets[leg] = (nom[0] + dx, nom[1] + dy, nom[2] + dz, 1.0)

        # IK
        try:
            angles = solve_all_legs(body_pose, foot_targets)
        except Exception:
            # Fall back to nominal stand if foot target leaves workspace
            angles = solve_all_legs(body_pose, nominal)
        return angles
