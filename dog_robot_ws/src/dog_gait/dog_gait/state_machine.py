"""3-state gait machine: OFF → STAND ↔ TROT, plus e-stop back to OFF."""
from enum import Enum


class State(Enum):
    OFF = "OFF"
    STAND = "STAND"
    TROT = "TROT"


CMD_VEL_THRESHOLD = 0.01  # m/s or rad/s — below this, treat as zero


class GaitStateMachine:
    def __init__(self):
        self.state = State.OFF

    def enable(self):
        if self.state == State.OFF:
            self.state = State.STAND

    def disable(self):
        self.state = State.OFF

    def update(self, cmd_vel_norm: float):
        """Transition based on commanded velocity magnitude."""
        if self.state == State.OFF:
            return
        if cmd_vel_norm > CMD_VEL_THRESHOLD:
            self.state = State.TROT
        else:
            self.state = State.STAND
