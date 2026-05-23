"""Trot phase generator — port of CHAMP's PhaseGenerator.

Outputs per-leg saw-tooth signals in [0, 1] indicating stance vs swing
progress. Leg order: [FL=0, FR=1, BL=2, BR=3]. Trot pairs: (0,3) and (1,2).
Swing phase is hard-coded to 0.25 s like CHAMP.
"""
from typing import List


SWING_PHASE_PERIOD = 0.25  # s, CHAMP-compatible


class PhaseGenerator:
    def __init__(self, stance_duration: float) -> None:
        self.stance_duration = stance_duration
        self.last_touchdown: float = 0.0
        self.has_started: bool = False
        self.has_swung: bool = False
        self.stance_phase_signal: List[float] = [0.0, 0.0, 0.0, 0.0]
        self.swing_phase_signal: List[float] = [0.0, 0.0, 0.0, 0.0]

    def run(self, target_velocity: float, step_length: float, t: float) -> None:
        stance_period = self.stance_duration
        swing_period = SWING_PHASE_PERIOD
        stride_period = stance_period + swing_period

        if target_velocity == 0.0:
            self.has_started = False
            self.has_swung = False
            self.last_touchdown = 0.0
            self.stance_phase_signal = [0.0] * 4
            self.swing_phase_signal = [0.0] * 4
            return

        if not self.has_started:
            self.has_started = True
            self.last_touchdown = t

        if (t - self.last_touchdown) >= stride_period:
            self.last_touchdown = t

        elapsed = t - self.last_touchdown
        if elapsed >= stride_period:
            elapsed = stride_period

        leg_clocks = [
            elapsed - 0.0 * stride_period,
            elapsed - 0.5 * stride_period,
            elapsed - 0.5 * stride_period,
            elapsed - 0.0 * stride_period,
        ]

        stance = [0.0] * 4
        swing = [0.0] * 4
        for i, c in enumerate(leg_clocks):
            if 0 < c < stance_period:
                stance[i] = c / stance_period
            if -swing_period < c < 0:
                swing[i] = (c + swing_period) / swing_period
            elif stance_period < c < stride_period:
                swing[i] = (c - stance_period) / swing_period

        if not self.has_swung and stance[0] < 0.5:
            stance[0] = 0.0
            stance[3] = 0.0
            swing[1] = 0.0
            swing[2] = 0.0
        else:
            self.has_swung = True

        self.stance_phase_signal = stance
        self.swing_phase_signal = swing
