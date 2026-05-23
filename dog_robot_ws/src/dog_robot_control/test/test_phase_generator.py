import math
import pytest

from dog_robot_control.gait.phase_generator import PhaseGenerator

STANCE = 0.30
SWING = 0.25  # CHAMP hard-coded
STRIDE = STANCE + SWING


def make_pg():
    return PhaseGenerator(stance_duration=STANCE)


def test_idle_signals_zero_when_velocity_zero():
    pg = make_pg()
    pg.run(target_velocity=0.0, step_length=0.1, t=0.0)
    assert pg.stance_phase_signal == [0.0, 0.0, 0.0, 0.0]
    assert pg.swing_phase_signal == [0.0, 0.0, 0.0, 0.0]


def test_warmup_holds_legs_0_3_in_stance():
    pg = make_pg()
    pg.run(target_velocity=0.1, step_length=0.05, t=0.0)
    pg.run(target_velocity=0.1, step_length=0.05, t=STANCE * 0.3)
    assert pg.stance_phase_signal[0] == 0.0
    assert pg.stance_phase_signal[3] == 0.0
    assert pg.swing_phase_signal[1] == 0.0
    assert pg.swing_phase_signal[2] == 0.0


def test_trot_anti_phase_after_warmup():
    pg = make_pg()
    pg.run(0.1, 0.05, 0.0)
    pg.run(0.1, 0.05, STANCE * 0.6)
    pg.run(0.1, 0.05, STRIDE * 1.25)
    assert pg.stance_phase_signal[0] == pytest.approx(pg.stance_phase_signal[3], abs=1e-6)
    assert pg.stance_phase_signal[1] == pytest.approx(pg.stance_phase_signal[2], abs=1e-6)


def test_velocity_zero_resets_state():
    pg = make_pg()
    pg.run(0.1, 0.05, 0.0)
    pg.run(0.1, 0.05, STRIDE * 0.5)
    pg.run(0.0, 0.0, STRIDE * 0.6)
    assert pg.stance_phase_signal == [0.0, 0.0, 0.0, 0.0]
    assert pg.swing_phase_signal == [0.0, 0.0, 0.0, 0.0]
    pg.run(0.1, 0.05, STRIDE * 0.7)
    assert pg.stance_phase_signal == [0.0, 0.0, 0.0, 0.0]
