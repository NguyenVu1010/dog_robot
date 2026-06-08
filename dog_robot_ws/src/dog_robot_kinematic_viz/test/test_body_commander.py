"""BodyCommander: trot pattern + cmd_vel state + phase advancement."""
import pytest

from dog_robot_kinematic_viz.body_commander import BodyCommander


def test_default_state_is_zero():
    b = BodyCommander()
    assert b.body_vel_xy() == (0.0, 0.0)
    assert b.body_yaw_rate() == 0.0
    assert b.time() == 0.0


def test_on_cmd_vel_updates_state():
    b = BodyCommander()
    b.on_cmd_vel(0.3, -0.1, 0.03, 0.5)
    assert b.body_vel_xy() == (0.3, -0.1)
    assert b.body_yaw_rate() == 0.5
    # linear_z is integrated via tick(); 1 s @ 0.03 m/s = 0.03 m (within default clamp).
    b.tick(1.0)
    assert b.body_z() == pytest.approx(0.03, abs=1e-9)


def test_tick_accumulates_time():
    b = BodyCommander(step_freq=1.0)
    b.tick(0.5)
    b.tick(0.25)
    assert b.time() == pytest.approx(0.75)


def test_phase_advances_at_step_freq():
    b = BodyCommander(step_freq=2.0)
    # phi = t * step_freq; FL has offset 0.
    b.tick(0.25)  # t=0.25 -> phi = 0.5
    assert b.phase("FL") == pytest.approx(0.5)
    b.tick(0.25)  # t=0.5 -> phi = 1.0 % 1 = 0.0
    assert b.phase("FL") == pytest.approx(0.0, abs=1e-12)


def test_trot_diagonals_in_phase():
    b = BodyCommander(step_freq=1.5)
    b.tick(0.123)
    assert b.phase("FL") == pytest.approx(b.phase("BR"))
    assert b.phase("FR") == pytest.approx(b.phase("BL"))


def test_trot_diagonals_180_out_of_phase():
    b = BodyCommander(step_freq=1.5)
    b.tick(0.0)  # phase still 0 mod offsets
    diff = (b.phase("FR") - b.phase("FL")) % 1.0
    assert diff == pytest.approx(0.5)


def test_phase_in_unit_interval_for_long_run():
    b = BodyCommander(step_freq=3.0)
    for _ in range(10000):
        b.tick(0.01)
    for leg in ("FL", "FR", "BL", "BR"):
        phi = b.phase(leg)
        assert 0.0 <= phi < 1.0


def test_unknown_leg_raises():
    b = BodyCommander()
    with pytest.raises(KeyError):
        b.phase("XX")


def test_default_body_z_is_zero():
    b = BodyCommander()
    assert b.body_z() == 0.0


def test_vz_integrates_into_body_z():
    b = BodyCommander()
    b.on_cmd_vel(0.0, 0.0, 0.02, 0.0)
    b.tick(0.1)
    assert b.body_z() == pytest.approx(0.002, abs=1e-9)
    b.tick(0.1)
    assert b.body_z() == pytest.approx(0.004, abs=1e-9)


def test_body_z_clamps_at_max():
    b = BodyCommander()  # default body_z_max = +0.04
    b.on_cmd_vel(0.0, 0.0, 0.10, 0.0)
    for _ in range(100):
        b.tick(0.01)   # total commanded delta = 0.10 * 1.0 = 0.10 m
    assert b.body_z() == pytest.approx(0.04, abs=1e-9)


def test_body_z_clamps_at_min():
    b = BodyCommander()  # default body_z_min = -0.04
    b.on_cmd_vel(0.0, 0.0, -0.10, 0.0)
    for _ in range(100):
        b.tick(0.01)
    assert b.body_z() == pytest.approx(-0.04, abs=1e-9)


def test_space_zeros_vz_halts_integration():
    b = BodyCommander()
    b.on_cmd_vel(0.0, 0.0, 0.02, 0.0)
    b.tick(0.5)
    z_after_drive = b.body_z()
    assert z_after_drive == pytest.approx(0.01, abs=1e-9)
    b.on_cmd_vel(0.0, 0.0, 0.0, 0.0)   # space
    b.tick(1.0)
    assert b.body_z() == pytest.approx(z_after_drive, abs=1e-9)


def test_body_z_min_max_params_respected():
    b = BodyCommander(body_z_min=-0.10, body_z_max=+0.10)
    b.on_cmd_vel(0.0, 0.0, 1.0, 0.0)
    for _ in range(50):
        b.tick(0.01)
    assert b.body_z() == pytest.approx(0.10, abs=1e-9)
