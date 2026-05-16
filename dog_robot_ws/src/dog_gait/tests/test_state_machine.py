from dog_gait.state_machine import GaitStateMachine, State


def test_initial_state_off():
    sm = GaitStateMachine()
    assert sm.state == State.OFF


def test_enable_transitions_off_to_stand():
    sm = GaitStateMachine()
    sm.enable()
    assert sm.state == State.STAND


def test_cmd_vel_zero_stays_stand():
    sm = GaitStateMachine()
    sm.enable()
    sm.update(cmd_vel_norm=0.0)
    assert sm.state == State.STAND


def test_cmd_vel_high_transitions_to_trot():
    sm = GaitStateMachine()
    sm.enable()
    sm.update(cmd_vel_norm=0.2)
    assert sm.state == State.TROT


def test_cmd_vel_zero_returns_to_stand():
    sm = GaitStateMachine()
    sm.enable()
    sm.update(cmd_vel_norm=0.2)
    sm.update(cmd_vel_norm=0.0)
    assert sm.state == State.STAND


def test_disable_returns_to_off():
    sm = GaitStateMachine()
    sm.enable()
    sm.update(cmd_vel_norm=0.2)
    sm.disable()
    assert sm.state == State.OFF
