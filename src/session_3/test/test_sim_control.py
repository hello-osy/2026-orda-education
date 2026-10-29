import math

import pytest

from session_3.sim_control_node import CommandState, motor_to_velocity


def test_ackermann_velocity_and_left_right_reverse():
    assert motor_to_velocity([0, 255]) == (1.0, 0.0)
    assert motor_to_velocity([0, 0]) == (0.0, 0.0)
    speed, yaw = motor_to_velocity([30, 128])
    assert speed == pytest.approx(128 / 255)
    assert yaw == pytest.approx(speed * math.tan(math.pi / 6) / .48)
    assert motor_to_velocity([-30, 128])[1] == -yaw
    assert motor_to_velocity([30, -128]) == (-speed, -yaw)


@pytest.mark.parametrize('data', [[], [1], [1, 2, 3], [31, 0], [-31, 0], [0, 256], [0, -256], [True, 0], [1.0, 0]])
def test_invalid_command_stops_previous_motion(data):
    state = CommandState()
    state.receive([10, 200], 10)
    assert state.current(10.1)[0] > 0
    with pytest.raises(ValueError):
        state.receive(data, 10.2)
    assert state.current(10.21) == (0, 0)


def test_wall_clock_watchdog_and_fresh_command():
    state = CommandState()
    assert state.current(100) == (0, 0)
    state.receive([0, 150], 100)
    assert state.current(100.49)[0] > 0
    assert state.current(100.5) == (0, 0)
    state.receive([0, -150], 101)
    assert state.current(101.1)[0] < 0
