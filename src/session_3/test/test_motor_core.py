from session_3.motor_core import SteeringControl, parse_feedback


def ready():
    control = SteeringControl(enabled=True, center=512)
    control.receive(512, 10.)
    control.heartbeat_stamp = 10.
    control.set_target([10, 0], 10.)
    return control


def test_steering_only_and_reject_drive():
    control = ready()
    assert control.command(10.) == b'150 0\n'
    for data in ([10, 150], [10, -150], [31, 0], [-31, 0], [], [0], [0, 0, 0]):
        control.set_target([10, 0], 10.)
        assert not control.set_target(data, 10.)
        assert control.command(10.) == b'0 0\n'


def test_three_independent_watchdogs_and_no_revival():
    for field, stale in [('feedback_stamp', 9.4), ('target_stamp', 9.4), ('heartbeat_stamp', 9.6)]:
        control = ready()
        setattr(control, field, stale)
        assert control.command(10.) == b'0 0\n'
        setattr(control, field, 10.)
        assert control.command(10.) == b'0 0\n'


def test_center_feedback_required_and_calibrate_idle():
    control = SteeringControl(enabled=True)
    control.receive(505, 10.)
    control.heartbeat_stamp = 10.
    assert control.command(10.) == b'0 0\n'
    assert control.calibrate(10.) == 505
    control.set_target([-10, 0], 10.)
    assert control.command(10.) == b'-150 0\n'
    try:
        control.calibrate(10.)
        assert False
    except ValueError:
        pass


def test_feedback_limits_and_disabled_output():
    assert parse_feedback('R,512,11.4,80') == 512
    for text in ('R,nan', 'R,1024,5,100', 'R,-1,0,0', 'X,512,2,50',
                 'R,512.5,2,50', 'R,512', 'R,512,nan,50', 'R,512,2,101'):
        assert parse_feedback(text) is None
    control = ready()
    control.receive(0, 10.)
    assert control.command(10.) == b'0 0\n'
    control = ready()
    control.enabled = False
    assert control.command(10.) == b'0 0\n'


def test_fixed_150_stops_at_target_and_repeated_target_stays_off():
    control = ready()
    assert control.command(10.) == b'150 0\n'
    control.receive(495, 10.05)  # +9.5625도: 목표 10도의 허용 오차 1도 이내
    assert control.command(10.05) == b'0 0\n'
    control.receive(500, 10.1)  # 출력 종료 후 흔들려도 같은 목표로 재출력하지 않음
    control.set_target([10, 0], 10.1)
    assert control.command(10.1) == b'0 0\n'
    control.set_target([0, 0], 10.1)
    assert control.command(10.1) == b'0 0\n'  # 새 목표도 앞선 펄스의 정착 시간을 건너뛰지 않는다.
    control.receive(500, 10.2)
    assert control.command(10.2) == b'-150 0\n'


def test_crossing_target_stops_instead_of_reversing():
    control = ready()
    control.command(10.)
    control.receive(485, 10.05)
    assert control.command(10.05) == b'0 0\n'
    assert control.reached


def test_twenty_millisecond_pulse_waits_for_feedback_after_settling():
    control = ready()
    assert control.command(10.) == b'150 0\n'
    control.set_target([10, 0], 10.005)
    assert control.command(10.005) == b'150 0\n'
    assert control.command(10.019) == b'150 0\n'
    assert control.command(10.021) == b'0 0\n'
    control.receive(512, 10.16)  # 새 측정이어도 20ms 펄스 +150ms 정착 전에 받은 값은 쓰지 않는다.
    assert control.command(10.17) == b'0 0\n'
    control.set_target([10, 0], 10.18)
    assert control.command(10.18) == b'0 0\n'  # 목표 재발행은 센서의 새 측정을 대신하지 않는다.
    control.receive(512, 10.18)
    assert control.command(10.18) == b'150 0\n'
    assert control.command(10.201) == b'0 0\n'


def test_distinct_targets_cannot_extend_pulse_or_bypass_fresh_feedback_gate():
    control = ready()
    assert control.command(10.) == b'150 0\n'
    control.set_target([11, 0], 10.005)
    assert control.command(10.005) == b'150 0\n'
    control.set_target([12, 0], 10.015)
    assert control.command(10.019) == b'150 0\n'
    assert control.command(10.021) == b'0 0\n'
    control.set_target([13, 0], 10.1)
    assert control.command(10.1) == b'0 0\n'
    control.receive(512, 10.18)
    assert control.command(10.18) == b'150 0\n'


def test_continuous_command_cannot_extend_five_second_goal_limit():
    control = ready()
    before_timeout = set()
    for step in range(2601):
        now = 10.+step*.002
        control.heartbeat_stamp = now
        if step % 100 == 0:
            control.receive(512, now)  # 실제 차량처럼 5Hz 피드백
        if step % 50 == 0:
            control.set_target([10, 0], now)  # 10Hz 목표 재발행
        actual = control.command(now)
        if step < 2500:
            before_timeout.add(actual)
        else:
            assert actual == b'0 0\n'
    assert before_timeout == {b'150 0\n', b'0 0\n'}
    assert control.expired


def test_reaching_target_during_pulse_immediately_cuts_power():
    control = ready()
    assert control.command(10.) == b'150 0\n'
    control.receive(495, 10.004)
    assert control.command(10.004) == b'0 0\n'
    control.receive(512, 10.2)
    control.set_target([10, 0], 10.2)
    assert control.command(10.2) == b'0 0\n'


def test_same_target_cannot_restart_after_feedback_fault():
    control = ready()
    assert control.command(10.) == b'150 0\n'
    control.heartbeat_stamp = 10.6
    control.set_target([10, 0], 10.6)
    assert control.command(10.6) == b'0 0\n'
    control.receive(512, 10.65)
    control.set_target([10, 0], 10.65)
    assert control.command(10.65) == b'0 0\n'
    control.set_target([-10, 0], 10.65)
    assert control.command(10.65) == b'-150 0\n'
