import time
import sys
import pytest
from session_2.driving import MotorControl,parse_rotation
from session_2 import driving, autonomous_driving


@pytest.fixture(autouse=True, params=[driving, autonomous_driving])
def control_implementation(request, monkeypatch):
    # 기존 구현과 3번 교육 코드에 동일한 통신·정지 검사를 적용합니다.
    module = sys.modules[__name__]
    monkeypatch.setattr(module, 'MotorControl', request.param.MotorControl)
    monkeypatch.setattr(module, 'parse_rotation', request.param.parse_rotation)


def prepared():
    motor=MotorControl('TEST-NOT-OPENED')
    now=time.monotonic()
    motor.state.connected=True
    motor.heartbeat()
    motor.receive(485,now)
    motor.calibrate()
    motor.set_target(10,now)
    return motor


def test_protocol_requires_explicit_arm_and_pid_is_bounded():
    motor=prepared()
    assert motor.command(time.monotonic())==b'0 0\n'
    motor.arm(150)
    steer,drive=map(int,motor.command(time.monotonic()).split())
    assert 40<=steer<=150 and drive==5
    motor.disarm()
    assert motor.command(time.monotonic())==b'0 0\n'


@pytest.mark.parametrize('field',['heartbeat_stamp','feedback_stamp','target_stamp'])
def test_stale_inputs_stop_and_latch_disarmed(field):
    motor=prepared();motor.arm(150)
    setattr(motor.state,field,time.monotonic()-1)
    assert motor.command(time.monotonic())==b'0 0\n'
    setattr(motor.state,field,time.monotonic())
    assert not motor.state.armed
    assert motor.command(time.monotonic())==b'0 0\n'


def test_lane_loss_and_missing_feedback_block_output():
    motor=MotorControl('TEST-NOT-OPENED')
    with pytest.raises(ValueError):motor.arm(10)
    motor=prepared();motor.arm(150);motor.set_target(None,time.monotonic())
    assert motor.command(time.monotonic())==b'0 0\n'
    assert not motor.state.armed


def test_rotation_frame_validation():
    assert parse_rotation('R,485,2.37,47')==485
    for line in ['U,1,2,3,4,5,6','R,1024,5,100','R,50,nan,5','R,50,2,101','R,x,2,5']:
        assert parse_rotation(line) is None


def test_serial_worker_sends_protocol_and_final_stop_without_hardware():
    class FakeSerial:
        def __init__(self,*args,**kwargs):
            self.writes=[];self.closed=False;self.last_rx=0
        @property
        def in_waiting(self):return 15
        def read(self,size):return b'R,485,2.37,47\n'
        def write(self,data):self.writes.append(data)
        def close(self):self.closed=True
    serial=FakeSerial()
    motor=MotorControl('MOCK',serial_factory=lambda *a,**kw:serial)
    motor.start()
    try:
        deadline=time.monotonic()+4
        while time.monotonic()<deadline and motor.snapshot().raw is None:time.sleep(.02)
        motor.heartbeat();motor.calibrate();motor.set_target(10,time.monotonic());motor.arm(150)
        time.sleep(.1)
        assert any(int(command.split()[0])>0 for command in serial.writes)
    finally:
        motor.close();motor.thread.join(2)
    assert not motor.thread.is_alive() and serial.closed and serial.writes[-1]==b'0 0\n'


def test_fresh_lane_missing_creeps_then_recovers_with_ramp():
    motor = prepared()
    motor.arm(180)
    for _ in range(36):
        motor.command(time.monotonic())
    assert motor.state.drive == 180
    motor.set_target(None, time.monotonic(), lane_missing=True)
    steer, drive = map(int, motor.command(time.monotonic()).split())
    assert motor.state.armed and motor.state.target == 0
    assert steer == 0 and drive == 150
    motor.set_target(10, time.monotonic())
    _, drive = map(int, motor.command(time.monotonic()).split())
    assert drive == 155 and not motor.state.lane_missing


@pytest.mark.parametrize('throttle,expected', [(0, 0), (150, 150), (230, 150)])
def test_lane_missing_respects_selected_throttle_and_explicit_arm(throttle, expected):
    motor = prepared()
    motor.set_target(None, time.monotonic(), lane_missing=True)
    assert motor.command(time.monotonic()) == b'0 0\n'
    motor.arm(throttle)
    for _ in range(30):
        motor.command(time.monotonic())
    assert int(motor.command(time.monotonic()).split()[1]) == expected


@pytest.mark.parametrize('field', ['heartbeat_stamp', 'feedback_stamp', 'target_stamp'])
def test_lane_missing_still_stops_on_stale_inputs(field):
    motor = prepared()
    motor.set_target(None, time.monotonic(), lane_missing=True)
    motor.arm(150)
    setattr(motor.state, field, time.monotonic()-1)
    assert motor.command(time.monotonic()) == b'0 0\n'
    assert not motor.state.armed
    motor.set_target(None, time.monotonic(), lane_missing=True)
    assert motor.command(time.monotonic()) == b'0 0\n'


def test_invalid_numeric_target_does_not_enable_creep():
    motor = prepared()
    motor.arm(150)
    motor.set_target(float('nan'), time.monotonic(), lane_missing=True)
    assert motor.command(time.monotonic()) == b'0 0\n'
    assert not motor.state.armed


def test_drive_pwm_range_matches_reference_controller():
    motor = prepared()
    motor.arm(230)
    for _ in range(60):
        motor.command(time.monotonic())
    assert motor.state.drive == 230
    motor.disarm()
    for value in (-1, 1, 20, 149, 231, 255):
        with pytest.raises(ValueError):
            motor.arm(value)
    assert motor.command(time.monotonic()) == b'0 0\n'
