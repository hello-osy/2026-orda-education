#!/usr/bin/env python3
"""가상 시리얼 + 실제 ROS로 조향 전용 노드를 검사한다. 실제 USB는 열지 않는다.

프로젝트 루트에서:
  source scripts/session_3_env.sh 30
  python src/session_3/scripts/check_motor_runtime.py

이 검사는 ±150 짧은 펄스·새 피드백 대기·목표 도달 정지·구동 PWM 차단을 검증한다.
실제 모터 방향, 중앙 센서값, 기구 동작은 검증하지 않는다.
"""
import errno
import json
import os
from pathlib import Path
import re
import select
import signal
import subprocess
import sys
import threading
import time
import tty

ROOT = Path(__file__).resolve().parents[3]
os.environ['ROS_DOMAIN_ID'] = '210'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
OUTPUT = ROOT / 'workspace/session_3/validation/motor_runtime'
OUTPUT.mkdir(parents=True, exist_ok=True)
os.environ['ROS_LOG_DIR'] = str(OUTPUT / 'ros_log')
os.environ['PYTHONPATH'] = str(ROOT / 'src/session_3') + os.pathsep + os.environ.get('PYTHONPATH', '')

import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from std_msgs.msg import Int16, Int16MultiArray
from std_srvs.srv import Trigger


class FakeArduino:
    """PTY master에서 피드백을 보내고 노드의 실제 시리얼 송신을 기록한다."""
    def __init__(self):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.port = os.ttyname(self.slave)
        self.feedback = threading.Event()
        self.feedback.set()
        self.stopping = threading.Event()
        self.lines = []
        self.error = None
        self.last_feedback = None
        self.raw = 512
        self.feedback_period = .2  # 실제 차량에서 확인한 약 5Hz R 피드백을 재현한다.
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        buffer = b''
        next_feedback = 0.0
        try:
            while not self.stopping.is_set():
                now = time.monotonic()
                if self.feedback.is_set() and now >= next_feedback:
                    os.write(self.master, f'R,{self.raw},12.0,100\n'.encode('ascii'))
                    self.last_feedback = now
                    next_feedback = now + self.feedback_period
                readable, _, _ = select.select([self.master], [], [], .01)
                if not readable:
                    continue
                try:
                    chunk = os.read(self.master, 4096)
                except OSError as exc:
                    if exc.errno == errno.EIO:
                        continue
                    raise
                buffer += chunk
                while b'\n' in buffer:
                    line, _, buffer = buffer.partition(b'\n')
                    match = re.fullmatch(rb'(-?\d+) (-?\d+)\r?', line)
                    if match:
                        self.lines.append((time.monotonic(), int(match[1]), int(match[2])))
        except Exception as exc:
            self.error = str(exc)

    def after(self, stamp):
        return [line for line in list(self.lines) if line[0] >= stamp]

    def set_raw(self, raw):
        assert isinstance(raw, int) and 0 <= raw <= 1023
        self.raw = raw

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)


class Driver:
    def __init__(self, board, process):
        self.board, self.process = board, process
        self.node = rclpy.create_node('motor_runtime_probe')
        self.publisher = self.node.create_publisher(Int16MultiArray, '/motor_control', 1)
        self.raw_messages = []
        self.node.create_subscription(Int16, '/steering/raw', self.raw_message, 1)
        self.calibration = self.node.create_client(Trigger, '/motor/calibrate')
        self.parameters = self.node.create_client(SetParameters, '/motor_node/set_parameters')
        self.command = None
        self.last_publish = None
        self.node.create_timer(.1, self.publish)
        self.checks = []

    def raw_message(self, msg):
        self.raw_messages.append((time.monotonic(), msg.data))

    def publish(self):
        if self.command is not None:
            message = Int16MultiArray()
            message.data = self.command
            self.publisher.publish(message)
            self.last_publish = time.monotonic()

    def wait(self, predicate, timeout=3.0, label='조건 대기'):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise AssertionError(f'노드가 일찍 종료됨: {self.process.returncode}')
            if self.board.error:
                raise AssertionError(self.board.error)
            rclpy.spin_once(self.node, timeout_sec=.01)
            if predicate():
                return
        raise AssertionError(f'{label}: {timeout}초 안에 조건을 충족하지 못함')

    def spin(self, duration):
        until = time.monotonic() + duration
        self.wait(lambda: time.monotonic() >= until, timeout=duration+.5)

    def passed(self, label):
        self.checks.append(label)
        print(f'PASS {label}', flush=True)

    def moving(self, target, sign):
        stamp = time.monotonic()
        self.command = [target, 0]
        self.wait(lambda: any(line[1] == 150*sign for line in self.board.after(stamp)),
                  label=f'{target:+}도 조향 출력')
        return next(line[0] for line in self.board.after(stamp) if line[1] == 150*sign)

    def zero_after(self, stamp, timeout=.9, quiet=.32):
        # 펄스 사이의 0을 목표 도달/오류 정지로 오인하지 않도록 연속 정지를 확인한다.
        def quiet_output():
            lines = self.board.after(stamp)
            if not lines or lines[-1][1:] != (0, 0):
                return False
            last_nonzero = max((line[0] for line in lines if line[1] != 0), default=stamp)
            return time.monotonic()-last_nonzero >= quiet
        self.wait(quiet_output, timeout=timeout, label='연속 정지 출력')

    def remains_zero(self, duration=.4):
        stamp = time.monotonic()
        self.spin(duration)
        lines = self.board.after(stamp)
        assert lines and all(line[1:] == (0, 0) for line in lines), '같은 목표 반복으로 출력이 재개됨'

    def call_calibrate(self):
        self.wait(self.calibration.service_is_ready, label='보정 서비스 검색')
        future = self.calibration.call_async(Trigger.Request())
        self.wait(future.done, label='보정 응답')
        return future.result()

    def set_sim_time(self, enabled):
        self.wait(self.parameters.service_is_ready, label='파라미터 서비스 검색')
        request = SetParameters.Request()
        request.parameters = [Parameter(name='use_sim_time', value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=enabled))]
        future = self.parameters.call_async(request)
        self.wait(future.done, label='파라미터 응답')
        assert all(result.successful for result in future.result().results)

    def check(self):
        self.command = [10, 0]
        self.wait(lambda: len(self.raw_messages) >= 3 and self.publisher.get_subscription_count() > 0,
                  timeout=8, label='시리얼 및 ROS 연결')
        self.spin(.3)
        assert self.board.lines and all(line[1:] == (0, 0) for line in self.board.lines)
        self.passed('중앙 보정 전에는 조향 및 구동 출력 0')

        self.command = None
        self.spin(.65)
        response = self.call_calibrate()
        assert response.success, response.message
        assert '512' in response.message
        self.passed('실제 ROS 서비스로 중앙 raw=512 보정')

        pulse_start = self.moving(10, 1)
        self.passed('양의 목표 조향각은 +150 PWM 펄스')
        self.board.feedback.clear()
        self.spin(.25)
        assert all(line[1:] == (0, 0) for line in self.board.after(pulse_start+.04)), (
            '새 피드백 없이 펄스를 다시 출력함')
        resumed = time.monotonic()
        self.board.feedback.set()
        self.wait(lambda: any(line[1] == 150 for line in self.board.after(resumed)),
                  label='새 실제 피드백 이후 다음 펄스')
        self.passed('펄스 이후 실제 새 피드백이 없으면 재출력하지 않음')
        response = self.call_calibrate()
        assert not response.success, '명령 발행 중 보정이 허용됨'
        self.passed('명령 수신 중 중앙 재보정 거부')

        # raw_direction=-1, raw_span=80: raw496은 +9도, +10도 목표의 허용 오차 1도.
        self.board.set_raw(496)
        self.zero_after(time.monotonic())
        self.board.set_raw(512)
        self.remains_zero()
        self.passed('목표 1도 이내 도달 후 같은 명령 반복 및 피드백 이탈에도 출력 0')

        self.moving(-10, -1)
        self.passed('음의 새 목표 조향각은 -150 PWM 펄스')
        # raw532는 -11.25도: 허용 오차를 거치지 않고 목표를 지나쳐도 역회전 없이 정지.
        self.board.set_raw(532)
        self.zero_after(time.monotonic())
        self.board.set_raw(512)
        self.remains_zero()
        self.passed('목표를 지나친 피드백에서 즉시 정지하고 자동 역회전하지 않음')

        self.moving(10, 1)
        self.command = [10, 100]
        self.zero_after(time.monotonic())
        self.command = [10, 0]
        self.remains_zero()
        self.passed('속도값이 0이 아닌 명령은 조향도 즉시 중단')
        for target, invalid in zip((11, 12, 13), ([10], [31, 0], [0, 0, 0])):
            self.moving(target, 1)
            self.command = invalid
            self.zero_after(time.monotonic())
            self.command = [target, 0]
            self.remains_zero()
        self.passed('잘못된 배열 길이 및 조향각 범위를 거부')

        self.moving(14, 1)
        self.command = None
        stamp = time.monotonic()
        self.spin(.65)
        self.zero_after(stamp+.55, quiet=.15)
        self.command = [14, 0]
        self.remains_zero()
        self.passed('명령 발행 중단 후 0.9초 이내 출력 0')

        self.moving(15, 1)
        self.board.feedback.clear()
        stamp = time.monotonic()
        self.spin(.65)
        self.zero_after(stamp+.55, quiet=.15)
        count = len(self.raw_messages)
        self.spin(.2)
        assert len(self.raw_messages) <= count + 1, '끊긴 raw 피드백을 계속 발행함'
        self.passed('명령을 계속 보내도 피드백 중단 후 출력 0')
        self.board.feedback.set()
        self.remains_zero()
        self.moving(16, 1)

        self.set_sim_time(True)
        self.spin(.4)
        self.zero_after(time.monotonic())
        self.passed('ROS heartbeat 타이머 정지 시 시리얼 스레드가 출력 차단')
        self.set_sim_time(False)
        self.remains_zero()
        self.passed('명령·피드백·heartbeat 회복 후에도 같은 목표는 자동 재시작하지 않음')

        started = self.moving(17, 1)
        # 10Hz 명령과 최신 R 피드백을 계속 보내지만 센서 위치는 바뀌지 않는다.
        self.spin(5.2)
        assert any(line[1] != 0 for line in self.board.after(started+4.5)), '5초 제한 전에 예기치 않게 중단됨'
        assert all(line[1:] == (0, 0) for line in self.board.after(started+5.1)), '5초 제한 뒤 펄스 출력'
        self.remains_zero()
        self.passed('같은 목표를 10Hz로 보내도 5초 제한 뒤 정지 상태 유지')

        self.moving(18, 1)
        self.command = [0, 0]
        self.zero_after(time.monotonic())
        self.passed('중앙 목표 + 중앙 피드백에서는 조향 PWM 0')
        self.command = None
        assert all(line[2] == 0 for line in self.board.lines), '구동 PWM이 0을 벗어남'
        self.passed('전체 시리얼 송신에서 구동 PWM은 항상 0')
        assert all(line[1] in (-150, 0, 150) for line in self.board.lines), '고정 조향 PWM 이외의 값 송신'
        self.passed('전체 시리얼 송신에서 조향 PWM은 -150 / 0 / +150만 사용')
        pulse_widths = []
        pulse_start = None
        for stamp, steering, _ in self.board.lines:
            if steering and pulse_start is None:
                pulse_start = stamp
            elif not steering and pulse_start is not None:
                pulse_widths.append(stamp-pulse_start)
                pulse_start = None
        assert pulse_start is None and pulse_widths
        assert max(pulse_widths) >= .015, '20ms 설정의 온전한 펄스가 관측되지 않음'
        assert max(pulse_widths) <= .05, f'PTY에서 관측한 펄스가 너무 김: {max(pulse_widths):.4f}초'
        self.passed(f'PTY의 실제 펄스 폭 50ms 이내 (최대 {max(pulse_widths)*1000:.1f}ms)')


def main():
    board = FakeArduino()
    process = driver = None
    log = (OUTPUT / 'motor_node.log').open('w', encoding='utf-8')
    result = {'hardware': 'pseudo terminal only', 'domain': 210, 'success': False}
    try:
        rclpy.init()
        process = subprocess.Popen([
            sys.executable, '-m', 'session_3.motor_node', '--ros-args',
            '-p', f'port:={board.port}', '-p', 'arm:=true',
            '-p', 'auto_calibrate:=false',
        ], cwd=ROOT, env=os.environ.copy(), stdout=log, stderr=subprocess.STDOUT)
        driver = Driver(board, process)
        driver.check()
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        assert process.returncode == 0, f'종료 코드 {process.returncode}'
        time.sleep(.1)
        assert board.lines[-1][1:] == (0, 0), '종료 시 정지 명령 누락'
        driver.passed('SIGINT 종료 시 마지막 시리얼 명령 0 0')
        result['success'] = True
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        if driver is not None:
            result['checks'] = driver.checks
            result['raw_messages'] = len(driver.raw_messages)
            driver.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        result['serial_commands'] = len(board.lines)
        result['nonzero_drive_commands'] = sum(line[2] != 0 for line in board.lines)
        start = board.lines[0][0] if board.lines else 0.0
        trace = [{'time_s': round(stamp-start, 4), 'steering_pwm': steer, 'drive_pwm': drive}
                 for stamp, steer, drive in board.lines]
        (OUTPUT / 'serial_commands.json').write_text(json.dumps(trace, indent=2)+'\n', encoding='utf-8')
        (OUTPUT / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        board.close()
        log.close()
    print(f'결과: {OUTPUT / "result.json"}')


if __name__ == '__main__':
    main()
