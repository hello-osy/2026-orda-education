"""같이 읽는 실제 제어 코드: 카메라 → PIDNet → scan line → PID → Arduino.

참고: hello-osy/ai-autonomous-driving-competition-2026의 sensor_topic 및
integrated_ardunio_code. Serial 115200, TX '<steer PWM> <drive PWM>\n',
RX 'R,<raw>,<voltage>,<percent>\n'. 실제 모터 연결은 GUI에서만 명시적으로 시작.
"""
from collections import deque
from dataclasses import dataclass
import math
import threading
import time

import cv2
import numpy as np
from .inference_runtime import inference_guard
from .learning import choose_device
from session_1.pid_view import PID
from session_1.scan_line_view import scan_line
from session_1.segmentation_view import PALETTE, Segmenter


# 1. 인지: session_1과 같은 segmentation 가중치로 실시간 프레임을 분류한다.
class DrivingPipeline:
    def __init__(self, device='auto'):
        self.device = choose_device(device)
        with inference_guard(self.device):
            self.segmenter = Segmenter(device=self.device)

    def process(self, frame, stamp, scan_y=.75, target_x=.79):
        image = cv2.resize(frame, (640,352))
        with inference_guard(self.device):
            labels = self.segmenter.predict(image)
        # 2. 판단: 오른쪽 실선의 픽셀 오차를 목표 조향각으로 변환한다.
        scan = scan_line(labels, scan_y, target_x)
        target = None if scan.error_px is None else float(np.clip(-scan.error_px/130*45,-45,45))
        overlay = cv2.addWeighted(image,.35,PALETTE[labels],.65,0)
        segmentation = overlay.copy()
        cv2.line(overlay,(0,scan.y),(639,scan.y),(255,220,0),2)
        cv2.line(overlay,(round(scan.target_x),0),(round(scan.target_x),351),(0,150,255),2)
        if scan.measured_x is not None:
            cv2.circle(overlay,(round(scan.measured_x),scan.y),7,(0,255,255),-1)
        return {'frame':image,'segmentation':segmentation,'overlay':overlay,'target':target,
                'stamp':stamp,'error_px':scan.error_px,'scan':scan}


def parse_rotation(line):
    parts = line.strip().split(',')
    if len(parts)!=4 or parts[0]!='R':
        return None
    try:
        raw, volts, percent = int(parts[1]),float(parts[2]),int(parts[3])
    except ValueError:
        return None
    return raw if 0<=raw<=1023 and math.isfinite(volts) and 0<=percent<=100 else None


@dataclass
class ControlState:
    connected: bool = False
    armed: bool = False
    raw: int | None = None
    center: int | None = None
    angle: float | None = None
    velocity: float = 0.
    target: float | None = None
    lane_missing: bool = False
    target_stamp: float = 0.
    feedback_stamp: float = 0.
    heartbeat_stamp: float = 0.
    steer: int = 0
    drive: int = 0
    p: float = 0.
    i: float = 0.
    d: float = 0.
    sent_command: str = ''
    sent_stamp: float = 0.
    reason: str = '연결 대기'


class MotorControl:
    """별도 I/O 스레드의 20Hz 제어. UI/추론 지연 중에도 stale 입력은 정지한다."""
    def __init__(self, port, raw_span=80, gains=(6.5,0.,.8), serial_factory=None):
        self.port = port
        self.raw_span = raw_span
        self.pid = PID(*gains)
        self.state = ControlState()
        self.drive_pwm = 0  # 연결만으로 구동하지 않는다.
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.samples = deque(maxlen=5)
        self.serial_factory = serial_factory
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.run,daemon=True)
        self.thread.start()

    def heartbeat(self):
        with self.lock:
            self.state.heartbeat_stamp = time.monotonic()

    def set_target(self, target, camera_stamp, lane_missing=False):
        with self.lock:
            # 새 영상에서 차선만 없는 경우를 입력 끊김과 구분합니다.
            missing = lane_missing and target is None
            if missing != self.state.lane_missing:
                self.pid.reset()
            self.state.lane_missing = missing
            if missing:
                target = 0.0  # 바퀴를 중앙으로 맞추며 천천히 전진합니다.
            self.state.target = target
            # 추론 완료 시각이 아니라 원본 카메라 프레임 수신 시각을 쓴다.
            self.state.target_stamp = camera_stamp

    def receive(self, raw, now):
        with self.lock:
            s = self.state
            self.samples.append(raw)
            raw = int(np.median(self.samples))
            angle = None if s.center is None else float(np.clip((s.center-raw)*45/self.raw_span,-45,45))
            dt = now-s.feedback_stamp
            if angle is not None and s.angle is not None and 0<dt<=.5:
                s.velocity = .25*(angle-s.angle)/dt+.75*s.velocity
            else:
                s.velocity = 0.
            s.raw,s.angle,s.feedback_stamp = raw,angle,now

    def calibrate(self):
        with self.lock:
            s = self.state
            if s.armed or s.raw is None or time.monotonic()-s.feedback_stamp>.5:
                raise ValueError('출력 중지 상태에서 최신 R 피드백이 필요합니다.')
            s.center,s.angle,s.velocity = s.raw,0.,0.
            self.pid.reset()
            s.reason = f'중앙 보정 완료: raw {s.center}'

    def fault(self, now):
        s = self.state
        if not s.connected: return 'Arduino 연결 없음'
        if s.center is None: return '바퀴를 중앙에 놓고 중앙 보정 필요'
        if now-s.heartbeat_stamp>.3: return 'GUI heartbeat 끊김'
        if now-s.feedback_stamp>.5: return '조향 피드백 끊김'
        if s.target is None or not math.isfinite(s.target): return '유효한 추론 결과 없음'
        if now-s.target_stamp>.5: return '카메라/segmentation 지연 또는 끊김'
        return ''

    def arm(self, drive_pwm):
        with self.lock:
            reason = self.fault(time.monotonic())
            if reason:
                raise ValueError(reason)
            if drive_pwm != 0 and not 150<=drive_pwm<=230:
                raise ValueError('전진 PWM은 150~230입니다. 0은 전진 정지입니다.')
            self.drive_pwm = int(drive_pwm)
            self.state.armed = True
            self.state.reason = '실제 출력 중'
            self.pid.reset()

    def disarm(self, reason='사용자 정지'):
        with self.lock:
            self.state.armed = False
            self.state.steer = self.state.drive = 0
            self.state.p = self.state.i = self.state.d = 0.
            self.state.reason = reason
            self.pid.reset()

    # 3. 제어: 목표각 - 실제각을 PID에 넣고, 실제 조향모터 PWM을 계산한다.
    def command(self, now):
        with self.lock:
            s = self.state
            if s.armed:
                reason = self.fault(now)
                if reason:
                    self.disarm(reason+' · 다시 주행 시작 필요')
                else:
                    result = self.pid.update(s.target,s.angle,s.velocity,now)
                    s.steer = result.output
                    s.p, s.i, s.d = result.p, result.i, result.d
                    # 전진 PWM은 20Hz마다 5씩 올린다. 정지는 즉시 0이다.
                    limit = self.drive_pwm
                    s.reason = '차선 따라 주행 중'
                    if s.lane_missing:
                        limit = min(limit, 150)
                        s.reason = '차선 미검출 · 중앙 조향 · 저속 전진'
                        if limit == 0:
                            s.reason = '차선 미검출 · 중앙 조향만 · 전진 0'
                    s.drive = min(limit, s.drive+5)
            return f'{s.steer if s.armed else 0} {s.drive if s.armed else 0}\n'.encode('ascii')

    def snapshot(self):
        with self.lock:
            return ControlState(**vars(self.state))

    def close(self):
        self.disarm('연결 종료')
        self.stop_event.set()

    def write_command(self, ser, command):
        written = ser.write(command)
        if written is not None and written != len(command):
            raise IOError('모터 명령을 모두 보내지 못했습니다.')
        with self.lock:
            self.state.sent_command = command.decode('ascii').strip()
            self.state.sent_stamp = time.monotonic()

    # 4. 송신: Arduino가 모터 드라이버에 적용하는 실제 Serial 명령이다.
    def run(self):
        import serial
        ser = None
        try:
            factory = self.serial_factory or serial.Serial
            ser = factory(self.port,115200,timeout=.02,write_timeout=.1)
            self.write_command(ser, b'0 0\n')
            self.stop_event.wait(2.)  # 보드 reset/boot 중에는 주행하지 않는다.
            with self.lock:
                self.state.connected = not self.stop_event.is_set()
                self.state.reason = '연결됨 · R 피드백과 중앙 보정을 기다려요'
            buffer = bytearray()
            next_send = 0.
            while not self.stop_event.is_set():
                now = time.monotonic()
                if now>=next_send:
                    # Serialize disarm with transmission so a prepared old command cannot follow it.
                    with self.lock:
                        self.write_command(ser, self.command(now))
                    next_send = now+.05
                waiting = min(ser.in_waiting,4096)
                if waiting:
                    buffer.extend(ser.read(waiting))
                    while b'\n' in buffer:
                        line,_,buffer = buffer.partition(b'\n')
                        raw = parse_rotation(line.decode('ascii',errors='replace'))
                        if raw is not None:
                            self.receive(raw,time.monotonic())
                    if len(buffer)>4096:
                        buffer.clear()
                self.stop_event.wait(.005)
        except Exception as exc:
            self.disarm('시리얼 오류: '+str(exc))
        finally:
            if ser is not None:
                try:
                    self.write_command(ser, b'0 0\n')
                except Exception:
                    pass
                ser.close()
            with self.lock:
                self.state.connected = False
                self.state.armed = False
                self.state.steer = self.state.drive = 0
