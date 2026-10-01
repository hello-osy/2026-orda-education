"""ORDA 2회차 교육 · 자율주행 코드 한 파일로 읽기.

1. 이미지 전처리 & 차선 검출: 사진에서 도로와 차선을 찾습니다.
2. 주행 오차 계산: 차선 위치의 차이를 목표 바퀴 각도로 바꿉니다.
3. 조향 안정화: 목표 각도와 실제 각도를 비교해 모터 힘을 계산합니다.
4. 전체 코드: 위 계산을 연결하고 모터 보드와 정보를 주고받습니다.

이 파일에는 차선 찾기, 방향 계산, PID, 모터 통신 코드가 들어 있습니다.
PID는 원하는 방향과 현재 방향의 차이를 줄이는 계산 방법입니다.
PWM은 모터에 전달하는 힘의 크기를 나타내는 숫자입니다.
GUI에서는 이 파일을 편집하고 저장하며, 자동으로 실행하지 않습니다.
신경망 구조와 학습된 차선 모델은 프로젝트의 session_1 자료를 사용합니다.
"""

# ========================================
# 준비. 필요한 도구 가져오기
# ========================================
from collections import deque
import copy
from pathlib import Path
import json
import math
import threading
import time
import cv2
import numpy as np
import torch
from session_1.vendor.pidnet import get_pred_model

# ========================================
# 1. 이미지 전처리 & 차선 검출
# ========================================
PALETTE = np.array([
    [30, 30, 30],     # 배경
    [235, 110, 60],   # 도로
    [40, 40, 235],    # 실선
    [0, 195, 255],    # 점선
    [70, 215, 40],    # 초록색 바닥
    [235, 220, 0],    # 밝은 회색 영역
], dtype=np.uint8)


# ========================================
# 1-1. 차선 모델 준비하기
# ========================================
class Segmenter:

    def __init__(self, model_path='', device='auto'):
        project = None
        for folder in Path(__file__).resolve().parents:
            if (folder / 'src/session_1/models').is_dir():
                project = folder
                break
        if project is None:
            raise FileNotFoundError('프로젝트의 src/session_1/models 폴더를 찾을 수 없습니다.')
        local = project / 'src/session_1/models'
        if model_path:
            path = Path(model_path).expanduser()
        else:
            path = local / 'lane_pidnet_s.pt'
        info = json.loads((path.parent / 'dataset_info.json').read_text(encoding='utf-8'))
        self.class_names = info['class_names']
        expected = ['background', 'road', 'lane_solid', 'lane_dashed', 'green_mat', 'light_gray']
        if self.class_names != expected:
            raise ValueError('이 수업은 6클래스 PIDNet 모델을 사용합니다. dataset_info.json 확인')
        if device == 'auto':
            if torch.cuda.is_available():
                device = 'cuda'
            elif torch.backends.mps.is_available():
                device = 'mps'
            else:
                device = 'cpu'
        self.device = device
        torch.set_num_threads(min(4, torch.get_num_threads()))
        self.model = get_pred_model('pidnet-s', len(self.class_names))
        state = torch.load(path, map_location='cpu', weights_only=True)
        state = state.get('state_dict', state)
        cleaned = {}
        for key, value in state.items():
            for prefix in ('module.', 'model.'):
                if key.startswith(prefix):
                    key = key[len(prefix):]
            cleaned[key] = value
        wanted = self.model.state_dict()
        missing = []
        for key, value in wanted.items():
            if key not in cleaned or cleaned[key].shape != value.shape:
                missing.append(key)
        if missing:
            raise ValueError(f'가중치 불일치: {missing[:5]}')
        weights = {}
        for key in wanted:
            weights[key] = cleaned[key]
        self.model.load_state_dict(weights, strict=True)
        self.model.to(device)
        self.model.eval()

    def predict(self, frame):
        # 이미지 전처리 & 차선 검출 ② 색 값 정리하기: BGR → RGB → 학습 때의 숫자 범위
        rgb = frame[:, :, ::-1].astype(np.float32) / 255.0
        rgb = (rgb - np.array([0.485, 0.456, 0.406], np.float32)) / np.array([0.229, 0.224, 0.225], np.float32)
        # 이미지 전처리 & 차선 검출 ③ 모델에 넣기: [색상, 세로, 가로] 순서의 사진 1장 묶음
        channels = rgb.transpose(2, 0, 1).copy()
        tensor = torch.from_numpy(channels)
        tensor = tensor.unsqueeze(0)
        tensor = tensor.to(self.device)
        with torch.inference_mode():
            logits = self.model(tensor)
            logits = torch.nn.functional.interpolate(logits, size=frame.shape[:2], mode='bilinear', align_corners=True)
            # 이미지 전처리 & 차선 검출 ④ 차선 번호 고르기: 실선 2, 점선 3
            labels = logits.argmax(1)[0]
            labels = labels.cpu().numpy()
            return labels.astype(np.uint8)


# ========================================
# 2. 주행 오차 계산
# ========================================
class ScanResult:

    def __init__(self, y, target_x, measured_x, error_px, mask, candidates):
        self.y = y
        self.target_x = target_x
        self.measured_x = measured_x
        self.error_px = error_px
        self.mask = mask
        self.candidates = candidates

def scan_line(labels, y_ratio=0.75, target_ratio=0.79, lane='right', band=5):
    """실선/점선을 섞지 않고 연속 픽셀 덩어리별 중심을 구한다.

    right: 오른쪽 절반의 실선 중 가장 오른쪽 덩어리.
    center: 중앙 점선 중 목표 x에 가장 가까운 덩어리.
    미검출은 None이며, 정상 오차 0과 구분한다. 자동 기준 전환은 하지 않는다.
    """
    h, w = labels.shape
    if not 0 <= y_ratio <= 1 or not 0 <= target_ratio <= 1 or band < 1:
        raise ValueError('scan_y/target_x는 0~1, band는 1 이상이어야 합니다')
    if lane not in ('right', 'center'):
        raise ValueError('lane: right 또는 center')
    y = round(y_ratio * (h - 1))
    target = target_ratio * (w - 1)
    if lane == 'right':
        lane_number = 2
    else:
        lane_number = 3
    mask = ((labels == lane_number) * 255).astype(np.uint8)
    # 주행 오차 계산 ① 차선 위치 찾기: 사진 아래쪽 75% 지점 주변 5줄 확인
    first_row = max(0, y - band // 2)
    last_row = min(h, y + band // 2 + 1)
    row_count = last_row - first_row
    needed = max(1, row_count // 2 + 1)

    # 절반이 넘는 줄에서 차선이 보이면 그 가로 위치를 기록합니다.
    positions = []
    for x in range(w):
        count = 0
        for row in range(first_row, last_row):
            if mask[row, x] > 0:
                count += 1
        if count >= needed:
            positions.append(x)

    # 붙어 있는 점들을 같은 차선으로 묶습니다.
    groups = []
    group = []
    for x in positions:
        if len(group) > 0 and x > group[-1] + 1:
            groups.append(group)
            group = []
        group.append(x)
    if len(group) > 0:
        groups.append(group)

    centers = []
    for group in groups:
        if len(group) >= 2:
            center = sum(group) / len(group)
            centers.append(center)
    candidates = tuple(centers)
    # 주행 오차 계산 ② 기준 차선 고르기: 기본값은 오른쪽 실선
    measured = None
    for center in candidates:
        if lane == 'right':
            if center < w / 2:
                continue
            if measured is None or center > measured:
                measured = center
        elif measured is None or abs(center - target) < abs(measured - target):
            measured = center
    # 주행 오차 계산 ③ 거리 차이 구하기: 목표 위치 − 차선 위치
    error = None
    if measured is not None:
        error = target - measured
    return ScanResult(y, target, measured, error, mask, candidates)


# ========================================
# 3. 조향 안정화
# ========================================
class PIDResult:

    def __init__(self, target=None, angle=None, error=None,
                 p=0.0, i=0.0, d=0.0, output=0, raw=0.0, dt=0.0,
                 status='RESET'):
        self.target = target
        self.angle = angle
        self.error = error
        self.p = p
        self.i = i
        self.d = d
        self.output = output
        self.raw = raw
        self.dt = dt
        self.status = status

class PID:
    """원본 시간주행 PID: P=6.5, I=0, D=0.8, PWM 상한150/최소40."""

    def __init__(self, kp=6.5, ki=0.0, kd=0.8):
        for gain in (kp, ki, kd):
            if not math.isfinite(gain) or gain < 0:
                raise ValueError('PID 설정값은 0 이상의 유한한 숫자여야 합니다')
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.stamp = None
        self.target = 0.0

    def update(self, target, angle, velocity, stamp):
        if target is None or angle is None:
            self.reset()
            return PIDResult(target=target, angle=angle, status='MISSING LANE / FEEDBACK')
        for value in (target, angle, velocity, stamp):
            if not math.isfinite(value):
                self.reset()
                return PIDResult(status='INVALID INPUT')
        if self.stamp is None:
            gap = 0.05
        else:
            gap = stamp - self.stamp
        if gap <= 0 or gap > 1.0:
            self.reset()
            gap = 0.05
        dt = max(0.001, min(gap, 0.1))
        self.stamp = stamp
        target = float(np.clip(target, -45, 45))
        # 목표가 바뀌어 오차 방향이 뒤집히면 누적 오차를 지웁니다.
        if (self.target - angle) * (target - angle) < 0:
            self.integral = 0.0
        self.target = target
        # 조향 안정화 ① 각도 차이 구하기: 목표 각도 − 실제 각도
        error = target - angle
        if abs(error) <= 1.0:
            self.integral = 0.0
            return PIDResult(target=target, angle=angle, error=error, dt=dt, status='WITHIN 1 DEG')
        # 조향 안정화 ② P와 I 계산하기: 현재 오차와 쌓인 오차를 반영 (기본 I는 0)
        candidate = self.integral + error * dt
        if self.ki > 0:
            candidate = float(np.clip(candidate, -30 / self.ki, 30 / self.ki))
        else:
            candidate = 0.0
        p = self.kp * error
        # 조향 안정화 ③ D로 흔들림 줄이기: 실제 바퀴 각속도 × −0.8
        d = -self.kd * velocity
        # 이미 최대 힘으로 돌고 있다면 같은 방향의 오차를 더 쌓지 않습니다.
        candidate_output = p + self.ki * candidate + d
        if not (abs(candidate_output) > 150 and candidate_output * error > 0):
            self.integral = candidate
        i = self.ki * self.integral
        raw = p + i + d
        # 조향 안정화 ④ 힘 제한하기: −150~150, 움직일 때 최소 크기 40
        output = float(np.clip(raw, -150, 150))
        if 0 < abs(output) < 40:
            if output > 0:
                output = 40.0
            else:
                output = -40.0
        status = 'TRACKING'
        if abs(raw) > 150:
            status = 'SATURATED'
        return PIDResult(target, angle, error, p, i, d, int(round(output)), raw, dt, status)


# ========================================
# 4. 전체 코드 · 차선 검출과 주행 오차 연결
# ========================================
class DrivingPipeline:

    def __init__(self, device='auto'):
        self.segmenter = Segmenter(device=device)

    def process(self, frame, stamp, scan_y=0.75, target_x=0.79):
        # 이미지 전처리 & 차선 검출 ① 크기 맞추기: 가로 640, 세로 352
        image = cv2.resize(frame, (640, 352))
        labels = self.segmenter.predict(image)
        # 주행 오차 계산 ①~③: 차선 위치 → 기준 차선 → 가로 오차
        scan = scan_line(labels, scan_y, target_x)
        # 주행 오차 계산 ④ 목표 각도 정하기: −오차 ÷ 130 × 45
        if scan.error_px is None:
            target = None
        else:
            target = float(np.clip(-scan.error_px / 130 * 45, -45, 45))
        overlay = cv2.addWeighted(image, 0.35, PALETTE[labels], 0.65, 0)
        cv2.line(overlay, (0, scan.y), (639, scan.y), (255, 220, 0), 2)
        cv2.line(overlay, (round(scan.target_x), 0), (round(scan.target_x), 351), (0, 150, 255), 2)
        if scan.measured_x is not None:
            cv2.circle(overlay, (round(scan.measured_x), scan.y), 7, (0, 255, 255), -1)
        return {'frame': image, 'overlay': overlay, 'target': target, 'stamp': stamp, 'error_px': scan.error_px}


# ========================================
# 4-1. 모터 보드에서 실제 바퀴 위치 읽기
# ========================================
def parse_rotation(line):
    parts = line.strip().split(',')
    if len(parts) != 4 or parts[0] != 'R':
        return None
    try:
        raw = int(parts[1])
        volts = float(parts[2])
        percent = int(parts[3])
    except ValueError:
        return None
    if 0 <= raw <= 1023 and math.isfinite(volts) and (0 <= percent <= 100):
        return raw
    else:
        return None


# ========================================
# 4-2. 현재 연결 상태와 바퀴 정보 보관하기
# ========================================
class ControlState:

    def __init__(self):
        self.connected = False
        self.armed = False
        self.raw = None
        self.center = None
        self.angle = None
        self.velocity = 0.0
        self.target = None
        self.lane_missing = False
        self.target_stamp = 0.0
        self.feedback_stamp = 0.0
        self.heartbeat_stamp = 0.0
        self.steer = 0
        self.drive = 0
        self.reason = '연결 대기'


# ========================================
# 4-3. 조향 안정화 결과를 모터 보드에 보내기
# ========================================
class MotorControl:
    """화면과 따로 모터를 확인합니다. 새 정보가 끊기면 모터를 멈춥니다."""

    def __init__(self, port, raw_span=80, gains=(6.5, 0.0, 0.8), serial_factory=None):
        self.port = port
        self.raw_span = raw_span
        self.pid = PID(gains[0], gains[1], gains[2])
        self.state = ControlState()
        self.drive_pwm = 0
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.samples = deque(maxlen=5)
        self.serial_factory = serial_factory
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
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
            # 계산 완료 시간이 아니라 사진을 받은 시간을 기록합니다.
            self.state.target_stamp = camera_stamp

    def receive(self, raw, now):
        with self.lock:
            s = self.state
            self.samples.append(raw)
            raw = int(np.median(self.samples))
            if s.center is None:
                angle = None
            else:
                angle = float(np.clip((s.center - raw) * 45 / self.raw_span, -45, 45))
            dt = now - s.feedback_stamp
            if angle is not None and s.angle is not None and (0 < dt <= 0.5):
                s.velocity = 0.25 * (angle - s.angle) / dt + 0.75 * s.velocity
            else:
                s.velocity = 0.0
            s.raw = raw
            s.angle = angle
            s.feedback_stamp = now

    def calibrate(self):
        with self.lock:
            s = self.state
            if s.armed or s.raw is None or time.monotonic() - s.feedback_stamp > 0.5:
                raise ValueError('출력 중지 상태에서 최신 R 피드백이 필요합니다.')
            s.center = s.raw
            s.angle = 0.0
            s.velocity = 0.0
            self.pid.reset()
            s.reason = f'중앙 보정 완료: raw {s.center}'

    # 연결이나 새 정보가 끊기면 모터를 멈출 이유를 돌려줍니다.
    def fault(self, now):
        s = self.state
        if not s.connected:
            return 'Arduino 연결 없음'
        if s.center is None:
            return '바퀴를 중앙에 놓고 중앙 보정 필요'
        if now - s.heartbeat_stamp > 0.3:
            return 'GUI heartbeat 끊김'
        if now - s.feedback_stamp > 0.5:
            return '조향 피드백 끊김'
        if s.target is None or not math.isfinite(s.target):
            return '유효한 추론 결과 없음'
        if now - s.target_stamp > 0.5:
            return '카메라/segmentation 지연 또는 끊김'
        return ''

    # 연결만으로 출발하지 않고, 이 함수를 호출해야 모터 출력을 허용합니다.
    def arm(self, drive_pwm):
        with self.lock:
            reason = self.fault(time.monotonic())
            if reason:
                raise ValueError(reason)
            if drive_pwm != 0 and not 150 <= drive_pwm <= 230:
                raise ValueError('전진 PWM은 150~230입니다. 0은 전진 정지입니다.')
            self.drive_pwm = int(drive_pwm)
            self.state.armed = True
            self.state.reason = '실제 출력 중'
            self.pid.reset()

    def disarm(self, reason='사용자 정지'):
        with self.lock:
            self.state.armed = False
            self.state.steer = 0
            self.state.drive = 0
            self.state.reason = reason
            self.pid.reset()

    def command(self, now):
        with self.lock:
            s = self.state
            if s.armed:
                reason = self.fault(now)
                if reason:
                    self.disarm(reason + ' · 다시 주행 시작 필요')
                else:
                    # 조향 안정화: 목표 각도와 실제 각도를 비교해 모터 힘 계산
                    result = self.pid.update(s.target, s.angle, s.velocity, now)
                    s.steer = result.output
                    # 전진 힘은 한 번에 5씩 높이고, 정지할 때는 즉시 0으로 만듭니다.
                    limit = self.drive_pwm
                    s.reason = '차선 따라 주행 중'
                    if s.lane_missing:
                        limit = min(limit, 150)
                        s.reason = '차선 미검출 · 중앙 조향 · 저속 전진'
                        if limit == 0:
                            s.reason = '차선 미검출 · 중앙 조향만 · 전진 0'
                    s.drive = min(limit, s.drive + 5)
            steer = 0
            drive = 0
            if s.armed:
                steer = s.steer
                drive = s.drive
            message = str(steer) + ' ' + str(drive) + '\n'
            return message.encode('ascii')

    def snapshot(self):
        with self.lock:
            return copy.copy(self.state)

    def close(self):
        self.disarm('연결 종료')
        self.stop_event.set()

    # 0.05초마다 모터 명령을 보내고 바퀴 위치를 읽습니다.
    def run(self):
        import serial
        ser = None
        try:
            if self.serial_factory is None:
                factory = serial.Serial
            else:
                factory = self.serial_factory
            ser = factory(self.port, 115200, timeout=0.02, write_timeout=0.1)
            ser.write(b'0 0\n')
            # 보드가 켜지는 2초 동안은 모터를 움직이지 않습니다.
            self.stop_event.wait(2.0)
            with self.lock:
                self.state.connected = not self.stop_event.is_set()
                self.state.reason = '연결됨 · R 피드백과 중앙 보정을 기다려요'
            buffer = bytearray()
            next_send = 0.0
            while not self.stop_event.is_set():
                now = time.monotonic()
                if now >= next_send:
                    ser.write(self.command(now))
                    next_send = now + 0.05
                waiting = min(ser.in_waiting, 4096)
                if waiting:
                    buffer.extend(ser.read(waiting))
                    while b'\n' in buffer:
                        line, _, buffer = buffer.partition(b'\n')
                        raw = parse_rotation(line.decode('ascii', errors='replace'))
                        if raw is not None:
                            self.receive(raw, time.monotonic())
                    if len(buffer) > 4096:
                        buffer.clear()
                self.stop_event.wait(0.005)
        except Exception as exc:
            self.disarm('시리얼 오류: ' + str(exc))
        finally:
            if ser is not None:
                try:
                    ser.write(b'0 0\n')
                except Exception:
                    pass
                ser.close()
            with self.lock:
                self.state.connected = False
                self.state.armed = False
                self.state.steer = 0
                self.state.drive = 0
