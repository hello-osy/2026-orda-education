"""4번 스케일카 전용 osy-260809 주행 파이프라인과 모터 어댑터."""
import time

import cv2
import numpy as np

from .competition_lane import CompetitionLaneTracker
from .driving import MotorControl as BaseMotorControl
from .inference_runtime import inference_guard
from .learning import choose_device
from session_1.segmentation_view import Segmenter, PALETTE

# seg_lane_offset_node.CLASS_TO_COLOR: 실선/점선은 둘 다 흰색으로 합친다.
CONTROL_PALETTE = np.array([
    [0, 0, 0], [105, 105, 105], [255, 255, 255],
    [255, 255, 255], [0, 255, 0], [211, 211, 211],
], dtype=np.uint8)


class DrivingPipeline:
    def __init__(self, device='auto', segmenter=None):
        self.device = choose_device(device)
        self.tracker = CompetitionLaneTracker()
        with inference_guard(self.device):
            self.segmenter = segmenter if segmenter is not None else Segmenter(device=self.device)
            # 원본 seg_lane_offset_node와 같이 첫 추론 지연을 주행 전에 끝낸다.
            for _ in range(3):
                self.segmenter.predict(np.zeros((360, 640, 3), dtype=np.uint8))

    def reset(self):
        self.tracker.reset()

    def process(self, frame, stamp):
        # 원본 camera.yaml의 640×360 좌표계를 사용한다. 1·2번 캡처 설정은 바꾸지 않는다.
        image = cv2.resize(frame, (640, 360))
        with inference_guard(self.device):
            labels = self.segmenter.predict(image)
        result = self.tracker.process(CONTROL_PALETTE[labels])
        result.update(frame=image, stamp=stamp,
                      segmentation=cv2.addWeighted(image, .45, PALETTE[labels], .55, 0))
        return result


class MotorControl(BaseMotorControl):
    """원본 timed 주행의 1초 직진/PID를 기존 GUI의 연결·정지 규약에 연결한다.

    전진 PWM은 GUI의 150~230 설정과 ramp를 유지한다. 자동 중앙 캡처 대신
    사용자가 중앙 보정을 수행하며, 통신/영상 끊김은 기존처럼 출력을 차단한다.
    """
    def __init__(self, *args, initial_straight_duration=1.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.initial_straight_duration = initial_straight_duration
        self.straight_until = 0.0
        self.requested_target = None

    def set_target(self, target, camera_stamp, lane_missing=False):
        with self.lock:
            self.requested_target = target
            # 차선 미검출 HOLD는 tracker가 직전 각도로 전달한다.
            # None은 영상/추론 실패이므로 기존 저속 전진 정책을 적용하지 않는다.
            super().set_target(target, camera_stamp, lane_missing=False)

    def arm(self, drive_pwm):
        with self.lock:
            super().arm(drive_pwm)
            self.straight_until = time.monotonic() + self.initial_straight_duration

    def command(self, now):
        with self.lock:
            self.state.target = self.requested_target
            initial = (self.state.armed and self.requested_target is not None
                       and np.isfinite(self.requested_target) and now < self.straight_until)
            if initial:
                self.state.target = 0.0
            output = super().command(now)
            if initial and self.state.armed:
                self.state.reason = '출발 1초 직진 · 이후 차선 추적'
            return output

    def receive(self, raw, now):
        # 원본 drive_control_node는 median 필터 없이 raw→각도, 각속도 EMA(0.25)를 쓴다.
        with self.lock:
            s = self.state
            angle = None if s.center is None else float(np.clip((s.center-raw)*45/self.raw_span, -45, 45))
            dt = now - s.feedback_stamp
            if angle is not None and s.angle is not None and 0 < dt <= 1.0:
                s.velocity = .25 * (angle-s.angle)/dt + .75 * s.velocity
            else:
                s.velocity = 0.0
            s.raw, s.angle, s.feedback_stamp = raw, angle, now
