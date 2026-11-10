"""공유용 정답 예시. 카메라·LiDAR·검출 결과만 사용하며 위치 정답은 읽지 않습니다."""
import math
import numpy as np
from .contracts import finite, no_path, no_perception


def detector_weights():
    """정답 주행에서는 함께 제공하는 Gazebo용 YOLO 가중치를 사용한다."""
    from .yolo_node import reference_weights
    return reference_weights()


def lane(camera):
    """낮은 카메라의 색 마스크를 지면 좌표로 옮겨 오른쪽 차선 중심을 구한다.

    이 역투영 값은 제공된 Gazebo 카메라 전용입니다. 실제 차에는 재보정이 필요합니다.
    """
    import cv2
    rgb = camera
    if rgb is None or rgb.ndim != 3 or rgb.shape[2] != 3:
        return no_path()
    height, width = rgb.shape[:2]
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)
    white = (hsv[:, :, 1] < 65) & (hsv[:, :, 2] > 170)
    yellow = ((hsv[:, :, 0] > 19) & (hsv[:, :, 0] < 39)
              & (hsv[:, :, 1] > 100) & (hsv[:, :, 2] > 100))
    focal = width / (2 * math.tan(1.2 / 2))
    v, u = np.mgrid[:height, :width]
    vertical = (v - height / 2) / focal
    denominator = math.sin(.12) + vertical * math.cos(.12)
    safe = denominator > .025
    divisor = np.maximum(denominator, .025)
    x = .18 * (math.cos(.12) - vertical * math.sin(.12)) / divisor
    y = -.18 * (u - width / 2) / focal / divisor
    roi = safe & (x > .55) & (x < 2.5)

    # 각 거리 구간의 중앙값을 써서 점선의 빈칸과 가로 정지선 영향을 줄입니다.
    centers = []
    for mask, offset, limits in [(white, .75, (-1.7, .15)),
                                  (yellow, -.75, (-.35, 1.8))]:
        selected = roi & mask & (y > limits[0]) & (y < limits[1])
        samples = []
        for lower in np.arange(.55, 2.4, .15):
            band = selected & (x >= lower) & (x < lower + .15)
            if np.count_nonzero(band) >= 3:
                samples.append((float(np.median(x[band])), float(np.median(y[band]))))
        if len(samples) < 3:
            continue
        points = np.asarray(samples)
        coefficients = np.polyfit(points[:, 0], points[:, 1], min(2, len(points) - 1))
        # 차선 폭은 1.5m. 곡선의 법선 방향 폭을 근사해 경계에서 중심으로 옮깁니다.
        lookahead = 1.25
        slope = float(np.polyval(np.polyder(coefficients), lookahead))
        center = float(np.polyval(coefficients, lookahead)) + offset * math.sqrt(1 + slope * slope)
        if abs(center) < 1.2:
            centers.append(center)
    if not centers:
        return no_path()
    return {'valid': True, 'error': float(np.median(centers))}


def _cone_follow(ranges, angle_min, angle_increment, range_min, range_max):
    """연속 LiDAR 점을 라바콘 후보로 묶고 앞쪽 좌우 후보의 중간을 따라간다."""
    values = np.asarray(ranges, dtype=float)
    if values.ndim != 1 or values.size < 5:
        return no_path()
    angles = angle_min + np.arange(values.size) * angle_increment
    angles = np.arctan2(np.sin(angles), np.cos(angles))
    valid = np.isfinite(values) & (values > range_min) & (values < range_max)
    valid &= np.abs(angles) < math.radians(80)
    clusters, current = [], []
    for index in range(values.size):
        if valid[index]:
            point = np.array([values[index] * math.cos(angles[index]),
                              values[index] * math.sin(angles[index])])
            if current and np.linalg.norm(point - current[-1]) > .25:
                clusters.append(np.asarray(current))
                current = []
            current.append(point)
        elif current:
            clusters.append(np.asarray(current))
            current = []
    if current:
        clusters.append(np.asarray(current))
    cones = []
    for cluster in clusters:
        center = cluster.mean(axis=0)
        diameter = np.linalg.norm(cluster[-1] - cluster[0])
        if .2 < center[0] < 3.0 and .18 < abs(center[1]) < 1.3 and diameter < .40:
            cones.append(center)
    left = [p for p in cones if p[1] > 0]
    right = [p for p in cones if p[1] < 0]
    pairs = [(a + b) / 2 for a in left for b in right
             if abs(a[0] - b[0]) < .55 and .8 < a[1] - b[1] < 1.7]
    if not pairs:
        return no_path()
    pairs.sort(key=lambda point: abs(point[0] - 1.0))
    target = pairs[0]
    # 앞쪽 목표점의 옆 방향 오차. 최종 조향은 통합 담당의 PID가 계산합니다.
    return {'valid': True, 'error': float(target[1])}


def _receive_detections(packet):
    """YOLO/시뮬레이터 색 검출기 모두 같은 결과 형식으로 받는다."""
    if not isinstance(packet, dict) or packet.get('valid', True) is not True:
        return no_perception()
    width, height = finite(packet.get('width')), finite(packet.get('height'))
    if width <= 0 or height <= 0 or not isinstance(packet.get('detections'), list):
        return no_perception()
    found = []
    result = no_perception()
    cones = result['rubbercone']
    for detection in packet['detections']:
        if not isinstance(detection, dict) or finite(detection.get('confidence')) < .35:
            continue
        bbox = detection.get('bbox', [])
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            continue
        x1, y1, x2, y2 = [finite(value) for value in bbox]
        x1, x2 = max(0, x1), min(width, x2)
        y1, y2 = max(0, y1), min(height, y2)
        if x2 <= x1 or y2 <= y1:
            continue
        label = detection.get('label')
        if label == 'cone':
            cones['bbox'].append([x1, y1])
            cones['size'].append([x2 - x1, y2 - y1])
            cones['conf'].append(finite(detection.get('confidence')))
        elif (label in ('red', 'yellow', 'green') and finite(detection.get('confidence')) >= .5
              and (y1 + y2) / 2 < height * .5
              and .5 <= (x2 - x1) / (y2 - y1) <= 1.9
              and (y2 - y1) / height <= .2):
            found.append({'detected': True, 'color': label, 'bbox': [x1, y1],
                          'size': [x2 - x1, y2 - y1],
                          'conf': finite(detection.get('confidence'))})
    # 빨강/노랑 정지 신호를 우선합니다. 미검출을 초록으로 판단하지 않습니다.
    found.sort(key=lambda item: ({'red': 2, 'yellow': 1, 'green': 0}[item['color']],
                                item['size'][0] * item['size'][1]), reverse=True)
    if found:
        result['traffic_light'] = found[0]
    cones['detected'] = bool(cones['bbox'])
    return result


def rubbercone(lidar):
    return _cone_follow(lidar, 0.0, math.pi / 180, .15, 12.0)


def yolo(labels, bbox, size, conf):
    packet = {'width': 320, 'height': 240, 'detections': []}
    for label, (x, y), (width, height), confidence in zip(labels, bbox, size, conf):
        packet['detections'].append({'label': label, 'bbox': [x, y, x + width, y + height],
                                     'confidence': confidence})
    return _receive_detections(packet)


class DrivingController:
    """인지 결과의 PID 조향과 빨강 정지 → 초록 출발 상태를 담당한다."""
    def __init__(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.mode = 'lane'
        self.stopped_at_signal = False
        self.passed_signal = False
        self.signal_missing = 0.0
        self.cone_missing = 10.0
        self.previous_time = None

    def step(self, lane_result, rubbercone_result, yolo_result):
        import time
        now = time.monotonic()
        dt = now - self.previous_time if self.previous_time is not None else .05
        self.previous_time = now
        return self._step(lane_result, rubbercone_result, yolo_result, dt)

    def _step(self, lane, cone, perception, dt):
        dt = min(.2, max(.01, finite(dt, .05)))
        # driving.py가 센서 수신 시각을 확인합니다. 한 입력이라도 끊기면 멈춥니다.
        if any(packet is None or packet.get('stale') for packet in (lane, cone, perception)):
            return [0, 0]
        light = perception['traffic_light']
        signal = light['color'] if light['detected'] else 'unknown'
        signal_height = finite(light['size'][1]) / 240
        if self.passed_signal:
            self.signal_missing = self.signal_missing + dt if signal == 'unknown' else 0.0
            if self.signal_missing > 2.0:
                self.passed_signal = False
        if not self.passed_signal and signal in ('red', 'yellow'):
            # 지름 0.22m인 제공 신호등이 약 2.3m 앞으로 다가오면 멈춥니다.
            if signal_height >= .085:
                self.stopped_at_signal = True
        if self.stopped_at_signal:
            if signal == 'green':
                self.stopped_at_signal = False
                self.passed_signal = True
                self.signal_missing = 0.0
            else:
                self.integral = 0.0
                return [0, 0]

        if cone.get('valid'):
            self.cone_missing = 0.0
        else:
            self.cone_missing += dt
        new_mode = 'cone' if cone.get('valid') else 'lane'
        chosen = cone if new_mode == 'cone' else lane
        if not chosen.get('valid'):
            return [0, 0]
        error = max(-1.2, min(1.2, finite(chosen.get('error'))))
        if new_mode != self.mode:
            self.previous_error, self.integral = error, 0.0
        self.mode = new_mode
        self.integral = max(-.4, min(.4, self.integral + error * dt))
        derivative = max(-2, min(2, (error - self.previous_error) / dt))
        self.previous_error = error
        steering = round(max(-30, min(30, 35 * error + 1.5 * self.integral + 1.2 * derivative)))
        pwm = 95 if self.mode == 'cone' else 125
        if abs(steering) > 15:
            pwm = min(pwm, 85)
        # 빨강·노랑이 가까워지면 먼저 감속해, 높인 주행 속도에서도 부드럽게 멈춘다.
        if (not self.passed_signal and signal in ('red', 'yellow')
                and signal_height >= .055):
            pwm = min(pwm, 70)
        return [int(steering), int(pwm)]
