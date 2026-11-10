# ========================================
# ========== DO NOT MODIFY ===============
# ===== 토픽 구독·발행 참고 (완성 코드) =====
# ========================================
# 이 영역은 읽고 참고하되 수정하지 않습니다. 수정할 네 역할은 아래 TODO에 있습니다.
# 구독: 토픽으로 들어온 데이터를 지정한 함수(callback)에 전달합니다.
# 발행: DrivingController가 반환한 [조향각, PWM]을 차량에 보냅니다.


def connect_topics(node):
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    from std_msgs.msg import Int16MultiArray, String

    # 발행 준비: /session4/motor_control, 자료형 Int16MultiArray.
    node.publisher = node.create_publisher(Int16MultiArray, '/session4/motor_control', 1)
    # 구독 준비: 자료형, 토픽 이름, 수신 시 실행할 함수, 수신 설정 순서입니다.
    node.sensor_subscriptions = [
        # 낮은 카메라 → 차선용 이미지 / 높은 카메라 → YOLO용 이미지.
        node.create_subscription(Image, '/camera/low/image_raw',
                                 lambda message: node.camera('low', message), qos_profile_sensor_data),
        node.create_subscription(Image, '/camera/high/image_raw',
                                 lambda message: node.camera('high', message), qos_profile_sensor_data),
        # 라이다 → 거리 배열. 센서 수신 설정은 qos_profile_sensor_data를 사용합니다.
        node.create_subscription(LaserScan, '/scan', node.scan, qos_profile_sensor_data),
        # 차량 상태 → 시각화에 표시합니다.
        node.create_subscription(String, '/session4/control/state', node.vehicle_state, 1),
    ]


def publish_motor(node, command):
    from std_msgs.msg import Int16MultiArray

    # command 예: [0, 125] 직진 / [10, 125] 좌회전 / [0, 0] 정지.
    node.publisher.publish(Int16MultiArray(data=command))


# ==============================
# ============ TODO ============
# ==============================
# 네 사람이 아래 네 부분을 나누어 수정합니다. 역할끼리는 반환값만 전달합니다.


def lane(camera):
    # TODO 1: 카메라에서 차로 가운데를 찾습니다.
    # 입력 camera: 240행 × 320열 × RGB 3색의 이미지 배열. camera[y, x]가 한 픽셀입니다.
    # 출력 dict: valid=차선을 찾았는지(bool), error=목표점의 좌우 거리(float, m).
    # error는 왼쪽 +, 오른쪽 -, 가운데 0입니다.
    # 예: {"valid": True, "error": 0.1} → 왼쪽으로 0.1m 떨어진 목표점.
    # 못 찾은 예: {"valid": False, "error": 0.0}
    return {"valid": False, "error": 0.0}


def rubbercone(lidar):
    # TODO 2: 라바콘 두 줄 사이의 가운데를 찾습니다.
    # 입력 lidar: 거리(m) 360개의 배열. lidar[0]=정면, [90]=왼쪽, [180]=뒤, [270]=오른쪽.
    # inf는 측정된 물체가 없다는 뜻입니다. 각도·범위 처리는 공통 코드가 합니다.
    # 출력 dict: valid=길을 찾았는지(bool), error=목표점 좌우 거리(float, m, 왼쪽 +).
    # 예: {"valid": True, "error": -0.1}
    # 못 찾은 예: {"valid": False, "error": 0.0}
    import numpy as np

    values = np.asarray(lidar, dtype=float)
    if values.ndim != 1 or values.size < 5:
        return no_path()
    angles = np.deg2rad((np.arange(values.size) + 180) % 360 - 180)
    valid = (np.isfinite(values) & (values > .15) & (values < 12.0)
             & (np.abs(angles) < np.deg2rad(80)))
    indices = np.flatnonzero(valid)
    if indices.size == 0:
        return no_path()

    # x는 차량 앞쪽, y는 왼쪽이며 유효한 점만 한 번에 변환합니다.
    distances, angles = values[indices], angles[indices]
    points = np.column_stack((distances * np.cos(angles), distances * np.sin(angles)))
    gaps = np.diff(points, axis=0)
    splits = np.flatnonzero((np.diff(indices) != 1)
                            | (np.einsum('ij,ij->i', gaps, gaps) > .25 ** 2)) + 1
    cones = []
    for cluster in np.split(points, splits):
        center = cluster.mean(axis=0)
        span = cluster[-1] - cluster[0]
        if (.2 < center[0] < 3.0 and .18 < abs(center[1]) < 1.3
                and span @ span < .40 ** 2):
            cones.append(center)

    left = [point for point in cones if point[1] > 0]
    right = [point for point in cones if point[1] < 0]
    pairs = [(a + b) * .5 for a in left for b in right
             if abs(a[0] - b[0]) < .55 and .8 < a[1] - b[1] < 1.7]
    if not pairs:
        return no_path()
    target = min(pairs, key=lambda point: abs(point[0] - 1.0))
    return {'valid': True, 'error': float(target[1])}


def yolo(labels, bbox, size, conf):
    # TODO 3: 모델의 검출 결과를 신호등과 라바콘으로 나눕니다.
    # 네 입력은 같은 순서의 목록입니다. i번째 값들은 같은 물체를 뜻합니다.
    # labels: 물체 이름. 예 ["red", "cone"] (red/yellow/green/cone).
    # bbox: 상자의 왼쪽 위 [x, y] 픽셀 좌표. 예 [[120, 40], [200, 100]].
    # size: 상자의 [너비, 높이] 픽셀 수. 예 [[20, 20], [30, 60]]. 이미지 크기는 320×240입니다.
    # conf: 모델의 검출 신뢰도(0~1). 예 [0.95, 0.88]. 검출이 없으면 네 목록 모두 []입니다.
    # 출력: traffic_light(신호등 1개), rubbercone(라바콘 여러 개)의 dict.
    # detected: 검출 여부(bool). bbox: 왼쪽 위 [x, y], size: [너비, 높이](픽셀).
    # conf: 신뢰도(0~1). 신호등 color: red/yellow/green/unknown.
    # 라바콘은 여러 개이므로 bbox·size·conf가 같은 순서의 목록입니다.
    # 검출 예:
    # {"traffic_light": {"detected": True, "color": "red",
    #                    "bbox": [120, 40], "size": [20, 20], "conf": 0.95},
    #  "rubbercone": {"detected": True, "bbox": [[200, 100], [250, 110]],
    #                 "size": [[30, 60], [25, 50]], "conf": [0.88, 0.90]}}
    # 못 찾으면 아래 형식입니다. 신호등이 여럿이면 정지 신호를 우선해 하나를 선택합니다.
    return {"traffic_light": {"detected": False, "color": "unknown",
                              "bbox": [0, 0], "size": [0, 0], "conf": 0.0},
            "rubbercone": {"detected": False, "bbox": [], "size": [], "conf": []}}


class DrivingController:
    def step(self, lane_result, rubbercone_result, yolo_result):
        # TODO 4: 위 세 함수의 결과로 주행 모드·PID 조향·신호 정지/출발을 결정합니다.
        # lane_result: lane(camera)가 반환한 dict.
        # rubbercone_result: rubbercone(lidar)가 반환한 dict.
        # yolo_result: yolo(...)가 반환한 dict. 센서가 끊기면 해당 결과는 None입니다.
        # 출력: [조향각, 속도 PWM] 정수 2개. 조향각 -30~30도(왼쪽 +), 전진 PWM 0~255.
        # 예: [10, 125] → 왼쪽 10도로 전진 / [0, 0] → 정지.
        # 입력 None이면 정지하고, 신호를 못 봤다는 이유만으로 출발하지 마세요.
        if any(result is None for result in (lane_result, rubbercone_result, yolo_result)):
            return [0, 0]
        # 우선 직진합니다. 위 publish_motor가 /session4/motor_control로 발행합니다.
        return [0, 125]


# ========================================
# ========== DO NOT MODIFY ===============
# ========================================
# 아래는 완성된 연결 코드입니다. 카메라·라이다를 받고 함수 결과로 차량을 조작합니다.
import os
from pathlib import Path
import sys

if __name__ == '__main__' and os.environ.get('SESSION4_ENV_READY') != '1':
    project = Path(__file__).resolve().parents[3]
    os.chdir(project)
    os.execvp('bash', ['bash', '-c', 'source "$1" || exit $?; shift; exec python "$@"',
        'session4', str(project / 'scripts/session_4_env.sh'), str(Path(__file__).resolve()), *sys.argv[1:]])

import argparse
import json
import time
from session_4.contracts import no_path, STALE_SECONDS


def create_node(implementation, detector):
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from session_3.image_utils import image_to_rgb
    from session_4.direct_runtime import distances, split_boxes, write_status

    class DrivingNode(Node):
        def __init__(self):
            super().__init__('session4_driving')
            self.controller = implementation.DrivingController()
            self.images, self.received, self.values = {}, {}, {}
            self.lidar = None
            self.scan_message = None
            self.viewer_open = False
            self.last_detection = -float('inf')
            connect_topics(self)
            self.timer = self.create_timer(.05, self.drive, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def camera(self, name, message):
            self.images[name] = cv2.resize(image_to_rgb(message), (320, 240))
            self.received[name] = time.monotonic()

        def scan(self, message):
            self.lidar = distances(message)
            self.scan_message = message
            self.received['scan'] = time.monotonic()

        def vehicle_state(self, message):
            self.values['actual'] = json.loads(message.data)
            self.received['actual'] = time.monotonic()

        def drive(self):
            now = time.monotonic()
            fresh = lambda name: now - self.received.get(name, -float('inf')) < STALE_SECONDS
            # 네 역할은 토픽 없이 함수의 반환값으로 이어집니다.
            lane_result = implementation.lane(self.images['low']) if fresh('low') else None
            rubbercone_result = implementation.rubbercone(self.lidar) if fresh('scan') else None
            if detector is not None and fresh('high') and now - self.last_detection >= .1:
                taken_at = self.received['high']
                packet = detector(self.images['high'])
                self.values['detections'] = packet
                self.last_detection = time.monotonic()
                if self.last_detection - taken_at < STALE_SECONDS and packet.get('valid', True):
                    self.values['perception'] = implementation.yolo(*split_boxes(packet))
                    self.received['perception'] = self.last_detection
                else:
                    self.values.pop('perception', None)
                self.received['detections'] = self.last_detection
            yolo_result = self.values.get('perception') if fresh('high') and fresh('perception') else None
            # 모델 추론이 오래 걸렸을 때는 다른 센서 결과도 다시 만료 확인합니다.
            now = time.monotonic()
            if not fresh('low'): lane_result = None
            if not fresh('scan'): rubbercone_result = None
            command = self.controller.step(lane_result, rubbercone_result, yolo_result)
            if (not isinstance(command, (list, tuple)) or len(command) != 2
                    or any(type(x) is not int for x in command)
                    or not -30 <= command[0] <= 30 or not 0 <= command[1] <= 255):
                command = [0, 0]
            if any(x is None for x in (lane_result, rubbercone_result, yolo_result)):
                command = [0, 0]
            publish_motor(self, command)
            for key, result in [('lane', lane_result), ('cone', rubbercone_result)]:
                self.values[key] = result if result is not None else dict(no_path(), stale=True)
                self.received[key] = now if result is not None else -float('inf')
            self.values['control'] = {'mode': getattr(self.controller, 'mode', 'pending'),
                'command': list(command), 'stopped_at_signal': getattr(self.controller, 'stopped_at_signal', False),
                'viewer_open': self.viewer_open}
            self.received['control'] = now
            write_status(self.values, self.received)

        def snapshot(self):
            return {'images': self.images, 'values': self.values, 'received': self.received,
                    'scan': self.scan_message}

        def stop(self):
            publish_motor(self, [0, 0])
    return DrivingNode()


def main(argv=None):
    parser = argparse.ArgumentParser(description='팀 자율주행 코드 실행')
    parser.add_argument('--solution', action='store_true')
    parser.add_argument('--yolo', action='store_true')
    parser.add_argument('--weights', default='')
    parser.add_argument('--device', default='auto')
    parser.add_argument('--class-map', default='')
    parser.add_argument('--no-viewer', action='store_true')
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--color-demo', action='store_true')
    options, ros_args = parser.parse_known_args(argv)
    implementation = sys.modules[__name__]
    if options.solution:
        from session_4 import solution as implementation
    from session_4.yolo_node import color_detections, YoloDetector
    detector = color_detections
    if options.weights or options.yolo or (options.solution and not options.color_demo):
        from session_4.training_tools import resolve_latest_model
        from session_4.dataset_tools import load_session2_module
        weights = options.weights or (str(resolve_latest_model()) if options.yolo else str(implementation.detector_weights()))
        device = load_session2_module('learning').choose_device(options.device)
        detector = YoloDetector(weights, options.class_map, device)
        print(f'YOLO 모델: {weights} / 장치: {device}', flush=True)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from session_4.control_mode import set_mode
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = create_node(implementation, detector)
    window = None
    try:
        node.timer.cancel()
        set_mode(node, 'auto')
        node.timer.reset()
        if (options.solution or options.viewer) and not options.no_viewer:
            from session_3.image_utils import prepare_qt
            prepare_qt()
            from PySide6.QtWidgets import QApplication
            from session_4.debug_viewer import create_window
            app = QApplication.instance() or QApplication([])
            window = create_window(node, state_provider=node.snapshot)
            import signal
            signal.signal(signal.SIGINT, lambda *_: window.close())
            window.show()
            node.viewer_open = True
            app.exec()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.timer.cancel()
        node.stop()
        if window is not None: window.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
