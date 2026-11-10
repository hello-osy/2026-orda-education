"""센서·네 역할 결과를 한 화면에서 확인한다. 차량 제어값은 발행하지 않는다."""
import math
import signal
import sys
import time

import numpy as np

from . import contracts


LABEL_COLORS = {'red': (255, 85, 85), 'yellow': (255, 211, 72),
                'green': (64, 220, 156), 'cone': (255, 159, 67)}


def visible_boxes(packet, width, height):
    """원본 영상 좌표를 화면 영상에 맞추고 잘못된 검출 박스는 제외한다."""
    if not isinstance(packet, dict) or packet.get('valid', True) is not True:
        return []
    original_width = contracts.finite(packet.get('width'))
    original_height = contracts.finite(packet.get('height'))
    if min(width, height, original_width, original_height) <= 0:
        return []
    detections = packet.get('detections', [])
    if not isinstance(detections, list):
        return []
    boxes = []
    for item in detections:
        if not isinstance(item, dict):
            continue
        bbox = item.get('bbox')
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            continue
        try:
            values = [float(value) for value in bbox]
            confidence = float(item.get('confidence', 0))
        except (TypeError, ValueError, OverflowError):
            continue
        if not all(math.isfinite(value) for value in values + [confidence]) or not 0 <= confidence <= 1:
            continue
        x1, y1, x2, y2 = values
        x1, x2 = max(0, x1), min(original_width, x2)
        y1, y2 = max(0, y1), min(original_height, y2)
        if x2 <= x1 or y2 <= y1:
            continue
        scale_x, scale_y = width / original_width, height / original_height
        box = (int(x1 * scale_x), int(y1 * scale_y),
               min(width - 1, round(x2 * scale_x)), min(height - 1, round(y2 * scale_y)))
        boxes.append((str(item.get('label', 'unknown'))[:32], confidence, box))
    return boxes


def detection_overlay(rgb, packet):
    """영상은 RGB 그대로 유지한다. 원본 배열과 검출 자료는 수정하지 않는다."""
    import cv2
    image = np.ascontiguousarray(rgb).copy()
    height, width = image.shape[:2]
    for label, confidence, (x1, y1, x2, y2) in visible_boxes(packet, width, height):
        color = LABEL_COLORS.get(label, (180, 210, 240))
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 1)
        text = f'{label} {confidence:.2f}'
        cv2.putText(image, text, (x1, max(11, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                    .34, (18, 24, 31), 3, cv2.LINE_AA)
        cv2.putText(image, text, (x1, max(11, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                    .34, color, 1, cv2.LINE_AA)
    return image


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, limit=6.0):
    """차량 앞쪽 x+, 왼쪽 y+ 좌표만 계산한다. NaN/inf 점은 그리지 않는다."""
    if not all(math.isfinite(float(value)) for value in (angle_min, angle_increment, range_min, limit)):
        return []
    if not math.isfinite(range_max):
        range_max = limit
    maximum = min(range_max, limit)
    if range_min < 0 or maximum <= 0 or maximum < range_min:
        return []
    points = []
    for index, distance in enumerate(ranges):
        angle = angle_min + index * angle_increment
        if math.isfinite(distance) and range_min <= distance <= maximum:
            points.append((distance * math.cos(angle), distance * math.sin(angle)))
    return points


def create_window(node, state_provider=None):
    """Qt를 늦게 불러 순수 오버레이·좌표 계산은 ROS 없이도 검사할 수 있다."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    from std_msgs.msg import String, Int16MultiArray
    from session_3.image_utils import image_to_rgb, prepare_qt

    prepare_qt()
    from PySide6.QtCore import QPointF, Qt, QTimer
    from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
    from PySide6.QtWidgets import QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget

    class ImagePanel(QLabel):
        def __init__(self):
            super().__init__('영상 수신 대기 중')
            self.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.setMinimumSize(320, 210)
            self.setStyleSheet('background:#111923; color:#b8c5d4; border-radius:6px;')
            self.source_pixmap = None

        def show_rgb(self, rgb):
            height, width = rgb.shape[:2]
            image = QImage(rgb.data, width, height, rgb.strides[0], QImage.Format.Format_RGB888).copy()
            self.source_pixmap = QPixmap.fromImage(image)
            self.fit()

        def fit(self):
            if self.source_pixmap is not None:
                self.setPixmap(self.source_pixmap.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                                        Qt.TransformationMode.SmoothTransformation))

        def resizeEvent(self, event):
            super().resizeEvent(event)
            self.fit()

    class ScanPanel(QWidget):
        def __init__(self):
            super().__init__()
            self.points = []
            self.setMinimumSize(320, 230)

        def paintEvent(self, event):
            painter = QPainter(self)
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
            painter.fillRect(self.rect(), QColor('#111923'))
            center = QPointF(self.width() / 2, self.height() / 2)
            radius = max(1.0, min(self.width(), self.height()) / 2 - 25)
            scale = radius / 6.0
            painter.setPen(QPen(QColor('#344557'), 1))
            for fraction in (.25, .5, .75, 1):
                painter.drawEllipse(center, radius * fraction, radius * fraction)
            painter.drawLine(QPointF(center.x() - radius, center.y()), QPointF(center.x() + radius, center.y()))
            painter.drawLine(QPointF(center.x(), center.y() - radius), QPointF(center.x(), center.y() + radius))
            painter.setPen(QColor('#cbd5e1'))
            painter.drawText(round(center.x() - 8), 17, '앞')
            painter.drawText(8, round(center.y()), '왼쪽')
            painter.drawText(9, self.height() - 9, '표시 반경 6 m')
            painter.setPen(QPen(QColor('#5edbb8'), 3, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
            for x, y in self.points:
                painter.drawPoint(QPointF(center.x() - y * scale, center.y() - x * scale))
            painter.setPen(QPen(QColor('#ffce67'), 3))
            painter.drawLine(center, QPointF(center.x(), center.y() - 15))

    class DebugViewer(QWidget):
        def __init__(self):
            super().__init__()
            self.setWindowTitle('ORDA · 자율주행 확인')
            self.resize(1060, 820)
            self.setStyleSheet('QWidget {font-size:14px;} QGroupBox {font-weight:600;}')
            layout = QVBoxLayout(self)
            self.summary = QLabel('자율주행 결과 수신 대기 중')
            self.summary.setStyleSheet('font-size:18px; font-weight:600; padding:8px;')
            layout.addWidget(self.summary)
            self.explanation = QLabel('카메라·라이다 결과를 기다립니다.')
            self.explanation.setWordWrap(True)
            self.explanation.setStyleSheet('padding:4px 8px; font-size:15px;')
            layout.addWidget(self.explanation)
            grid = QGridLayout()
            layout.addLayout(grid, 1)
            self.low = ImagePanel()
            self.high = ImagePanel()
            self.scan = ScanPanel()
            self.low_caption = QLabel('차선 결과 수신 대기 중')
            self.high_caption = QLabel('검출 결과 수신 대기 중')
            self.scan_caption = QLabel('라이다 수신 대기 중')
            for caption in (self.low_caption, self.high_caption, self.scan_caption):
                caption.setWordWrap(True)
                caption.setMinimumHeight(42)
            for title, panel, caption, row, column in [
                    ('차선 카메라', self.low, self.low_caption, 0, 0),
                    ('물체 검출 카메라', self.high, self.high_caption, 0, 1),
                    ('라이다 · 위에서 본 거리', self.scan, self.scan_caption, 1, 0)]:
                group = QGroupBox(title)
                content = QVBoxLayout(group)
                content.addWidget(panel, 1)
                content.addWidget(caption)
                grid.addWidget(group, row, column)
            results = QGroupBox('역할별 결과')
            result_layout = QVBoxLayout(results)
            self.control_status = QLabel()
            self.role_status = QLabel()
            self.perception_status = QLabel()
            for label in (self.control_status, self.role_status, self.perception_status):
                label.setWordWrap(True)
                label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
                result_layout.addWidget(label)
            result_layout.addStretch()
            hint = QLabel('선이 안 보이거나 결과가 이상하면 담당 역할의 입력과 출력을 함께 확인하세요.')
            hint.setWordWrap(True)
            hint.setStyleSheet('color:#64748b;')
            result_layout.addWidget(hint)
            grid.addWidget(results, 1, 1)
            self.received = {}
            self.values = {}
            self.images = {}
            self.subscriptions = []
            self.last_drawn = 0.0
            for name in (() if state_provider else ('low', 'high')):
                self.subscriptions.append(node.create_subscription(
                    Image, f'/camera/{name}/image_raw', lambda message, key=name: self.image(key, message),
                    qos_profile_sensor_data))
            if not state_provider:
                self.subscriptions.append(node.create_subscription(LaserScan, '/scan', self.scan_received,
                                                                  qos_profile_sensor_data))
            for key, topic in ([] if state_provider else [('actual', '/session4/control/state')]):
                self.subscriptions.append(node.create_subscription(
                    String, topic, lambda message, name=key: self.packet(name, message), 1))
            if not state_provider:
                self.subscriptions.append(node.create_subscription(Int16MultiArray, contracts.MOTOR_TOPIC,
                                                                   self.motor, 1))
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.tick)
            self.timer.start(30)

        def image(self, key, message):
            try:
                self.images[key] = image_to_rgb(message)
                self.received[key] = time.monotonic()
            except (TypeError, ValueError) as exc:
                node.get_logger().warning(str(exc))

        def packet(self, key, message):
            try:
                self.values[key] = contracts.loads(message.data)
                self.received[key] = time.monotonic()
            except (TypeError, ValueError):
                self.values.pop(key, None)
                self.received.pop(key, None)

        def motor(self, message):
            if len(message.data) == 2:
                self.values['motor'] = list(message.data)
                self.received['motor'] = time.monotonic()

        def scan_received(self, message):
            self.scan.points = scan_points(message.ranges, message.angle_min, message.angle_increment,
                                           message.range_min, message.range_max)
            self.received['scan'] = time.monotonic()
            self.scan.update()

        def fresh(self, key, now):
            return key in self.received and now - self.received[key] < contracts.STALE_SECONDS

        def status(self, key, now):
            packet = self.values.get(key)
            if not self.fresh(key, now) or (isinstance(packet, dict) and packet.get('stale')):
                return '수신 대기 / 중단'
            if isinstance(packet, dict) and not packet.get('valid', True):
                return {'lane': '차선 없음', 'cone': '구간 없음'}.get(key, '결과 없음')
            return '정상'

        def tick(self):
            if not rclpy.ok():
                self.close()
                return
            # 한 번에 너무 많은 메시지를 처리해 화면이 멈추지 않도록 제한합니다.
            for _ in range(12):
                rclpy.spin_once(node, timeout_sec=0)
            now = time.monotonic()
            if state_provider:
                state = state_provider()
                self.images = state['images']
                self.values = state['values']
                self.received = state['received']
                if state.get('scan') is not None:
                    scan = state['scan']
                    self.scan.points = scan_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                                    scan.range_min, scan.range_max)
                    self.scan.update()
            else:
                import json
                from .direct_runtime import status_path
                try:
                    state = json.loads(status_path().read_text())
                    self.values.update(state['values'])
                    self.received.update(state['received'])
                except (OSError, ValueError, KeyError):
                    pass
            if now - self.last_drawn < .1:
                return
            self.last_drawn = now
            lane = self.values.get('lane', {})
            cone = self.values.get('cone', {})
            perception = self.values.get('perception', {})
            control = self.values.get('control', {})
            actual = self.values.get('actual', {})
            detections = self.values.get('detections', {})
            if 'low' in self.images:
                self.low.show_rgb(self.images['low'])
            if 'high' in self.images:
                packet = detections if self.fresh('detections', now) else None
                self.high.show_rgb(detection_overlay(self.images['high'], packet))
            lane_error = (f'{contracts.finite(lane.get("error")):+.3f} m (왼쪽 +)'
                          if lane.get('valid') and self.fresh('lane', now) and not lane.get('stale') else '—')
            cone_error = (f'{contracts.finite(cone.get("error")):+.3f} m'
                          if cone.get('valid') and self.fresh('cone', now) and not cone.get('stale') else '—')
            self.low_caption.setText(
                f'영상 {self.status("low", now)} · 차선 {self.status("lane", now)}\n'
                f'목표점 좌우 오차 {lane_error}')
            source = {'simulator_color': '색상 예제', 'yolo': 'YOLO'}.get(str(detections.get('source')), '검출기')
            self.high_caption.setText(f'영상 {self.status("high", now)} · {source} {self.status("detections", now)}')
            self.scan_caption.setText(
                f'라이다 {self.status("scan", now)} · 표시 {len(self.scan.points)}점\n'
                f'라바콘 {self.status("cone", now)} · 목표점 오차 {cone_error}')
            mode = {'lane': '차선', 'cone': '라바콘', 'pending': 'TODO 구현 대기'}.get(str(control.get('mode')), '대기')
            command = actual.get('command', [0, 0]) if self.fresh('actual', now) else [0, 0]
            if not isinstance(command, list) or len(command) != 2:
                command = [0, 0]
            command = [int(contracts.finite(value)) for value in command]
            stopped = bool(control.get('stopped_at_signal'))
            if self.fresh('actual', now):
                manual = actual.get('mode') == 'manual'
                title = '수동주행' if manual else f'자율주행 · {mode} 모드'
                if not actual.get('fresh'):
                    title += ' · 명령 대기 / 정지'
                elif stopped and not manual:
                    title += ' · 신호 정지 중'
                self.summary.setText(title)
            else:
                self.summary.setText('차량 제어 상태 수신 대기 / 중단')
            from .sim_control_node import MAX_SPEED_MPS
            speed = MAX_SPEED_MPS * command[1] / 255
            self.control_status.setText(f'차량에 적용된 명령\n조향 {command[0]:+d}° · PWM {command[1]:d}\n명령 속도 {speed:.2f} m/s')
            turn = '왼쪽으로 조향합니다.' if command[0] > 1 else (
                '오른쪽으로 조향합니다.' if command[0] < -1 else '직진합니다.')
            if not self.fresh('actual', now):
                explanation = '차량과 연결을 확인하고 있습니다.'
            elif command[1] == 0 and stopped:
                explanation = '정지 신호를 확인했습니다. 초록불이 보일 때까지 기다립니다.'
            elif command[1] == 0:
                explanation = '현재 정지 중입니다. 센서와 주행 경로를 확인합니다.'
            elif actual.get('mode') == 'manual':
                explanation = '키보드로 직접 운전하고 있습니다. ' + turn
            elif control.get('mode') == 'cone':
                explanation = '라이다로 라바콘 사이의 가운데를 따라갑니다. ' + turn
            else:
                explanation = '카메라로 차선을 찾고 오른쪽 차로를 따라갑니다. ' + turn
            self.explanation.setText(explanation)
            self.role_status.setText('입력 결과\n' + '\n'.join(
                f'{label}: {self.status(key, now)}' for key, label in
                [('lane', '차선'), ('cone', '라바콘'), ('perception', '객체 인식')]))
            signal_text = {'red': '빨강', 'yellow': '노랑', 'green': '초록', 'unknown': '미검출'}.get(
                str(perception.get('traffic_light', {}).get('color')), '미검출')
            self.perception_status.setText(
                f'신호: {signal_text} · 신호 높이 비율 {contracts.finite(perception.get("traffic_light", {}).get("size", [0, 0])[1]) / 240:.3f}\n'
                f'라바콘 검출 {len(perception.get("rubbercone", {}).get("bbox", []))}개')

        def closeEvent(self, event):
            self.timer.stop()
            for subscription in self.subscriptions:
                node.destroy_subscription(subscription)
            self.subscriptions.clear()
            super().closeEvent(event)

    return DebugViewer()


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from session_3.image_utils import prepare_qt
    prepare_qt()
    from PySide6.QtWidgets import QApplication

    rclpy.init(args=args)
    node = Node('session4_debug_viewer')
    app = QApplication.instance() or QApplication([sys.argv[0]])
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    window = None
    try:
        window = create_window(node)
        window.show()
        app.exec()
    finally:
        if window is not None:
            window.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
