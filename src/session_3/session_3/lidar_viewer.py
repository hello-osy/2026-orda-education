"""LaserScan 토픽을 위에서 본 거리 점으로 표시한다. 위=앞, 왼쪽=왼쪽."""
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .image_utils import prepare_qt

prepare_qt()
from PySide6.QtCore import QPointF, Qt, QTimer
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget


class ScanCanvas(QWidget):
    def __init__(self, limit):
        super().__init__()
        self.limit = limit
        self.points = []
        self.setMinimumSize(360, 360)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor('#151c24'))
        center = QPointF(self.width() / 2, self.height() / 2)
        radius = max(1.0, min(self.width(), self.height()) / 2 - 30)
        scale = radius / self.limit
        painter.setPen(QPen(QColor('#405264'), 1))
        for portion in (0.25, 0.5, 0.75, 1.0):
            painter.drawEllipse(center, radius * portion, radius * portion)
        painter.drawLine(QPointF(center.x() - radius, center.y()), QPointF(center.x() + radius, center.y()))
        painter.drawLine(QPointF(center.x(), center.y() - radius), QPointF(center.x(), center.y() + radius))
        painter.setPen(QColor('#d9e3ee'))
        painter.drawText(int(center.x() - 10), 18, '앞')
        painter.drawText(8, int(center.y() - 6), '왼쪽')
        painter.drawText(self.width() - 34, int(center.y() - 6), '오른쪽')
        painter.drawText(10, self.height() - 12, f'표시 반경 {self.limit:g} m')
        painter.setPen(QPen(QColor('#66e0bd'), 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        for x, y in self.points:
            painter.drawPoint(QPointF(center.x() - y * scale, center.y() - x * scale))
        painter.setPen(QPen(QColor('#ffd37a'), 3))
        painter.drawLine(center, QPointF(center.x(), center.y() - 16))


class LidarViewer(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.topic = node.declare_parameter('topic', '/scan').value
        limit = float(node.declare_parameter('max_range', 6.0).value)
        if not math.isfinite(limit) or limit <= 0:
            raise ValueError('max_range는 0보다 큰 유한한 거리여야 합니다.')
        self.setWindowTitle(f'ORDA · 라이다 · {self.topic}')
        self.resize(580, 620)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel(self.topic))
        self.canvas = ScanCanvas(limit)
        self.status = QLabel('거리 수신 대기 중 · 발행 노드와 ROS_DOMAIN_ID를 확인하세요.')
        self.status.setWordWrap(True)
        layout.addWidget(self.canvas, 1)
        layout.addWidget(self.status)
        self.last_scan = 0.0
        self.point_count = 0
        self.frame_id = ''
        self.subscription = node.create_subscription(LaserScan, self.topic, self.receive, qos_profile_sensor_data)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)

    def receive(self, message):
        points = []
        for i, distance in enumerate(message.ranges):
            angle = message.angle_min + i * message.angle_increment
            if (math.isfinite(distance) and math.isfinite(angle)
                    and message.range_min <= distance <= min(message.range_max, self.canvas.limit)):
                points.append((distance * math.cos(angle), distance * math.sin(angle)))
        self.canvas.points = points
        self.canvas.update()
        self.point_count = len(points)
        self.frame_id = message.header.frame_id
        self.last_scan = time.monotonic()

    def tick(self):
        if not rclpy.ok():
            self.close()
            return
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.last_scan:
            age = time.monotonic() - self.last_scan
            suffix = f' · 수신 중단 {age:.1f}초 (마지막 거리)' if age > 2 else ' · 수신 중'
            self.status.setText(f'표시 {self.point_count}점 · {self.frame_id}' + suffix)
            self.status.setStyleSheet('color:#b42318;' if age > 2 else '')


def main(args=None):
    rclpy.init(args=args)
    node = Node('lidar_viewer')
    app = QApplication.instance() or QApplication([sys.argv[0]])
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    try:
        window = LidarViewer(node)
        window.show()
        app.exec()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
