"""카메라 토픽을 구독하는 독립 PySide6 창."""
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .image_utils import image_to_rgb, prepare_qt

prepare_qt()
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget


class CameraViewer(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.topic = node.declare_parameter('topic', '/camera/high/image_raw').value
        self.setWindowTitle(f'ORDA · 카메라 · {self.topic}')
        self.resize(640, 480)
        layout = QVBoxLayout(self)
        self.title = QLabel(self.topic)
        self.picture = QLabel('영상 수신 대기 중')
        self.picture.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.picture.setMinimumSize(240, 180)
        self.picture.setStyleSheet('background:#151c24; color:#d9e3ee; border-radius:6px;')
        self.status = QLabel('발행 노드와 ROS_DOMAIN_ID를 확인하세요.')
        layout.addWidget(self.title)
        layout.addWidget(self.picture, 1)
        layout.addWidget(self.status)
        self.pixmap = None
        self.last_frame = 0.0
        self.details = ''
        self.error = ''
        self.subscription = node.create_subscription(Image, self.topic, self.receive, qos_profile_sensor_data)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)

    def receive(self, message):
        try:
            rgb = image_to_rgb(message)
            height, width = rgb.shape[:2]
            picture = QImage(rgb.data, width, height, rgb.strides[0], QImage.Format.Format_RGB888).copy()
            self.pixmap = QPixmap.fromImage(picture)
            self.last_frame = time.monotonic()
            self.details = f'{width} × {height} · {message.encoding}'
            self.error = ''
            self.show_image()
        except (ValueError, TypeError) as exc:
            self.error = str(exc)

    def show_image(self):
        if self.pixmap is not None:
            self.picture.setPixmap(self.pixmap.scaled(
                self.picture.size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.show_image()

    def tick(self):
        if not rclpy.ok():
            self.close()
            return
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.error:
            self.status.setText(self.error)
            self.status.setStyleSheet('color:#b42318;')
        elif self.last_frame:
            age = time.monotonic() - self.last_frame
            suffix = f' · 수신 중단 {age:.1f}초 (마지막 영상)' if age > 2 else ' · 수신 중'
            self.status.setText(self.details + suffix)
            self.status.setStyleSheet('color:#b42318;' if age > 2 else '')


def main(args=None):
    rclpy.init(args=args)
    node = Node('camera_viewer')
    app = QApplication.instance() or QApplication([sys.argv[0]])
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    try:
        window = CameraViewer(node)
        window.show()
        app.exec()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
