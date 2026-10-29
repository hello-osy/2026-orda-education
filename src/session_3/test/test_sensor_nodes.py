"""실물 장치 대신 고정 측정을 넣고 실제 ROS 메시지 송수신을 확인한다."""
import math
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan


def receive_until(nodes, condition, timeout=5):
    until = time.monotonic() + timeout
    while time.monotonic() < until and not condition():
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.02)
    assert condition(), 'ROS 메시지를 제한 시간 안에 받지 못했습니다.'


class FakeSensor:
    payload = None

    def __init__(self, *args):
        self.closed = False

    def read(self):
        return [('data', time.monotonic(), self.payload)]

    def close(self):
        self.closed = True


def test_camera_publishes_actual_dimensions_and_uncalibrated_info(monkeypatch):
    from session_3 import camera_node
    monkeypatch.setattr(camera_node, 'SensorProcess', FakeSensor)
    FakeSensor.payload = np.full((12, 16, 3), [10, 20, 30], dtype=np.uint8)
    rclpy.init(args=[])
    producer = camera_node.CameraNode()
    listener = Node('camera_test_listener')
    images, infos = [], []
    listener.create_subscription(Image, '/camera/high/image_raw', images.append, qos_profile_sensor_data)
    listener.create_subscription(CameraInfo, '/camera/high/camera_info', infos.append, qos_profile_sensor_data)
    try:
        receive_until([producer, listener], lambda: bool(images and infos))
        assert (images[-1].width, images[-1].height, images[-1].step) == (16, 12, 48)
        assert images[-1].encoding == 'bgr8'
        assert bytes(images[-1].data[:3]) == bytes([10, 20, 30])
        assert infos[-1].k[0] == 0
        assert (infos[-1].width, infos[-1].height) == (16, 12)
    finally:
        producer.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()
    assert producer.worker.closed


def test_lidar_publishes_scan_with_matching_geometry(monkeypatch):
    from session_3 import lidar_node
    monkeypatch.setattr(lidar_node, 'SensorProcess', FakeSensor)
    FakeSensor.payload = [(10, 0, 1000), (10, 90, 2000)]
    rclpy.init(args=['--ros-args', '-p', 'port:=test-only'])
    producer = lidar_node.LidarNode()
    listener = Node('lidar_test_listener')
    scans = []
    listener.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    try:
        receive_until([producer, listener], lambda: bool(scans))
        scan = scans[-1]
        assert len(scan.ranges) == 360
        assert scan.ranges[180] == 1.0
        assert scan.ranges[90] == 2.0
        assert math.isinf(scan.ranges[0])
        assert scan.angle_min + 359 * scan.angle_increment == pytest.approx(scan.angle_max, abs=1e-6)
        assert scan.header.frame_id == 'lidar_link'
    finally:
        producer.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()
    assert producer.worker.closed
