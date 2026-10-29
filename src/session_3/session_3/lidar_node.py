"""RPLIDAR A1 한 개의 거리 측정을 표준 LaserScan으로 발행한다."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .sensor_utils import SensorProcess, lidar_worker, scan_ranges


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        for name, value in [('port', ''), ('baudrate', 115200), ('mount_yaw_deg', 0.0)]:
            self.declare_parameter(name, value)
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.mount_yaw = float(self.get_parameter('mount_yaw_deg').value)
        if not port:
            raise ValueError('port에 라이다 장치 경로를 지정하세요. 예: -p port:=/dev/ttyUSB0')
        if baudrate <= 0 or not math.isfinite(self.mount_yaw):
            raise ValueError('baudrate 또는 mount_yaw_deg를 확인하세요.')
        self.publisher = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.worker = SensorProcess(lidar_worker, port, baudrate)
        self.last_scan = time.monotonic()
        self.received_scan = False
        self.warned = False
        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info(f'/scan 발행 · {port} · 0도=앞, +90도=왼쪽')

    def tick(self):
        latest = None
        for kind, received, value in self.worker.read():
            if kind == 'error':
                self.get_logger().error(value)
            elif kind == 'data':
                latest = (received, value)
        if latest is not None:
            received, samples = latest
            if time.monotonic() - received > 1:
                return
            message = LaserScan()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'lidar_link'
            message.angle_min = -math.pi
            message.angle_increment = math.pi / 180.0
            message.angle_max = message.angle_min + 359 * message.angle_increment
            message.range_min, message.range_max = 0.15, 12.0
            message.scan_time = max(0.0, received - self.last_scan) if self.received_scan else 0.0
            # 각도별로 재배열했으므로 일정한 광선 간 취득 시각은 가정하지 않는다.
            message.time_increment = 0.0
            message.ranges = scan_ranges(samples, self.mount_yaw)
            self.publisher.publish(message)
            self.last_scan = received
            self.received_scan = True
            self.warned = False
        elif not self.warned and time.monotonic() - self.last_scan > (2 if self.received_scan else 6):
            self.warned = True
            self.get_logger().warning('라이다 수신 대기 중입니다. 포트와 연결 상태를 확인하세요.')

    def destroy_node(self):
        self.worker.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
