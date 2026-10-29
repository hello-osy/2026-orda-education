"""USB 카메라 한 개를 ROS 2 Image와 미보정 CameraInfo로 발행한다."""
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .sensor_utils import SensorProcess, camera_worker


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        for name, value in [('camera_name', 'high'), ('device', 0), ('width', 320),
                            ('height', 240), ('fps', 10.0)]:
            self.declare_parameter(name, value)
        camera_name = self.get_parameter('camera_name').value
        if camera_name not in ('high', 'low'):
            raise ValueError('camera_name은 high 또는 low입니다.')
        device = self.get_parameter('device').value
        width, height = (self.get_parameter(name).value for name in ('width', 'height'))
        fps = float(self.get_parameter('fps').value)
        if width <= 0 or height <= 0 or not 0 < fps <= 120:
            raise ValueError('width/height는 양수, fps는 0보다 크고 120 이하여야 합니다.')
        self.frame_id = f'camera_{camera_name}_optical_frame'
        prefix = f'/camera/{camera_name}'
        self.image_pub = self.create_publisher(Image, prefix + '/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, prefix + '/camera_info', qos_profile_sensor_data)
        self.worker = SensorProcess(camera_worker, device, width, height, fps)
        self.last_frame = time.monotonic()
        self.warned = False
        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info(f'{prefix}/image_raw · 카메라 {device}; CameraInfo는 미보정(K=0)입니다.')

    def tick(self):
        latest = None
        for kind, received, value in self.worker.read():
            if kind == 'error':
                self.get_logger().error(value)
            elif kind == 'status':
                self.get_logger().info(value)
            elif kind == 'data':
                latest = (received, value)
        if latest is not None:
            received, frame = latest
            if time.monotonic() - received > 1.0:
                return
            self.last_frame = received
            self.warned = False
            message = Image()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.frame_id
            message.height, message.width = frame.shape[:2]
            message.encoding = 'bgr8'
            message.is_bigendian = 0
            message.step = message.width * 3
            message.data = frame.tobytes()
            self.image_pub.publish(message)
            info = CameraInfo()
            info.header = message.header
            info.height, info.width = message.height, message.width
            # K[0] == 0은 미보정 카메라를 뜻한다. 임의의 보정값을 만들지 않는다.
            self.info_pub.publish(info)
        elif not self.warned and time.monotonic() - self.last_frame > 2:
            self.warned = True
            self.get_logger().warning('카메라 영상 수신 대기 중입니다. 장치 번호와 권한을 확인하세요.')

    def destroy_node(self):
        self.worker.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
