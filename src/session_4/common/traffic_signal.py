"""Gazebo 안의 신호등 색을 실제로 바꾸는 수업 환경 노드."""
import queue
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


COLORS = {'red': (1.0, .02, .01), 'yellow': (1.0, .83, .015), 'green': (.015, 1.0, .035)}
DARK = (.015, .015, .015)


def phase_at(seconds):
    """sim time 기준: 빨강 12초 → 초록 10초 → 노랑 2초."""
    tick = seconds % 24
    return 'red' if tick < 12 else 'green' if tick < 22 else 'yellow'


def visual_request(lamp, active):
    rgb = COLORS[lamp] if active else DARK
    color = 'r: %.4f g: %.4f b: %.4f a: 1' % rgb
    return (f'name: "{lamp}_lamp" parent_name: "traffic_light_body" '
            f'material {{ ambient {{ {color} }} diffuse {{ {color} }} '
            f'emissive {{ {color} }} specular {{ r: 0 g: 0 b: 0 a: 1 }} }}')


class TrafficSignal(Node):
    def __init__(self):
        super().__init__('session4_traffic_signal')
        self.declare_parameter('mode', 'cycle')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, '/session4/traffic_light/state', qos)
        self.commands = queue.Queue(maxsize=1)
        self.completed = queue.Queue()
        self.stopped = threading.Event()
        self.requested = None
        self.actual = 'unknown'
        self.last_retry = 0.0
        self.worker = threading.Thread(target=self.update_visuals, daemon=True)
        self.worker.start()
        self.timer = self.create_timer(.2, self.tick)

    def tick(self):
        while not self.completed.empty():
            phase, success, detail = self.completed.get_nowait()
            if success:
                self.actual = phase
                self.get_logger().info(f'Gazebo 신호등: {phase}')
            else:
                self.actual = 'unknown'
                self.requested = None
                self.get_logger().warning('신호등 갱신 재시도: '+detail[:180])
        mode = str(self.get_parameter('mode').value)
        if mode not in ('cycle', 'red', 'green', 'yellow'):
            self.get_logger().error('mode는 cycle/red/green/yellow 중 하나여야 합니다.')
            mode = 'red'
        phase = phase_at(self.get_clock().now().nanoseconds/1e9) if mode == 'cycle' else mode
        now = time.monotonic()
        if self.requested != phase and now-self.last_retry >= 1.0:
            if self.commands.empty():
                self.commands.put_nowait(phase)
                self.requested = phase
                self.last_retry = now
        message = String()
        message.data = self.actual
        self.publisher.publish(message)

    def update_visuals(self):
        # CLI에는 리스트 인자를 전달한다. shell이나 Gazebo Python 바인딩이 필요 없다.
        while not self.stopped.is_set():
            try:
                phase = self.commands.get(timeout=.2)
            except queue.Empty:
                continue
            success, detail = True, ''
            for lamp in COLORS:
                if self.stopped.is_set():
                    return
                try:
                    result = subprocess.run(
                        ['gz', 'service', '-s', '/world/session4_track/visual_config',
                         '--reqtype', 'gz.msgs.Visual', '--reptype', 'gz.msgs.Boolean',
                         '--timeout', '1500', '--req', visual_request(lamp, lamp == phase)],
                        capture_output=True, text=True, timeout=3, check=False)
                    if result.returncode or 'data: true' not in result.stdout:
                        success, detail = False, result.stderr or result.stdout or 'Gazebo 응답 없음'
                        break
                except (OSError, subprocess.TimeoutExpired) as error:
                    success, detail = False, str(error)
                    break
            self.completed.put((phase, success, detail))

    def destroy_node(self):
        self.stopped.set()
        self.worker.join(timeout=3.5)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
