"""Arduino 115200 baud: TX '조향PWM 0\n', RX 'R,raw,전압,백분율\n'."""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Int16, Int16MultiArray
from std_srvs.srv import Trigger

from .auto_center import StartupCenter
from .motor_core import SteeringControl, parse_feedback


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        defaults = {'port': '', 'arm': False, 'center_raw': -1, 'raw_span': 80.0,
                    'raw_direction': -1, 'pwm_direction': 1,
                    'auto_calibrate': True, 'startup_wait': 10.0}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.port = p('port')
        if not self.port:
            raise ValueError('Arduino port를 지정하세요. 자동으로 장치를 선택하지 않습니다.')
        self.auto_calibrate = p('auto_calibrate')
        self.startup_wait = p('startup_wait')
        StartupCenter(0.0, self.startup_wait)  # 잘못된 대기 설정은 포트를 열기 전에 거부한다.
        self.startup = None
        # 자동 등록에서는 이전 부팅 때의 center_raw를 재사용하지 않는다.
        initial_center = -1 if self.auto_calibrate else p('center_raw')
        self.control = SteeringControl(p('arm'), initial_center, p('raw_span'),
                                       p('raw_direction'), p('pwm_direction'))
        self.control.reason = 'Arduino 연결 대기'
        self.lock = threading.RLock()
        self.stopping = threading.Event()
        self.connected = False
        self.error = ''
        self.last_status = ''
        self.sent_pwm = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Int16MultiArray, '/motor_control', self.target, qos)
        self.raw_pub = self.create_publisher(Int16, '/steering/raw', 1)
        self.pwm_pub = self.create_publisher(Int16, '/steering/pwm', 1)
        self.center_pub = self.create_publisher(Int16, '/steering/center', 1)
        self.create_service(Trigger, '/motor/calibrate', self.calibrate)
        self.create_timer(.05, self.tick)
        self.worker = threading.Thread(target=self.serial_loop, daemon=True)
        self.worker.start()
        self.get_logger().info('구동 PWM은 항상 0입니다. ' + (
            '펌웨어 부팅·센서 안정화 후 중앙 raw를 자동 등록합니다.' if self.auto_calibrate else
            '바퀴를 중앙에 놓고 /motor/calibrate를 호출하세요.'))

    def automatic_pending(self):
        return self.auto_calibrate and (self.startup is None or self.startup.center is None)

    def target(self, msg):
        with self.lock:
            if not self.connected:
                self.control.stop(self.startup.status(time.monotonic()) if self.startup is not None else
                                  'Arduino 연결 대기')
                return
            if self.automatic_pending():
                # 부팅 중 수신한 목표는 보관하지 않는다. 등록 뒤 새로 받은 명령만 처리한다.
                self.control.stop(self.startup.status(time.monotonic()))
                return
            if not self.control.set_target(list(msg.data), time.monotonic()):
                self.get_logger().warning(self.control.reason)

    def calibrate(self, request, response):
        with self.lock:
            try:
                if not self.connected:
                    raise ValueError('Arduino가 연결되지 않았습니다.')
                if self.automatic_pending():
                    raise ValueError('부팅 후 자동 중앙 등록이 끝날 때까지 기다리세요.')
                raw = self.control.calibrate(time.monotonic())
                response.success, response.message = True, f'중앙 raw={raw}, 조향 준비 완료'
            except ValueError as exc:
                response.success, response.message = False, str(exc)
        return response

    def tick(self):
        with self.lock:
            self.control.heartbeat_stamp = time.monotonic()
            if (self.connected and self.control.raw is not None
                    and self.control.feedback_stamp is not None
                    and time.monotonic()-self.control.feedback_stamp <= .5):
                msg = Int16()
                msg.data = self.control.raw
                self.raw_pub.publish(msg)
            status = self.error or self.control.reason
            if self.connected:
                pwm = Int16()
                pwm.data = self.sent_pwm
                self.pwm_pub.publish(pwm)
                if self.control.center is not None:
                    center = Int16()
                    center.data = self.control.center
                    self.center_pub.publish(center)
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def serial_loop(self):
        import serial
        port = None
        try:
            port = serial.Serial(self.port, 115200, timeout=.02, write_timeout=.1)
            opened_at = time.monotonic()
            with self.lock:
                if self.auto_calibrate:
                    self.startup = StartupCenter(opened_at, self.startup_wait)
                    self.control.stop(self.startup.status(opened_at))
            self.write(port, b'0 0\n')
            if self.stopping.wait(2.0):  # Arduino 리셋 중에는 정지 상태를 유지한다.
                return
            port.reset_input_buffer()  # 부팅 중 쌓인 오래된 R 프레임은 안정화 표본으로 쓰지 않는다.
            with self.lock:
                self.connected = not self.stopping.is_set()
                self.control.stop(self.startup.status(time.monotonic()) if self.auto_calibrate else
                                  'R 피드백 / 중앙 보정 대기')
            buffer = bytearray()
            next_send = 0.0
            while not self.stopping.is_set():
                now = time.monotonic()
                with self.lock:
                    if self.automatic_pending():
                        self.control.stop(self.startup.status(now))
                        command = b'0 0\n'
                    else:
                        command = self.control.command(now)
                    requested_pwm = int(command.split()[0])
                    changed = requested_pwm != self.sent_pwm
                    if changed or now >= next_send:
                        self.write(port, command)
                        self.sent_pwm = requested_pwm
                        if changed:
                            message = Int16()
                            message.data = self.sent_pwm
                            self.pwm_pub.publish(message)
                        next_send = now+.05
                size = min(port.in_waiting, 4096)
                if size:
                    buffer.extend(port.read(size))
                    while b'\n' in buffer:
                        line, _, buffer = buffer.partition(b'\n')
                        raw = parse_feedback(line.decode('ascii', errors='replace'))
                        if raw is not None:
                            with self.lock:
                                received_at = time.monotonic()
                                self.control.receive(raw, received_at)
                                if self.automatic_pending():
                                    center = self.startup.receive(raw, received_at)
                                    if center is not None:
                                        self.control.register_center(center, '새 조향 명령 대기')
                                        self.get_logger().info(
                                            f'자동 중앙 등록 완료: raw={center}, 등록 전 명령 폐기 완료')
                    if len(buffer) > 4096:
                        buffer.clear()
                self.stopping.wait(.002)
        except Exception as exc:
            with self.lock:
                self.error = f'시리얼 오류: {exc}'
        finally:
            if port is not None:
                try:
                    self.write(port, b'0 0\n')
                except Exception:
                    pass
                port.close()
            with self.lock:
                self.connected = False
                self.control.stop('시리얼 연결 종료')

    @staticmethod
    def write(port, data):
        if port.write(data) != len(data):
            raise IOError('시리얼 명령 일부만 송신됨')

    def destroy_node(self):
        with self.lock:
            self.control.stop('종료')
        self.stopping.set()
        self.worker.join(timeout=3)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
