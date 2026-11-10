"""[조향각(deg), 전진 PWM]을 Gazebo Ackermann 차량의 속도로 변환한다."""
import math
import time
import json


WHEELBASE_M = 0.48
STEER_LIMIT_DEG = 30
MAX_SPEED_MPS = 2.0
COMMAND_TIMEOUT_S = 0.5


def motor_to_velocity(data):
    """+조향은 왼쪽, +PWM은 전진. 잘못된 입력은 ValueError로 구분한다."""
    if len(data) != 2 or any(not isinstance(x, int) or isinstance(x, bool) for x in data):
        raise ValueError('data는 [조향각, 전진PWM] 정수 2개여야 합니다.')
    steer, pwm = data
    if not -STEER_LIMIT_DEG <= steer <= STEER_LIMIT_DEG or not -255 <= pwm <= 255:
        raise ValueError('조향각은 -30~30도, 전진PWM은 -255~255입니다.')
    speed = MAX_SPEED_MPS * pwm / 255.0
    # Gazebo의 angular.z는 바퀴 각도가 아니라 차체의 회전 속도(rad/s)다.
    yaw_rate = speed * math.tan(math.radians(steer)) / WHEELBASE_M
    return speed, yaw_rate


class CommandState:
    """ROS 시뮬레이션 시각이 멈춰도 실제 경과 시간으로 명령을 만료한다."""
    def __init__(self):
        self.velocity = (0.0, 0.0)
        self.received_at = -math.inf
        self.command = [0, 0]

    def receive(self, data, now):
        self.velocity = (0.0, 0.0)
        self.received_at = -math.inf
        self.command = [0, 0]
        self.velocity = motor_to_velocity(data)
        self.command = list(data)
        self.received_at = now

    def current(self, now):
        if now - self.received_at >= COMMAND_TIMEOUT_S:
            return 0.0, 0.0
        return self.velocity


class ControlSelector:
    """선택한 입력만 적용한다. 모드 전환 전에 받은 명령은 다시 사용하지 않는다."""
    def __init__(self, mode='auto'):
        self.set_mode(mode)

    def set_mode(self, mode):
        if mode not in ('auto', 'manual'):
            raise ValueError('mode는 auto 또는 manual입니다.')
        self.mode = mode
        self.state = CommandState()

    def receive(self, source, data, now):
        if source == self.mode:
            self.state.receive(data, now)

    def status(self, now):
        fresh = 0 <= now - self.state.received_at < COMMAND_TIMEOUT_S
        return {'mode': self.mode, 'fresh': fresh,
                'command': list(self.state.command) if fresh else [0, 0]}


def main(args=None):
    # 순수 계산은 ROS 없이도 테스트할 수 있게 ROS import를 실행 진입점에 둔다.
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int16MultiArray, String
    from rcl_interfaces.msg import SetParametersResult

    class SimControlNode(Node):
        def __init__(self):
            super().__init__('session4_sim_control')
            self.declare_parameter('mode', 'auto')
            self.selector = ControlSelector(self.get_parameter('mode').value)
            self.publisher = self.create_publisher(Twist, '/session4/sim/cmd_vel', 1)
            self.status_publisher = self.create_publisher(String, '/session4/control/state', 1)
            self.subscription = self.create_subscription(
                Int16MultiArray, '/session4/motor_control', lambda message: self.receive('auto', message), 1)
            self.manual_subscription = self.create_subscription(
                Int16MultiArray, '/session4/manual_control', lambda message: self.receive('manual', message), 1)
            self.add_on_set_parameters_callback(self.parameters_changed)
            self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(0.05, self.publish_command, clock=self.wall_clock)
            self.get_logger().info('/session4/motor_control [조향각 ±30도, PWM ±255], 0.5초 수신 중단 시 정지')

        def parameters_changed(self, parameters):
            selected = next((parameter.value for parameter in parameters if parameter.name == 'mode'), None)
            if selected is not None:
                if selected not in ('auto', 'manual'):
                    return SetParametersResult(successful=False, reason='auto/manual 중 선택하세요.')
                self.selector.set_mode(selected)
                self.publish_command()
            return SetParametersResult(successful=True)

        def receive(self, source, message):
            try:
                self.selector.receive(source, list(message.data), time.monotonic())
            except ValueError as exc:
                self.get_logger().warning(str(exc) + ' 차량을 정지합니다.')
            self.publish_command()

        def publish_command(self):
            now = time.monotonic()
            speed, yaw_rate = self.selector.state.current(now)
            message = Twist()
            message.linear.x = speed
            message.angular.z = yaw_rate
            self.publisher.publish(message)
            self.status_publisher.publish(String(data=json.dumps(self.selector.status(now))))

        def stop(self):
            self.selector.set_mode(self.selector.mode)
            self.publish_command()

    # Ctrl+C 뒤에도 정지 메시지를 먼저 발행할 수 있도록 context를 유지한다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SimControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
