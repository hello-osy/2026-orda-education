"""[조향각(deg), 전진 PWM]을 Gazebo Ackermann 차량의 속도로 변환한다."""
import math
import time


WHEELBASE_M = 0.48
STEER_LIMIT_DEG = 30
MAX_SPEED_MPS = 1.0
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

    def receive(self, data, now):
        self.velocity = (0.0, 0.0)
        self.received_at = -math.inf
        self.velocity = motor_to_velocity(data)
        self.received_at = now

    def current(self, now):
        if now - self.received_at >= COMMAND_TIMEOUT_S:
            return 0.0, 0.0
        return self.velocity


def main(args=None):
    # 순수 계산은 ROS 없이도 테스트할 수 있게 ROS import를 실행 진입점에 둔다.
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int16MultiArray

    class SimControlNode(Node):
        def __init__(self):
            super().__init__('sim_control')
            self.state = CommandState()
            self.publisher = self.create_publisher(Twist, '/session_3/sim/cmd_vel', 1)
            self.subscription = self.create_subscription(
                Int16MultiArray, '/motor_control', self.receive, 1)
            self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(0.05, self.publish_command, clock=self.wall_clock)
            self.get_logger().info('/motor_control [조향각 ±30도, PWM ±255], 0.5초 수신 중단 시 정지')

        def receive(self, message):
            try:
                self.state.receive(list(message.data), time.monotonic())
            except ValueError as exc:
                self.get_logger().warning(str(exc) + ' 차량을 정지합니다.')
            self.publish_command()

        def publish_command(self):
            speed, yaw_rate = self.state.current(time.monotonic())
            message = Twist()
            message.linear.x = speed
            message.angular.z = yaw_rate
            self.publisher.publish(message)

        def stop(self):
            self.state = CommandState()
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
