"""오상영 사전 준비: USB 장치를 소유하는 노드만 차량 연결 PC에서 실행."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {'camera_high': '0', 'camera_low': '1', 'lidar_port': '', 'motor_port': '',
                'arm': 'false', 'center_raw': '-1', 'raw_span': '80.0',
                'raw_direction': '-1', 'pwm_direction': '1', 'lidar_yaw': '0.0',
                'auto_calibrate': 'true', 'startup_wait': '10.0'}
    actions = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    value = lambda key, kind: ParameterValue(LaunchConfiguration(key), value_type=kind)
    nodes = []
    for name in ('high', 'low'):
        nodes.append(Node(package='session_3', executable='camera_node',
                          name=f'camera_{name}', output='screen', parameters=[{
                              'camera_name': name, 'device': value(f'camera_{name}', int)}]))
    nodes.append(Node(package='session_3', executable='lidar_node', output='screen',
                      parameters=[{'port': value('lidar_port', str),
                                   'mount_yaw_deg': value('lidar_yaw', float)}]))
    nodes.append(Node(package='session_3', executable='motor_node', output='screen',
                      parameters=[{'port': value('motor_port', str), 'arm': value('arm', bool),
                                   'center_raw': value('center_raw', int),
                                   'raw_span': value('raw_span', float),
                                   'raw_direction': value('raw_direction', int),
                                   'pwm_direction': value('pwm_direction', int),
                                   'auto_calibrate': value('auto_calibrate', bool),
                                   'startup_wait': value('startup_wait', float)}]))
    for node in nodes:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='차량 노드 종료'))])))
    return LaunchDescription(actions+nodes)
