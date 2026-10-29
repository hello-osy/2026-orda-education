"""두 카메라와 라이다 시각화 창을 함께 연다."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('high_topic', default_value='/camera/high/image_raw'),
        DeclareLaunchArgument('low_topic', default_value='/camera/low/image_raw'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    viewers = [
        Node(package='session_3', executable='camera_viewer', name='high_camera_viewer',
             parameters=[clock, {'topic': LaunchConfiguration('high_topic')}]),
        Node(package='session_3', executable='camera_viewer', name='low_camera_viewer',
             parameters=[clock, {'topic': LaunchConfiguration('low_topic')}]),
        Node(package='session_3', executable='lidar_viewer', name='lidar_viewer',
             parameters=[clock, {'topic': LaunchConfiguration('scan_topic')}]),
    ]
    return LaunchDescription(arguments + viewers)
