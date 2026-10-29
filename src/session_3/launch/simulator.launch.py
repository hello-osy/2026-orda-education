"""Gazebo 서버/GUI를 분리 실행해 macOS에서도 같은 명령을 사용한다."""
import os
import platform
import shutil
import uuid
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory('session_3'))
    if shutil.which('gz') is None:
        raise RuntimeError('gz 명령이 없습니다. docs/SESSION_3.md의 Gazebo 추가 설치를 완료하세요.')
    get_package_share_directory('ros_gz_bridge')
    engine = LaunchConfiguration('render_engine').perform(context)
    backend = LaunchConfiguration('render_backend').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    # 모델 경로는 설치 위치에서 찾는다. 외부 Fuel 모델이나 절대 경로가 필요 없다.
    resources = str(share / 'models')
    previous = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if previous:
        resources += os.pathsep + previous
    # 같은 핫스팟/ROS domain을 쓰는 다른 수강생의 Gazebo와 섞이지 않는다.
    env = {'GZ_SIM_RESOURCE_PATH': resources,
           'GZ_PARTITION': LaunchConfiguration('partition').perform(context)}
    options = ['--render-engine', engine, '--render-engine-api-backend', backend]
    server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v', '2', str(share / 'worlds' / 'solid_track.sdf')] + options,
        output='screen', additional_env=env)
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='gazebo_bridge',
                  parameters=[{'config_file': str(share / 'config' / 'bridge.yaml')}],
                  additional_env=env, output='screen')
    # 제어 watchdog에는 sim time을 사용하지 않는다.
    control = Node(package='session_3', executable='sim_control', name='sim_control', output='screen')
    actions = []
    for process, reason in [(server, 'Gazebo 서버 종료'), (bridge, 'Gazebo 연결 종료'),
                            (control, '시뮬레이터 제어 노드 종료')]:
        # 제어/브리지 오류 때도 Gazebo가 마지막 속도로 계속 달리지 않게 함께 종료한다.
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process, on_exit=[
            EmitEvent(event=Shutdown(reason=reason))])))
    actions.extend([server, bridge, control])
    if gui:
        window = ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', '2'] + options,
                                output='screen', additional_env=env)
        actions += [window, RegisterEventHandler(OnProcessExit(target_action=window, on_exit=[
            EmitEvent(event=Shutdown(reason='Gazebo 창 종료'))]))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('partition', default_value='orda_session3_' + uuid.uuid4().hex,
                              description='Gazebo 통신 구분값. 기본값은 실행마다 달라집니다.'),
        DeclareLaunchArgument('render_engine', default_value='ogre2', choices=['ogre2', 'ogre']),
        DeclareLaunchArgument('render_backend',
                              default_value='metal' if platform.system() == 'Darwin' else 'opengl',
                              choices=['opengl', 'metal', 'vulkan']),
        OpaqueFunction(function=start),
    ])
