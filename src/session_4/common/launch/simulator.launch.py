"""Gazebo 2차선 트랙·카메라·라이다·가상 차량·신호등을 함께 실행한다."""
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
    share = Path(get_package_share_directory('session_4'))
    if shutil.which('gz') is None:
        raise RuntimeError('Gazebo가 없습니다. docs/SESSION_3.md의 추가 설치를 완료하세요.')
    get_package_share_directory('ros_gz_bridge')
    resources = str(share/'models')
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        resources += os.pathsep+os.environ['GZ_SIM_RESOURCE_PATH']
    env = {'GZ_SIM_RESOURCE_PATH': resources,
           'GZ_PARTITION': LaunchConfiguration('partition').perform(context)}
    options = ['--render-engine', LaunchConfiguration('render_engine').perform(context),
               '--render-engine-api-backend', LaunchConfiguration('render_backend').perform(context)]
    server = ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '-v', '2',
                                str(share/'worlds/collaboration_track.sdf')]+options,
                            output='screen', additional_env=env)
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', name='session4_bridge',
                  parameters=[{'config_file': str(share/'config/bridge.yaml')}],
                  additional_env=env, output='screen')
    control = Node(package='session_4', executable='sim_control', output='screen')
    signal = Node(package='session_4', executable='traffic_signal', output='screen', additional_env=env,
                  parameters=[{'use_sim_time': True,
                               'mode': LaunchConfiguration('signal_mode').perform(context)}])
    processes = [server, bridge, control, signal]
    if LaunchConfiguration('gui').perform(context).lower() == 'true':
        processes.append(ExecuteProcess(cmd=['gz', 'sim', '-g', '-v', '2']+options,
                                        output='screen', additional_env=env))
    actions = []
    for process in processes:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=process, on_exit=[
            EmitEvent(event=Shutdown(reason='Session 4 구성 요소 종료'))])))
    return actions+processes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('partition', default_value='orda_session4_'+uuid.uuid4().hex),
        DeclareLaunchArgument('signal_mode', default_value='cycle', choices=['cycle', 'red', 'green', 'yellow']),
        DeclareLaunchArgument('render_engine', default_value='ogre2', choices=['ogre2', 'ogre']),
        DeclareLaunchArgument('render_backend',
                              default_value='metal' if platform.system() == 'Darwin' else 'opengl',
                              choices=['opengl', 'metal', 'vulkan']),
        OpaqueFunction(function=start),
    ])
