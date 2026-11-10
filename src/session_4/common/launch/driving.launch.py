"""학생 파일 하나를 실행하는 호환 launch."""
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from session_4.student_runner import student_source

def start(context):
    command = [sys.executable, str(student_source())]
    if LaunchConfiguration('solution').perform(context) == 'true': command.append('--solution')
    return [ExecuteProcess(cmd=command, output='screen')]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('solution',default_value='false'),OpaqueFunction(function=start)])
