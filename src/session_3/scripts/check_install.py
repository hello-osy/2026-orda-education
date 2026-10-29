"""추가 설치/ROS Python ABI/시뮬레이터 의존성을 실행 전에 검사한다."""
import argparse
import importlib
import platform
import shutil
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true')
    args = parser.parse_args()
    errors = []
    for name in ('rclpy', 'launch', 'launch_ros', 'sensor_msgs.msg', 'std_msgs.msg', 'std_srvs.srv',
                 'geometry_msgs.msg', 'numpy', 'cv2', 'PySide6.QtWidgets', 'serial', 'rplidar'):
        try:
            importlib.import_module(name)
        except Exception as exc:
            errors.append(f'{name}: {exc}')
    if args.sim:
        if not shutil.which('gz'):
            errors.append('Gazebo 없음: docs/SESSION_3.md의 OS별 추가 설치를 실행하세요.')
        try:
            from ament_index_python.packages import get_package_prefix
            get_package_prefix('ros_gz_bridge')
        except Exception:
            errors.append('ros_gz_bridge 없음: Ubuntu/WSL은 ros-jazzy-ros-gz, Mac은 session_3_macos_bridge.sh 설치 필요')
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        print('docs/SESSION_3.md의 환경 세팅을 확인하세요. Gazebo는 pip로 설치하지 않습니다.', file=sys.stderr)
        return 1
    print(f'Session 3 라이브러리 OK · {platform.system()} · Python {platform.python_version()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
