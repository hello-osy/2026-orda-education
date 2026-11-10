"""수업 실행에 필요한 ROS Python, Gazebo bridge를 검사한다."""
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
    for name in ('rclpy', 'launch_ros', 'sensor_msgs.msg', 'std_msgs.msg',
                 'geometry_msgs.msg', 'numpy', 'cv2', 'yaml'):
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
            errors.append('ros_gz_bridge 없음: docs/SESSION_3.md의 bridge 설치를 진행하세요.')
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        print('docs/SESSION_4.md의 환경 세팅을 확인하세요.', file=sys.stderr)
        return 1
    print(f'Session 4 라이브러리 OK · {platform.system()} · Python {platform.python_version()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
