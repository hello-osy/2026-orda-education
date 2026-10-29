"""실제 Gazebo 센서·물리 이동·명령 중단 정지를 검사한다(실물 장치 사용 안 함).

source scripts/session_3_env.sh 30
python src/session_3/scripts/check_sim_runtime.py

검사는 별도 로컬 ROS 도메인 82와 고유 Gazebo partition에서 실행한다.
센서 PNG, 실제 world pose, 실행 로그와 result.json을 workspace에 보존한다.
"""
from collections import deque
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid


def stop(process):
    if process is not None and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)


class GazeboPoses:
    """SceneBroadcaster의 world pose를 읽는다. 바퀴 적분 odometry가 아니다."""
    def __init__(self, environment, output):
        self.samples = deque(maxlen=20000)
        self.lock = threading.Lock()
        self.error_log = (output / 'pose.log').open('w')
        self.process = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', '/world/orda_track/dynamic_pose/info', '--json-output'],
            env=environment, stdout=subprocess.PIPE, stderr=self.error_log,
            text=True, bufsize=1, start_new_session=True)
        self.reader = threading.Thread(target=self.read, daemon=True)
        self.reader.start()

    def read(self):
        decoder = json.JSONDecoder()
        buffer = ''
        for line in self.process.stdout:
            buffer += line
            while buffer.strip():
                buffer = buffer.lstrip()
                if not buffer.startswith('{'):
                    start = buffer.find('{')
                    buffer = buffer[start:] if start >= 0 else ''
                    if not buffer:
                        break
                try:
                    message, length = decoder.raw_decode(buffer)
                except json.JSONDecodeError:
                    break
                buffer = buffer[length:]
                for pose in message.get('pose', []):
                    if pose.get('name') != 'orda_car':
                        continue
                    position = pose.get('position', {})
                    q = pose.get('orientation', {})
                    x, y, z, w = (float(q.get(k, 0 if k != 'w' else 1)) for k in ('x', 'y', 'z', 'w'))
                    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                    sample = {'wall_time': time.monotonic(), 'x': float(position.get('x', 0)),
                              'y': float(position.get('y', 0)), 'z': float(position.get('z', 0)), 'yaw': yaw}
                    with self.lock:
                        self.samples.append(sample)

    def latest(self):
        with self.lock:
            return dict(self.samples[-1]) if self.samples else None

    def close(self, output):
        stop(self.process)
        self.reader.join(timeout=3)
        self.error_log.close()
        with self.lock:
            (output / 'world_poses.json').write_text(json.dumps(list(self.samples), indent=2) + '\n')


def main():
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from std_msgs.msg import Int16MultiArray
    from session_3.image_utils import image_to_rgb

    ros2 = shutil.which('ros2')
    if not ros2 or not shutil.which('gz'):
        raise RuntimeError('ROS 환경과 Gazebo를 먼저 설치하세요.')
    base = Path(__file__).resolve().parents[3] / 'workspace/session_3/validation'
    base.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='sim_runtime_', dir=base))
    partition = 'orda_check_' + uuid.uuid4().hex
    os.environ.update(ROS_DOMAIN_ID='82', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', GZ_PARTITION=partition,
                      ROS_LOG_DIR=str(output / 'ros_log'))
    for name in ('ROS_LOCALHOST_ONLY', 'ROS_STATIC_PEERS', 'ROS_DISCOVERY_SERVER',
                 'ROS_SUPER_CLIENT', 'FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE'):
        os.environ.pop(name, None)
    environment = dict(os.environ)
    rclpy.init(args=[], domain_id=82, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('session3_sim_runtime_check')
    motor = node.create_publisher(Int16MultiArray, '/motor_control', 1)
    command = [None]
    images, infos = {}, {}
    counts = {'high': 0, 'low': 0, 'scan': 0, 'clock': 0}
    scan = [None]
    clock = [0.0]

    def send_command():
        if command[0] is not None:
            motor.publish(Int16MultiArray(data=command[0]))

    def image_received(name, message):
        images[name] = image_to_rgb(message)
        counts[name] += 1

    def scan_received(message):
        scan[0] = message
        counts['scan'] += 1

    def clock_received(message):
        clock[0] = message.clock.sec + message.clock.nanosec / 1e9
        counts['clock'] += 1

    node.create_timer(0.05, send_command)
    for name in ('high', 'low'):
        node.create_subscription(Image, f'/camera/{name}/image_raw',
                                 lambda msg, key=name: image_received(key, msg), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, f'/camera/{name}/camera_info',
                                 lambda msg, key=name: infos.update({key: msg}), qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', scan_received, qos_profile_sensor_data)
    node.create_subscription(Clock, '/clock', clock_received, qos_profile_sensor_data)
    launch = None
    poses = None
    report = {'status': 'running', 'domain': 82, 'partition': partition, 'hardware_used': False}

    def until(condition, timeout=90):
        deadline = time.monotonic() + timeout
        while not condition():
            if launch.poll() is not None:
                raise RuntimeError(f'Gazebo launch 종료({launch.returncode}); simulator.log 확인')
            if poses.process.poll() is not None:
                raise RuntimeError('Gazebo pose 구독 종료; pose.log 확인')
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Gazebo 응답 시간 초과: counts={counts}, pose={poses.latest()}')
            rclpy.spin_once(node, timeout_sec=0.05)

    def advance(seconds):
        deadline = time.monotonic() + seconds
        until(lambda: time.monotonic() >= deadline, timeout=seconds + 5)

    def fresh_pose():
        value = poses.latest()
        if value is None or time.monotonic() - value['wall_time'] > 2:
            raise RuntimeError('Gazebo world pose 수신이 중단되었습니다.')
        return value

    def distance(a, b):
        return math.hypot(a['x'] - b['x'], a['y'] - b['y'])

    try:
        with (output / 'simulator.log').open('w') as log:
            print('Gazebo 센서와 world pose 수신을 기다립니다.', flush=True)
            launch = subprocess.Popen([sys.executable, ros2, 'launch', 'session_3',
                                       'simulator.launch.py', 'gui:=false', f'partition:={partition}'],
                                      env=environment, stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            poses = GazeboPoses(environment, output)
            until(lambda: all(value >= 3 for value in counts.values()) and len(infos) == 2
                  and poses.latest() is not None and motor.get_subscription_count() > 0)
            for name, rgb in images.items():
                if rgb.shape != (240, 320, 3) or float(rgb.std(axis=(0, 1)).max()) < 3:
                    raise RuntimeError(f'{name} 카메라 크기/렌더링 이상: {rgb.shape}')
                if infos[name].k[0] <= 0:
                    raise RuntimeError(f'{name} 시뮬레이션 CameraInfo가 보정되지 않았습니다.')
                cv2.imwrite(str(output / f'{name}_camera.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            valid_ranges = [x for x in scan[0].ranges if math.isfinite(x)
                            and scan[0].range_min <= x <= scan[0].range_max]
            if len(scan[0].ranges) != 360 or len(valid_ranges) < 10:
                raise RuntimeError('360도 라이다의 유효 거리 수신 실패')
            report['sensor_counts'] = dict(counts)
            report['finite_lidar_rays'] = len(valid_ranges)
            report['clock_seconds'] = clock[0]
            print('두 카메라·라이다·clock 수신 완료. 실제 차량 이동·정지를 확인합니다.', flush=True)
            command[0] = [0, 0]
            advance(0.5)
            initial = fresh_pose()
            command[0] = [0, 150]
            until(lambda: distance(initial, fresh_pose()) > 0.3, timeout=12)
            moved = fresh_pose()
            report['straight_distance_m'] = distance(initial, moved)
            command[0] = [15, 150]
            until(lambda: abs(math.atan2(math.sin(fresh_pose()['yaw'] - moved['yaw']),
                                        math.cos(fresh_pose()['yaw'] - moved['yaw']))) > 0.08, timeout=12)
            turned = fresh_pose()
            report['left_turn_radians'] = math.atan2(math.sin(turned['yaw'] - moved['yaw']),
                                                    math.cos(turned['yaw'] - moved['yaw']))
            if report['left_turn_radians'] <= 0:
                raise RuntimeError('양수 조향에서 차체가 오른쪽으로 회전했습니다.')
            command[0] = [0, 0]
            advance(1)
            stopped = fresh_pose()
            advance(0.8)
            report['explicit_stop_drift_m'] = distance(stopped, fresh_pose())
            if report['explicit_stop_drift_m'] > 0.03:
                raise RuntimeError('정지 명령 후 차량 이동이 계속됩니다.')
            command[0] = [0, 150]
            until(lambda: distance(stopped, fresh_pose()) > 0.2, timeout=12)
            command[0] = None  # 새 명령을 보내지 않아 watchdog을 검사한다.
            advance(1.2)
            timed_out = fresh_pose()
            advance(0.8)
            report['watchdog_stop_drift_m'] = distance(timed_out, fresh_pose())
            if report['watchdog_stop_drift_m'] > 0.03:
                raise RuntimeError('명령 발행 중단 후 차량 이동이 계속됩니다.')
            report['status'] = 'passed'
    except Exception as exc:
        report.update(status='failed', error=str(exc))
        raise
    finally:
        command[0] = [0, 0]
        send_command()
        if poses is not None:
            poses.close(output)
        stop(launch)
        node.destroy_node()
        rclpy.shutdown()
        report['final_counts'] = counts
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        print(f'검증 기록: {output}', flush=True)


if __name__ == '__main__':
    main()
