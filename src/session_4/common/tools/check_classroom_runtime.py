"""실제 ROS 제어 모드·녹화·사진 추출을 검사한다. Gazebo나 실물 장치는 쓰지 않는다.

source scripts/session_4_env.sh
python src/session_4/common/tools/check_classroom_runtime.py

격리 도메인 87에서 검사하며, 직접 만든 bag만 사용하고 최신 bag 표시는 원래대로 돌린다.
"""
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid


DOMAIN = 87


def stop_group(process):
    """이 검사에서 start_new_session으로 만든 프로세스 그룹만 종료한다."""
    if process is None:
        return
    for signum, timeout in ((signal.SIGINT, 12), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue
    raise RuntimeError(f'검사 프로세스를 종료하지 못했습니다: {process.pid}')


def restore_owned_marker(marker, previous, own_bag):
    """다른 녹화가 최신 표시를 갱신했다면 덮어쓰지 않는다."""
    marker, own_bag = Path(marker), Path(own_bag).resolve()
    if not marker.exists():
        return {'restored': previous is None, 'reason': 'latest marker absent'}
    try:
        current = json.loads(marker.read_text(encoding='utf-8'))
        path = Path(current.get('path', '')).expanduser().resolve()
    except (OSError, ValueError, TypeError):
        return {'restored': False, 'reason': 'unrecognized marker left untouched'}
    if path != own_bag:
        return {'restored': False, 'reason': 'another recording owns latest marker'}
    if previous is None:
        marker.unlink()
    else:
        temporary = marker.with_name(marker.name + '.' + uuid.uuid4().hex + '.restore')
        try:
            temporary.write_bytes(previous)
            temporary.replace(marker)
        finally:
            temporary.unlink(missing_ok=True)
    return {'restored': True, 'reason': 'previous marker restored' if previous else 'test marker removed'}


def main():
    import numpy as np
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image
    from std_msgs.msg import Int16MultiArray, String
    from session_4.control_mode import set_mode
    from session_4.dataset_tools import extract_dataset_frames, find_project_root
    from session_4.recording import bag_summary
    from session_4.sim_control_node import motor_to_velocity

    root = find_project_root()
    base = root / 'workspace/session_4/validation'
    base.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='classroom_runtime_', dir=base))
    bag = output / 'bag'
    marker = root / 'workspace/session_4/latest_bag.json'
    previous_marker = marker.read_bytes() if marker.exists() else None
    report = {'status': 'running', 'domain': DOMAIN, 'hardware_used': False,
              'gazebo_used': False, 'real_ros_processes': True,
              'previous_latest_bag_existed': previous_marker is not None}
    environment = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN),
                       RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                       ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                       ROS_LOG_DIR=str(output / 'ros_log'))
    for key in ('ROS_LOCALHOST_ONLY', 'ROS_STATIC_PEERS', 'ROS_DISCOVERY_SERVER',
                'ROS_SUPER_CLIENT', 'FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE'):
        environment.pop(key, None)
        os.environ.pop(key, None)
    os.environ.update({key: environment[key] for key in
                       ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_LOG_DIR')})
    rclpy.init(args=[], domain_id=DOMAIN, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('session4_classroom_runtime_probe')
    processes, logs = {}, []
    states, velocities = deque(maxlen=4000), deque(maxlen=4000)
    enabled = {'auto': False, 'manual': True}
    commands = {'auto': [10, 120], 'manual': [-17, 190]}
    sent = {'high': 0, 'low': 0, 'auto': 0, 'manual': 0}
    publishers = {
        'auto': node.create_publisher(Int16MultiArray, '/session4/motor_control', 1),
        'manual': node.create_publisher(Int16MultiArray, '/session4/manual_control', 1),
        'high': node.create_publisher(Image, '/camera/high/image_raw', qos_profile_sensor_data),
        'low': node.create_publisher(Image, '/camera/low/image_raw', qos_profile_sensor_data),
    }
    pixels = np.full((240, 320, 3), (65, 70, 75), dtype=np.uint8)
    pixels[:100] = (205, 220, 230)
    pixels[35:55, 155:175] = (255, 0, 0)
    pixels[145:210, 220:240] = (255, 115, 0)
    pixel_bytes = pixels.tobytes()

    def state_received(message):
        states.append((time.monotonic(), json.loads(message.data)))

    subscriptions = [
        node.create_subscription(String, '/session4/control/state', state_received, 1),
        node.create_subscription(Twist, '/session4/sim/cmd_vel',
            lambda message: velocities.append((time.monotonic(), message.linear.x, message.angular.z)), 1),
    ]

    def publish():
        for source in ('auto', 'manual'):
            if enabled[source]:
                publishers[source].publish(Int16MultiArray(data=list(commands[source])))
                sent[source] += 1
        for name in ('high', 'low'):
            message = Image()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = 'camera_' + name
            message.width, message.height, message.step = 320, 240, 960
            message.encoding = 'rgb8'
            message.data = pixel_bytes
            publishers[name].publish(message)
            sent[name] += 1

    timer = node.create_timer(.1, publish)

    def start(name, arguments):
        log = (output / (name + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen([sys.executable] + arguments, env=environment, cwd=root,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes[name] = process
        return process

    def until(condition, timeout=15, description='ROS 상태 대기'):
        deadline = time.monotonic() + timeout
        while not condition():
            failed = {name: process.returncode for name, process in processes.items()
                      if process.poll() is not None}
            if failed:
                raise RuntimeError(f'{description}: 검사 프로세스 조기 종료 {failed}')
            if time.monotonic() >= deadline:
                latest = states[-1] if states else None
                raise TimeoutError(f'{description}: state={latest}, velocities={list(velocities)[-3:]}')
            rclpy.spin_once(node, timeout_sec=.03)

    def advance(seconds):
        deadline = time.monotonic() + seconds
        until(lambda: time.monotonic() >= deadline, timeout=seconds + 3)

    def matches(mode, command, fresh=True):
        if not states or not velocities:
            return False
        status = states[-1][1]
        speed, yaw = motor_to_velocity(command)
        return (status['mode'] == mode and status['fresh'] is fresh
                and status['command'] == command
                and math.isclose(velocities[-1][1], speed, abs_tol=1e-9)
                and math.isclose(velocities[-1][2], yaw, abs_tol=1e-9))

    try:
        print(f'제어·녹화 수업 흐름 검사: {output}', flush=True)
        start('sim_control', ['-m', 'session_4.sim_control_node'])
        until(lambda: len(states) >= 3 and publishers['manual'].get_subscription_count() > 0)
        advance(.4)
        if not matches('auto', [0, 0], False):
            raise AssertionError('기본 auto 모드에서 manual 명령을 무시하지 않았습니다.')
        report['default_auto_ignores_manual'] = True
        enabled['auto'] = True
        until(lambda: matches('auto', commands['auto']))
        advance(.3)
        if not matches('auto', commands['auto']):
            raise AssertionError('auto 모드에서 다른 manual 명령과 충돌했습니다.')
        report['auto_applied'] = dict(states[-1][1])

        enabled['manual'] = False
        set_mode(node, 'manual')
        until(lambda: matches('manual', [0, 0], False))
        report['switch_manual_resets_command'] = True
        commands['manual'] = [-14, 160]
        enabled['manual'] = True
        until(lambda: matches('manual', commands['manual']))
        since = time.monotonic()
        advance(.4)
        recent = [status for stamp, status in states if stamp > since + .1]
        if not recent or any(status['command'] != commands['manual'] for status in recent):
            raise AssertionError('manual 모드에서 auto 명령이 적용됐습니다.')
        report['manual_applied_auto_ignored'] = dict(states[-1][1])

        enabled['manual'] = False
        paused = time.monotonic()
        until(lambda: matches('manual', [0, 0], False), timeout=3,
              description='manual 미수신 watchdog 정지')
        first_zero = next(stamp for stamp, speed, yaw in velocities
                          if stamp > paused + .1 and speed == 0 and yaw == 0)
        advance(.3)
        if not matches('manual', [0, 0], False):
            raise AssertionError('manual 미수신 이후 정지를 유지하지 못했습니다.')
        report['manual_timeout_stop_after_s'] = first_zero - paused

        enabled['auto'] = False
        enabled['manual'] = True
        until(lambda: matches('manual', commands['manual']))
        enabled['manual'] = False
        set_mode(node, 'auto')
        until(lambda: matches('auto', [0, 0], False))
        advance(.25)
        if not matches('auto', [0, 0], False):
            raise AssertionError('auto 전환 이후 이전 명령을 다시 사용했습니다.')
        report['switch_auto_requires_fresh_auto'] = True
        commands['auto'] = [6, 90]
        enabled['auto'] = True
        until(lambda: matches('auto', commands['auto']))
        report['fresh_auto_after_switch'] = dict(states[-1][1])

        recorder = start('recording', ['-m', 'session_4.recording', '--record-only', '--output', str(bag)])
        until(lambda: all(publishers[name].get_subscription_count() > 0 for name in ('high', 'low')),
              description='실제 ros2 bag 카메라 구독 준비')
        first_high = sent['high']
        until(lambda: sent['high'] - first_high >= 18, timeout=6,
              description='카메라 약 18장 녹화')
        advance(.2)
        processes.pop('recording')
        stop_group(recorder)
        if recorder.returncode != 0:
            raise RuntimeError(f'실제 녹화 도구 종료 코드 {recorder.returncode}; recording.log 확인')
        counts = bag_summary(bag)
        if counts.get('/camera/high/image_raw', 0) < 15 or counts.get('/camera/low/image_raw', 0) < 15:
            raise AssertionError(f'카메라 녹화 수 부족: {counts}')
        latest = json.loads(marker.read_text(encoding='utf-8'))
        if latest.get('status') != 'completed' or Path(latest.get('path', '')).resolve() != bag:
            raise AssertionError('정상 녹화의 최신 완료 표시가 올바르지 않습니다.')
        before = {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in bag.glob('*.db3')}
        extraction = extract_dataset_frames(bag=bag, output=output / 'extracted', stride=5,
                                            project_root=root)
        expected = counts['/camera/high/image_raw'] // 5
        if extraction['counts'] != {'/camera/high/image_raw': expected}:
            raise AssertionError(f'5프레임 간격 추출 수 불일치: {extraction}')
        after = {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in bag.glob('*.db3')}
        if before != after:
            raise AssertionError('사진 추출 중 원본 SQLite bag 내용이 바뀌었습니다.')
        report['recording'] = {'status': 'passed', 'bag': str(bag), 'counts': counts,
                               'latest_marker_completed': True}
        report['extraction'] = {'status': 'passed', 'count': expected,
                               'source_bag_unchanged': True, 'output': extraction['path']}
        report['status'] = 'passed'
    except BaseException as error:
        report.update(status='failed', error=str(error))
        raise
    finally:
        commands['auto'] = commands['manual'] = [0, 0]
        for source in ('auto', 'manual'):
            publishers[source].publish(Int16MultiArray(data=[0, 0]))
        for process in reversed(list(processes.values())):
            stop_group(process)
        for log in logs:
            log.close()
        report['latest_bag_restore'] = restore_owned_marker(marker, previous_marker, bag)
        report['sent'] = sent
        (output / 'control_trace.json').write_text(json.dumps(list(states), indent=2) + '\n')
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        print(f'검증 기록: {output}', flush=True)


if __name__ == '__main__':
    main()
