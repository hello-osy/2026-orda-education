"""합성 센서 → 실제 ROS bag SQLite 녹화 → 별도 도메인 재생을 검사한다.

프로젝트 루트에서 실행:
  source scripts/session_3_env.sh 30
  python src/session_3/scripts/check_ros_runtime.py
USB 장치나 모터에는 접근하지 않는다. 결과와 로그는 workspace에 보존한다.
"""
import json
import math
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import tempfile
import time


TOPICS = ('/camera/high/image_raw', '/camera/low/image_raw', '/scan')
RECORD_DOMAIN, REPLAY_DOMAIN = 82, 83


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)


def wait_for(node, process, condition, timeout=30):
    import rclpy
    deadline = time.monotonic() + timeout
    while not condition():
        if process.poll() is not None:
            raise RuntimeError(f'ros2 bag 프로세스가 먼저 종료했습니다: {process.returncode}')
        if time.monotonic() >= deadline:
            raise TimeoutError('30초 안에 ROS 토픽 송수신을 확인하지 못했습니다.')
        rclpy.spin_once(node, timeout_sec=0.05)


def record(ros2, output):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan
    rclpy.init(args=[], domain_id=RECORD_DOMAIN)
    node = Node('session3_synthetic_sensors')
    publishers = [node.create_publisher(kind, topic, qos_profile_sensor_data)
                  for kind, topic in zip((Image, Image, LaserScan), TOPICS)]
    sent = {topic: 0 for topic in TOPICS}

    def publish():
        stamp = node.get_clock().now().to_msg()
        for index, name in enumerate(('high', 'low')):
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = f'camera_{name}_optical_frame'
            image.width, image.height, image.step = 64, 48, 192
            image.encoding = 'bgr8'
            image.data = bytes((20, 100, 50)) * (64 * 48)
            publishers[index].publish(image)
            sent[TOPICS[index]] += 1
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'lidar_link'
        scan.angle_min, scan.angle_increment = -math.pi, math.pi / 180
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        scan.range_min, scan.range_max = 0.15, 12.0
        scan.scan_time = 0.1
        scan.ranges = [1.0 + i / 360 for i in range(360)]
        publishers[2].publish(scan)
        sent[TOPICS[2]] += 1

    timer = node.create_timer(0.1, publish)
    environment = dict(os.environ, ROS_DOMAIN_ID=str(RECORD_DOMAIN))
    bag = output / 'bag'
    process = None
    try:
        with (output / 'record.log').open('w') as log:
            process = subprocess.Popen([ros2, 'bag', 'record', '--storage', 'sqlite3',
                                        '-o', str(bag), *TOPICS],
                                       env=environment, stdout=log, stderr=subprocess.STDOUT)
            wait_for(node, process, lambda: all(p.get_subscription_count() > 0 for p in publishers))
            ready = time.monotonic()
            wait_for(node, process, lambda: time.monotonic() - ready >= 1.5)
            stop(process)
        if process.returncode != 0:
            raise RuntimeError(f'녹화 종료 코드 {process.returncode}: record.log 확인')
    finally:
        if process is not None:
            stop(process)
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()
    if not (bag / 'metadata.yaml').is_file():
        raise RuntimeError('녹화 metadata.yaml이 생성되지 않았습니다.')
    counts = {topic: 0 for topic in TOPICS}
    for database in bag.glob('*.db3'):
        with sqlite3.connect(f'file:{database}?mode=ro', uri=True) as connection:
            rows = connection.execute('SELECT topics.name, COUNT(messages.id) FROM topics '
                                      'LEFT JOIN messages ON topics.id = messages.topic_id GROUP BY topics.id')
            for topic, count in rows:
                if topic in counts:
                    counts[topic] += count
    if not all(counts.values()):
        raise RuntimeError(f'녹화되지 않은 센서 토픽이 있습니다: {counts}')
    return bag, sent, counts


def replay(ros2, output, bag):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, LaserScan
    rclpy.init(args=[], domain_id=REPLAY_DOMAIN)
    node = Node('session3_bag_receiver', parameter_overrides=[Parameter('use_sim_time', value=True)])
    counts = {topic: 0 for topic in TOPICS}
    clocks = set()
    errors = []

    def receive(topic, message):
        if isinstance(message, Image):
            if (message.width, message.height, message.encoding) != (64, 48, 'bgr8'):
                errors.append(f'영상 필드 불일치: {topic}')
            if len(message.data) != message.height * message.step:
                errors.append(f'영상 바이트 수 불일치: {topic}')
        elif len(message.ranges) != 360 or message.ranges[0] != 1.0:
            errors.append('라이다 거리 불일치')
        counts[topic] += 1

    for kind, topic in zip((Image, Image, LaserScan), TOPICS):
        node.create_subscription(kind, topic, lambda msg, name=topic: receive(name, msg), qos_profile_sensor_data)
    node.create_subscription(Clock, '/clock',
                             lambda msg: clocks.add((msg.clock.sec, msg.clock.nanosec)), qos_profile_sensor_data)
    environment = dict(os.environ, ROS_DOMAIN_ID=str(REPLAY_DOMAIN))
    process = None
    try:
        with (output / 'play.log').open('w') as log:
            process = subprocess.Popen([ros2, 'bag', 'play', str(bag), '--clock', '--loop'],
                                       env=environment, stdout=log, stderr=subprocess.STDOUT)
            wait_for(node, process, lambda: all(value >= 3 for value in counts.values()) and len(clocks) >= 3)
            if errors:
                raise RuntimeError('; '.join(errors))
            if node.get_clock().now().nanoseconds <= 0:
                raise RuntimeError('use_sim_time 노드의 /clock 적용 실패')
            stop(process)
        if process.returncode != 0:
            raise RuntimeError(f'재생 종료 코드 {process.returncode}: play.log 확인')
    finally:
        if process is not None:
            stop(process)
        node.destroy_node()
        rclpy.shutdown()
    return counts, len(clocks)


def main():
    ros2 = shutil.which('ros2')
    if not ros2:
        raise RuntimeError('먼저 source scripts/session_3_env.sh 30을 실행하세요.')
    base = Path(__file__).resolve().parents[3] / 'workspace/session_3/validation'
    base.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='ros_runtime_', dir=base))
    os.environ.update(RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_LOG_DIR=str(output / 'ros_log'))
    for name in ('ROS_LOCALHOST_ONLY', 'ROS_STATIC_PEERS', 'ROS_DISCOVERY_SERVER',
                 'ROS_SUPER_CLIENT', 'FASTDDS_DEFAULT_PROFILES_FILE', 'FASTRTPS_DEFAULT_PROFILES_FILE'):
        os.environ.pop(name, None)
    report = {'record_domain': RECORD_DOMAIN, 'replay_domain': REPLAY_DOMAIN,
              'ros2_executable': ros2, 'hardware_used': False}
    try:
        bag, sent, recorded = record(ros2, output)
        received, clocks = replay(ros2, output, bag)
        report.update(status='passed', bag=str(bag), sent=sent, recorded=recorded,
                      replayed=received, distinct_clock_messages=clocks)
    except Exception as exc:
        report.update(status='failed', error=str(exc))
        raise
    finally:
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        print(f'검증 기록: {output}', flush=True)


if __name__ == '__main__':
    main()
