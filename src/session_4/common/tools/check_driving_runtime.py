"""실제 Gazebo 센서만으로 정답코드를 달리며 빨강 정지·초록 재출발을 검사한다.

source scripts/session_4_env.sh
python src/session_4/common/tools/check_driving_runtime.py [--lap]

world pose는 평가기에만 사용한다. 학생/정답 노드에는 전달하지 않는다.
"""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid

from check_sim_runtime import GazeboPoses, stop


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--lap', action='store_true', help='재출발 뒤 한 바퀴도 검사')
    options = parser.parse_args()
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from rcl_interfaces.srv import SetParameters
    from sensor_msgs.msg import Image
    from std_msgs.msg import String, Int16MultiArray
    from session_3.image_utils import image_to_rgb

    base = Path(__file__).resolve().parents[4] / 'workspace/session_4/validation'
    base.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='driving_runtime_', dir=base))
    partition = 'session4_drivecheck_' + uuid.uuid4().hex
    os.environ.update(ROS_DOMAIN_ID='84', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp', GZ_PARTITION=partition,
                      ROS_LOG_DIR=str(output / 'ros_log'))
    env = dict(os.environ)
    rclpy.init(args=[], domain_id=84, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('session4_driving_runtime_check')
    command = [0, 0]
    packets, images, counts = {}, {}, {}
    detector_sources = set()
    control_status = {}
    trace = []
    processes, logs = [], []
    poses = None
    report = {'status': 'running', 'domain': 84, 'partition': partition,
              'hardware_used': False, 'detector': 'reference_yolo', 'lap_requested': options.lap,
              'driver': 'student/driving.py'}
    (output / 'run.json').write_text(json.dumps(report, indent=2) + '\n')

    def motor(message):
        command[:] = list(message.data)

    def packet(key, message):
        packets[key] = json.loads(message.data)
        counts[key] = counts.get(key, 0) + 1

    def camera(key, message):
        images[key] = image_to_rgb(message)
        counts[key] = counts.get(key, 0) + 1

    subscriptions = [node.create_subscription(Int16MultiArray, '/session4/motor_control', motor, 1)]
    for key in ('high', 'low'):
        subscriptions.append(node.create_subscription(Image, f'/camera/{key}/image_raw',
                              lambda message, k=key: camera(k, message), qos_profile_sensor_data))
    parameter_client = node.create_client(SetParameters, '/session4_traffic_signal/set_parameters')

    def launch(name, arguments, direct=False):
        log = (output / f'{name}.log').open('w')
        logs.append(log)
        command_line = [sys.executable] + ([] if direct else [shutil.which('ros2')]) + arguments
        process = subprocess.Popen(command_line,
                                   env=env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    def latest():
        pose = poses.latest()
        if pose is None or time.monotonic() - pose['wall_time'] > 3:
            raise RuntimeError('평가용 world pose 수신 중단')
        return pose

    def until(condition, timeout, label):
        deadline, next_sample = time.monotonic() + timeout, 0.0
        while True:
            from session_4.direct_runtime import status_path
            try:
                state = json.loads(status_path().read_text())
                for key in ('lane', 'cone', 'perception'):
                    if key in state['values']:
                        packets[key] = state['values'][key]
                control_status.update(state['values'].get('control', {}))
                detector_sources.add(state['values'].get('detections', {}).get('source'))
            except (OSError, ValueError, KeyError):
                pass
            if condition():
                return
            if any(process.poll() is not None for process in processes):
                raise RuntimeError(f'{label}: 실행 프로세스 종료. *.log 확인')
            if time.monotonic() >= deadline:
                raise TimeoutError(f'{label}: pose={poses.latest()}, command={command}, inputs={packets}')
            rclpy.spin_once(node, timeout_sec=.03)
            now = time.monotonic()
            if now >= next_sample and poses.latest():
                item = dict(poses.latest(), command=list(command), inputs=dict(packets))
                trace.append(item)
                (output / 'live.json').write_text(json.dumps(item, ensure_ascii=False, indent=2) + '\n')
                next_sample = now + .2

    def save_images(prefix):
        for key, rgb in images.items():
            cv2.imwrite(str(output / f'{prefix}_{key}.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))

    try:
        launch('simulator', ['launch', 'session_4', 'simulator.launch.py', 'gui:=false',
                            f'partition:={partition}', 'signal_mode:=red'])
        poses = GazeboPoses(env, output)
        until(lambda: poses.latest() is not None and len(images) == 2, 90, '센서 준비')
        save_images('spawn')
        student = Path(__file__).resolve().parents[2] / 'student/driving.py'
        launch('driving', [str(student), '--solution'], direct=True)
        until(lambda: len(packets) == 3 and command[1] > 0, 30, '정답코드 출발')
        until(lambda: 'yolo' in detector_sources, 20, '실제 YOLO 검출 결과 수신')
        report['yolo_inference_confirmed'] = True
        until(lambda: control_status.get('viewer_open'), 20, '자동 시각화 창 실행')
        report['automatic_viewer_started'] = True
        report['initial_pose'] = latest()
        print('정답코드 출발. 빨간불 정지를 기다립니다.', flush=True)
        until(lambda: latest()['x'] > 1.8 and command[1] == 0
              and control_status.get('stopped_at_signal')
              and packets.get('perception', {}).get('traffic_light', {}).get('color') == 'red', 180, '빨간불 정지')
        first_stop = latest()
        deadline = time.monotonic() + 1.5
        until(lambda: time.monotonic() >= deadline, 5, '정지 유지')
        stopped = latest()
        report['red_stop_pose'] = stopped
        report['red_stop_drift_m'] = math.hypot(stopped['x'] - first_stop['x'], stopped['y'] - first_stop['y'])
        if report['red_stop_drift_m'] > .08 or stopped['x'] >= 5.5:
            raise RuntimeError('빨간불 정지 유지 또는 신호 전 정지 실패')
        save_images('red_stop')
        until(lambda: parameter_client.service_is_ready(), 10, '신호 변경 서비스')
        request = SetParameters.Request(parameters=[Parameter(name='mode',
            value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='green'))])
        future = parameter_client.call_async(request)
        until(lambda: future.done(), 10, '초록불 변경')
        if not all(result.successful for result in future.result().results):
            raise RuntimeError('신호등 파라미터 변경 실패')
        until(lambda: packets.get('perception', {}).get('traffic_light', {}).get('color') == 'green' and command[1] > 0,
              20, '초록불 영상 인식 및 재출발')
        save_images('green_restart')
        until(lambda: latest()['x'] > 8 and latest()['yaw'] > .15, 90, '신호 통과와 곡선 진입')
        report['after_signal_pose'] = latest()
        if options.lap:
            print('초록불 재출발 통과. 한 바퀴 차선 주행을 확인합니다.', flush=True)
            until(lambda: latest()['y'] > 5.2 and latest()['x'] < 0, 240, '반대쪽 직선')
            until(lambda: latest()['y'] < -5.2
                  and report['initial_pose']['x'] <= latest()['x'] < -3
                  and abs(latest()['yaw']) < .5, 240, '한 바퀴 주행')
            report['cone_mode_observed'] = any(item['inputs'].get('cone', {}).get('valid')
                                           and item['command'][1] > 0 for item in trace)
            if not report['cone_mode_observed']:
                raise RuntimeError('라바콘 유효 결과를 사용한 주행 없음')
            report['lap_completed'] = True
            report['lap_end_pose'] = latest()
            save_images('lap_end')
        # 타원 도로의 중심 직선까지 거리: 오른쪽 차로는 5.0~6.5m 띠입니다.
        lateral_errors = [abs(math.hypot(max(abs(item['x']) - 8, 0), item['y']) - 5.75)
                          for item in trace]
        report['max_right_lane_center_error_m'] = max(lateral_errors)
        if report['max_right_lane_center_error_m'] > .70:
            raise RuntimeError('도로 오른쪽 차로를 벗어난 주행 경로')
        # 역할 프로세스를 중단했을 때 물리 위치가 정지하는지도 확인한다.
        driver = processes.pop()
        # SIGINT는 정답 실행의 Qt 창 close()에 연결되어 있습니다.
        driver.send_signal(signal.SIGINT)
        deadline = time.monotonic() + 8
        try:
            until(lambda: driver.poll() is not None, 8, '시각화 종료 후 주행 종료')
            if driver.returncode != 0:
                raise RuntimeError(f'주행 종료 오류: {driver.returncode}')
            report['viewer_close_stops_driver'] = True
        finally:
            stop(driver)
        deadline = time.monotonic() + 1.2
        until(lambda: time.monotonic() >= deadline, 5, 'watchdog 대기')
        before = latest()
        deadline = time.monotonic() + .8
        until(lambda: time.monotonic() >= deadline, 5, 'watchdog 정지')
        after = latest()
        report['shutdown_drift_m'] = math.hypot(after['x'] - before['x'], after['y'] - before['y'])
        if report['shutdown_drift_m'] > .04:
            raise RuntimeError('주행 노드 중단 후 차량 정지 실패')
        report['status'] = 'passed'
    except Exception as exc:
        report.update(status='failed', error=str(exc))
        raise
    finally:
        for process in reversed(processes):
            stop(process)
        if poses:
            poses.close(output)
        for log in logs:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report['counts'] = counts
        (output / 'trace.json').write_text(json.dumps(trace, ensure_ascii=False, indent=2) + '\n')
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        print(f'검증 기록: {output}', flush=True)


if __name__ == '__main__':
    main()
