"""직접 운전하는 창과 SQLite bag 녹화를 함께 실행하는 완성된 수업 도구."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import sqlite3
import subprocess
import sys
import time

from .dataset_tools import find_project_root, write_json_atomic

TOPICS = ['/camera/high/image_raw', '/camera/low/image_raw', '/scan',
          '/session4/motor_control', '/session4/manual_control', '/session4/control/state', '/clock']


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    for signum, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def bag_summary(folder):
    """카메라가 실제로 기록된 bag만 다음 이미지 추출의 기본값으로 등록한다."""
    folder = Path(folder)
    if not (folder / 'metadata.yaml').is_file():
        raise ValueError('녹화가 정상 종료되지 않았습니다. metadata.yaml이 없습니다.')
    counts = {}
    for path in sorted(folder.glob('*.db3')):
        with sqlite3.connect(path.resolve().as_uri() + '?mode=ro', uri=True) as db:
            for name, count in db.execute('SELECT topics.name, COUNT(messages.id) FROM topics '
                    'LEFT JOIN messages ON topics.id=messages.topic_id GROUP BY topics.id'):
                counts[name] = counts.get(name, 0) + count
    if counts.get('/camera/high/image_raw', 0) == 0:
        raise ValueError('카메라 영상이 녹화되지 않았습니다. 시뮬레이터를 먼저 켜고 다시 실행하세요.')
    return counts


def main(argv=None):
    parser = argparse.ArgumentParser(description='직접 운전하며 카메라·라이다 녹화')
    parser.add_argument('--record-only', action='store_true', help='운전 창 없이 녹화만 실행')
    parser.add_argument('--output', help='새 bag 폴더. 생략하면 날짜별 새 폴더 사용')
    args = parser.parse_args(argv)
    root = find_project_root()
    base = root / 'workspace/session_4'
    path = Path(args.output).expanduser().resolve() if args.output else (
        base / 'bags' / datetime.now().strftime('drive_%Y%m%d_%H%M%S_%f'))
    if path.exists():
        parser.error('이미 있는 폴더에는 덮어쓰지 않습니다. 새 폴더를 지정하세요.')
    path.parent.mkdir(parents=True, exist_ok=True)
    ros2 = shutil.which('ros2')
    if not ros2:
        parser.error('ROS 환경을 먼저 불러오세요: source scripts/session_4_env.sh')
    record = manual = None
    failure = None
    try:
        command = [sys.executable, ros2, 'bag', 'record', '--disable-keyboard-controls',
                   '-s', 'sqlite3', '-o', str(path), '--topics'] + TOPICS
        print(f'녹화 저장 위치: {path}', flush=True)
        record = subprocess.Popen(command, cwd=root, start_new_session=True)
        if not args.record_only:
            manual = subprocess.Popen([sys.executable, ros2, 'run', 'session_4', 'manual_drive'],
                                      cwd=root, start_new_session=True)
            print('운전이 끝나면 운전 창을 닫으세요. 녹화도 함께 저장합니다.', flush=True)
        else:
            print('녹화를 마치려면 Ctrl+C를 누르세요.', flush=True)
        while record.poll() is None and (manual is None or manual.poll() is None):
            time.sleep(.1)
        if record.poll() is not None:
            failure = f'녹화 프로그램이 먼저 종료됐습니다(종료 코드 {record.returncode}).'
        elif manual is not None and manual.returncode != 0:
            failure = f'직접 운전 창이 오류로 종료됐습니다(종료 코드 {manual.returncode}).'
    except KeyboardInterrupt:
        pass
    finally:
        stop_process(manual)
        stop_process(record)
    if failure:
        raise RuntimeError(failure)
    if record is None or record.returncode != 0:
        raise RuntimeError('녹화 종료 오류. 출력 메시지와 bag 폴더를 확인하세요.')
    counts = bag_summary(path)
    marker = {'status': 'completed', 'path': str(path.resolve()), 'storage': 'sqlite3',
              'counts': counts, 'created_at': datetime.now().isoformat(timespec='seconds')}
    write_json_atomic(base / 'latest_bag.json', marker)
    print(json.dumps({'녹화 완료': str(path), '기록한 데이터 수': counts}, ensure_ascii=False, indent=2))
    print('다음 명령: bash scripts/session_4.sh extract', flush=True)


if __name__ == '__main__':
    main()
