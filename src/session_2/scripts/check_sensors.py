"""제한 시간 동안 실수신 확인. 자동/명시한 카메라와 명시한 A1 포트만 사용."""
import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from session_2.sensors import camera_worker, lidar_worker
from session_2.viewer import SensorProcess
from session_2.platform_support import default_camera_indices


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port')
    parser.add_argument('--cameras', nargs='*', type=int, default=default_camera_indices())
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=15)
    args = parser.parse_args()
    workers = {f'camera_{i}': SensorProcess(camera_worker, (i, args.width, args.height, args.fps)) for i in args.cameras}
    if args.port:
        workers['lidar'] = SensorProcess(lidar_worker, (args.port, 115200))
    stats = {name: {'received': 0, 'errors': [], 'first': None, 'last': None} for name in workers}
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            for name, worker in workers.items():
                for kind, stamp, payload in worker.drain():
                    stat = stats[name]
                    if kind == 'data':
                        stat['received'] += 1
                        stat['first'] = stat['first'] or stamp
                        stat['last'] = stamp
                        stat['shape'] = list(payload.shape)
                    elif kind == 'error':
                        stat['errors'].append(payload)
            time.sleep(0.03)
    finally:
        finished_at = time.monotonic()
        for worker in workers.values():
            worker.request_stop()
        pending = list(workers.values())
        while pending:
            pending = [w for w in pending if not w.finished()]
            time.sleep(0.05)
    for name, stat in stats.items():
        first, last = stat.pop('first'), stat.pop('last')
        stat['last_age_s'] = round(finished_at-last, 2) if last is not None else None
        stat['received_hz'] = round((stat['received'] - 1) / (last - first), 2) if first and last > first else 0
        stat['forced_shutdown'] = workers[name].forced
    print(json.dumps(stats, ensure_ascii=False, indent=2))
    return 0 if stats and all(s['received'] and s['last_age_s'] < 2 and not s['errors'] and not s['forced_shutdown'] for s in stats.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
