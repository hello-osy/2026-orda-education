"""ROS 2 SQLite bag 기록/탐색. ROS 데몬 없이 표준 CDR 메시지를 저장한다."""
from collections import Counter
import bisect
import csv
import json
from pathlib import Path
import queue
import sqlite3
import threading
import time

import cv2
import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

CAMERA_TOPICS = ('/camera/high/image_raw', '/camera/low/image_raw')
SCAN_TOPIC = '/scan'
IMAGE_TYPES = ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')
SCAN_TYPE = 'sensor_msgs/msg/LaserScan'


def decode_image(msg):
    if msg.__msgtype__ == IMAGE_TYPES[1]:
        frame = cv2.imdecode(msg.data, cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError('압축 영상을 해석하지 못했습니다.')
        return frame
    enc = msg.encoding.lower()
    channels = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1,
                'yuyv': 2, 'yuy2': 2, 'yuv422_yuy2': 2, 'yuv422': 2, 'uyvy': 2}
    if enc not in channels:
        raise ValueError(f'지원하지 않는 영상 encoding: {enc}')
    n = channels[enc]
    if msg.step < msg.width * n or len(msg.data) < msg.height * msg.step:
        raise ValueError('영상 데이터 길이/step이 잘못되었습니다.')
    image = np.asarray(msg.data, dtype=np.uint8)[:msg.height * msg.step].reshape(msg.height, msg.step)
    image = image[:, :msg.width * n].reshape(msg.height, msg.width, n)
    codes = {'rgb8': cv2.COLOR_RGB2BGR, 'bgra8': cv2.COLOR_BGRA2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
             'mono8': cv2.COLOR_GRAY2BGR, 'yuyv': cv2.COLOR_YUV2BGR_YUY2, 'yuy2': cv2.COLOR_YUV2BGR_YUY2,
             'yuv422_yuy2': cv2.COLOR_YUV2BGR_YUY2, 'yuv422': cv2.COLOR_YUV2BGR_UYVY, 'uyvy': cv2.COLOR_YUV2BGR_UYVY}
    return np.ascontiguousarray(image) if enc == 'bgr8' else cv2.cvtColor(image, codes[enc])


def decode_scan(msg):
    distances = np.asarray(msg.ranges)
    angles = msg.angle_min + np.arange(len(distances)) * msg.angle_increment
    valid = np.isfinite(distances) & (distances >= msg.range_min) & (distances <= msg.range_max) & (distances > 0)
    # 대회 sllidar_ros2(inverted=false): ROS 각도 = 180° − A1 원시 각도.
    # 화면 내부는 실시간 센서와 같은 A1 원시 각도를 사용한다.
    return np.column_stack((np.ones(valid.sum()), 180.0 - np.rad2deg(angles[valid]), distances[valid] * 1000)).astype(np.float32)


class BagWriter:
    def __init__(self, path, topics):
        self.path = Path(path)
        self.store = get_typestore(Stores.ROS2_JAZZY)
        self.writer = Writer(self.path, version=9)
        self.writer.open()
        self.connections = {}
        self.previous_scan = None
        try:
            for topic in topics:
                msgtype = SCAN_TYPE if topic == SCAN_TOPIC else IMAGE_TYPES[0]
                self.connections[topic] = self.writer.add_connection(topic, msgtype, typestore=self.store)
        except BaseException:
            self.writer.close()
            raise

    def header(self, ns, frame):
        types = self.store.types
        return types['std_msgs/msg/Header'](types['builtin_interfaces/msg/Time'](ns // 10**9, ns % 10**9), frame)

    def write(self, topic, ns, payload):
        if topic == SCAN_TOPIC:
            # 대회 sllidar_ros2와 같은 180° − 원시 각도로 1도 bin에 배치한다.
            ranges = np.full(360, np.inf, dtype=np.float32)
            intensities = np.zeros(360, dtype=np.float32)
            for quality, angle, distance in payload:
                if not np.isfinite([quality, angle, distance]).all() or quality <= 0 or not 150 <= distance <= 12000:
                    continue
                index = round(180.0 - float(angle)) % 360
                if distance / 1000 < ranges[index]:
                    ranges[index], intensities[index] = distance / 1000, quality
            period = (ns - self.previous_scan) / 1e9 if self.previous_scan is not None else 0.
            self.previous_scan = ns
            msg = self.store.types[SCAN_TYPE](self.header(ns, 'laser'), 0., float(np.deg2rad(359)),
                float(np.deg2rad(1)), 0., max(0., period), .15, 12., ranges, intensities)
        else:
            image = np.ascontiguousarray(payload, dtype=np.uint8)
            h, w = image.shape[:2]
            msg = self.store.types[IMAGE_TYPES[0]](self.header(ns, 'camera_high' if topic == CAMERA_TOPICS[0] else 'camera_low'),
                h, w, 'bgr8', 0, w * 3, image.ravel())
        self.writer.write(self.connections[topic], ns, self.store.serialize_cdr(msg, msg.__msgtype__))

    def close(self):
        self.writer.close()


class Recorder:
    """디스크 기록이 GUI를 막지 않도록 제한된 큐 + 기록 스레드를 사용한다."""
    def __init__(self, path, topics):
        self.path = Path(path)
        self.topics = tuple(topics)
        self.queue = queue.Queue(maxsize=60)
        self.counts = Counter()
        self.dropped = 0
        self.error = ''
        self.accepting = True
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.wall_offset = time.time_ns() - int(time.monotonic() * 1e9)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def submit(self, topic, stamp, frame):
        if not self.accepting or self.error or topic not in self.topics:
            return
        try:
            self.queue.put_nowait((topic, self.wall_offset + int(stamp * 1e9), frame))
        except queue.Full:
            self.dropped += 1

    def stop(self):
        self.accepting = False
        self.stop_event.set()

    def run(self):
        writer = None
        try:
            writer = BagWriter(self.path, self.topics)
            self.ready.set()
            while not self.stop_event.is_set() or not self.queue.empty():
                try:
                    topic, ns, frame = self.queue.get(timeout=.1)
                except queue.Empty:
                    continue
                writer.write(topic, ns, frame)
                self.counts[topic] += 1
        except Exception as exc:
            self.error = str(exc)
            self.accepting = False
        finally:
            if writer:
                try:
                    writer.close()
                    (self.path / 'session2_recording.json').write_text(json.dumps({
                        'counts': dict(self.counts), 'queue_dropped': self.dropped,
                        'error': self.error, 'timestamp': 'host receive time',
                        'note': 'preview queue may skip sensor frames; not hardware synchronized',
                    }, ensure_ascii=False, indent=2), encoding='utf-8')
                except Exception as exc:
                    self.error = str(exc)
            self.ready.set()


class BagArchive:
    """SQLite는 읽기 전용. 메모리에는 프레임 인덱스만 보관한다."""
    def __init__(self, path):
        self.path = Path(path).expanduser().resolve()
        self.dbs, self.rows, self.types, self.stamps = [], {}, {}, {}
        self.store = get_typestore(Stores.ROS2_JAZZY)
        files = sorted(self.path.glob('*.db3')) if self.path.is_dir() else [self.path]
        if not files or any(p.suffix != '.db3' for p in files):
            raise ValueError('SQLite ROS 2 bag 폴더(metadata.yaml + .db3) 또는 .db3 파일을 선택하세요. MCAP은 지원하지 않습니다.')
        try:
            for file in files:
                db = sqlite3.connect(file.as_uri() + '?mode=ro', uri=True, check_same_thread=False)
                self.dbs.append(db)
                for tid, name, msgtype, fmt in db.execute('select id,name,type,serialization_format from topics'):
                    if msgtype not in (*IMAGE_TYPES, SCAN_TYPE):
                        continue
                    if fmt != 'cdr':
                        raise ValueError('CDR 직렬화 bag만 지원합니다.')
                    if name in self.types and self.types[name] != msgtype:
                        raise ValueError(f'분할 파일의 토픽 타입이 다릅니다: {name}')
                    self.types[name] = msgtype
                    self.rows.setdefault(name, []).extend((stamp, len(self.dbs)-1, mid) for mid, stamp in db.execute(
                        'select id,timestamp from messages where topic_id=? order by timestamp,id', (tid,)))
            self.rows = {k: sorted(v) for k, v in self.rows.items() if v}
            if not self.rows:
                raise ValueError('bag에 Image/CompressedImage/LaserScan 메시지가 없습니다.')
            self.stamps = {k: [r[0] for r in v] for k, v in self.rows.items()}
            self.start = min(v[0][0] for v in self.rows.values())
            self.end = max(v[-1][0] for v in self.rows.values())
            self.duration = (self.end-self.start)/1e9
        except BaseException:
            self.close()
            raise

    def read(self, topic, index):
        stamp, dbi, mid = self.rows[topic][index]
        raw = self.dbs[dbi].execute('select data from messages where id=?', (mid,)).fetchone()[0]
        msg = self.store.deserialize_cdr(raw, self.types[topic])
        return stamp, decode_scan(msg) if self.types[topic] == SCAN_TYPE else decode_image(msg)

    def snapshot(self, seconds, topics):
        target = self.start + round(seconds * 1e9)
        result = {}
        for topic in topics:
            if topic not in self.rows:
                continue
            index = bisect.bisect_right(self.stamps[topic], target)-1
            if index >= 0:
                stamp, data = self.read(topic, index)
                result[topic] = {'data': data, 'age': (target-stamp)/1e9, 'index': index}
        return result

    def close(self):
        for db in self.dbs:
            db.close()
        self.dbs = []


def extract_frames(bag_path, output, topics, stride=5, progress=print):
    if stride < 1:
        raise ValueError('추출 간격은 1 이상이어야 합니다.')
    output = Path(output)
    bag = BagArchive(bag_path)
    try:
        topics = list(dict.fromkeys(topics))
        if not topics or any(t not in bag.rows or bag.types[t] not in IMAGE_TYPES for t in topics):
            raise ValueError('카메라 영상 토픽을 선택하세요.')
        output.mkdir(parents=True, exist_ok=False)
        counts = {}
        with (output/'manifest.csv').open('w', newline='', encoding='utf-8') as handle:
            csvout = csv.writer(handle)
            csvout.writerow(['file', 'topic', 'frame_1based', 'timestamp_ns', 'source_bag'])
            for number, topic in enumerate(topics, 1):
                folder = output/f'camera_{number}'
                folder.mkdir()
                counts[topic] = 0
                # 요청한 5프레임마다 = 5, 10, 15번째. 각 카메라에서 독립적으로 센다.
                for index in range(stride-1, len(bag.rows[topic]), stride):
                    stamp, frame = bag.read(topic, index)
                    name = f'camera_{number}/frame_{index+1:08d}_{stamp}.jpg'
                    ok, encoded = cv2.imencode('.jpg', frame)
                    if not ok:
                        raise OSError(f'이미지 저장 실패: {name}')
                    (output/name).write_bytes(encoded.tobytes())
                    csvout.writerow([name, topic, index+1, stamp, str(bag.path)])
                    counts[topic] += 1
                    if counts[topic] % 50 == 0:
                        progress(f'{topic}: {counts[topic]}장 저장', flush=True)
        (output/'summary.json').write_text(json.dumps({'stride':stride, 'counts':counts}, ensure_ascii=False, indent=2), encoding='utf-8')
        return counts
    finally:
        bag.close()
