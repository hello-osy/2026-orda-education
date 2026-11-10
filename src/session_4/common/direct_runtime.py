"""센서 배열 변환과 화면용 파일 저장. 역할 간 ROS 토픽은 사용하지 않는다."""
import math
import os
import time
import numpy as np
from .dataset_tools import find_project_root, write_json_atomic


def distances(message):
    output = np.full(360, math.inf)
    for index, distance in enumerate(message.ranges):
        if math.isfinite(distance) and message.range_min <= distance <= message.range_max:
            degree = round(math.degrees(message.angle_min + index * message.angle_increment)) % 360
            output[degree] = min(output[degree], distance)
    return output


def split_boxes(packet):
    labels, bbox, size, conf = [], [], [], []
    for item in packet.get('detections', []):
        x1, y1, x2, y2 = item['bbox']
        labels.append(item['label'])
        bbox.append([x1, y1])
        size.append([x2 - x1, y2 - y1])
        conf.append(item['confidence'])
    return labels, bbox, size, conf


def status_path():
    return find_project_root() / 'workspace/session_4/runtime' / ('status_' + os.environ.get('ROS_DOMAIN_ID', '0') + '.json')


_last_write = 0.0
def write_status(values, received):
    global _last_write
    if time.monotonic() - _last_write < .15:
        return
    write_json_atomic(status_path(), {'values': values, 'received': {
        key: value if math.isfinite(value) else -1e30 for key, value in received.items()}})
    _last_write = time.monotonic()
