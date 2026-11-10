"""네 역할이 Git으로 코드를 교환할 때 함께 지킬 토픽·자료 형식."""
import json
import math

MOTOR_TOPIC = '/session4/motor_control'
STALE_SECONDS = 0.8


def no_path():
    return {'valid': False, 'error': 0.0}


def no_perception():
    return {'traffic_light': {'detected': False, 'color': 'unknown',
                              'bbox': [0, 0], 'size': [0, 0], 'conf': 0.0},
            'rubbercone': {'detected': False, 'bbox': [], 'size': [], 'conf': []}}


def dumps(packet):
    return json.dumps(packet, ensure_ascii=False, allow_nan=False, separators=(',', ':'))


def loads(text):
    packet = json.loads(text)
    if not isinstance(packet, dict):
        raise ValueError('토픽 자료는 JSON object여야 합니다.')
    return packet


def finite(value, fallback=0.0):
    try:
        number = float(value)
        return number if math.isfinite(number) else fallback
    except (TypeError, ValueError):
        return fallback


class Latest:
    """학생 노드 중 하나가 꺼지면 이전 결과로 계속 달리지 않게 한다."""
    def __init__(self):
        self.value = None
        self.received_at = -math.inf

    def receive(self, value, now):
        self.value, self.received_at = value, now

    def current(self, now):
        return self.value if 0 <= now - self.received_at < STALE_SECONDS else None
