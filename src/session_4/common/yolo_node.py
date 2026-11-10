"""YOLO 추론 또는 Gazebo 색 검출 예제 → 같은 검출 토픽으로 발행.

color는 학습 모델이 아닌 시뮬레이터 색 기반 예제입니다. 학생 YOLO 모드는 최근 학습
결과 또는 직접 지정한 red/yellow/green/cone 가중치를 사용합니다. COCO 기본 모델로 신호 색을
판별했다고 주장하지 않습니다. 정답 주행용 Gazebo 가중치는 common/weights에 제공합니다.
"""
import json
from pathlib import Path
import time
import numpy as np


def reference_weights():
    """정답에 포함된 Gazebo용 모델 경로를 찾는다."""
    path = Path(__file__).resolve().parent / 'weights' / 'traffic_cone_yolov8n.pt'
    if path.is_file():
        return path
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('session_4')) / 'weights/traffic_cone_yolov8n.pt'
    if not path.is_file():
        raise FileNotFoundError('정답 YOLO 모델이 없습니다. Session 4 파일을 다시 내려받으세요.')
    return path


def color_detections(rgb):
    """시뮬레이터에서 즉시 실습하도록 제공한 HSV 예제. YOLO와 구분해 표시합니다."""
    import cv2
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)
    hue, saturation, value = cv2.split(hsv)
    vivid = (saturation > 125) & (value > 110)
    masks = {
        'red': (((hue < 6) | (hue > 173)) & vivid),
        'yellow': ((hue > 23) & (hue < 37) & vivid),
        'green': ((hue > 42) & (hue < 92) & vivid),
        'cone': ((hue >= 6) & (hue < 23) & vivid),
    }
    height, width = rgb.shape[:2]
    detections = []
    for label, mask in masks.items():
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if cv2.contourArea(contour) < 5 or min(w, h) < 3:
                continue
            if label != 'cone':
                # 제공한 높은 카메라에서 신호등은 수평선 위, 도로는 아래에 보입니다.
                # 곡선의 노란 점선이 둥글게 보여도 신호로 읽지 않도록 위 절반만 사용.
                if y + h / 2 >= height * .50 or not .5 <= w / h <= 1.9:
                    continue
            elif h / w < .75:
                continue
            detections.append({'label': label, 'confidence': 1.0,
                               'bbox': [int(x), int(y), int(x + w), int(y + h)]})
    return {'valid': True, 'source': 'simulator_color', 'width': width,
            'height': height, 'detections': detections}


class YoloDetector:
    def __init__(self, weights, class_map='', device='cpu'):
        path = Path(weights).expanduser()
        if not weights or not path.is_file():
            raise ValueError('detector:=yolo에는 실제 수업용 weights:=/절대경로/model.pt가 필요합니다.')
        mapping = json.loads(class_map) if class_map else {}
        if not isinstance(mapping, dict) or any(value not in ('red', 'yellow', 'green', 'cone')
                                                for value in mapping.values()):
            raise ValueError('class_map은 모델 클래스 이름을 red/yellow/green/cone로 바꾸는 JSON object입니다.')
        from ultralytics import YOLO
        self.model = YOLO(str(path))
        self.mapping = mapping
        self.device = device
        names = set(self.model.names.values())
        canonical = {mapping.get(name, name) for name in names}
        if not {'red', 'yellow', 'green', 'cone'}.issubset(canonical):
            raise ValueError('모델 클래스 또는 class_map에 red, yellow, green, cone 4종이 필요합니다. COCO 기본 모델은 색별 신호등 모델이 아닙니다.')

    def __call__(self, rgb):
        # Ultralytics numpy 입력은 BGR입니다. ROS 입력 RGB를 변환합니다.
        result = self.model.predict(source=np.ascontiguousarray(rgb[:, :, ::-1]),
                                    verbose=False, device=self.device, imgsz=320, conf=.35)[0]
        detections = []
        if result.boxes is not None:
            for box, score, index in zip(result.boxes.xyxy.cpu().tolist(),
                                          result.boxes.conf.cpu().tolist(),
                                          result.boxes.cls.cpu().tolist()):
                name = self.model.names[int(index)]
                label = self.mapping.get(name, name)
                if label in ('red', 'yellow', 'green', 'cone'):
                    detections.append({'label': label, 'confidence': float(score), 'bbox': box})
        height, width = rgb.shape[:2]
        return {'valid': True, 'source': 'yolo', 'width': width, 'height': height,
                'detections': detections}

def main(args=None):
    from .student_runner import load_student
    return load_student().main(['--yolo', *(args or [])])


if __name__ == '__main__':
    main()
