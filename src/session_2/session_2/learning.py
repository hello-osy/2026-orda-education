"""YOLOv8 데이터 검증, CUDA/MPS/CPU 선택, 학습과 추론. GUI와 분리한다."""
import hashlib
from pathlib import Path
import math

import yaml
from .inference_runtime import inference_guard


def choose_device(requested='auto'):
    import torch
    if requested == 'auto':
        return 'cuda' if torch.cuda.is_available() else ('mps' if torch.backends.mps.is_available() else 'cpu')
    if requested == 'mps' and not torch.backends.mps.is_available():
        raise ValueError('이 Python 환경에서 MPS를 사용할 수 없습니다. CPU를 선택하세요.')
    if requested == 'cuda' and not torch.cuda.is_available():
        raise ValueError('이 Python 환경에서 CUDA를 사용할 수 없습니다. PyTorch 설치를 확인하세요.')
    if requested not in ('cpu', 'mps', 'cuda'):
        raise ValueError('지원하지 않는 실행 장치입니다.')
    return requested


def validate_dataset(path):
    """Roboflow의 ../train/images 경로를 내보낸 폴더 기준으로 해석한다."""
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise ValueError('Roboflow에서 내려받은 data.yaml 파일을 선택하세요.')
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(config, dict):
        raise ValueError('data.yaml 형식이 잘못되었습니다.')
    names = config.get('names')
    if isinstance(names, dict):
        if sorted(names) != list(range(len(names))):
            raise ValueError('클래스 번호는 0부터 연속이어야 합니다.')
        names = [names[i] for i in range(len(names))]
    if not isinstance(names, list) or not names or not all(isinstance(n, str) and n.strip() for n in names):
        raise ValueError('names에 클래스 이름 목록이 필요합니다.')
    if len(set(names)) != len(names) or config.get('nc', len(names)) != len(names):
        raise ValueError('클래스 이름 중복 또는 nc와 names 개수 불일치입니다.')
    declared = Path(str(config.get('path', '.'))).expanduser()
    root = declared if declared.is_absolute() else path.parent/declared
    normalized = {'names': names, 'nc': len(names)}
    summary = {}
    hashes = {}
    for split in ('train', 'val'):
        value = config.get(split)
        if not isinstance(value, str) or not value:
            raise ValueError(f'{split} 이미지 폴더 경로가 필요합니다.')
        candidate = Path(value).expanduser()
        candidates = [candidate] if candidate.is_absolute() else [root/candidate, path.parent/candidate,
            path.parent/str(candidate).removeprefix('../')]
        folder = next((p.resolve() for p in candidates if p.is_dir()), None)
        if folder is None:
            raise ValueError(f'{split} 폴더를 찾지 못했습니다: {value}')
        images = sorted(p for p in folder.rglob('*') if p.suffix.lower() in ('.jpg','.jpeg','.png','.bmp','.webp'))
        if not images:
            raise ValueError(f'{split}에 이미지가 없습니다.')
        boxes, missing = 0, 0
        split_hashes = set()
        for image in images:
            # Ultralytics 표준 images/... -> labels/... 경로.
            parts = list(image.parts)
            if 'images' not in parts:
                raise ValueError('train/images 및 train/labels 구조의 YOLOv8 데이터셋이 필요합니다.')
            parts[len(parts)-1-parts[::-1].index('images')] = 'labels'
            label = Path(*parts).with_suffix('.txt')
            if not label.is_file():
                missing += 1
            else:
                for line in label.read_text(encoding='utf-8').splitlines():
                    if not line.strip():
                        continue
                    try:
                        row = [float(x) for x in line.split()]
                        if len(row) != 5 or not all(math.isfinite(x) for x in row):
                            raise ValueError()
                        cls, x, y, w, h = row
                        if cls != int(cls) or not 0 <= cls < len(names) or not (0 <= x <= 1 and 0 <= y <= 1 and 0 < w <= 1 and 0 < h <= 1):
                            raise ValueError()
                    except ValueError:
                        raise ValueError(f'잘못된 bbox 라벨: {label} (class x y width height, 0~1 좌표)') from None
                    boxes += 1
            split_hashes.add(hashlib.sha256(image.read_bytes()).digest())
        if not boxes:
            raise ValueError(f'{split}에 bbox 라벨이 없습니다. Roboflow에서 라벨링 후 YOLOv8로 내보내세요.')
        hashes[split] = split_hashes
        normalized[split] = str(folder)
        summary[split] = {'images': len(images), 'boxes': boxes, 'missing_labels': missing}
    if hashes['train'] & hashes['val']:
        raise ValueError('train과 val에 동일 이미지가 있습니다. 촬영 구간별로 나누어 다시 내보내세요.')
    return normalized, summary


class Detector:
    def __init__(self, weights, device='auto'):
        from ultralytics import YOLO
        path = Path(weights).expanduser()
        if not path.is_file() or path.suffix != '.pt':
            raise ValueError('학습된 best.pt 또는 YOLOv8 .pt 파일을 선택하세요.')
        self.device = choose_device(device)
        with inference_guard(self.device):
            self.model = YOLO(str(path), task='detect')
        if self.model.task != 'detect':
            raise ValueError('bbox 객체 검출 모델을 선택하세요.')

    def predict(self, frame, confidence=.25, imgsz=640):
        import time
        start = time.monotonic()
        with inference_guard(self.device):
            result = self.model.predict(frame, device=self.device, conf=confidence, imgsz=imgsz,
                                        quantize=32, verbose=False)[0]
            # plot / boxes도 GPU 텐서를 읽을 수 있으므로 보호 구간 안에서 처리한다.
            plotted, count = result.plot(), len(result.boxes)
        return plotted, count, (time.monotonic()-start)*1000


def train(data, output, model='yolov8n.pt', epochs=30, batch=4, imgsz=640, device='auto'):
    from ultralytics import YOLO
    output = Path(output)
    config, summary = validate_dataset(data)
    output.mkdir(parents=True, exist_ok=False)
    normalized = output/'dataset.yaml'
    normalized.write_text(yaml.safe_dump(config, allow_unicode=True), encoding='utf-8')
    selected = choose_device(device)
    print(f'학습 장치: {selected} | 데이터: {summary}', flush=True)
    # AMP는 CPU/MPS에서 끄고, DataLoader는 macOS 프로세스 충돌을 피하도록 worker=0 (Windows에서도 추가 프로세스 없음).
    yolo = YOLO(model, task='detect')
    yolo.train(data=str(normalized), epochs=epochs, batch=batch, imgsz=imgsz, device=selected,
               workers=0, amp=False, project=str(output), name='model', exist_ok=False,
               plots=False, cache=False, seed=0)
    best = Path(yolo.trainer.best)
    if not best.is_file():
        raise RuntimeError('학습이 끝났지만 best.pt가 없습니다. 로그를 확인하세요.')
    return {'best': str(best.resolve()), 'device': selected, 'dataset': summary}
