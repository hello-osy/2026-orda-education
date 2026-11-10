"""Session 2의 검증·학습 함수를 이용한 Session 4 YOLO 학습 명령."""
import argparse
from datetime import datetime
from pathlib import Path
import uuid

from .dataset_tools import (completed_marker_path, find_project_root,
                            load_session2_module, write_json_atomic)


CLASS_NAMES = ('red', 'yellow', 'green', 'cone')


def resolve_training_model(model, project_root):
    """기본 가중치 다운로드도 Git에서 제외되는 수업 작업 폴더 안에 둔다."""
    path = Path(model).expanduser()
    if str(model) == 'yolov8n.pt':
        return Path(project_root).resolve() / 'workspace/session_4/cache/models' / path.name
    if path.is_file():
        return path.resolve()
    if path.suffix.lower() != '.pt':
        raise ValueError('--model에는 기본 YOLO .pt 이름 또는 이미 있는 .pt 파일을 지정하세요.')
    if len(path.parts) == 1:
        return Path(project_root).resolve() / 'workspace/session_4/cache/models' / path.name
    raise ValueError(f'학습 시작 모델 파일이 없습니다: {path}')


def resolve_latest_model(project_root=None):
    """추론 실행용 최신 완료 경로. PyTorch/Ultralytics를 불러오지 않는다."""
    root = Path(project_root).resolve() if project_root else find_project_root()
    path = completed_marker_path(root / 'workspace/session_4/latest_model.json', '최신 YOLO 학습')
    if not path.is_file() or path.suffix.lower() != '.pt':
        raise ValueError(f'학습된 .pt 모델 파일이 아닙니다: {path}')
    return path


def validate_training_dataset(data, project_root=None):
    """기존 데이터 검사에 수업의 4개 클래스·박스 경계·구간별 분포 검사를 더한다."""
    learning = load_session2_module('learning', project_root)
    config, summary = learning.validate_dataset(data)
    names = config['names']
    if len(names) != 4 or set(names) != set(CLASS_NAMES):
        raise ValueError('data.yaml의 클래스 이름은 red, yellow, green, cone 네 가지여야 합니다. '
                         'Roboflow 클래스 이름을 확인하세요.')
    for split in ('train', 'val'):
        counts = {name: 0 for name in names}
        folder = Path(config[split])
        images = [p for p in folder.rglob('*')
                  if p.suffix.lower() in ('.jpg', '.jpeg', '.png', '.bmp', '.webp')]
        for image in images:
            parts = list(image.parts)
            parts[len(parts) - 1 - parts[::-1].index('images')] = 'labels'
            label = Path(*parts).with_suffix('.txt')
            if not label.is_file():
                continue  # 물체가 없는 음성 사진은 기존 검증기의 missing_labels로 집계한다.
            for number, line in enumerate(label.read_text(encoding='utf-8').splitlines(), 1):
                if not line.strip():
                    continue
                cls, x, y, width, height = [float(value) for value in line.split()]
                if (x - width / 2 < -1e-6 or x + width / 2 > 1 + 1e-6
                        or y - height / 2 < -1e-6 or y + height / 2 > 1 + 1e-6):
                    raise ValueError(f'영상 밖으로 나가는 bbox입니다: {label}:{number}')
                counts[names[int(cls)]] += 1
        missing = [name for name, count in counts.items() if count == 0]
        if missing:
            raise ValueError(f'{split}에 다음 클래스의 라벨이 없습니다: {", ".join(missing)}. '
                             '각 신호 색과 라바콘 사진을 추가해 다시 내보내세요.')
        summary[split]['class_counts'] = counts
    return config, summary


def train_dataset(data=None, device='auto', epochs=30, batch=4, imgsz=320,
                  model='yolov8n.pt', project_root=None):
    root = Path(project_root).resolve() if project_root else find_project_root()
    data = Path(data).expanduser().resolve() if data else root / 'workspace/session_4/dataset/data.yaml'
    if not data.is_file():
        raise ValueError(f'학습 데이터 목록이 없습니다: {data}\n'
                         'YOLOv8 데이터셋 압축을 workspace/session_4/dataset에 풀거나 --data 경로를 지정하세요.')
    if device not in ('auto', 'cpu', 'mps', 'cuda'):
        raise ValueError('--device는 auto, cpu, mps, cuda 중 하나입니다.')
    for name, value in (('epochs', epochs), ('batch', batch), ('imgsz', imgsz)):
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise ValueError(f'--{name}은 1 이상의 정수여야 합니다.')
    config, summary = validate_training_dataset(data, root)
    print(f'데이터 검사 완료: {summary}', flush=True)
    learning = load_session2_module('learning', root)
    # Session 2의 학습 함수도 이 설정 함수를 호출하므로 import 전에 Session 4 경로를 정한다.
    platform_support = load_session2_module('platform_support', root)
    platform_support.prepare_learning_config(root / 'workspace/session_4')
    output_parent = root / 'workspace/session_4/training'
    output_parent.mkdir(parents=True, exist_ok=True)
    output = output_parent / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:6])
    model_path = resolve_training_model(model, root)
    model_path.parent.mkdir(parents=True, exist_ok=True)
    print(f'시작 모델: {model_path} (처음 학습할 때 없으면 이 위치로 다운로드)', flush=True)
    result = learning.train(data, output, model=str(model_path), epochs=epochs, batch=batch,
                            imgsz=imgsz, device=device)
    best = Path(result['best']).expanduser().resolve()
    if not best.is_file() or best.suffix.lower() != '.pt':
        raise RuntimeError('학습이 정상 종료되지 않았습니다. best.pt와 학습 로그를 확인하세요.')
    packet = {'path': str(best), 'status': 'completed', 'device': result['device'],
              'class_names': list(config['names']), 'data': str(data),
              'output': str(output), 'dataset': summary}
    write_json_atomic(root / 'workspace/session_4/latest_model.json', packet)
    print(f'학습 완료 · 실제 장치: {result["device"]}', flush=True)
    print(f'학습 모델: {best}', flush=True)
    print('다음 실행: python3 src/session_4/student/driving.py --yolo', flush=True)
    return packet


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data')
    parser.add_argument('--device', default='auto', choices=['auto', 'cpu', 'mps', 'cuda'])
    parser.add_argument('--epochs', default=30, type=int)
    parser.add_argument('--batch', default=4, type=int)
    parser.add_argument('--imgsz', default=320, type=int)
    parser.add_argument('--model', default='yolov8n.pt')
    parser.add_argument('--latest-path', action='store_true', help='최신 완료 모델 경로만 출력')
    options = parser.parse_args(args)
    try:
        if options.latest_path:
            print(resolve_latest_model())
        else:
            values = vars(options)
            values.pop('latest_path')
            train_dataset(**values)
    except (ValueError, OSError, ImportError, RuntimeError) as error:
        parser.exit(1, f'YOLO 학습 도구: {error}\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
