"""완료한 SQLite ROS 2 bag에서 YOLO 라벨링용 카메라 사진을 꺼낸다."""
import argparse
import importlib
import json
from pathlib import Path
import sys
import uuid


HIGH_CAMERA_TOPIC = '/camera/high/image_raw'


def find_project_root(start=None):
    """소스 실행·colcon symlink 설치 모두에서 교육 저장소를 찾는다."""
    starts = [Path(start).expanduser().resolve()] if start else []
    starts.extend([Path(__file__).resolve(), Path.cwd().resolve()])
    for origin in starts:
        for candidate in (origin, *origin.parents):
            if ((candidate / 'src/session_2/session_2/learning.py').is_file()
                    and (candidate / 'scripts/session_4.sh').is_file()):
                return candidate
    raise ValueError('교육 저장소를 찾지 못했습니다. 프로젝트 폴더에서 다시 실행하세요.')


def load_session2_module(name, project_root=None):
    """기존 수업의 bag·학습 함수를 재사용한다. import만으로 학습하지 않는다."""
    if name not in ('bagio', 'learning', 'platform_support'):
        raise ValueError('지원하지 않는 Session 2 도구입니다.')
    root = Path(project_root).resolve() if project_root else find_project_root()
    source = root / 'src/session_2'
    if not (source / 'session_2' / f'{name}.py').is_file():
        raise ValueError(f'Session 2 공통 코드를 찾지 못했습니다: {source}')
    # session_2는 ROS 패키지가 아니므로, 이 어댑터를 호출할 때만 소스 경로를 추가한다.
    source_text = str(source)
    if source_text not in sys.path:
        sys.path.insert(0, source_text)
    return importlib.import_module(f'session_2.{name}')


def write_json_atomic(path, packet):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        temporary.write_text(json.dumps(packet, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def completed_marker_path(marker, description):
    """기록/학습 도중의 경로를 최신 완료 결과로 잘못 사용하지 않는다."""
    marker = Path(marker)
    try:
        packet = json.loads(marker.read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise ValueError(f'{description} 기록이 없습니다: {marker}') from error
    if (not isinstance(packet, dict) or packet.get('status') != 'completed'
            or not isinstance(packet.get('path'), str) or not packet['path']):
        raise ValueError(f'{description}이 정상 완료된 기록이 아닙니다: {marker}')
    path = Path(packet['path']).expanduser()
    if not path.is_absolute():
        raise ValueError(f'{description} 기록의 경로는 절대경로여야 합니다: {marker}')
    if not path.exists():
        raise ValueError(f'{description} 파일/폴더가 없습니다: {path}')
    return path.resolve()


def resolve_bag(bag=None, project_root=None):
    root = Path(project_root).resolve() if project_root else find_project_root()
    if bag is None or str(bag) == 'latest':
        path = completed_marker_path(root / 'workspace/session_4/latest_bag.json', '최신 bag 녹화')
    else:
        path = Path(bag).expanduser().resolve()
    if path.is_dir():
        if not any(path.glob('*.db3')):
            raise ValueError(f'SQLite .db3 파일이 없는 bag 폴더입니다: {path}')
    elif not path.is_file() or path.suffix.lower() != '.db3':
        raise ValueError('SQLite ROS 2 bag 폴더 또는 .db3 파일을 지정하세요. MCAP은 지원하지 않습니다.')
    return path


def extract_dataset_frames(bag=None, output=None, stride=5, topic=HIGH_CAMERA_TOPIC,
                           project_root=None):
    root = Path(project_root).resolve() if project_root else find_project_root()
    if isinstance(stride, bool) or not isinstance(stride, int) or stride < 1:
        raise ValueError('--stride는 1 이상의 정수여야 합니다.')
    source = resolve_bag(bag, root)
    output = (Path(output).expanduser().resolve() if output else
              root / 'workspace/session_4/extracted' / (source.stem if source.is_file() else source.name))
    if output.exists():
        raise ValueError(f'추출 폴더가 이미 있습니다: {output}\n--output으로 새 폴더를 지정하거나 새로 녹화하세요.')
    bagio = load_session2_module('bagio', root)
    archive = bagio.BagArchive(source)
    try:
        if topic not in archive.rows or archive.types[topic] not in bagio.IMAGE_TYPES:
            raise ValueError(f'bag에 카메라 토픽이 없습니다: {topic}')
        if len(archive.rows[topic]) < stride:
            raise ValueError(f'영상이 {stride}장보다 적습니다. 더 녹화하거나 --stride 1로 추출하세요.')
    finally:
        archive.close()
    # 원본 bag은 읽기 전용. 사진·원본 timestamp의 manifest는 기존 추출기가 저장한다.
    counts = bagio.extract_frames(source, output, [topic], stride=stride)
    print(f'사진 저장 완료: {output / "camera_1"}', flush=True)
    print(f'추출 결과: {counts} / {stride}프레임마다 1장', flush=True)
    print('사진에 신호등 색(red/yellow/green)과 라바콘(cone) 박스를 표시한 뒤 YOLOv8 형식으로 내보내세요.', flush=True)
    return {'path': str(output), 'source_bag': str(source), 'counts': counts, 'stride': stride}


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', help='생략하면 마지막 정상 완료 bag')
    parser.add_argument('--stride', type=int, default=5)
    parser.add_argument('--output', help='새 추출 폴더 (기존 폴더는 덮어쓰지 않음)')
    parser.add_argument('--topic', default=HIGH_CAMERA_TOPIC)
    options = parser.parse_args(args)
    try:
        extract_dataset_frames(**vars(options))
    except (ValueError, OSError, ImportError) as error:
        parser.exit(1, f'사진 추출 실패: {error}\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
