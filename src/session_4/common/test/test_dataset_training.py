"""실제 SQLite 사진 추출과 학습 입력/완료 표시를 검증한다. 모델 다운로드는 하지 않는다."""
import csv
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import cv2
import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from session_4 import dataset_tools, training_tools


def make_dataset(folder, names=('red', 'yellow', 'green', 'cone')):
    for split_number, split in enumerate(('train', 'valid')):
        (folder / split / 'images').mkdir(parents=True)
        (folder / split / 'labels').mkdir()
        for index in range(4):
            image = np.full((16, 16, 3), 20 + split_number * 90 + index * 10, dtype=np.uint8)
            cv2.imwrite(str(folder / split / 'images' / f'{index}.png'), image)
            (folder / split / 'labels' / f'{index}.txt').write_text(f'{index} 0.5 0.5 0.25 0.25\n')
    data = folder / 'data.yaml'
    data.write_text(yaml.safe_dump({'train': 'train/images', 'val': 'valid/images',
                                    'nc': 4, 'names': list(names)}))
    return data


def test_extract_latest_recording_keeps_every_fifth_high_frame_and_source_untouched(tmp_path):
    bagio = dataset_tools.load_session2_module('bagio')
    root = tmp_path / 'project'
    bag = root / 'workspace/session_4/bags/run_1'
    bag.parent.mkdir(parents=True)
    writer = bagio.BagWriter(bag, [dataset_tools.HIGH_CAMERA_TOPIC])
    try:
        for index in range(11):
            writer.write(dataset_tools.HIGH_CAMERA_TOPIC, (index + 1) * 100_000_000,
                         np.full((20, 30, 3), index * 15, np.uint8))
    finally:
        writer.close()
    source = {p.name: p.read_bytes() for p in bag.iterdir() if p.is_file()}
    dataset_tools.write_json_atomic(root / 'workspace/session_4/latest_bag.json',
                                   {'path': str(bag), 'status': 'completed'})
    real_loader = dataset_tools.load_session2_module
    # The fixture project has no Session 2 source; use the real repository adapter.
    import unittest.mock
    with unittest.mock.patch.object(dataset_tools, 'load_session2_module',
                                    side_effect=lambda name, _: real_loader(name)):
        result = dataset_tools.extract_dataset_frames(project_root=root)
        output = Path(result['path'])
        assert result['counts'] == {dataset_tools.HIGH_CAMERA_TOPIC: 2}
        with (output / 'manifest.csv').open() as handle:
            rows = list(csv.DictReader(handle))
        assert [int(row['frame_1based']) for row in rows] == [5, 10]
        assert len(list((output / 'camera_1').glob('*.jpg'))) == 2
        assert {p.name: p.read_bytes() for p in bag.iterdir() if p.is_file()} == source
        with pytest.raises(ValueError, match='이미 있습니다'):
            dataset_tools.extract_dataset_frames(project_root=root)


@pytest.mark.parametrize('status', ['recording', 'failed'])
def test_latest_bag_rejects_incomplete_recording(tmp_path, status):
    dataset_tools.write_json_atomic(tmp_path / 'workspace/session_4/latest_bag.json',
                                   {'path': str(tmp_path), 'status': status})
    with pytest.raises(ValueError, match='정상 완료'):
        dataset_tools.resolve_bag(project_root=tmp_path)


def test_training_dataset_accepts_canonical_names_in_any_class_order(tmp_path):
    data = make_dataset(tmp_path, ('cone', 'green', 'red', 'yellow'))
    config, summary = training_tools.validate_training_dataset(data)
    assert config['names'] == ['cone', 'green', 'red', 'yellow']
    assert summary['train']['class_counts'] == dict.fromkeys(config['names'], 1)
    assert summary['val']['class_counts'] == dict.fromkeys(config['names'], 1)


def test_training_dataset_requires_all_classes_and_rejects_out_of_frame_boxes(tmp_path):
    data = make_dataset(tmp_path)
    label = tmp_path / 'valid/labels/2.txt'
    label.write_text('0 0.5 0.5 0.25 0.25\n')
    with pytest.raises(ValueError, match='val.*green'):
        training_tools.validate_training_dataset(data)
    label.write_text('2 0.99 0.5 0.25 0.25\n')
    with pytest.raises(ValueError, match='영상 밖'):
        training_tools.validate_training_dataset(data)


def test_training_dataset_rejects_wrong_class_names_and_train_val_overlap(tmp_path):
    data = make_dataset(tmp_path, ('red', 'yellow', 'green', 'traffic_cone'))
    with pytest.raises(ValueError, match='클래스 이름'):
        training_tools.validate_training_dataset(data)
    data.write_text(yaml.safe_dump({'train': 'train/images', 'val': 'valid/images',
                                    'names': list(training_tools.CLASS_NAMES)}))
    (tmp_path / 'valid/images/0.png').write_bytes((tmp_path / 'train/images/0.png').read_bytes())
    with pytest.raises(ValueError, match='동일 이미지'):
        training_tools.validate_training_dataset(data)


def test_completed_model_path_is_resolved_without_torch_and_failed_training_keeps_old_marker(tmp_path, monkeypatch):
    old_model = tmp_path / 'previous.pt'
    old_model.write_bytes(b'fixture')
    marker = tmp_path / 'workspace/session_4/latest_model.json'
    original = {'path': str(old_model), 'status': 'completed', 'device': 'cpu',
                'class_names': list(training_tools.CLASS_NAMES)}
    dataset_tools.write_json_atomic(marker, original)
    assert training_tools.resolve_latest_model(tmp_path) == old_model
    data = tmp_path / 'data.yaml'
    data.write_text('fixture data path')
    monkeypatch.setattr(training_tools, 'validate_training_dataset',
                        lambda *args: ({'names': list(training_tools.CLASS_NAMES)}, {}))

    def fail(*args, **kwargs):
        raise RuntimeError('fixture training failed')

    fake_learning = SimpleNamespace(train=fail)
    fake_platform = SimpleNamespace(prepare_learning_config=lambda _: None)
    monkeypatch.setattr(training_tools, 'load_session2_module',
                        lambda name, _: fake_platform if name == 'platform_support' else fake_learning)
    with pytest.raises(RuntimeError, match='fixture training failed'):
        training_tools.train_dataset(data, project_root=tmp_path)
    assert json.loads(marker.read_text()) == original
    assert training_tools.resolve_latest_model(tmp_path) == old_model


def test_success_marker_contains_actual_device_and_best_model_only_after_return(tmp_path, monkeypatch):
    data = tmp_path / 'data.yaml'
    data.write_text('fixture data path')
    best = tmp_path / 'best.pt'
    marker = tmp_path / 'workspace/session_4/latest_model.json'
    monkeypatch.setattr(training_tools, 'validate_training_dataset',
                        lambda *args: ({'names': list(training_tools.CLASS_NAMES)}, {'train': {}}))

    def train(*args, **kwargs):
        assert not marker.exists()
        assert kwargs['imgsz'] == 320 and kwargs['device'] == 'auto'
        assert kwargs['model'] == str(tmp_path / 'workspace/session_4/cache/models/yolov8n.pt')
        best.write_bytes(b'fixture best')
        return {'best': str(best), 'device': 'mps'}

    fake_learning = SimpleNamespace(train=train)
    fake_platform = SimpleNamespace(prepare_learning_config=lambda _: None)
    monkeypatch.setattr(training_tools, 'load_session2_module',
                        lambda name, _: fake_platform if name == 'platform_support' else fake_learning)
    result = training_tools.train_dataset(data, project_root=tmp_path)
    assert result['device'] == 'mps'
    assert json.loads(marker.read_text())['path'] == str(best)
    assert training_tools.resolve_latest_model(tmp_path) == best


def test_pretrained_cache_keeps_model_out_of_repository_root_and_preserves_explicit_file(tmp_path):
    cache = training_tools.resolve_training_model('yolov8n.pt', tmp_path)
    assert cache == tmp_path / 'workspace/session_4/cache/models/yolov8n.pt'
    assert not cache.exists()  # 경로 계산은 모델을 다운로드하지 않는다.
    local = tmp_path / 'my_custom.pt'
    local.write_bytes(b'custom fixture')
    assert training_tools.resolve_training_model(str(local), tmp_path) == local
    assert local.read_bytes() == b'custom fixture'
    with pytest.raises(ValueError, match='없습니다'):
        training_tools.resolve_training_model(str(tmp_path / 'missing/custom.pt'), tmp_path)
