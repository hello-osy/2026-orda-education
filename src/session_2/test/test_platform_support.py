import sys
from types import SimpleNamespace

import pytest

from session_2.learning import choose_device
from session_2.platform_support import camera_backend, camera_help, prepare_qt


def test_learning_config_created_before_first_import(monkeypatch, tmp_path):
    from pathlib import Path
    from session_2.platform_support import prepare_learning_config
    monkeypatch.delenv('YOLO_CONFIG_DIR', raising=False)
    monkeypatch.delenv('MPLCONFIGDIR', raising=False)
    workspace = tmp_path / 'new' / 'workspace'
    paths = prepare_learning_config(workspace)
    assert Path(paths['YOLO_CONFIG_DIR']) == workspace / '.ultralytics'
    assert all(Path(path).is_dir() for path in paths.values())
    assert prepare_learning_config(workspace) == paths


def test_learning_config_preserves_custom_paths(monkeypatch, tmp_path):
    from session_2.platform_support import prepare_learning_config
    custom = tmp_path / 'custom'
    monkeypatch.setenv('YOLO_CONFIG_DIR', str(custom / 'yolo'))
    monkeypatch.setenv('MPLCONFIGDIR', str(custom / 'mpl'))
    paths = prepare_learning_config(tmp_path / 'workspace')
    assert paths == {'YOLO_CONFIG_DIR': str(custom / 'yolo'), 'MPLCONFIGDIR': str(custom / 'mpl')}
    assert (custom / 'yolo').is_dir() and (custom / 'mpl').is_dir()


def test_default_cameras_skip_metadata_and_missing_device_numbers(monkeypatch, tmp_path):
    from session_2.platform_support import default_camera_indices
    monkeypatch.setattr(sys, 'platform', 'linux')
    for name, index in [('video0', '0'), ('video2', '1'), ('video3', '0'),
                        ('video4', '1'), ('video10', '0')]:
        node = tmp_path / name
        node.mkdir()
        (node / 'index').write_text(index)
    (tmp_path / 'video5').mkdir()  # unplugged during discovery
    assert default_camera_indices(tmp_path) == [0, 3]
    (tmp_path / 'video0' / 'index').unlink()
    assert default_camera_indices(tmp_path) == [3, 10]


@pytest.mark.parametrize('platform', ['linux', 'darwin', 'win32'])
def test_default_cameras_without_linux_devices(monkeypatch, tmp_path, platform):
    from session_2.platform_support import default_camera_indices
    monkeypatch.setattr(sys, 'platform', platform)
    assert default_camera_indices(tmp_path) == [0, 1]


@pytest.mark.parametrize('platform,expected,hint', [
    ('darwin', 1200, '실행 앱'), ('win32', 700, '데스크톱 앱'),
    ('linux', 200, '/dev/video'),
])
def test_camera_platform(monkeypatch, platform, expected, hint):
    monkeypatch.setattr(sys, 'platform', platform)
    cv2 = SimpleNamespace(CAP_AVFOUNDATION=1200, CAP_DSHOW=700, CAP_V4L2=200, CAP_ANY=0)
    assert camera_backend(cv2) == expected
    assert hint in camera_help()


@pytest.mark.parametrize('cuda,mps,expected', [
    (True, False, 'cuda'), (False, True, 'mps'), (False, False, 'cpu'),
])
def test_device_selection(monkeypatch, cuda, mps, expected):
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(
        cuda=SimpleNamespace(is_available=lambda: cuda),
        backends=SimpleNamespace(mps=SimpleNamespace(is_available=lambda: mps))))
    assert choose_device() == expected
    assert choose_device('cpu') == 'cpu'
    if not cuda:
        with pytest.raises(ValueError):
            choose_device('cuda')
    if not mps:
        with pytest.raises(ValueError):
            choose_device('mps')


def test_qt_removes_only_opencv_paths(monkeypatch, tmp_path):
    monkeypatch.setitem(sys.modules, 'cv2', SimpleNamespace(__file__=str(tmp_path/'cv2'/'__init__.py')))
    monkeypatch.setenv('QT_QPA_PLATFORM_PLUGIN_PATH', str(tmp_path/'cv2'/'qt'/'plugins'))
    monkeypatch.setenv('QT_QPA_FONTDIR', str(tmp_path/'user-fonts'))
    prepare_qt()
    import os
    assert 'QT_QPA_PLATFORM_PLUGIN_PATH' not in os.environ
    assert os.environ['QT_QPA_FONTDIR'] == str(tmp_path/'user-fonts')
