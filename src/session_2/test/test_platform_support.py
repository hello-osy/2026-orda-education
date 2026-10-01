import sys
from types import SimpleNamespace

import pytest

from session_2.learning import choose_device
from session_2.platform_support import camera_backend, camera_help, prepare_qt


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
