"""OS별 장치 선택과 Qt 실행 준비."""
import os
import sys


def camera_backend(cv2):
    if sys.platform == 'darwin':
        return cv2.CAP_AVFOUNDATION
    if sys.platform == 'win32':
        return cv2.CAP_DSHOW
    if sys.platform.startswith('linux'):
        return cv2.CAP_V4L2
    return cv2.CAP_ANY


def camera_help():
    if sys.platform == 'darwin':
        return '시스템 설정 → 개인정보 보호 및 보안 → 카메라에서 실행 앱의 권한도 확인하세요.'
    if sys.platform == 'win32':
        return '설정 → 개인정보 및 보안 → 카메라에서 카메라 액세스와 데스크톱 앱 액세스를 허용하세요.'
    return '/dev/video 장치의 접근 권한과 다른 앱의 카메라 사용 여부를 확인하세요.'


def prepare_qt():
    # Linux OpenCV wheel의 Qt5 경로가 PySide6(Qt6)를 가리지 않게 한다.
    # 먼저 import해야 뒤늦은 OpenCV import가 환경변수를 다시 설정하지 않는다.
    import cv2
    from pathlib import Path
    bundled = Path(cv2.__file__).resolve().parent / 'qt'
    for name in ('QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_QPA_FONTDIR'):
        value = os.environ.get(name)
        if value and Path(value).resolve().is_relative_to(bundled):
            os.environ.pop(name, None)
