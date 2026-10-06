"""OS별 장치 선택과 Qt 실행 준비."""
import os
import sys
from pathlib import Path


def prepare_learning_config(workspace=None):
    """Ultralytics import 전에 설정 경로를 생성한다. 사용자 환경변수는 유지한다."""
    workspace = Path(workspace) if workspace is not None else Path(__file__).resolve().parents[3] / 'workspace' / 'session_2'
    paths = {}
    for name, folder in [('YOLO_CONFIG_DIR', '.ultralytics'), ('MPLCONFIGDIR', '.matplotlib')]:
        path = Path(os.environ.get(name) or workspace / folder).expanduser().resolve()
        path.mkdir(parents=True, exist_ok=True)
        os.environ[name] = str(path)
        paths[name] = str(path)
    # Ubuntu의 .pth가 시스템 mpl_toolkits를 미리 로드할 수 있다.
    # 실제 선택된 Matplotlib과 같은 설치 위치의 확장을 우선한다.
    import importlib.util
    spec = importlib.util.find_spec('matplotlib')
    if spec is not None and spec.origin:
        toolkit = Path(spec.origin).parent.parent / 'mpl_toolkits'
        if toolkit.is_dir():
            import mpl_toolkits
            mpl_toolkits.__path__ = [str(toolkit), *[p for p in mpl_toolkits.__path__ if p != str(toolkit)]]
    return paths


def default_camera_indices(sysfs_root='/sys/class/video4linux'):
    """Linux UVC 기본 영상 노드를 선택하고 메타데이터 노드는 제외한다.

    USB 재연결 후 번호에 빈 곳이 생길 수 있다. 사용자가 지정한
    --cameras는 이 기본값보다 우선하며, 다른 OS의 열거 방식은 유지한다.
    """
    if not sys.platform.startswith('linux'):
        return [0, 1]
    indices = []
    for node in Path(sysfs_root).glob('video*'):
        try:
            index = int(node.name[5:])
            if 0 <= index <= 15 and (node / 'index').read_text().strip() == '0':
                indices.append(index)
        except (OSError, ValueError):
            continue
    indices.sort()
    if len(indices) >= 2:
        return indices[:2]
    if indices:
        return [indices[0], next(i for i in range(16) if i != indices[0])]
    return [0, 1]


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
