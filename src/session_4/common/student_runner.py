"""설치된 공유 파일 또는 소스의 student/driving.py를 한 번만 읽는다."""
import importlib.util
from pathlib import Path
import sys


def student_source():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('session_4')) / 'student' / 'driving.py'
        if path.is_file():
            return path
    except (ImportError, LookupError):
        pass
    path = Path(__file__).resolve().parents[1] / 'student' / 'driving.py'
    if not path.is_file():
        raise FileNotFoundError('student/driving.py를 찾지 못했습니다. Session 4를 다시 빌드하세요.')
    return path


def load_student():
    name = 'session4_student_driving'
    path = student_source()
    cached = sys.modules.get(name)
    if cached is not None and Path(cached.__file__).resolve() == path.resolve():
        return cached
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(name, None)
        raise
    return module
