"""common/ 폴더를 설치 전에도 session_4 패키지 이름으로 검사한다."""
import importlib.util
from pathlib import Path
import sys


root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('session_4', root / '__init__.py',
                                             submodule_search_locations=[str(root)])
module = importlib.util.module_from_spec(spec)
sys.modules['session_4'] = module
spec.loader.exec_module(module)
