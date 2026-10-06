"""학습 전 설치 검사: NumPy 확장 모듈의 ABI와 설정 폴더를 실제로 사용한다."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main():
    try:
        from session_2.platform_support import prepare_learning_config
        prepare_learning_config()
        import numpy as np
        import matplotlib
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        import contourpy
        import scipy
        from scipy.optimize import linear_sum_assignment
        from ultralytics import YOLO

        figure = Figure()
        figure.subplots().contour(np.array([[0., 1.], [2., 3.]]))
        FigureCanvasAgg(figure).draw()
        linear_sum_assignment(np.eye(2))
        for module in (np, matplotlib, contourpy, scipy):
            print(f'{module.__name__} {module.__version__}: {module.__file__}')
        print('Session 2 학습 라이브러리 OK')
        return 0
    except Exception as exc:
        print(f'설치 검사 실패: {exc}', file=sys.stderr)
        print('프로젝트 가상환경의 Python으로 다음을 실행하세요: '
              'python -m pip install -r src/session_2/requirements.txt', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
