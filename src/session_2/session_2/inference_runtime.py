"""PIDNet과 YOLO가 같은 MPS 명령 스트림을 동시에 사용하지 않도록 한다."""
from contextlib import contextmanager
import threading

_MPS_LOCK = threading.RLock()


@contextmanager
def inference_guard(device):
    if str(device) != 'mps':
        yield
        return
    import torch
    # 모델 생성·첫 warmup·추론 모두 같은 잠금을 사용한다.
    # 비동기 GPU 명령까지 끝난 후 다음 모델에 실행권을 넘긴다.
    with _MPS_LOCK:
        try:
            yield
        finally:
            torch.mps.synchronize()
