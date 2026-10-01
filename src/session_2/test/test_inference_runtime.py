from concurrent.futures import ThreadPoolExecutor
import sys
import threading
from types import SimpleNamespace

import pytest
from session_2.inference_runtime import inference_guard


def test_mps_waits_for_previous_gpu_completion(monkeypatch):
    synchronizing = threading.Event()
    finish_sync = threading.Event()
    second_started = threading.Event()
    second_entered = threading.Event()

    def synchronize():
        synchronizing.set()
        assert finish_sync.wait(3)

    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(mps=SimpleNamespace(synchronize=synchronize)))

    def first():
        with inference_guard('mps'):
            pass

    def second():
        second_started.set()
        with inference_guard('mps'):
            second_entered.set()

    with ThreadPoolExecutor(2) as executor:
        a = executor.submit(first)
        assert synchronizing.wait(3)
        b = executor.submit(second)
        try:
            assert second_started.wait(3)
            assert not second_entered.wait(.1)
        finally:
            finish_sync.set()
        a.result(timeout=3)
        b.result(timeout=3)
    assert second_entered.is_set()


def test_guard_releases_after_failure_and_skips_cpu_cuda(monkeypatch):
    calls = []
    monkeypatch.setitem(sys.modules, 'torch', SimpleNamespace(mps=SimpleNamespace(synchronize=lambda: calls.append(1))))
    with pytest.raises(ValueError):
        with inference_guard('mps'):
            raise ValueError('inference failed')
    with ThreadPoolExecutor(1) as executor:
        def recover():
            with inference_guard('mps'):
                return True
        assert executor.submit(recover).result(timeout=3)
    for device in ('cpu', 'cuda'):
        with inference_guard(device):
            pass
    assert calls == [1, 1]
