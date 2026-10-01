import queue
import numpy as np
from session_2.sensors import publish, scan_xy


def test_camera_open_failure_reports_permission_and_releases_device(monkeypatch):
    import sys
    from types import SimpleNamespace
    from session_2.sensors import camera_worker

    class Output(queue.Queue):
        def cancel_join_thread(self):
            pass

    released = []
    camera = SimpleNamespace(isOpened=lambda: False, release=lambda: released.append(True))
    monkeypatch.setitem(sys.modules, 'cv2', SimpleNamespace(
        CAP_AVFOUNDATION=1200, CAP_DSHOW=700, CAP_V4L2=200, CAP_ANY=0, VideoCapture=lambda *args: camera))
    output = Output()
    camera_worker(output, None, 0, 320, 240, 10)
    kind, _, message = output.get_nowait()
    assert kind == 'error'
    assert '카메라 번호' in message and '확인하세요' in message
    assert released == [True]


def test_scan_coordinates_units_and_invalid_returns():
    scan = [[10, 0, 1000], [10, 90, 2000], [0, 0, 1000], [10, 0, 0],
            [10, 0, 7000], [10, float('nan'), 1000]]
    np.testing.assert_allclose(scan_xy(scan, 6), [[0, -1], [2, 0]], atol=1e-6)
    np.testing.assert_allclose(scan_xy([[10, 0, 1000]], 6, 90), [[1, 0]], atol=1e-6)
    assert scan_xy([], 6).shape == (0, 2)


def test_queue_keeps_newest_when_full():
    out = queue.Queue(maxsize=2)
    for n in range(10):
        publish(out, 'data', n)
    assert [out.get_nowait()[2], out.get_nowait()[2]] == [8, 9]
