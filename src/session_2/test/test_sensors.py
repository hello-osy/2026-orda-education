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


def test_camera_recovers_after_second_device_initialization(monkeypatch):
    import sys
    from types import SimpleNamespace
    from session_2 import sensors

    class Output(queue.Queue):
        def cancel_join_thread(self):
            pass

    now = [0.0]
    stopped = [False]
    released = []
    samples = iter([(0.1, True), (1.2, False), (2.4, False), (3.0, True)])

    def read():
        try:
            now[0], ok = next(samples)
        except StopIteration:
            stopped[0] = True
            return False, None
        return ok, np.zeros((2, 2, 3), np.uint8) if ok else None

    camera = SimpleNamespace(isOpened=lambda: True, set=lambda *args: True,
                             read=read, release=lambda: released.append(True))
    monkeypatch.setitem(sys.modules, 'cv2', SimpleNamespace(
        CAP_AVFOUNDATION=1200, CAP_DSHOW=700, CAP_V4L2=200, CAP_ANY=0,
        CAP_PROP_FRAME_WIDTH=3, CAP_PROP_FRAME_HEIGHT=4, CAP_PROP_FPS=5,
        VideoCapture=lambda *args: camera))
    monkeypatch.setattr(sensors.time, 'monotonic', lambda: now[0])
    out = Output()
    sensors.camera_worker(out, SimpleNamespace(is_set=lambda: stopped[0], wait=lambda _: None), 0, 320, 240, 10)
    messages = list(out.queue)
    assert [kind for kind, _, _ in messages] == ['data', 'data']
    assert [stamp for _, stamp, _ in messages] == [0.1, 3.0]
    assert released == [True]


def test_camera_persistent_failure_still_exits_and_releases(monkeypatch):
    import sys
    from types import SimpleNamespace
    from session_2 import sensors

    class Output(queue.Queue):
        def cancel_join_thread(self):
            pass

    now = [0.0]
    released = []

    def read():
        now[0] += 1.0
        return False, None

    camera = SimpleNamespace(isOpened=lambda: True, set=lambda *args: True,
                             read=read, release=lambda: released.append(True))
    monkeypatch.setitem(sys.modules, 'cv2', SimpleNamespace(
        CAP_AVFOUNDATION=1200, CAP_DSHOW=700, CAP_V4L2=200, CAP_ANY=0,
        CAP_PROP_FRAME_WIDTH=3, CAP_PROP_FRAME_HEIGHT=4, CAP_PROP_FPS=5,
        VideoCapture=lambda *args: camera))
    monkeypatch.setattr(sensors.time, 'monotonic', lambda: now[0])
    out = Output()
    sensors.camera_worker(out, SimpleNamespace(is_set=lambda: False, wait=lambda _: None), 0, 320, 240, 10)
    kind, _, text = out.get_nowait()
    assert kind == 'error' and '10초' in text and now[0] == 10.0
    assert released == [True]
