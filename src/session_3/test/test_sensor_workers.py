"""장치가 요청 크기를 무시하거나 응답을 멈출 때의 동작을 검사한다."""
import cv2
import numpy as np
import pytest
import rplidar

from session_3.sensor_utils import SensorProcess, camera_worker, lidar_worker


class Output:
    def __init__(self):
        self.messages = []

    def cancel_join_thread(self):
        pass

    def put_nowait(self, message):
        self.messages.append(message)


class Stop:
    def __init__(self, frames=None):
        self.stopped = False
        self.frames = frames
        self.waits = []

    def is_set(self):
        return self.stopped

    def set(self):
        self.stopped = True

    def wait(self, timeout):
        self.waits.append(timeout)
        if self.frames is not None:
            self.frames -= 1
            if self.frames <= 0:
                self.set()
        return self.stopped


@pytest.mark.parametrize('input_size', [(640, 480), (320, 240), (160, 120)])
def test_camera_ignoring_requested_size_still_emits_requested_size(monkeypatch, input_size):
    width, height = input_size
    captured = np.full((height, width, 3), [10, 80, 150], dtype=np.uint8)

    class Camera:
        released = False

        def isOpened(self):
            return True

        def set(self, *args):
            return False  # 드라이버가 CAP_PROP_FRAME_WIDTH/HEIGHT 요청을 무시한다.

        def read(self):
            return True, captured

        def release(self):
            self.released = True

    camera = Camera()
    monkeypatch.setattr(cv2, 'VideoCapture', lambda *args: camera)
    output = Output()
    camera_worker(output, Stop(frames=3), 2, 320, 240, 10)
    frames = [value for kind, _, value in output.messages if kind == 'data']
    statuses = [value for kind, _, value in output.messages if kind == 'status']
    assert len(frames) == 3
    assert all(frame.shape == (240, 320, 3) and frame.nbytes == 230400 for frame in frames)
    assert all(np.array_equal(frame[0, 0], [10, 80, 150]) for frame in frames)
    assert len(statuses) == 1
    assert f'입력 {width}×{height} → 발행 320×240' in statuses[0]
    if input_size == (320, 240):
        assert frames[0] is captured
    assert camera.released


def run_lidar(monkeypatch, data, stop=None, stop_during_read=False, health=('Good', 0)):
    stop = stop or Stop()
    initially_stopped = stop.is_set()
    calls, events, instances = [], [], []

    class Serial:
        def read(self, count):
            calls.append(count)
            if stop_during_read:
                stop.set()
            return data

        def inWaiting(self):
            raise AssertionError('시간 제한 없는 inWaiting 반복을 사용하면 안 됩니다.')

    class Lidar:
        def __init__(self, port, baudrate, timeout):
            assert timeout == 3
            self._serial = Serial()
            instances.append(self)

        def get_info(self):
            events.append('get_info')
            return {'model': 24}

        def get_health(self):
            events.append('get_health')
            return health

        def clean_input(self):
            events.append('clean_input')

        def start_motor(self):
            events.append('start_motor')

        def iter_scans(self, **kwargs):
            assert kwargs == {'max_buf_meas': 3000}
            events.append('iter_scans')
            self._read_response(5)
            yield [(5, 0, 1000)]

        def stop(self):
            events.append('stop')

        def stop_motor(self):
            events.append('stop_motor')

        def disconnect(self):
            events.append('disconnect')

    monkeypatch.setattr(rplidar, 'RPLidar', Lidar)
    output = Output()
    lidar_worker(output, stop, 'test-only-no-device', 115200)
    if initially_stopped:
        assert instances == [] and events == []
    else:
        assert len(instances) == 1
        assert events[:5] == ['stop', 'stop_motor', 'clean_input', 'get_info', 'get_health']
        assert events[-3:] == ['stop', 'stop_motor', 'disconnect']
        if health in (('Good', 0), ('Warning', 0)):
            assert events[5] == 'start_motor'
            assert stop.waits == [3.0]
        else:
            assert 'start_motor' not in events
    return output, calls


@pytest.mark.parametrize('data', [b'', b'12'])
def test_lidar_short_read_reports_error_and_releases(monkeypatch, data):
    output, calls = run_lidar(monkeypatch, data)
    assert calls == [5]
    errors = [value for kind, _, value in output.messages if kind == 'error']
    assert len(errors) == 1
    assert f'5바이트 중 {len(data)}바이트 수신' in errors[0]
    assert not any(kind == 'data' for kind, _, _ in output.messages)


def test_lidar_complete_read_emits_scan(monkeypatch):
    output, calls = run_lidar(monkeypatch, b'12345')
    assert calls == [5]
    assert [(kind, value) for kind, _, value in output.messages] == [('data', [(5, 0, 1000)])]


def test_lidar_stop_before_read_is_quiet(monkeypatch):
    stop = Stop()
    stop.set()
    output, calls = run_lidar(monkeypatch, b'12345', stop=stop)
    assert calls == []
    assert output.messages == []


def test_lidar_stop_during_read_is_quiet(monkeypatch):
    output, calls = run_lidar(monkeypatch, b'12345', stop_during_read=True)
    assert calls == [5]
    assert output.messages == []


def test_lidar_stop_during_warmup_releases_without_scanning(monkeypatch):
    output, calls = run_lidar(monkeypatch, b'12345', stop=Stop(frames=1))
    assert calls == []
    assert output.messages == []


@pytest.mark.parametrize('health', [('Error', 2), 'Data in buffer'])
def test_lidar_invalid_health_never_starts_motor(monkeypatch, health):
    output, calls = run_lidar(monkeypatch, b'12345', health=health)
    assert calls == []
    assert len(output.messages) == 1
    assert output.messages[0][0] == 'error'
    assert '상태' in output.messages[0][2]


def test_sensor_shutdown_allows_three_second_read_to_finish():
    class Process:
        alive = True

        def join(self, timeout):
            if timeout >= 3.1:  # 3초 read 뒤 stop/stop_motor 정리 시간
                self.alive = False

        def is_alive(self):
            return self.alive

        def terminate(self):
            pytest.fail('정상적인 제한 시간 읽기가 끝나기 전에 강제 종료했습니다.')

    class Queue:
        closed = False

        def close(self):
            self.closed = True

    sensor = SensorProcess.__new__(SensorProcess)
    sensor.stop, sensor.process, sensor.out = Stop(), Process(), Queue()
    sensor.close()
    assert sensor.stop.is_set() and sensor.out.closed and not sensor.process.is_alive()
