import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import argparse
import time
from types import SimpleNamespace
from threading import Event

import numpy as np
from PySide6.QtWidgets import QApplication
from session_2.studio import Studio
from session_2.competition_driving import MotorControl


class SerialRecorder:
    def __init__(self):
        self.writes = []
        self.closed = False

    @property
    def in_waiting(self):
        return 15

    def read(self, size):
        return b'R,485,2.37,47\n'

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        self.closed = True


class Pipeline:
    def process(self, frame, stamp):
        return dict(stamp=stamp, target=10., error_px=-28.9,
                    segmentation=frame, overlay=frame)


def make_window():
    app = QApplication.instance() or QApplication([])
    window = Studio(argparse.Namespace(cameras=[0, 1], port='', range=6., width=320,
                                       height=240, fps=10, autostart=False, bag=None))
    window.live.timer.stop()
    window.select_stage(3)
    for panel in window.live.cameras:
        panel.worker = SimpleNamespace(stopping=None)
    return app, window


def pump(app, window, predicate, seconds=4, feed=True):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if feed:
            stamp = time.monotonic()
            window.live.cameras[0].last = stamp
            window.receive_live_frame(0, stamp, np.zeros((32, 48, 3), np.uint8))
            window.live.cameras[1].last = stamp
            window.receive_live_frame(1, stamp, np.full((32, 48, 3), 99, np.uint8))
        app.processEvents()
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('scale-car operation timed out')


def cleanup(app, window):
    for panel in window.live.cameras:
        panel.worker = None
    window.close()
    pump(app, window, lambda: not window.timer.isActive(), feed=False)


def test_scale_car_real_serial_path_requires_arm_and_stops_on_navigation():
    app, window = make_window()
    page = window.scale_car
    assert page.throttle.maximum() == 230 and page.throttle.value() == 150 and page.throttle.minimum() == 150
    serial = SerialRecorder()
    page.pipeline = Pipeline()
    page.motor_factory = lambda port, **kw: MotorControl(port, serial_factory=lambda *a, **k: serial, **kw)
    try:
        page.start_button.click()
        page.port.setCurrentText('MOCK')
        page.connect_button.click()
        pump(app, window, lambda: page.motor.snapshot().raw is not None and page.last_result is not None)
        assert serial.writes and all(data == b'0 0\n' for data in serial.writes)
        page.calibrate_button.click()
        assert page.motor.snapshot().center == 485
        page.arm_button.click()
        pump(app, window, lambda: page.motor.snapshot().p == 65.)
        assert page.segmentation.pixmap is not None and page.scan.pixmap is not None
        state = page.motor.snapshot()
        assert state.armed and state.p == 65. and state.sent_command
        page.stop_button.click()
        pump(app, window, lambda: serial.writes[-1] == b'0 0\n')
        assert not page.motor.snapshot().armed
        page.arm_button.click()
        pump(app, window, lambda: page.motor.snapshot().armed)
        window.select_stage(2)
        pump(app, window, lambda: serial.writes[-1] == b'0 0\n')
        assert not page.enabled and not page.motor.snapshot().armed
    finally:
        cleanup(app, window)
    assert serial.closed and serial.writes[-1] == b'0 0\n'


def test_camera_loss_stops_motor_and_slow_yolo_does_not_block_lane_inference():
    app, window = make_window()
    page = window.scale_car
    release = Event()
    serial = SerialRecorder()
    page.pipeline = Pipeline()

    class SlowYolo:
        def predict(self, frame, confidence):
            assert int(frame[0, 0, 0]) == 99
            release.wait(4)
            return frame, 1, 1.

    try:
        page.start_inference()
        page.yolo_model = SlowYolo()
        page.motor = MotorControl('MOCK', serial_factory=lambda *a, **kw: serial)
        page.motor.start()
        pump(app, window, lambda: page.motor.snapshot().raw is not None and page.last_result is not None)
        assert page.yolo_future is not None and not page.yolo_future.done()
        page.calibrate()
        page.arm()
        pump(app, window, lambda: page.motor.snapshot().drive > 0)
        window.live.cameras[0].worker = None
        pump(app, window, lambda: not page.motor.snapshot().armed and serial.writes[-1] == b'0 0\n', feed=False)
        assert page.segmentation.pixmap is None
        window.live.cameras[1].worker = None
        release.set()
        pump(app, window, lambda: page.yolo_future is None, feed=False)
        assert page.yolo_picture.pixmap is None
    finally:
        release.set()
        cleanup(app, window)


def test_busy_port_and_missing_connections_do_not_open_or_arm():
    app, window = make_window()
    page = window.scale_car
    try:
        page.arm()
        assert page.motor is None
        page.port.setCurrentText('')
        page.toggle_motor()
        assert page.motor is None
        window.live.lidar.worker = SimpleNamespace(stopping=None)
        window.live.port.setCurrentText('MOCK-LIDAR')
        page.port.setCurrentText('MOCK-LIDAR')
        page.toggle_motor()
        assert page.motor is None
        assert '라이다' in page.motor_status.text()
    finally:
        window.live.lidar.worker = None
        cleanup(app, window)


def test_fresh_lane_hold_keeps_last_angle_but_camera_loss_stops():
    app, window = make_window()
    page = window.scale_car
    assert page.throttle.maximum() == 230 and page.throttle.value() == 150 and page.throttle.minimum() == 150
    serial = SerialRecorder()

    class NoLane:
        def process(self, frame, stamp):
            return dict(stamp=stamp, target=12., error_px=None,
                        status='CENTER LOST HOLD', line_kind='CENTER',
                        segmentation=frame, overlay=frame)

    page.pipeline = NoLane()
    page.motor_factory = lambda port, **kw: MotorControl(port, serial_factory=lambda *a, **k: serial, **kw)
    try:
        page.start_button.click()
        page.port.setCurrentText('MOCK')
        page.connect_button.click()
        pump(app, window, lambda: page.motor.snapshot().raw is not None and page.last_result is not None)
        page.calibrate_button.click()
        page.arm_button.click()
        pump(app, window, lambda: page.motor.snapshot().drive == 150)
        assert not page.motor.snapshot().lane_missing and page.motor.snapshot().target == 12
        assert any(int(data.split()[0]) > 0 and int(data.split()[1]) == 150 for data in serial.writes)
        assert all(0 <= int(data.split()[1]) <= 150 for data in serial.writes)
        window.live.cameras[0].error = '카메라 끊김'
        pump(app, window, lambda: not page.motor.snapshot().armed and serial.writes[-1] == b'0 0\n', feed=False)
    finally:
        cleanup(app, window)
    assert serial.closed and serial.writes[-1] == b'0 0\n'
