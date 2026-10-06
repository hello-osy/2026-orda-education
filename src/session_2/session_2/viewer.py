"""실시간 USB 카메라 2대 + RPLIDAR A1 시각화 (ROS 불필요)."""
import argparse
from collections import deque
import multiprocessing as mp
import queue
import signal
import sys
import time

import numpy as np
from PySide6.QtCore import QPointF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QPushButton, QSizePolicy, QSpinBox, QVBoxLayout, QWidget,
)
from .sensors import camera_worker, lidar_worker, scan_xy, serial_ports, suggested_port


class SensorProcess:
    def __init__(self, target, args):
        ctx = mp.get_context('spawn')
        self.queue = ctx.Queue(maxsize=2)
        self.stop = ctx.Event()
        self.process = ctx.Process(target=target, args=(self.queue, self.stop, *args), daemon=True)
        self.process.start()
        self.stopping = None
        self.forced = False

    def drain(self):
        messages = []
        for _ in range(8):
            try:
                messages.append(self.queue.get_nowait())
            except queue.Empty:
                break
        return messages

    def request_stop(self):
        if self.stopping is None:
            self.stopping = time.monotonic()
            self.stop.set()

    def finished(self):
        if self.stopping is not None and time.monotonic() - self.stopping > 5 and self.process.is_alive():
            self.process.terminate()
            self.forced = True
        if self.process.is_alive():
            return False
        self.process.join()
        self.queue.close()
        return True


class ScanCanvas(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(320, 300)
        self.scan = np.empty((0, 3))
        self.limit = 2.0
        self.rotation = 0.0
        self.stale = True

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#111c2d'))
        cx, cy = self.width() / 2, self.height() / 2
        radius = min(cx, cy) - 30
        painter.setPen(QPen(QColor('#34465e'), 1))
        for part in range(1, 5):
            r = radius * part / 4
            painter.drawEllipse(QPointF(cx, cy), r, r)
            painter.drawText(int(cx + 5), int(cy - r + 16), f'{self.limit * part / 4:g} m')
        painter.drawLine(QPointF(cx - radius, cy), QPointF(cx + radius, cy))
        painter.drawLine(QPointF(cx, cy - radius), QPointF(cx, cy + radius))
        painter.setPen(QColor('#c5d4e8'))
        painter.drawText(int(cx - 38), 20, '±180° / 앞')
        painter.drawText(self.width() - 65, int(cy - 5), '+90° / 우')
        painter.drawText(6, int(cy - 5), '−90° / 좌')
        painter.drawText(int(cx - 25), self.height() - 30, '0° / 뒤')
        painter.setPen(QPen(QColor('#65748b' if self.stale else '#42e2b8'), 3))
        for x, y in scan_xy(self.scan, self.limit, self.rotation):
            painter.drawPoint(QPointF(cx + x * radius / self.limit, cy + y * radius / self.limit))
        painter.setBrush(QColor('#ffcc66'))
        painter.drawEllipse(QPointF(cx, cy), 5, 5)
        if self.stale:
            painter.setPen(QColor('#ffcc66'))
            painter.drawText(15, self.height() - 15, '실시간 데이터 없음 / 수신 대기')


class SensorPanel(QGroupBox):
    received = Signal(float, object)
    def __init__(self, title, is_lidar=False):
        super().__init__(title)
        self.is_lidar = is_lidar
        self.worker = None
        self.last = None
        self.timestamps = deque(maxlen=60)
        self.error = ''
        self.pixmap = None
        layout = QVBoxLayout(self)
        self.settings = QFormLayout()
        layout.addLayout(self.settings)
        self.button = QPushButton('시작')
        layout.addWidget(self.button)
        self.status = QLabel('연결 대기')
        self.status.setWordWrap(True)
        self.status.setMinimumHeight(55)
        layout.addWidget(self.status)
        if is_lidar:
            self.display = ScanCanvas()
        else:
            self.display = QLabel('카메라 시작 버튼을 누르세요')
            self.display.setMinimumSize(300, 220)
            self.display.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
            self.display.setAlignment(Qt.AlignCenter)
            self.display.setStyleSheet('background: #111c2d; color: #c5d4e8')
        layout.addWidget(self.display, 1)

    def start(self, target, args):
        self.error = ''
        self.last = None
        self.timestamps.clear()
        self.pixmap = None
        if self.is_lidar:
            self.display.scan = np.empty((0, 3))
        else:
            self.display.clear()
            self.display.setText('연결 중…')
        self.worker = SensorProcess(target, args)
        self.button.setText('중지')
        self.status.setText('연결 중…')

    def stop(self):
        if self.worker:
            self.worker.request_stop()
            self.button.setEnabled(False)
            self.status.setText('장치 정리 중…')

    def tick(self):
        if self.worker:
            for kind, stamp, payload in self.worker.drain():
                if kind == 'error':
                    self.error = payload
                elif kind == 'data':
                    self.last = stamp
                    self.timestamps.append(stamp)
                    self.received.emit(stamp, payload)
                    if self.is_lidar:
                        self.display.scan = payload
                    else:
                        rgb = np.ascontiguousarray(payload[:, :, ::-1])
                        image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy()
                        self.pixmap = QPixmap.fromImage(image)
            if self.worker.finished():
                if self.worker.forced:
                    self.error = '응답 지연으로 종료됨. 다시 시작해 주세요.'
                    if self.is_lidar:
                        self.error += ' 라이다가 계속 돌면 USB 전원을 분리하세요.'
                self.worker = None
                self.button.setEnabled(True)
                self.button.setText('시작')
        age = time.monotonic() - self.last if self.last is not None else float('inf')
        active = self.worker is not None and self.worker.stopping is None
        if self.is_lidar:
            self.display.stale = not active or age > 2
            self.display.update()
        elif active and age > 2:
            self.display.clear()
            self.display.setText('연결 중…' if self.last is None else '영상 수신 일시 중단 · 회복 대기 중…')
        elif not active and self.last is not None:
            self.display.clear()
            self.display.setText('카메라 중지됨 · 다시 연결하세요')
        elif self.pixmap is not None:
            self.display.setPixmap(self.pixmap.scaled(self.display.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        if self.error:
            self.status.setText('오류: ' + self.error)
            if not self.is_lidar and self.pixmap is None:
                self.display.setText('영상 없음 · 위 상태 안내를 확인하세요')
        elif self.worker and self.worker.stopping is not None:
            self.status.setText('장치 정리 중…')
        elif not active:
            self.status.setText('중지됨' if self.last else '연결 대기')
        elif self.last is None:
            self.status.setText('연결 중… 권한 / 장치 번호 / 포트 확인')
        else:
            span = self.timestamps[-1] - self.timestamps[0]
            rate = (len(self.timestamps) - 1) / span if span > 0 else 0
            detail = f'{len(self.display.scan)} points' if self.is_lidar else f'{self.pixmap.width()}×{self.pixmap.height()}'
            unit = 'Hz' if self.is_lidar else 'FPS'
            state = '수신 지연 · 회복 대기' if age > 2 else '수신 중'
            self.status.setText(f'{state} · {rate:.1f} {unit} · {detail}\n마지막 수신 {age:.1f}초 전')
        self.status.setToolTip(self.status.text())


class Viewer(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.closing = False
        self.setWindowTitle('ORDA Session 2 · USB 카메라 & RPLIDAR A1')
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        title = QLabel('Session 2  |  실시간 센서 모니터')
        title.setStyleSheet('font-size: 24px; font-weight: bold;')
        layout.addWidget(title)
        help_text = QLabel('카메라 번호는 내장 카메라를 포함합니다. 각 화면을 확인해 선택하세요. '
                           'Mac: 시스템 설정 → 개인정보 보호 및 보안 → 카메라에서 실행 앱 허용.\n'
                           '허브에서 끊기면 320×240 / 10 FPS를 선택하세요. FPS는 수신 기준 · 센서 간 시간 동기화 없음')
        help_text.setWordWrap(True)
        layout.addWidget(help_text)
        capture_settings = QHBoxLayout()
        self.resolution = QComboBox()
        for width, height in [(320,240),(640,360),(640,480),(1280,720)]:
            self.resolution.addItem(f'{width}×{height}', (width,height))
        selected = self.resolution.findData((args.width,args.height))
        if selected < 0:
            self.resolution.addItem(f'{args.width}×{args.height}',(args.width,args.height))
            selected = self.resolution.count()-1
        self.resolution.setCurrentIndex(selected)
        self.camera_fps = QSpinBox()
        self.camera_fps.setRange(1,60)
        self.camera_fps.setValue(args.fps)
        self.resolution.currentIndexChanged.connect(self.change_capture_settings)
        self.camera_fps.valueChanged.connect(self.change_capture_settings)
        capture_settings.addWidget(QLabel('요청 해상도'))
        capture_settings.addWidget(self.resolution)
        capture_settings.addWidget(QLabel('요청 FPS'))
        capture_settings.addWidget(self.camera_fps)
        capture_settings.addStretch()
        layout.addLayout(capture_settings)
        toolbar = QHBoxLayout()
        layout.addLayout(toolbar)
        for label, action in [('전체 시작', self.start_all), ('전체 중지', self.stop_all), ('포트 새로고침', self.refresh_ports)]:
            button = QPushButton(label)
            button.clicked.connect(action)
            toolbar.addWidget(button)
        body = QHBoxLayout()
        layout.addLayout(body, 1)
        self.cameras = []
        for i, index in enumerate(args.cameras):
            panel = SensorPanel(f'카메라 {i + 1}')
            panel.index = QSpinBox()
            panel.index.setRange(0, 15)
            panel.index.setValue(index)
            panel.settings.addRow('장치 번호', panel.index)
            panel.button.clicked.connect(lambda checked=False, p=panel: self.toggle_camera(p))
            body.addWidget(panel, 1)
            self.cameras.append(panel)
        self.lidar = SensorPanel('RPLIDAR A1 · 115200 baud', True)
        self.port = QComboBox()
        self.port.setEditable(True)
        self.lidar.settings.addRow('시리얼 포트', self.port)
        self.range = QDoubleSpinBox()
        self.range.setRange(0.5, 16)
        self.range.setSingleStep(0.5)
        self.range.setDecimals(1)
        self.range.setValue(args.range)
        self.range.setSuffix(' m')
        self.range.valueChanged.connect(self.change_range)
        self.lidar.settings.addRow('최대 표시 거리', self.range)
        self.rotation = QSpinBox()
        self.rotation.setRange(-180, 180)
        self.rotation.setSuffix('°')
        self.rotation.valueChanged.connect(self.change_rotation)
        self.lidar.settings.addRow('설치 방향 보정', self.rotation)
        self.lidar.button.clicked.connect(self.toggle_lidar)
        body.addWidget(self.lidar, 1)
        self.refresh_ports()
        self.port.setCurrentText(args.port or suggested_port())
        self.change_range(args.range)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)
        self.resize(1420, 740)
        if args.autostart:
            QTimer.singleShot(300, self.start_all)

    def change_range(self, value):
        self.lidar.display.limit = value
        self.lidar.display.update()

    def change_capture_settings(self, *_):
        self.args.width, self.args.height = self.resolution.currentData()
        self.args.fps = self.camera_fps.value()

    def change_rotation(self, value):
        self.lidar.display.rotation = value
        self.lidar.display.update()

    def refresh_ports(self):
        current = self.port.currentText()
        self.port.clear()
        self.port.addItems([p.device for p in serial_ports()])
        self.port.setCurrentText(current)

    def toggle_camera(self, panel):
        if panel.worker:
            panel.stop()
        else:
            for other in self.cameras:
                if other is not panel and other.worker and other.active_index == panel.index.value():
                    panel.error = '다른 화면이 사용 중인 카메라 번호입니다.'
                    return
            panel.active_index = panel.index.value()
            panel.start(camera_worker, (panel.active_index, self.args.width, self.args.height, self.args.fps))
            panel.index.setEnabled(False)

    def toggle_lidar(self):
        if self.lidar.worker:
            self.lidar.stop()
        elif not self.port.currentText().strip():
            self.lidar.error = 'RPLIDAR A1 시리얼 포트를 선택하세요.'
        else:
            self.lidar.start(lidar_worker, (self.port.currentText().strip(), 115200))
            self.port.setEnabled(False)

    def start_all(self):
        if self.closing:
            return
        for panel in self.cameras:
            if panel.worker is None:
                self.toggle_camera(panel)
        if self.lidar.worker is None:
            self.toggle_lidar()

    def stop_all(self):
        for panel in [*self.cameras, self.lidar]:
            panel.stop()

    def tick(self):
        for panel in [*self.cameras, self.lidar]:
            panel.tick()
        for panel in self.cameras:
            panel.index.setEnabled(panel.worker is None)
        camera_idle = all(panel.worker is None for panel in self.cameras)
        self.resolution.setEnabled(camera_idle)
        self.camera_fps.setEnabled(camera_idle)
        self.port.setEnabled(self.lidar.worker is None)
        if self.closing and all(p.worker is None for p in [*self.cameras, self.lidar]):
            self.close()

    def closeEvent(self, event):
        if any(p.worker for p in [*self.cameras, self.lidar]):
            self.closing = True
            self.centralWidget().setEnabled(False)
            self.stop_all()
            event.ignore()
        else:
            self.timer.stop()
            event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cameras', type=int, nargs=2, default=[0, 1], metavar=('LEFT', 'RIGHT'))
    parser.add_argument('--port', default='')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--range', type=float, default=2.0)
    parser.add_argument('--autostart', action='store_true')
    parser.add_argument('--list-ports', action='store_true')
    args = parser.parse_args()
    if args.list_ports:
        for port in serial_ports():
            print(f'{port.device}: {port.description} [{port.hwid}]')
        return
    if min(args.width, args.height, args.fps) <= 0 or not 0.5 <= args.range <= 16:
        parser.error('해상도/FPS는 양수, 표시 반경은 0.5~16 m이어야 합니다.')
    if min(args.cameras) < 0 or max(args.cameras) > 15 or args.cameras[0] == args.cameras[1]:
        parser.error('서로 다른 카메라 번호 0~15를 지정하세요.')
    from .platform_support import prepare_qt
    prepare_qt()
    app = QApplication(sys.argv[:1])
    viewer = Viewer(args)
    signal.signal(signal.SIGINT, lambda *_: viewer.close())
    signal.signal(signal.SIGTERM, lambda *_: viewer.close())
    viewer.show()
    return app.exec()


if __name__ == '__main__':
    mp.freeze_support()
    sys.exit(main())
