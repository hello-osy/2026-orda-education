"""4번 수업: 실시간 차선 추론, 조향 시각화, 명시적으로 시작하는 모터 출력."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import time
from pathlib import Path

from PySide6.QtCore import Qt, QPointF, QRectF
from PySide6.QtGui import QColor, QPainter, QPen, QPainterPath, QKeySequence, QShortcut
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
                              QLabel, QPushButton, QComboBox, QSpinBox, QSizePolicy, QFileDialog)
from .driving import DrivingPipeline, MotorControl
from .sensors import serial_ports
from .learning import Detector


class PIDChart(QWidget):
    def __init__(self):
        super().__init__()
        self.values = deque(maxlen=180)
        self.setMinimumSize(180, 110)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def add(self, target, angle, output):
        self.values.append((target, angle, output))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#101c30'))
        height = (self.height() - 56) / 2
        for part, title, limit, traces in [
                (0, '바퀴 각도 · 주황: 목표 / 하늘: 실제', 45, [(0, '#ffae52'), (1, '#5ed8ff')]),
                (1, '조향 모터 힘 · 초록: PWM', 150, [(2, '#6ee7b7')])]:
            top = 28 + part * (height + 20)
            box = QRectF(34, top, max(1, self.width()-44), max(1, height-15))
            painter.setPen(QColor('#cbd5e1'))
            painter.drawText(8, int(top-9), title)
            painter.setPen(QColor('#334155'))
            painter.drawRect(box)
            painter.drawLine(QPointF(box.left(), box.center().y()), QPointF(box.right(), box.center().y()))
            painter.setPen(QColor('#cbd5e1'))
            painter.drawText(2, int(box.top()+12), str(limit))
            painter.drawText(2, int(box.bottom()), str(-limit))
            for field, color in traces:
                path = QPainterPath()
                started = False
                for index, values in enumerate(self.values):
                    value = values[field]
                    if value is None:
                        started = False
                        continue
                    x = box.left() + index / 179 * box.width()
                    y = box.center().y() - max(-limit, min(limit, value)) / limit * box.height()/2
                    if started:
                        path.lineTo(x, y)
                    else:
                        path.moveTo(x, y)
                        started = True
                painter.setPen(QPen(QColor(color), 2))
                painter.drawPath(path)


class ScaleCarPage(QWidget):
    def __init__(self, studio, picture_factory, pipeline_factory=DrivingPipeline,
                 motor_factory=MotorControl):
        super().__init__()
        self.studio = studio
        self.pipeline_factory = pipeline_factory
        self.motor_factory = motor_factory
        self.pipeline = None
        self.motor = None
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.yolo_executor = ThreadPoolExecutor(max_workers=1)
        self.yolo_future = None
        self.yolo_model = None
        self.yolo_weights = ''
        self.yolo_stamp = None
        self.future = None
        self.operation = ''
        self.generation = 0
        self.enabled = False
        self.closing = False
        self.last_stamp = None
        self.last_result = None
        self.message_until = 0.
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        row = QHBoxLayout()
        self.camera = QComboBox()
        self.camera.addItems(['카메라 1', '카메라 2'])
        self.camera.currentIndexChanged.connect(self.change_camera)
        row.addWidget(self.camera)
        self.start_button = QPushButton('추론 시작')
        self.start_button.clicked.connect(self.start_inference)
        row.addWidget(self.start_button)
        self.infer_stop = QPushButton('추론 중지')
        self.infer_stop.clicked.connect(self.stop_inference)
        row.addWidget(self.infer_stop)
        self.inference_status = QLabel('추론 시작 → 모터 연결 → 바퀴 중앙 맞추기 → 주행 시작')
        self.inference_status.setWordWrap(True)
        row.addWidget(self.inference_status, 1)
        layout.addLayout(row)
        views = QHBoxLayout()
        self.segmentation = picture_factory('Segmentation · 도로와 차선')
        self.scan = picture_factory('Scan line · 주행 오차')
        self.segmentation.clear('추론 시작을 누르세요')
        self.scan.clear('차선 위치와 목표 위치를 비교합니다')
        views.addWidget(self.segmentation, 1)
        views.addWidget(self.scan, 1)
        pid_box = QGroupBox('PID · 조향 안정화')
        pid_layout = QVBoxLayout(pid_box)
        self.chart = PIDChart()
        pid_layout.addWidget(self.chart, 1)
        self.pid_status = QLabel('모터 보드 연결 후 실제 바퀴 각도를 표시합니다.')
        self.pid_status.setWordWrap(True)
        self.pid_status.setMaximumHeight(65)
        pid_layout.addWidget(self.pid_status)
        right = QVBoxLayout()
        right.addWidget(pid_box, 1)
        self.yolo_picture = picture_factory('YOLO · 물체 찾기')
        self.yolo_picture.image.setMinimumHeight(80)
        self.yolo_picture.clear('YOLO 모델을 선택하세요')
        self.yolo_pick = QPushButton('YOLO 모델 선택 · best.pt')
        self.yolo_pick.clicked.connect(self.pick_yolo)
        self.yolo_picture.layout().addWidget(self.yolo_pick)
        right.addWidget(self.yolo_picture, 1)
        views.addLayout(right, 1)
        layout.addLayout(views, 1)
        hardware = QHBoxLayout()
        hardware.addWidget(QLabel('모터 보드'))
        self.port = QComboBox()
        self.port.setEditable(True)
        self.port.setMinimumWidth(180)
        hardware.addWidget(self.port, 1)
        refresh = QPushButton('포트 새로고침')
        refresh.clicked.connect(self.refresh_ports)
        hardware.addWidget(refresh)
        self.connect_button = QPushButton('모터 연결')
        self.connect_button.clicked.connect(self.toggle_motor)
        hardware.addWidget(self.connect_button)
        self.calibrate_button = QPushButton('바퀴 중앙 맞추기')
        self.calibrate_button.clicked.connect(self.calibrate)
        hardware.addWidget(self.calibrate_button)
        layout.addLayout(hardware)
        controls = QHBoxLayout()
        controls.addWidget(QLabel('45도까지 센서 변화량'))
        self.raw_span = QSpinBox()
        self.raw_span.setRange(20, 1023)
        self.raw_span.setValue(80)
        self.raw_span.setToolTip('중앙에서 바퀴 45도까지 움직일 때의 센서 숫자 변화량. 차량에 맞게 설정하고 연결하세요.')
        controls.addWidget(self.raw_span)
        controls.addWidget(QLabel('전진 힘'))
        self.throttle = QSpinBox()
        self.throttle.setRange(150, 230)
        self.throttle.setValue(150)
        self.throttle.setToolTip('전진 PWM 150~230 / 기본 150 / 차선 미검출 150')
        controls.addWidget(self.throttle)
        self.arm_button = QPushButton('주행 시작 · 실제 모터 출력')
        self.arm_button.clicked.connect(self.arm)
        controls.addWidget(self.arm_button, 1)
        self.stop_button = QPushButton('정지 · 모터 출력 0')
        self.stop_button.setStyleSheet('QPushButton {background:#b91c1c;color:white;font-weight:700;padding:10px;}')
        self.stop_button.clicked.connect(self.emergency_stop)
        controls.addWidget(self.stop_button, 1)
        layout.addLayout(controls)
        shortcut = QShortcut(QKeySequence('Space'), self)
        shortcut.setContext(Qt.WidgetWithChildrenShortcut)
        shortcut.activated.connect(self.emergency_stop)
        self.motor_status = QLabel('연결만으로 출발하지 않습니다. 바퀴를 똑바로 놓고 중앙을 맞춘 뒤 주행을 시작하세요.')
        self.motor_status.setWordWrap(True)
        self.motor_status.setMaximumHeight(50)
        layout.addWidget(self.motor_status)
        self.refresh_ports()

    def report_motor(self, text):
        self.motor_status.setText(text)
        self.message_until = time.monotonic() + 3

    def refresh_ports(self):
        if self.motor and self.motor.thread and self.motor.thread.is_alive():
            return
        current = self.port.currentText()
        self.port.clear()
        self.port.addItem('')
        arduino_ports = []
        for port in serial_ports():
            self.port.addItem(port.device)
            self.port.setItemData(self.port.count()-1, port.description, Qt.ToolTipRole)
            if 'arduino' in (port.manufacturer or '').lower():
                arduino_ports.append(port.device)
        if not current and len(arduino_ports) == 1:
            current = arduino_ports[0]
        self.port.setCurrentText(current)

    def sample(self):
        index = self.camera.currentIndex()
        panel = self.studio.live.cameras[index]
        sample = self.studio.live_frames[index]
        if (panel.worker is None or panel.worker.stopping is not None or panel.error or
                sample is None or panel.last != sample[0] or time.monotonic()-sample[0] > .5):
            return None
        return sample

    def change_camera(self, *_):
        self.stop_inference()
        self.last_result = None
        self.chart.values.clear()
        self.segmentation.clear('카메라 변경 · 추론 시작을 누르세요')
        self.scan.clear('새 카메라 수신 대기')

    def start_inference(self):
        if self.closing or self.future is not None:
            return
        if self.studio.job and self.studio.job_kind == 'train':
            self.inference_status.setText('학습을 중지하거나 마친 뒤 추론을 시작하세요.')
            return
        self.studio.stop_inference()
        self.generation += 1
        self.enabled = True
        self.last_stamp = None
        self.start_yolo()
        panel = self.studio.live.cameras[self.camera.currentIndex()]
        if panel.worker is None:
            self.studio.live.toggle_camera(panel)
        if self.pipeline is None:
            self.operation = 'model'
            self.future_generation = self.generation
            self.future = self.executor.submit(self.pipeline_factory)
            self.inference_status.setText('차선 모델을 불러오는 중…')
        else:
            self.inference_status.setText('실시간 카메라 수신 대기')

    def pick_yolo(self):
        self.stop_inference()
        path, _ = QFileDialog.getOpenFileName(self, 'YOLO 모델 선택', '', 'YOLO 모델 (*.pt)')
        if path:
            self.studio.weights.setText(path)
            self.yolo_weights = path
            self.yolo_model = None
            self.yolo_stamp = None
            self.yolo_picture.clear('모델 선택 완료 · 추론 시작을 누르세요')

    def start_yolo(self):
        if self.yolo_future is not None:
            self.yolo_picture.clear('YOLO 작업 중 · 끝난 뒤 모델을 다시 선택하세요')
            return
        path = self.studio.weights.text().strip()
        if not Path(path).is_file() or Path(path).suffix.lower() != '.pt':
            self.yolo_picture.clear('YOLO 모델 선택에서 best.pt를 고르세요')
            return
        if self.yolo_model is not None and path == self.yolo_weights:
            self.yolo_stamp = None
            return
        self.yolo_weights = path
        self.yolo_model = None
        self.yolo_stamp = None
        self.yolo_generation = self.generation
        self.yolo_operation = 'model'
        self.yolo_future = self.yolo_executor.submit(Detector, path, 'auto')
        self.yolo_picture.clear('YOLO 모델을 불러오는 중…')

    def tick_yolo(self, active):
        if self.yolo_future and self.yolo_future.done():
            future = self.yolo_future
            self.yolo_future = None
            try:
                result = future.result()
                if self.yolo_generation == self.generation and self.enabled and active and not self.closing:
                    if self.yolo_operation == 'model':
                        self.yolo_model = result
                    elif self.sample() is not None and time.monotonic()-self.yolo_stamp < 2:
                        frame, count, ms = result
                        self.yolo_picture.display(frame, f'실시간 · {count}개 물체 · {ms:.0f} ms')
            except Exception as exc:
                self.yolo_model = None
                self.yolo_picture.clear('YOLO 처리 실패: ' + str(exc))
        if self.closing or not active or not self.enabled:
            return
        sample = self.sample()
        if sample is None:
            self.yolo_picture.clear('실시간 카메라 수신 대기')
            return
        if self.yolo_model is None and self.yolo_future is None and not Path(self.studio.weights.text().strip()).is_file():
            self.yolo_picture.clear('YOLO 모델 선택에서 best.pt를 고르세요')
            return
        if self.yolo_model is None or self.yolo_future is not None or self.yolo_stamp == sample[0]:
            return
        self.yolo_stamp = sample[0]
        self.yolo_generation = self.generation
        self.yolo_operation = 'frame'
        self.yolo_future = self.yolo_executor.submit(self.yolo_model.predict, sample[1], .25)

    def emergency_stop(self):
        if self.motor:
            self.motor.disarm('사용자 정지 · 다시 주행 시작 필요')
        self.report_motor('모터 정지 요청 · 다시 주행 시작을 눌러야 출발합니다.')

    def stop_inference(self):
        self.enabled = False
        self.generation += 1
        self.last_stamp = None
        if self.motor:
            self.motor.set_target(None, 0.)
            self.motor.disarm('추론 중지 · 다시 주행 시작 필요')
        self.inference_status.setText('추론 중지')

    def toggle_motor(self):
        if self.motor and self.motor.thread and self.motor.thread.is_alive():
            self.motor.close()
            self.report_motor('모터 연결 종료 중…')
            return
        port = self.port.currentText().strip()
        if not port:
            self.report_motor('모터 보드의 포트를 선택하세요.')
            return
        if self.studio.live.lidar.worker and port == self.studio.live.port.currentText().strip():
            self.report_motor('라이다가 사용 중인 포트입니다. 모터 보드 포트를 선택하세요.')
            return
        self.motor = self.motor_factory(port, raw_span=self.raw_span.value())
        self.motor.start()
        self.chart.values.clear()
        self.report_motor('모터 보드 연결 중 · 바퀴 위치 수신 대기')

    def calibrate(self):
        try:
            if self.motor is None:
                raise ValueError('모터 보드를 먼저 연결하세요.')
            self.motor.calibrate()
            self.report_motor('바퀴 중앙 보정 완료')
        except ValueError as exc:
            self.report_motor(str(exc))

    def arm(self):
        try:
            if not self.enabled or self.sample() is None or self.motor is None:
                raise ValueError('실시간 추론과 모터 연결을 먼저 시작하세요.')
            self.motor.heartbeat()
            self.motor.arm(self.throttle.value())
            self.report_motor('실제 모터 출력 중')
        except ValueError as exc:
            self.report_motor(str(exc))

    def tick(self, active):
        self.tick_yolo(active)
        now = time.monotonic()
        if self.motor:
            if active and not self.closing:
                self.motor.heartbeat()
            state = self.motor.snapshot()
            alive = bool(self.motor.thread and self.motor.thread.is_alive())
            self.port.setEnabled(not alive)
            self.raw_span.setEnabled(not alive)
            self.connect_button.setText('모터 연결 해제' if alive else '모터 연결')
            self.calibrate_button.setEnabled(not state.armed)
            self.throttle.setEnabled(not state.armed)
            self.arm_button.setEnabled(not state.armed)
            actual = state.angle if state.connected and now-state.feedback_stamp <= .5 else None
            target = state.target if self.enabled and now-state.target_stamp <= .5 else None
            self.chart.add(target, actual, state.steer)
            angle_text = '수신 대기' if actual is None else f'{actual:+.1f}°'
            target_text = '없음' if target is None else f'{target:+.1f}°'
            self.pid_status.setText(f'목표 {target_text} / 실제 {angle_text}\nP {state.p:+.1f} · I {state.i:+.1f} · D {state.d:+.1f}\n조향 {state.steer} / 전진 {state.drive}')
            sent = state.sent_command or '아직 없음'
            detail = f'{state.reason} · 마지막 송신: {sent}'
            # Keep actionable button errors visible until connection state changes.
            if now >= self.message_until or state.armed or self.closing:
                self.motor_status.setText(detail)
            self.motor_status.setToolTip(detail)
        else:
            target = None
            if self.enabled and self.last_result and now-self.last_result['stamp'] <= .5:
                target = self.last_result['target']
            self.chart.add(target, None, 0)
            target_text = '없음' if target is None else f'{target:+.1f}°'
            self.pid_status.setText(f'목표 {target_text} / 실제 각도 수신 대기\n모터 미연결 · 명령 전송 없음')
        if self.future and self.future.done():
            future = self.future
            self.future = None
            try:
                result = future.result()
                if self.future_generation == self.generation and self.enabled and active and not self.closing:
                    if self.operation == 'model':
                        self.pipeline = result
                    elif self.sample() is not None and now-result['stamp'] <= .5:
                        self.last_result = result
                        if self.motor:
                            self.motor.set_target(result['target'], result['stamp'],
                                                  lane_missing=result['target'] is None)
                        self.segmentation.display(result['segmentation'], '도로·실선·점선을 색으로 표시')
                        error = result['error_px']
                        caption = '차선 미검출 · 주행 시작 상태에서는 중앙 조향 / 전진 PWM 150' if error is None else f'주행 오차 {error:+.1f}px · 목표 {result["target"]:+.1f}°'
                        self.scan.display(result['overlay'], caption)
            except Exception as exc:
                self.stop_inference()
                self.inference_status.setText('추론 실패: ' + str(exc))
        if self.closing or not active or not self.enabled:
            return
        sample = self.sample()
        if sample is None:
            if self.motor:
                self.motor.set_target(None, 0.)
                self.motor.disarm('카메라 수신 끊김 · 다시 주행 시작 필요')
            self.segmentation.clear('실시간 카메라 수신 대기')
            self.scan.clear('새 영상이 없으면 모터를 멈춥니다')
            panel = self.studio.live.cameras[self.camera.currentIndex()]
            self.inference_status.setText(panel.error or '카메라 수신 대기 · 연결과 권한을 확인하세요')
            return
        if self.pipeline is None or self.future is not None or sample[0] == self.last_stamp:
            return
        self.last_stamp = sample[0]
        self.operation = 'frame'
        self.future_generation = self.generation
        self.future = self.executor.submit(self.pipeline.process, sample[1], sample[0])
        self.inference_status.setText('차선 추론 중 · 오른쪽 실선 기준')

    def shutdown(self):
        if not self.closing:
            self.closing = True
            self.stop_inference()
            if self.motor:
                self.motor.close()

    def busy(self):
        return self.future is not None or self.yolo_future is not None or bool(self.motor and self.motor.thread and self.motor.thread.is_alive())

    def finish_shutdown(self):
        self.executor.shutdown(wait=False)
        self.yolo_executor.shutdown(wait=False)
