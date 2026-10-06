"""session_1과 동일한 수업 디자인: 센서 → bag → 데이터 → YOLOv8."""
import argparse
import bisect
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import sys
import time

import numpy as np
from PySide6.QtCore import QProcess, QProcessEnvironment, Qt, QTimer, QUrl
from PySide6.QtGui import QDesktopServices, QImage, QPixmap
from PySide6.QtWidgets import (QApplication, QButtonGroup, QComboBox,
    QFileDialog, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QPlainTextEdit, QPushButton, QScrollArea, QSizePolicy, QSlider, QStackedWidget, QVBoxLayout, QWidget)

from .bagio import BagArchive, CAMERA_TOPICS, IMAGE_TYPES, Recorder, SCAN_TOPIC, SCAN_TYPE
from .learning import Detector, validate_dataset
from .viewer import ScanCanvas, Viewer
from .platform_support import default_camera_indices, prepare_learning_config

ROOT = Path(__file__).resolve().parents[3]
WORKSPACE = ROOT/'workspace'/'session_2'
STYLE = '''QWidget {font-size:14px;color:#18304b;background:#f3f6fb;}
QPushButton {padding:6px 8px;border:1px solid #c3cede;border-radius:8px;background:white;}
QPushButton:hover {background:#e1edff;} QPushButton:checked {background:#215aca;color:white;border-color:#215aca;}
QPushButton:disabled {color:#8895a6;background:#e9eef5;}
QGroupBox {font-weight:600;border:1px solid #c3cede;border-radius:12px;margin-top:12px;padding:8px 6px 6px;}
QGroupBox::title {subcontrol-origin:margin;left:12px;} QLabel {background:transparent;}
QComboBox,QDoubleSpinBox,QSpinBox,QLineEdit {background:white;padding:6px;border:1px solid #c3cede;border-radius:6px;}
QPlainTextEdit {background:#101c30;color:#dbeafe;border-radius:12px;padding:10px;font-size:13px;}
QSlider::groove:horizontal {height:6px;background:#cad6e7;border-radius:3px;}
QSlider::handle:horizontal {background:#215aca;width:20px;margin:-7px 0;border-radius:10px;}
QSlider::sub-page:horizontal {background:#6c9ceb;border-radius:3px;}'''
STAGES = [
    ('1. ROS2 bag 녹화 & 재생', '', ''),
    ('2. YOLOv8 학습 & 추론', '', ''),
    ('3. 자율주행 코드', '', ''),
    ('4. 스케일카 굴려보기', '', ''),
]
# 수업용 고정 설정: 화면에서 여러 값을 고를 필요가 없다.
YOLO_CAMERA_INDEX = 1  # 화면의 카메라 2. USB 장치 번호는 사용자가 별도로 지정합니다.

TRAINING = {'model': 'yolov8n.pt', 'epochs': '30', 'batch': '4', 'imgsz': '640', 'device': 'auto'}



def new_path(kind):
    parent = WORKSPACE/kind
    parent.mkdir(parents=True, exist_ok=True)
    return parent/datetime.now().strftime('%Y%m%d_%H%M%S_%f')


def button(text, fn):
    widget = QPushButton(text)
    widget.clicked.connect(fn)
    return widget


class StatusNote(QLabel):
    def setText(self, text):
        super().setText(text)
        self.setToolTip(text)


def note(text):
    widget = StatusNote(text)
    widget.setToolTip(text)
    widget.setMaximumHeight(56)
    widget.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
    widget.setWordWrap(True)
    return widget


class Picture(QGroupBox):
    def __init__(self, title):
        super().__init__(title)
        layout = QVBoxLayout(self)
        self.image = QLabel('bag 파일을 열어주세요')
        self.image.setAlignment(Qt.AlignCenter)
        self.image.setMinimumSize(140, 100)
        self.image.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Expanding)
        self.image.setStyleSheet('background:#101c30;border-radius:12px;color:#dbeafe;')
        self.caption = note('입력 대기')
        self.caption.setMinimumHeight(22)
        layout.addWidget(self.image, 1)
        layout.addWidget(self.caption)
        self.pixmap = None

    def clear(self, text='이 시각에 수신된 데이터가 없어요'):
        self.pixmap = None
        self.image.clear()
        self.image.setText(text)
        self.caption.setText('입력 없음')

    def display(self, image, caption):
        rgb = np.ascontiguousarray(image[:, :, ::-1])
        self.pixmap = QPixmap.fromImage(QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy())
        self.caption.setText(caption)
        self.fit()

    def fit(self):
        if self.pixmap is not None:
            self.image.setPixmap(self.pixmap.scaled(self.image.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fit()


class Studio(QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.setWindowTitle('ORDA 2회차 교육')
        self.stage = 0
        self.bag_mode = 0
        self.recorder = None
        self.last_recording = None
        self.archive = None
        self.detector = None
        self.loaded_weights = None
        self.live_frames = [None, None]
        self.infer_enabled = False
        self.live_generation = 0
        self.last_inferred = None
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.operation = ''
        self.generation = 0
        self.playing = False
        self.position = 0.
        self.anchor = time.monotonic()
        self.needs_frame = False
        self.last_rendered = None
        self.closing = False
        self.job = None
        self.job_cancelled = False
        self.job_output = None
        self.root = QWidget()
        self.setCentralWidget(self.root)
        layout = QVBoxLayout(self.root)
        layout.setContentsMargins(16,12,16,12)
        layout.setSpacing(8)
        title = QLabel('ORDA 2회차 교육')
        title.setStyleSheet('font-size:28px;font-weight:700;')
        layout.addWidget(title)
        nav = QHBoxLayout()
        self.nav_group = QButtonGroup(self)
        self.nav_buttons = []
        for i, (name, _, __) in enumerate(STAGES):
            item = QPushButton(name.replace('&', '&&'))
            item.setCheckable(True)
            item.toggled.connect(lambda checked, j=i: self.select_stage(j) if checked else None)
            self.nav_group.addButton(item)
            self.nav_buttons.append(item)
            nav.addWidget(item)
        layout.addLayout(nav)
        self.pages = QStackedWidget()
        layout.addWidget(self.pages, 1)
        self.live = Viewer(args)
        self.live_container = self.live.takeCentralWidget()
        self.live_container.hide()
        for panel, topic in zip(self.live.cameras, CAMERA_TOPICS):
            panel.received.connect(lambda stamp, data, t=topic: self.record_sample(t, stamp, data))
        for index, panel in enumerate(self.live.cameras):
            panel.received.connect(lambda stamp, data, i=index: self.receive_live_frame(i, stamp, data))
        self.live.lidar.received.connect(lambda stamp, data: self.record_sample(SCAN_TOPIC, stamp, data))
        self.build_sensor_page()
        self.build_learning_page()
        from .driving_page import DrivingPage
        self.driving = DrivingPage(WORKSPACE / 'autonomous_driving.py')
        self.pages.addWidget(self.driving)
        from .scale_car import ScaleCarPage
        self.scale_car = ScaleCarPage(self, Picture)
        self.pages.addWidget(self.scale_car)
        footer = QHBoxLayout()
        self.notice = note('')
        footer.addWidget(self.notice, 1)
        footer.addWidget(button('수업 종료', self.close))
        layout.addLayout(footer)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(40)
        self.resize(1440, 900)
        self.setMinimumSize(1100, 760)
        self.select_stage(0)
        if args.bag:
            self.load_bag(Path(args.bag))

    def add_page(self, page):
        page.layout().setContentsMargins(0, 0, 0, 0)
        page.layout().setSpacing(6)
        self.pages.addWidget(page)

    def build_transport(self, layout):
        timeline = QSlider(Qt.Horizontal)
        timeline.setAccessibleName('영상 재생 위치')
        timeline.setRange(0, 10000)
        timeline.sliderPressed.connect(self.pause)
        timeline.valueChanged.connect(self.seek)
        layout.addWidget(timeline)
        row = QHBoxLayout()
        play = button('재생', self.toggle_play)
        row.addWidget(play)
        row.addWidget(button('다음 장면', self.next_frame))
        clock = QLabel('0.0 / 0.0초')
        row.addWidget(clock, 1)
        layout.addLayout(row)
        self.timeline, self.play_button, self.clock = timeline, play, clock
        self.rate = 1.

    def build_sensor_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        cards = QHBoxLayout()
        self.pictures = [Picture('카메라 1 기록'), Picture('카메라 2 기록')]
        self.scan_group = QGroupBox('저장된 거리 정보')
        scan_layout = QVBoxLayout(self.scan_group)
        self.bag_scan = ScanCanvas()
        self.bag_scan.setMinimumSize(180, 140)
        self.bag_scan_status = note('기록 없음')
        scan_layout.addWidget(self.bag_scan, 1)
        scan_layout.addWidget(self.bag_scan_status)
        self.bag_scan_convention = QComboBox()
        self.bag_scan_convention.addItem('각도 기준: 대회 코드 / SLLIDAR', 0)
        self.bag_scan_convention.addItem('각도 기준: 이전 Session 2 녹화', 180)
        self.bag_scan_convention.setToolTip('이번 수정 전에 Session 2에서 녹화한 bag은 이전 Session 2 녹화를 선택하세요.')
        self.bag_scan_convention.currentIndexChanged.connect(self.change_bag_scan_convention)
        scan_layout.addWidget(self.bag_scan_convention)
        self.sensor_displays = []
        for field in (self.live.rotation,):
            self.live.lidar.settings.labelForField(field).hide()
            field.hide()
        self.bag_scan.limit = self.live.range.value()
        self.live.range.valueChanged.connect(self.change_bag_scan_range)
        self.live.lidar.settings.labelForField(self.live.port).setText('연결 장치')
        for panel in self.live.cameras:
            panel.settings.labelForField(panel.index).setText('카메라 번호')
        for i, panel in enumerate([*self.live.cameras, self.live.lidar]):
            panel.setTitle(('카메라 1', '카메라 2', '라이다 · 주변 물체까지의 거리')[i])
            inner = panel.layout()
            inner.setContentsMargins(6, 6, 6, 6)
            inner.setSpacing(4)
            # 센서 화면을 위로, 연결 설정을 아래로 옮긴다.
            while inner.count():
                inner.takeAt(0)
            stack = QStackedWidget()
            panel.display.setMinimumSize(140, 120)
            stack.addWidget(panel.display)
            stack.addWidget(self.pictures[i] if i < 2 else self.scan_group)
            self.sensor_displays.append(stack)
            inner.addWidget(stack, 1)
            panel.status.setFixedHeight(40)
            inner.addWidget(panel.status)
            settings = QWidget()
            settings.setLayout(panel.settings)
            settings.setFixedHeight(78 if panel.is_lidar else 48)
            inner.addWidget(settings)
            inner.addWidget(panel.button)
            panel.button.setText('연결 시작')
            cards.addWidget(panel, 1)
        layout.addLayout(cards, 1)
        controls = QHBoxLayout()
        self.source_label = note('현재 화면: 실시간 카메라·라이다')
        controls.addWidget(self.source_label, 1)
        controls.addWidget(button('실시간 보기', self.show_live))
        controls.addWidget(button('모두 연결', self.live.start_all))
        controls.addWidget(button('모두 중지', self.live.stop_all))
        layout.addLayout(controls)
        self.recording_controls = QGroupBox('녹화 · 재생')
        record_layout = QVBoxLayout(self.recording_controls)
        row = QHBoxLayout()
        self.record_button = button('● 녹화 시작', self.toggle_record)
        self.record_label = note('연결한 센서의 영상을 bag 파일로 저장합니다.')
        self.open_recorded = button('최근 녹화 열기', self.open_last_recording)
        self.open_recorded.setEnabled(False)
        row.addWidget(self.record_button)
        row.addWidget(self.record_label, 1)
        row.addWidget(self.open_recorded)
        row.addWidget(button('bag 파일 열기', self.pick_bag))
        record_layout.addLayout(row)
        self.bag_name = note('bag: 카메라 영상과 거리 정보를 함께 저장한 기록 파일')
        record_layout.addWidget(self.bag_name)
        topics = QHBoxLayout()
        self.topic_boxes = []
        for title in ('카메라 1 기록', '카메라 2 기록', '라이다 기록'):
            combo = QComboBox()
            combo.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
            combo.setMinimumContentsLength(10)
            combo.currentIndexChanged.connect(self.request_frame)
            self.topic_boxes.append(combo)
            topics.addWidget(QLabel(title))
            topics.addWidget(combo, 1)
        record_layout.addLayout(topics)
        self.build_transport(record_layout)
        layout.addWidget(self.recording_controls)
        self.add_page(page)

    def change_bag_scan_range(self, value):
        self.bag_scan.limit = value
        self.bag_scan.update()

    def change_bag_scan_convention(self, _index):
        self.bag_scan.rotation = self.bag_scan_convention.currentData()
        self.bag_scan.update()

    def build_learning_page(self):
        page = QWidget()
        layout = QHBoxLayout(page)
        self.capture_box = QGroupBox('사진 캡처하기')
        capture = QVBoxLayout(self.capture_box)
        capture.addWidget(note('녹화한 카메라 2 영상에서 학습에 쓸 사진을 꺼냅니다.\n5장면마다 사진 1장을 저장합니다.'))
        capture.addWidget(button('bag 파일 열기', self.pick_bag))
        self.extract_bag = note('선택한 기록 없음')
        capture.addWidget(self.extract_bag)
        capture.addWidget(note('학습용 사진은 카메라 2 기록에서만 수집합니다.'))
        self.extract_button = button('사진 캡처하기', self.extract)
        capture_actions = QHBoxLayout()
        capture_actions.addWidget(self.extract_button)
        capture_actions.addWidget(button('캡처 중지', self.cancel_job))
        capture.addLayout(capture_actions)
        capture.addWidget(button('사진 폴더 열기', lambda: self.open_folder(WORKSPACE/'images')))
        capture.addWidget(note('Roboflow에서 물체에 사각형과 이름을 표시하고 YOLOv8 형식으로 내려받으세요.'))
        capture.addWidget(button('YOLOv8 학습용 데이터셋 만들기\n- Roboflow로 이동', lambda: QDesktopServices.openUrl(QUrl('https://universe.roboflow.com/helloosy/2026-3cmko'))))
        self.capture_status = note('')
        capture.addWidget(self.capture_status)
        self.extract_log = QPlainTextEdit()
        self.extract_log.setReadOnly(True)
        self.extract_log.setMinimumHeight(65)
        self.extract_log.setPlaceholderText('사진 저장 내역')
        self.extract_log.setMaximumBlockCount(1000)
        capture.addWidget(self.extract_log, 1)
        layout.addWidget(self.capture_box, 1)

        self.training_box = QGroupBox('YOLOv8 모델 학습')
        training = QVBoxLayout(self.training_box)
        training.addWidget(note('정답이 표시된 사진을 보여 주며 물체의 모양과 이름을 모델에 가르칩니다.'))
        training.addWidget(button('학습 사진 목록 선택 · data.yaml', self.pick_yaml))
        self.data_yaml = QLineEdit()
        self.data_yaml.setPlaceholderText('내려받은 폴더 안의 data.yaml')
        training.addWidget(self.data_yaml)
        training.addWidget(button('학습 사진 확인', self.check_dataset))
        training.addWidget(note('고정 설정: YOLOv8n · 30회 · 사진 640\n한 번에 4장 · 계산 장치 자동 선택'))
        self.train_button = button('모델 학습 시작', self.train)
        train_actions = QHBoxLayout()
        train_actions.addWidget(self.train_button)
        train_actions.addWidget(button('학습 중지', self.cancel_job))
        training.addLayout(train_actions)
        self.training_status = note('학습이 끝나면 오른쪽에서 물체 찾기를 시작할 수 있습니다.')
        training.addWidget(self.training_status)
        self.train_log = QPlainTextEdit()
        self.train_log.setReadOnly(True)
        self.train_log.setMinimumHeight(65)
        self.train_log.setPlaceholderText('학습 진행 내역')
        self.train_log.setMaximumBlockCount(2000)
        training.addWidget(self.train_log, 1)
        layout.addWidget(self.training_box, 1)

        self.infer_box = QGroupBox('YOLOv8 모델 추론')
        inference = QVBoxLayout(self.infer_box)
        inference.addWidget(note('카메라 2 영상에서 물체를 찾습니다. 물체 찾기 시작을 누르세요.'))
        self.weights = QLineEdit()
        self.weights.setPlaceholderText('학습 결과 best.pt · 학습 후 자동 선택')
        inference.addWidget(self.weights)
        row = QHBoxLayout()
        inference.addWidget(button('모델 파일 선택', self.pick_weights))
        self.load_model = button('물체 찾기 시작', self.apply_model)
        row.addWidget(self.load_model)
        row.addWidget(button('물체 찾기 중지', self.stop_inference))
        inference.addLayout(row)
        inference.addWidget(note('추론 입력: 카메라 2 (고정)'))
        self.infer_picture = Picture('물체 찾기 결과')
        self.infer_picture.clear('카메라 2를 연결하고 학습한 모델을 선택하세요')
        inference.addWidget(self.infer_picture, 1)
        self.infer_connect = button('카메라 2 연결', self.toggle_infer_camera)
        inference.addWidget(self.infer_connect)
        self.infer_status = note('카메라 2 연결 대기 · 장치 번호는 1번 화면에서 설정합니다.')
        inference.addWidget(self.infer_status)
        layout.addWidget(self.infer_box, 1)
        self.add_page(page)

    def select_stage(self, stage):
        if self.stage != stage:
            self.pause()
            if self.stage == 3:
                self.scale_car.stop_inference()
            if stage == 3:
                self.stop_inference()
                self.scale_car.refresh_ports()
        self.stage = stage
        self.nav_buttons[stage].setChecked(True)
        self.pages.setCurrentIndex(stage)
        self.request_frame()

    def inference_active(self):
        return self.stage == 1

    def receive_live_frame(self, index, stamp, frame):
        self.live_frames[index] = (stamp, frame.copy())

    def live_sample(self):
        index = YOLO_CAMERA_INDEX
        panel = self.live.cameras[index]
        sample = self.live_frames[index]
        if (panel.worker is None or panel.worker.stopping is not None or panel.error or
                panel.last is None or sample is None or sample[0] != panel.last or
                time.monotonic() - sample[0] >= 2):
            return None
        return sample

    def ensure_live_camera(self):
        panel = self.live.cameras[YOLO_CAMERA_INDEX]
        if panel.worker is None:
            self.live.toggle_camera(panel)

    def toggle_infer_camera(self):
        self.live_generation += 1
        self.last_inferred = None
        panel = self.live.cameras[YOLO_CAMERA_INDEX]
        self.live_frames[YOLO_CAMERA_INDEX] = None
        self.live.toggle_camera(panel)
        self.infer_picture.clear('카메라 연결 상태를 확인하세요')

    def stop_inference(self):
        self.infer_enabled = False
        self.live_generation += 1
        self.last_inferred = None
        self.infer_picture.clear('물체 찾기 중지 · 카메라 원본 보기')

    def tick_live_inference(self):
        if not self.inference_active():
            return
        panel = self.live.cameras[YOLO_CAMERA_INDEX]
        self.infer_connect.setText('카메라 2 연결' if panel.worker is None else '카메라 2 중지')
        sample = self.live_sample()
        if sample is None:
            self.infer_status.setText(panel.error or '실시간 영상 없음 · 카메라 연결과 권한을 확인하세요.')
            self.infer_picture.clear('실시간 영상 수신 대기')
            return
        stamp, frame = sample
        if not self.infer_enabled or self.detector is None:
            self.infer_status.setText('카메라 수신 중 · 물체 찾기 시작을 누르세요.')
            self.infer_picture.display(frame, '실시간 카메라 원본')
            return
        if self.job and self.job_kind == 'train':
            self.infer_status.setText('학습 중 · 물체 찾기 대기')
            return
        self.infer_status.setText('실시간 물체 찾는 중 · 최신 영상부터 처리합니다.')
        key = (YOLO_CAMERA_INDEX, stamp)
        if self.future is not None or self.last_inferred == key:
            return
        self.last_inferred = key
        detector = self.detector
        generation = self.live_generation
        index = YOLO_CAMERA_INDEX
        self.submit('live', lambda: (generation, index, stamp, detector.predict(frame, .25)))

    def show_live(self):
        self.pause()
        self.bag_mode = 0
        for stack in self.sensor_displays:
            stack.setCurrentIndex(0)
        self.source_label.setText('현재 화면: 실시간 카메라·라이다')

    def show_recording(self):
        self.bag_mode = 1
        for stack in self.sensor_displays:
            stack.setCurrentIndex(1)
        self.source_label.setText('현재 화면: 저장된 bag 기록 · 녹화는 실시간 센서를 저장합니다')

    def open_folder(self, path):
        path.mkdir(parents=True, exist_ok=True)
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    def record_sample(self, topic, stamp, data):
        if self.recorder:
            self.recorder.submit(topic, stamp, data)

    def toggle_record(self):
        if self.recorder:
            self.recorder.stop()
            self.record_button.setEnabled(False)
            return
        now = time.monotonic()
        panels = [*self.live.cameras, self.live.lidar]
        topics = [topic for panel, topic in zip(panels, (*CAMERA_TOPICS,SCAN_TOPIC))
                  if panel.worker and panel.last is not None and now-panel.last < 2]
        if not topics:
            self.notice.setText('녹화 전에 카메라/라이다를 시작하고 실제 수신 중인지 확인하세요.')
            return
        self.recorder = Recorder(new_path('bags'), topics)
        self.open_recorded.setEnabled(False)
        self.record_button.setText('■ 녹화 종료 · 저장')

    def open_last_recording(self):
        if self.last_recording:
            self.select_stage(0)
            self.show_recording()
            self.load_bag(self.last_recording)

    def pick_bag(self):
        path = QFileDialog.getExistingDirectory(self,'ROS 2 SQLite bag 폴더',str(WORKSPACE/'bags'))
        if path:
            self.load_bag(Path(path))

    def load_bag(self, path):
        if self.future:
            self.notice.setText('현재 파일/모델 작업이 끝난 뒤 다시 선택하세요.')
            return
        if self.recorder and Path(path).resolve() == self.recorder.path.resolve():
            self.notice.setText('녹화를 종료하고 저장을 마친 뒤 열어주세요.')
            return
        self.show_recording()
        self.pause()
        self.generation += 1
        self.timeline.setValue(0)
        self.position = 0.
        old = self.archive
        self.archive = None
        self.bag_name.setText('기록을 불러오는 중…')
        self.extract_bag.setText('기록을 불러오는 중…')
        self.bag_scan.scan = np.empty((0, 3))
        self.bag_scan.stale = True
        self.bag_scan.update()
        for picture in self.pictures:
            picture.clear('bag을 불러오는 중…')
        def load():
            if old:
                old.close()
            return BagArchive(path)
        self.submit('load', load)
        self.notice.setText('녹화 기록을 불러오는 중…')

    def submit(self, operation, fn):
        self.operation = operation
        self.future_generation = self.generation
        self.future = self.executor.submit(fn)

    def request_frame(self, *_):
        self.generation += 1
        self.needs_frame = True

    def current_position(self):
        return self.position + (time.monotonic()-self.anchor)*self.rate if self.playing else self.position

    def pause(self):
        if self.playing:
            self.position = self.current_position()
        self.playing = False
        if hasattr(self,'play_button'):
            self.play_button.setText('재생')

    def toggle_play(self):
        if not self.archive:
            self.notice.setText('bag 파일을 먼저 선택하세요.')
            return
        self.show_recording()
        if self.playing:
            self.pause()
        else:
            if self.position >= self.archive.duration:
                self.position = 0.
            self.anchor = time.monotonic()
            self.playing = True
            self.play_button.setText('일시정지')
            self.needs_frame = True

    def set_position(self, seconds):
        self.pause()
        self.show_recording()
        self.position = max(0., min(seconds, self.archive.duration)) if self.archive else 0.
        self.request_frame()

    def seek(self, value):
        if self.archive:
            self.set_position(self.archive.duration*value/10000)

    def next_frame(self):
        if not self.archive:
            return
        index = 0
        topic = self.topic_boxes[index].currentData() or next(iter(self.archive.rows))
        stamps = self.archive.stamps[topic]
        target = self.archive.start + round(self.current_position()*1e9)
        index = min(bisect.bisect_right(stamps,target),len(stamps)-1)
        self.set_position((stamps[index]-self.archive.start)/1e9)

    def pick_weights(self):
        path, _ = QFileDialog.getOpenFileName(self,'학습한 YOLOv8 모델',str(WORKSPACE/'runs'),'YOLO 모델 (*.pt)')
        if path:
            self.weights.setText(path)

    def apply_model(self):
        if self.future:
            self.notice.setText('현재 프레임 처리가 끝난 뒤 모델을 적용하세요.')
            self.pause()
            return
        if self.job and self.job_kind == 'train':
            self.notice.setText('학습이 끝난 뒤 추론 모델을 적용하세요.')
            return
        self.pause()
        weights, device = self.weights.text(), 'auto'
        if not Path(weights).is_file() or Path(weights).suffix.lower() != '.pt':
            self.notice.setText('모델 파일 선택에서 학습한 best.pt 파일을 먼저 선택하세요.')
            return
        self.stop_inference()
        self.model_generation = self.live_generation
        self.ensure_live_camera()
        self.detector = None
        self.submit('model', lambda: Detector(weights, device))
        self.loaded_weights = weights
        self.notice.setText('물체를 찾는 모델을 불러오는 중…')

    def pick_yaml(self):
        path, _ = QFileDialog.getOpenFileName(self,'Roboflow data.yaml',str(WORKSPACE),'YAML (*.yaml *.yml)')
        if path:
            self.data_yaml.setText(path)

    def check_dataset(self):
        if self.future:
            self.notice.setText('현재 작업이 끝난 뒤 다시 검사하세요.')
            return
        self.pause()
        path = self.data_yaml.text()
        self.submit('validate', lambda: validate_dataset(path))
        self.training_status.setText('사진 파일과 정답 표시를 확인하는 중…')

    def extract(self):
        if not self.archive:
            self.notice.setText('bag 파일 열기를 눌러 녹화한 기록을 먼저 선택하세요.')
            return
        topic = self.topic_boxes[YOLO_CAMERA_INDEX].currentData()
        if not topic or topic == self.topic_boxes[0].currentData():
            self.notice.setText('카메라 2 기록이 필요합니다. 1번 화면에서 카메라 2 기록을 선택하세요.')
            return
        topics = [topic]
        output = new_path('images')
        self.start_job('extract',['--bag',str(self.archive.path),'--output',str(output),'--topics',*topics,'--stride','5'],output)

    def train(self):
        if not Path(self.data_yaml.text()).is_file():
            self.notice.setText('data.yaml을 먼저 선택하세요.')
            return
        self.pause()
        self.stop_inference()
        output = new_path('runs')
        self.start_job('train',['--data',self.data_yaml.text(),'--output',str(output),
            '--model',TRAINING['model'],'--epochs',TRAINING['epochs'],
            '--batch',TRAINING['batch'],'--imgsz',TRAINING['imgsz'],'--device',TRAINING['device']],output)

    def start_job(self, kind, arguments, output):
        if self.job:
            self.notice.setText('진행 중인 추출/학습이 끝나거나 중지된 뒤 시작하세요.')
            return
        self.job_kind, self.job_output = kind, output
        self.job_cancelled = False
        self.job_log = self.extract_log if kind == 'extract' else self.train_log
        self.job_log.clear()
        self.job = QProcess(self)
        self.job.setProgram(sys.executable)
        self.job.setArguments(['-u','-m','session_2.jobs',kind,*arguments])
        env = QProcessEnvironment.systemEnvironment()
        env.insert('PYTHONPATH', os.pathsep.join([str(ROOT/'src'/'session_2'), str(ROOT/'src'/'session_1')]))
        env.insert('PYTHONUTF8', '1')
        env.insert('PYTHONIOENCODING', 'utf-8')
        for name, path in prepare_learning_config(WORKSPACE).items():
            env.insert(name, path)
        self.job.setProcessEnvironment(env)
        self.job.setWorkingDirectory(str(ROOT))
        self.job.setProcessChannelMode(QProcess.MergedChannels)
        self.job.readyReadStandardOutput.connect(self.job_read)
        self.job.finished.connect(self.job_finished)
        self.job.errorOccurred.connect(self.job_error)
        self.job.start()
        self.extract_button.setEnabled(False)
        self.train_button.setEnabled(False)
        self.job_status = self.training_status if kind == 'train' else self.capture_status
        self.job_status.setText('학습 중 · 아래에서 반복 학습 진행을 확인하세요.' if kind == 'train' else '사진 저장 중')
        self.notice.setText(f'{kind} 작업 시작 · 저장 위치: {output}')

    def job_read(self):
        if self.job:
            text = bytes(self.job.readAllStandardOutput()).decode('utf-8',errors='replace')
            self.job_log.insertPlainText(text.replace('\r','\n'))
            self.job_log.verticalScrollBar().setValue(self.job_log.verticalScrollBar().maximum())

    def job_error(self, error):
        if error == QProcess.FailedToStart:
            self.job_log.appendPlainText('작업 프로세스를 시작하지 못했습니다: '+self.job.errorString())
            self.job_finished(-1,QProcess.CrashExit)

    def cancel_job(self):
        if self.job:
            self.job_cancelled = True
            proc = self.job
            proc.terminate()
            QTimer.singleShot(5000,lambda: proc.kill() if proc.state()!=QProcess.NotRunning else None)
            self.notice.setText('작업 중지 중… 일부 파일은 작업 폴더에 남습니다.')

    def job_finished(self, code, status):
        self.job_read()
        result_file = self.job_output/'result.json'
        successful = code == 0 and not self.job_cancelled and result_file.is_file()
        if successful:
            result = json.loads(result_file.read_text(encoding='utf-8'))
            if self.job_kind == 'train':
                self.weights.setText(result['best'])
                self.training_status.setText(f"학습 완료 · {result['device']} · 오른쪽의 물체 찾기 시작을 누르세요.")
            else:
                self.capture_status.setText('사진 저장 완료 · 사진 폴더에서 확인하세요.')
            self.notice.setText('완료 · '+str(self.job_output))
        else:
            self.notice.setText('작업 중지됨 · 일부 결과만 저장될 수 있어요.' if self.job_cancelled else '작업 실패 · 화면 로그를 확인하세요.')
            self.job_status.setText(self.notice.text())
        self.job = None
        self.extract_button.setEnabled(True)
        self.train_button.setEnabled(True)

    def finish_future(self):
        future, operation = self.future, self.operation
        self.future = None
        try:
            result = future.result()
            if operation == 'load':
                self.archive = result
                self.position = 0.
                self.last_rendered = None
                images = [t for t in result.rows if result.types[t] in IMAGE_TYPES]
                scans = [t for t in result.rows if result.types[t] == SCAN_TYPE]
                for i, combo in enumerate(self.topic_boxes):
                    combo.blockSignals(True)
                    combo.clear(); combo.addItem('선택 안 함',None)
                    available = images if i < 2 else scans
                    for topic in available:
                        combo.addItem(f'{topic} ({len(result.rows[topic])})',topic)
                    preferred = (*CAMERA_TOPICS,SCAN_TOPIC)[i]
                    selected = preferred if preferred in available else (available[min(i,len(available)-1)] if available else None)
                    if i == 1 and len(images) == 1:
                        selected = None
                    combo.setCurrentIndex(max(0,combo.findData(selected)))
                    combo.blockSignals(False)
                self.bag_name.setText(result.path.name)
                self.bag_name.setToolTip(str(result.path))
                self.extract_bag.setText(result.path.name)
                self.extract_bag.setToolTip(str(result.path))
                self.notice.setText(f'bag 준비 완료 · {result.duration:.1f}초 · 재생 버튼을 누르세요.')
                self.request_frame()
            elif operation == 'model':
                if self.model_generation != self.live_generation:
                    return
                self.detector = result
                self.infer_enabled = True
                self.last_inferred = None
                self.notice.setText(f'모델 준비 완료 · {result.device} · {Path(self.loaded_weights).name}')
                self.request_frame()
            elif operation == 'live':
                generation, index, stamp, prediction = result
                if generation == self.live_generation and index == YOLO_CAMERA_INDEX and self.infer_enabled and self.live_sample() is not None and time.monotonic() - stamp < 2:
                    frame, count, ms = prediction
                    self.infer_picture.display(frame, f'실시간 · {count}개 물체 · {ms:.0f} ms')
            elif operation == 'validate':
                self.training_status.setText('학습 사진 확인 완료 · '+json.dumps(result[1],ensure_ascii=False))
            elif operation == 'frame' and self.future_generation == self.generation:
                self.render_frame(result)
        except Exception as exc:
            self.pause()
            self.notice.setText('처리 실패: '+str(exc))
            if operation in ('live', 'model'):
                self.stop_inference()
            if operation == 'validate':
                self.training_status.setText('검사 실패: '+str(exc))
            elif operation == 'load':
                self.bag_name.setText('기록 열기 실패')
                self.extract_bag.setText('기록 열기 실패')

    def render_frame(self, result):
        seconds, samples, inference, topics = result
        self.last_rendered = seconds
        for i, picture in enumerate(self.pictures):
            sample = samples.get(topics[i])
            if sample is None:
                picture.clear()
            else:
                age = sample['age']
                caption = f"{sample['index']+1}번째 장면 · 선택 시각보다 {age:.2f}초 전"
                if age > 2:
                    caption += ' · 오래된 영상'
                picture.display(sample['data'], caption)
        scan = samples.get(topics[2])
        self.bag_scan.scan = scan['data'] if scan else np.empty((0,3))
        self.bag_scan.stale = scan is None or scan['age'] > 2
        self.bag_scan.update()
        self.bag_scan_status.setText(f"{len(scan['data'])}개 거리 측정점 · 기록 시각 차이 {scan['age']:.2f}초" if scan else '이 시각에 라이다 기록 없음')
        for timeline in (self.timeline,):
            timeline.blockSignals(True)
            timeline.setValue(round(seconds/max(self.archive.duration,.001)*10000))
            timeline.blockSignals(False)
        for clock in (self.clock,):
            clock.setText(f'{seconds:.1f} / {self.archive.duration:.1f}초')

    def tick(self):
        self.scale_car.tick(self.stage == 3 and not self.closing)
        if self.recorder:
            r = self.recorder
            counts = ' · '.join(f'{t}: {r.counts[t]}' for t in r.topics)
            self.record_label.setText(f"{'녹화 중' if r.accepting else '저장 중'} · {sum(r.counts.values())} 메시지 · 기록 큐 누락 {r.dropped}\n{counts}")
            if not r.thread.is_alive():
                self.last_recording = r.path if not r.error and sum(r.counts.values()) else None
                self.record_label.setText('녹화 실패: '+r.error if r.error else f'저장 완료 · {sum(r.counts.values())} 메시지 · 누락 {r.dropped} · {r.path.name}')
                self.open_recorded.setEnabled(self.last_recording is not None)
                self.recorder = None
                self.record_button.setEnabled(True)
                self.record_button.setText('● 녹화 시작')
        if self.future and self.future.done():
            self.finish_future()
        if self.closing:
            if self.future is None and self.recorder is None and self.job is None and not self.scale_car.busy() and all(p.worker is None for p in [*self.live.cameras,self.live.lidar]):
                self.close()
            return
        self.tick_live_inference()
        if self.archive and not self.future and (self.stage == 0 and self.bag_mode == 1) and (self.playing or self.needs_frame):
            seconds = min(self.current_position(),self.archive.duration)
            if self.playing and seconds >= self.archive.duration:
                self.pause()
                self.position = self.archive.duration
            self.needs_frame = False
            topics = [c.currentData() for c in self.topic_boxes]
            archive = self.archive
            def work():
                samples = archive.snapshot(seconds,[t for t in topics if t])
                return seconds,samples,None,topics
            self.submit('frame',work)

    def closeEvent(self, event):
        self.scale_car.stop_inference()
        if not self.closing and not self.driving.confirm_close():
            event.ignore()
            return
        self.scale_car.shutdown()
        pending = self.scale_car.busy() or self.future is not None or self.recorder is not None or self.job is not None or any(p.worker for p in [*self.live.cameras,self.live.lidar])
        if pending:
            event.ignore()
            if not self.closing:
                self.closing = True
                self.pause()
                self.root.setEnabled(False)
                self.live.stop_all()
                if self.recorder:
                    self.recorder.stop()
                self.cancel_job()
                self.notice.setText('센서와 기록 파일을 안전하게 닫고 있어요…')
        else:
            self.timer.stop()
            self.live.timer.stop()
            if self.archive:
                self.archive.close()
            self.scale_car.finish_shutdown()
            self.executor.shutdown(wait=False)
            event.accept()


def main():
    parser = argparse.ArgumentParser(description='ORDA Session 2 GUI 수업')
    parser.add_argument('--cameras',nargs=2,type=int,default=default_camera_indices())
    parser.add_argument('--port',default='')
    parser.add_argument('--width',type=int,default=320)
    parser.add_argument('--height',type=int,default=240)
    parser.add_argument('--fps',type=int,default=10)
    parser.add_argument('--range',type=float,default=2.)
    parser.add_argument('--autostart',action='store_true')
    parser.add_argument('--bag')
    parser.add_argument('--list-ports',action='store_true')
    args = parser.parse_args()
    if args.list_ports:
        from .sensors import serial_ports
        for port in serial_ports():
            print(f'{port.device}: {port.description}')
        return 0
    if min(args.cameras)<0 or max(args.cameras)>15 or len(set(args.cameras))!=2:
        parser.error('서로 다른 카메라 번호 0~15를 지정하세요.')
    if min(args.width,args.height,args.fps)<=0 or not .5<=args.range<=16:
        parser.error('해상도/FPS는 양수, 반경은 0.5~16m이어야 합니다.')
    WORKSPACE.mkdir(parents=True,exist_ok=True)
    prepare_learning_config(WORKSPACE)
    from .platform_support import prepare_qt
    prepare_qt()
    app = QApplication.instance() or QApplication(sys.argv[:1])
    app.setStyle('Fusion')
    app.setStyleSheet(STYLE)
    window = Studio(args)
    signal.signal(signal.SIGINT,lambda *_: window.close())
    signal.signal(signal.SIGTERM,lambda *_: window.close())
    window.show()
    return app.exec()


if __name__ == '__main__':
    sys.exit(main())
