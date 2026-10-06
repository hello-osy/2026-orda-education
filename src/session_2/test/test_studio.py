import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import argparse
import ast
import time

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QTextCursor
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QPushButton, QMessageBox, QScrollArea
from session_2.studio import Studio, TRAINING
from session_2.driving_page import DrivingPage
from test_bagio import make_bag


def wait_until(app, predicate, seconds=5):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        app.processEvents()
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('GUI 작업 시간 초과')


def test_unified_pages_recording_replay_inference_and_fixed_training(tmp_path):
    app = QApplication.instance() or QApplication([])
    args = argparse.Namespace(cameras=[0, 1], port='', range=6., width=320, height=240,
                              fps=10, autostart=False, bag=None)
    window = Studio(args)
    try:
        window.show()
        window.resize(1100, 760)
        app.processEvents()
        assert window.pages.count() == 4
        assert window.windowTitle() == 'ORDA 2회차 교육'
        assert not window.notice.text()
        assert '작업 폴더' not in [b.text() for b in window.findChildren(QPushButton)]
        assert all(panel.isVisible() for panel in [*window.live.cameras, window.live.lidar])
        window.load_bag(make_bag(tmp_path/'bag'))
        wait_until(app, lambda: window.last_rendered is not None and window.future is None)
        window.next_frame()
        wait_until(app, lambda: window.last_rendered == .1)
        assert window.pictures[0].pixmap.width() == 64
        assert all(stack.currentIndex() == 1 for stack in window.sensor_displays)
        window.show_live()
        assert all(stack.currentIndex() == 0 for stack in window.sensor_displays)
        window.set_position(.5)
        wait_until(app, lambda: window.last_rendered == .5)
        assert all(stack.currentIndex() == 1 for stack in window.sensor_displays)
        window.nav_buttons[1].setChecked(True)
        assert all(box.isVisible() for box in [window.capture_box, window.training_box, window.infer_box])
        assert not window.recording_controls.isVisible()
        assert window.pages.currentIndex() == 1
        for stage in (0, 1):
            window.select_stage(stage)
            app.processEvents()
            assert not isinstance(window.pages.currentWidget(), QScrollArea)
            assert window.size().width() == 1100 and window.size().height() == 760
            for control in window.pages.currentWidget().findChildren(QPushButton):
                if control.isVisible():
                    corner = control.mapTo(window, control.rect().bottomRight())
                    assert corner.y() < window.height() and corner.x() < window.width()
        window.select_stage(1)

        class FakeDetector:
            device = 'cpu'
            def predict(self, frame, confidence):
                self.pixel = int(frame[0, 0, 0])
                return frame, 1, 2.

        window.detector = FakeDetector()
        active_detector = window.detector
        window.weights.setText(str(tmp_path/'missing.pt'))
        window.apply_model()
        assert window.detector is active_detector
        assert window.future is None
        assert 'best.pt' in window.notice.text()
        assert not hasattr(window, 'infer_timeline')
        assert not window.infer_enabled

        yaml = tmp_path/'data.yaml'
        yaml.write_text('names: [car]')
        window.data_yaml.setText(str(yaml))
        jobs = []
        window.start_job = lambda *args: jobs.append(args)
        window.extract_button.click()
        capture_arguments = jobs.pop()[1]
        assert capture_arguments[capture_arguments.index('--topics')+1:capture_arguments.index('--stride')] == ['/camera/low/image_raw']
        # 카메라 2가 없으면 카메라 1로 대체하지 않는다.
        camera2_index = window.topic_boxes[1].currentIndex()
        window.topic_boxes[1].setCurrentIndex(0)
        window.extract_button.click()
        assert not jobs and '카메라 2 기록이 필요' in window.notice.text()
        window.topic_boxes[1].setCurrentIndex(camera2_index)
        window.train_button.click()
        arguments = jobs[0][1]
        for key, value in TRAINING.items():
            assert arguments[arguments.index('--'+key)+1] == value
        window.nav_buttons[2].setChecked(True)
        assert not window.driving.code.isReadOnly()
        ast.parse(window.driving.code.toPlainText())
        assert 'class PID:' in window.driving.code.toPlainText()
        assert 'class Segmenter:' in window.driving.code.toPlainText()
        assert 'from session_1.pid_view' not in window.driving.code.toPlainText()
    finally:
        window.close()
        wait_until(app, lambda: not window.timer.isActive())


def test_editor_save_reload_search_indent_and_close(tmp_path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    path = tmp_path/'autonomous_driving.py'
    editor = DrivingPage(path)
    editor.select_lesson(3)
    editor.show()
    app.processEvents()
    original = editor.code.toPlainText()
    assert editor.code.document().findBlockByNumber(14).layout().formats()
    editor.code.moveCursor(QTextCursor.End)
    editor.code.insertPlainText('\n# 저장 확인\n')
    assert editor.code.document().isModified()
    editor.save_button.click()
    assert path.read_text() == original+'\n# 저장 확인\n'
    assert not editor.code.document().isModified()
    reopened = DrivingPage(path)
    reopened.select_lesson(3)
    assert reopened.code.toPlainText() == path.read_text()
    reopened.open_search()
    reopened.search.setText('class PID:')
    reopened.find_next()
    assert reopened.code.textCursor().selectedText() == 'class PID:'
    reopened.code.setPlainText('if True:')
    reopened.code.moveCursor(QTextCursor.End)
    QTest.keyClick(reopened.code, Qt.Key_Return)
    assert reopened.code.toPlainText() == 'if True:\n    '
    monkeypatch.setattr(QMessageBox, 'question', lambda *args: QMessageBox.Cancel)
    assert not reopened.confirm_close()
    monkeypatch.setattr(QMessageBox, 'question', lambda *args: QMessageBox.Discard)
    assert reopened.confirm_close()
    assert path.read_text() == original+'\n# 저장 확인\n'
    editor.close()
    reopened.close()


def test_code_lessons_follow_unsaved_edits_and_preserve_whole_file(tmp_path):
    app = QApplication.instance() or QApplication([])
    path = tmp_path/'autonomous_driving.py'
    page = DrivingPage(path)
    page.show()
    app.processEvents()
    try:
        for index, included, excluded in [(0, 'class Segmenter:', 'class PID:'),
                                          (1, 'def scan_line(', 'class Segmenter:'),
                                          (2, 'class PID:', 'def scan_line(')]:
            page.lesson_buttons[index].setChecked(True)
            assert page.lesson_index == index
            assert included in page.preview.toPlainText()
            assert excluded not in page.preview.toPlainText()
            assert page.preview.isReadOnly()
            assert '이 코드의 역할' in page.explanation.toPlainText()
            assert '계산 순서' in page.explanation.toPlainText()
            assert not page.save_button.isVisible()
        page.lesson_buttons[3].setChecked(True)
        assert page.code.isVisible() and page.save_button.isVisible()
        edited = page.code.toPlainText().replace('kp=6.5', 'kp=7.0')
        page.code.setPlainText(edited)
        page.code.document().setModified(True)
        page.lesson_buttons[2].setChecked(True)
        assert 'kp=7.0' in page.preview.toPlainText()
        page.open_search()
        page.search.setText('class PID:')
        page.find_next()
        assert page.preview.textCursor().selectedText() == 'class PID:'
        assert page.save()
        assert path.read_text() == edited
        assert 'class Segmenter:' in path.read_text()
        page.code.setPlainText('def unfinished(')
        page.select_lesson(0)
        assert '부분 코드 표시 불가' in page.status.text()
        assert page.code.toPlainText() == 'def unfinished('
        assert path.read_text() == edited
    finally:
        page.close()


def test_live_inference_uses_latest_camera_frame_and_discards_old_results(tmp_path):
    from types import SimpleNamespace
    from threading import Event
    app = QApplication.instance() or QApplication([])
    args = argparse.Namespace(cameras=[0, 1], port='', range=6., width=320, height=240,
                              fps=10, autostart=False, bag=None)
    window = Studio(args)
    window.live.timer.stop()
    release = Event()
    pixels = []

    class Detector:
        def predict(self, frame, confidence):
            release.wait(2)
            pixels.append(int(frame[0, 0, 0]))
            return frame, 1, 3.

    def feed(index, pixel, stamp=None):
        stamp = time.monotonic() if stamp is None else stamp
        window.live.cameras[index].last = stamp
        window.receive_live_frame(index, stamp, np.full((30, 40, 3), pixel, dtype=np.uint8))

    try:
        window.select_stage(1)
        for panel in window.live.cameras:
            panel.worker = SimpleNamespace(stopping=None)
        window.detector = Detector()
        window.infer_enabled = True
        assert window.archive is None
        feed(0, 200)
        window.tick()
        assert window.future is None
        feed(1, 11)
        window.tick()
        assert window.operation == 'live'
        feed(1, 22)
        feed(1, 33)
        # Camera 1 must never replace the latest camera 2 input.
        feed(0, 201)
        feed(1, 99)
        release.set()
        wait_until(app, lambda: window.future is None and pixels == [11, 99])
        assert window.infer_picture.pixmap.width() == 40
        assert '실시간' in window.infer_picture.caption.text()
        # A disconnected source must clear the last result, not replay it.
        feed(1, 100, time.monotonic() - 3)
        window.tick()
        assert window.infer_picture.pixmap is None
        assert len(pixels) == 2
        window.stop_inference()
        feed(1, 101)
        window.tick()
        assert window.infer_picture.caption.text() == '실시간 카메라 원본'
        assert len(pixels) == 2
    finally:
        release.set()
        window.stop_inference()
        for panel in window.live.cameras:
            panel.worker = None
        window.close()
        wait_until(app, lambda: not window.timer.isActive())


def test_camera_panel_hides_stale_image_until_recovery():
    from types import SimpleNamespace
    from session_2.viewer import SensorPanel
    app = QApplication.instance() or QApplication([])
    panel = SensorPanel('카메라')
    now = time.monotonic()
    messages = [('data', now-3, np.zeros((8, 8, 3), np.uint8))]

    def drain():
        batch = messages[:]
        messages.clear()
        return batch

    panel.worker = SimpleNamespace(drain=drain, finished=lambda: False, stopping=None)
    panel.tick()
    assert '회복 대기' in panel.display.text()
    assert panel.display.pixmap().isNull()
    messages.append(('data', time.monotonic(), np.ones((8, 8, 3), np.uint8)))
    panel.tick()
    assert not panel.display.pixmap().isNull()
    assert '수신 중' in panel.status.text()
    panel.worker = None
    panel.close()
