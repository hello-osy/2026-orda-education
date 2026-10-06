"""4번 전용 원본 이식: 실제 원문과 같은 입력 시퀀스의 결과를 비교한다."""
import ast
from pathlib import Path
from types import SimpleNamespace
import time

import cv2
import numpy as np
import pytest

from session_2.competition_lane import CompetitionLaneTracker
from session_2.competition_driving import DrivingPipeline, MotorControl


def scene(right=False, center=False, x=40):
    frame = np.full((360, 640, 3), 105, np.uint8)
    if right:
        cv2.rectangle(frame, (592, 145), (600, 359), (255, 255, 255), -1)
        cv2.rectangle(frame, (601, 145), (639, 359), (0, 255, 0), -1)
    if center:
        for y in (160, 220, 280, 340):
            cv2.rectangle(frame, (x, y), (x+8, min(y+25, 359)), (255, 255, 255), -1)
    return frame


def original_node():
    # 원문은 변경하지 않는다. 테스트에서만 ROS import/Node 상속을 제거하고 입출력을 대체한다.
    path = Path(__file__).parents[1]/'session_2/reference_osy_260809/timed_lane_offset_node.py.txt'
    tree = ast.parse(path.read_text())
    body = [n for n in tree.body if isinstance(n, (ast.Assign, ast.ClassDef))]
    cls = next(n for n in body if isinstance(n, ast.ClassDef))
    cls.bases = []
    env = {'np': np, 'cv2': cv2, 'Int16': SimpleNamespace}
    exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])), str(path), 'exec'), env)
    node = object.__new__(env['TimedLaneOffsetNggNode'])
    node.__dict__.update(vars(CompetitionLaneTracker()))
    node.to_bgr = lambda frame: frame
    node.segment_colors = lambda frame: frame
    node.show_mask_windows = lambda *args: None
    node.get_logger = lambda: SimpleNamespace(warn=lambda *args, **kwargs: None)
    node.offset_pub = SimpleNamespace(publish=lambda msg: setattr(node, 'output', msg.data))

    def debug(msg, frame, status, mask, measured_x, mode, raw_offset=None,
              target_x=None, line_kind='RIGHT', held_x=None):
        node.result = dict(status=status, measured_x=measured_x, target_x=target_x,
                           line_kind=line_kind, held_x=held_x, target=node.output)
    node.publish_debug = debug
    return node


def test_port_matches_pinned_original_across_transitions():
    port, original = CompetitionLaneTracker(), original_node()
    frames = ([scene()]*2 + [scene(center=True)]*3
              + [scene(right=True, center=True)]*12
              + [scene(center=True)]*2 + [scene(center=True, x=160)]*3
              + [scene()]*3 + [scene(center=True, x=70)]*4)
    states = []
    for frame in frames:
        actual = port.process(frame)
        original.image_callback(frame)
        for key, expected in original.result.items():
            if isinstance(expected, float):
                assert actual[key] == pytest.approx(expected)
            else:
                assert actual[key] == expected
        assert port.last_offset == original.last_offset
        assert port.last_center_line_x == original.last_center_line_x
        assert port.right_green_stable_count == original.right_green_stable_count
        states.append(actual['status'])
    assert {'OK', 'CENTER FALLBACK', 'CENTER LOST HOLD', 'CENTER JUMP HOLD'} <= set(states)


def test_right_recovery_requires_ten_frames_and_loss_holds_steering():
    tracker = CompetitionLaneTracker()
    for _ in range(9):
        result = tracker.process(scene(right=True, center=True))
        assert result['line_kind'] == 'CENTER'
    result = tracker.process(scene(right=True, center=True))
    assert result['line_kind'] == 'RIGHT' and result['status'] == 'OK'
    target = result['target']
    lost = tracker.process(scene())
    assert lost['status'] == 'CENTER LOST HOLD' and lost['target'] == target
    tracker.reset()
    assert tracker.process(scene())['target'] == 0


def test_horizontal_lines_removed_and_steering_sign():
    tracker = CompetitionLaneTracker()
    mask = np.zeros((120, 640), np.uint8)
    mask[60, 100:200] = 255
    mask[10:30, 500:508] = 255
    filtered = tracker.remove_horizontal_white_bands(mask)
    assert not filtered[55:66].any() and filtered[10:30, 500:508].all()
    assert tracker.map_line_x_to_offset(375, 505) == -45
    assert tracker.map_line_x_to_offset(505, 505) == 0
    assert tracker.map_line_x_to_offset(635, 505) == 45


def test_pipeline_class_palette_and_original_camera_geometry():
    class Segmenter:
        def predict(self, image):
            assert image.shape == (360, 640, 3)
            labels = np.ones((360, 640), np.uint8)
            labels[150:, 592:601] = 2
            labels[150:, 601:] = 4
            return labels
    pipeline = DrivingPipeline('cpu', segmenter=Segmenter())
    for _ in range(10):
        result = pipeline.process(np.zeros((240, 320, 3), np.uint8), 123.0)
    assert result['stamp'] == 123 and result['line_kind'] == 'RIGHT'
    assert result['overlay'].shape == (216, 640, 3)
    pipeline.reset()
    assert pipeline.tracker.last_offset == 0


def prepared_motor():
    motor = MotorControl('TEST-NOT-OPENED')
    now = time.monotonic()
    motor.state.connected = True
    motor.heartbeat()
    motor.receive(480, now)
    motor.calibrate()
    motor.set_target(20, now)
    return motor


def test_initial_straight_starts_at_arm_then_tracks_and_watchdog_still_stops():
    motor = prepared_motor()
    assert motor.command(time.monotonic()) == b'0 0\n'
    motor.arm(150)
    steer, drive = map(int, motor.command(time.monotonic()).split())
    assert steer == 0 and drive == 5
    motor.straight_until = time.monotonic()-1
    steer, drive = map(int, motor.command(time.monotonic()).split())
    assert steer > 0 and drive == 10
    motor.state.target_stamp = time.monotonic()-1
    assert motor.command(time.monotonic()) == b'0 0\n'
    assert not motor.state.armed


def test_missing_input_during_initial_straight_stops_not_creeps():
    motor = prepared_motor()
    motor.arm(150)
    motor.set_target(None, time.monotonic(), lane_missing=True)
    assert motor.command(time.monotonic()) == b'0 0\n'
    assert not motor.state.armed
