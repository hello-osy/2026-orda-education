"""학생 계약, 센서 미수신 정지, 실제 알고리즘의 좌표/신호 상태를 검사합니다."""
import math
import numpy as np
import pytest

from session_4 import contracts, starter, solution
from session_4.yolo_node import color_detections, YoloDetector


def scene(center=0.0, curvature=0.0):
    height, width = 240, 320
    focal = width / (2 * math.tan(.6))
    v, u = np.mgrid[:height, :width]
    dz = (v - height / 2) / focal
    denominator = np.maximum(.03, math.sin(.12) + dz * math.cos(.12))
    x = .18 * (math.cos(.12) - dz * math.sin(.12)) / denominator
    y = -.18 * (u - width / 2) / focal / denominator
    image = np.full((height, width, 3), 45, dtype=np.uint8)
    road_center = center + curvature * x * x
    offset = .75 * np.sqrt(1 + (2 * curvature * x) ** 2)
    white = (np.abs(y - (road_center - offset)) < .025) & (x < 5)
    yellow = (np.abs(y - (road_center + offset)) < .025) & (x < 5) & (x % .7 < .4)
    image[white] = (240, 240, 240)
    image[yellow] = (245, 210, 0)
    return image


def scan_cones(center_offset=0.0):
    angles = -math.pi + np.arange(720) * (2 * math.pi / 720)
    ranges = np.full(angles.shape, math.inf)
    directions = np.stack((np.cos(angles), np.sin(angles)), axis=1)
    for x in (1.2, 2.2):
        for y in (-.65 + center_offset, .65 + center_offset):
            projection = directions[:, 0] * x + directions[:, 1] * y
            discriminant = .12 ** 2 - (x * x + y * y - projection * projection)
            intersects = (discriminant >= 0) & (projection > 0)
            ranges[intersects] = np.minimum(ranges[intersects],
                projection[intersects] - np.sqrt(discriminant[intersects]))
    return ranges, -math.pi, 2 * math.pi / 720, .15, 12.0


def perception(signal='unknown', size=0):
    result = contracts.no_perception()
    result['traffic_light'] = {'detected': signal != 'unknown', 'color': signal,
                               'bbox': [120, 40], 'size': [20, size * 240], 'conf': .95}
    return result


def test_starter_has_complete_transport_contract_and_drives_straight():
    lane = starter.lane(scene())
    cone = starter.rubbercone(scan_distances())
    detected = starter.yolo([], [], [], [])
    assert lane == contracts.no_path()
    assert cone['valid']
    assert detected == contracts.no_perception()
    assert starter.DrivingController().step(lane, cone, detected) == [0, 125]
    assert starter.DrivingController().step(None, cone, detected) == [0, 0]


@pytest.mark.parametrize('offset', [-.18, 0, .18])
def test_lane_projection_sign_and_metric_offset(offset):
    result = solution.lane(scene(offset))
    assert result['valid']
    assert result['error'] == pytest.approx(offset, abs=.05)


def test_lane_lookahead_turns_left_on_left_bend():
    result = solution.lane(scene(curvature=.08))
    assert result['valid']
    assert .06 < result['error'] < .22
    assert not solution.lane(np.zeros((240, 320, 3), dtype=np.uint8))['valid']


def test_lidar_corridor_detection_and_sign():
    straight = solution._cone_follow(*scan_cones())
    offset = solution._cone_follow(*scan_cones(.12))
    assert straight['valid'] and abs(straight['error']) < .03
    assert offset['valid'] and .07 < offset['error'] < .18
    assert not solution._cone_follow(np.full(360, math.inf), -math.pi, math.pi / 180, .15, 12)['valid']


def test_detection_contract_prioritizes_red_and_filters_broken_boxes():
    packet = {'width': 320, 'height': 240, 'detections': [
        {'label': 'green', 'confidence': .95, 'bbox': [100, 20, 140, 60]},
        {'label': 'red', 'confidence': .9, 'bbox': [200, 25, 220, 45]},
        {'label': 'cone', 'confidence': .8, 'bbox': [150, 160, 165, 200]},
        {'label': 'red', 'confidence': .99, 'bbox': [2, 2, 1, 1]},
    ]}
    result = solution._receive_detections(packet)
    assert result['traffic_light']['detected'] and result['traffic_light']['color'] == 'red' and len(result['rubbercone']['bbox']) == 1
    assert result['traffic_light']['bbox'] == [200, 25]
    assert result['traffic_light']['size'] == [20, 20]
    assert result['traffic_light']['conf'] == .9
    assert result['rubbercone'] == {'detected': True, 'bbox': [[150, 160]], 'size': [[15, 40]], 'conf': [.8]}
    assert solution._receive_detections({'valid': False}) == contracts.no_perception()


@pytest.mark.parametrize('index', [0, 1, 2])
def test_controller_stops_when_any_role_or_sensor_becomes_stale(index):
    values = [{'valid': True, 'error': 0.0}, {'valid': True, 'error': 0.0}, perception()]
    controller = solution.DrivingController()
    assert controller._step(*values, .05)[1] > 0
    values[index] = dict(values[index], stale=True)
    assert controller._step(*values, .05) == [0, 0]
    values[index] = None
    assert controller._step(*values, .05) == [0, 0]


def test_signal_stop_latches_until_green_and_ignores_same_signal_after_departure():
    lane, cone = {'valid': True, 'error': 0.0}, {'valid': False, 'error': 0.0}
    controller = solution.DrivingController()
    assert controller._step(lane, cone, perception('red', .04), .05)[1] > 0
    assert controller._step(lane, cone, perception('red', .10), .05) == [0, 0]
    assert controller._step(lane, cone, perception('unknown'), .05) == [0, 0]
    assert controller._step(lane, cone, perception('green', .1), .05)[1] > 0
    assert controller._step(lane, cone, perception('red', .1), .05)[1] > 0
    for _ in range(42):
        controller._step(lane, cone, perception('unknown'), .05)
    assert controller._step(lane, cone, perception('red', .1), .05) == [0, 0]


def test_controller_uses_cones_then_returns_to_lanes():
    controller = solution.DrivingController()
    lane = {'valid': True, 'error': -.15}
    cone = {'valid': True, 'error': .15}
    assert controller._step(lane, cone, perception(), .05)[0] > 0
    cone['valid'] = False
    assert controller._step(lane, cone, perception(), .05)[0] < 0
    lane['valid'] = False
    assert controller._step(lane, cone, perception(), .05) == [0, 0]


def test_simulator_color_detector_is_labeled_and_finds_red_and_cones():
    import cv2
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.circle(image, (180, 50), 12, (255, 0, 0), -1)
    cv2.fillPoly(image, [np.array([[120, 160], [110, 200], [130, 200]])], (255, 100, 0))
    packet = color_detections(image)
    assert packet['source'] == 'simulator_color'
    result = solution._receive_detections(packet)
    assert result['traffic_light']['color'] == 'red' and len(result['rubbercone']['bbox']) == 1


def test_color_detector_rejects_round_yellow_road_dash_below_horizon():
    import cv2
    image = np.full((240, 320, 3), 70, dtype=np.uint8)
    # 실제 곡선 주행 영상에서 오인식한 점선과 같은 위치·크기의 노란 영역.
    cv2.ellipse(image, (10, 157), (8, 9), 20, 0, 360, (255, 230, 0), -1)
    assert color_detections(image)['detections'] == []
    assert solution._receive_detections(color_detections(image))['traffic_light']['color'] == 'unknown'
    # 아래쪽 점선이 있어도 위쪽의 초록 신호를 가리지 않아야 출발할 수 있습니다.
    cv2.circle(image, (205, 52), 11, (0, 255, 0), -1)
    result = solution._receive_detections(color_detections(image))
    assert result['traffic_light']['color'] == 'green'


def test_missing_yolo_weights_fails_before_network_or_model_load(tmp_path):
    with pytest.raises(ValueError, match='weights'):
        YoloDetector(str(tmp_path / 'missing.pt'))


def test_red_approach_slows_before_full_stop():
    controller = solution.DrivingController()
    lane, cone = {'valid': True, 'error': 0.0}, {'valid': False}
    cruise = controller._step(lane, cone, perception('red', .04), .05)
    approach = controller._step(lane, cone, perception('red', .06), .05)
    assert 0 < approach[1] < cruise[1]
    assert controller._step(lane, cone, perception('red', .10), .05) == [0, 0]


def test_latest_input_expires_at_watchdog_threshold():
    latest = contracts.Latest()
    latest.receive({'valid': True}, 10)
    assert latest.current(10.79) == {'valid': True}
    assert latest.current(10.8) is None


def scan_distances():
    from types import SimpleNamespace
    from session_4.direct_runtime import distances
    r, minimum, step, low, high = scan_cones()
    return distances(SimpleNamespace(ranges=r, angle_min=minimum, angle_increment=step,
                                     range_min=low, range_max=high))


def test_simple_public_functions_and_three_result_controller():
    result = solution.rubbercone(scan_distances())
    assert result['valid'] and abs(result['error']) < .04
    signal = solution.yolo(['red', 'cone'], [[120, 40], [200, 100]], [[20, 20], [30, 60]], [.95, .88])
    assert signal['traffic_light']['color'] == 'red' and len(signal['rubbercone']['bbox']) == 1
    assert signal['traffic_light']['size'] == [20, 20]
    assert solution.DrivingController().step({'valid': True, 'error': 0}, {'valid': False},
        contracts.no_perception())[1] > 0
