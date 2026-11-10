import math

import numpy as np

from session_4.debug_viewer import detection_overlay, scan_points, visible_boxes


def test_boxes_scale_clip_and_ignore_non_finite_or_reversed_coordinates():
    packet = {'width': 640, 'height': 480, 'detections': [
        {'label': 'red', 'confidence': .9, 'bbox': [-20, 40, 80, 100]},
        {'label': 'cone', 'confidence': .8, 'bbox': [620, 400, 700, 500]},
        {'label': 'green', 'confidence': .8, 'bbox': [100, 100, 90, 110]},
        {'label': 'yellow', 'confidence': .9, 'bbox': [0, math.nan, 20, 40]},
        {'label': 'red', 'confidence': math.inf, 'bbox': [0, 0, 20, 40]},
    ]}
    assert visible_boxes(packet, 320, 240) == [
        ('red', .9, (0, 20, 40, 50)), ('cone', .8, (310, 200, 319, 239))]
    assert visible_boxes(dict(packet, valid=False), 320, 240) == []
    assert visible_boxes(dict(packet, width=0), 320, 240) == []


def test_overlay_does_not_change_original_rgb_or_draw_stale_packet():
    rgb = np.full((100, 100, 3), 35, np.uint8)
    packet = {'width': 100, 'height': 100, 'detections': [
        {'label': 'cone', 'confidence': .7, 'bbox': [20, 25, 60, 80]}]}
    output = detection_overlay(rgb, packet)
    assert np.all(rgb == 35)
    assert np.any(output != rgb)
    assert np.array_equal(detection_overlay(rgb, None), rgb)


def test_scan_coordinates_preserve_left_positive_and_ignore_invalid_ranges():
    points = scan_points([1, math.inf, 2, math.nan, .1, 8], 0, math.pi / 4, .15, 12)
    assert len(points) == 2
    assert points[0] == (1.0, 0.0)
    assert abs(points[1][0]) < 1e-10 and points[1][1] == 2.0
    assert scan_points([1], math.nan, .1, .15, 12) == []
