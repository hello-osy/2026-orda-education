import math
from types import SimpleNamespace

import numpy as np
import pytest

from session_3.image_utils import image_to_rgb
from session_3.sensor_utils import scan_ranges


def image(encoding, width, height, step, data):
    return SimpleNamespace(encoding=encoding, width=width, height=height, step=step, data=bytes(data))


def test_bgr_rows_honor_padding_and_rgb_order():
    message = image('bgr8', 1, 2, 4, [10, 20, 30, 255, 40, 50, 60, 255])
    assert image_to_rgb(message).tolist() == [[[30, 20, 10]], [[60, 50, 40]]]


def test_mono_and_rgb():
    assert image_to_rgb(image('mono8', 1, 1, 1, [80])).tolist() == [[[80, 80, 80]]]
    assert image_to_rgb(image('rgb8', 1, 1, 3, [20, 30, 40])).tolist() == [[[20, 30, 40]]]


def test_yuy2_with_padding():
    message = image('yuv422_yuy2', 2, 1, 6, [16, 128, 235, 128, 0, 0])
    rgb = image_to_rgb(message)
    assert rgb.shape == (1, 2, 3)
    assert rgb[0, 0].max() <= 1
    assert rgb[0, 1].min() >= 254


@pytest.mark.parametrize('message', [
    image('bgr8', 2, 1, 5, [0] * 5), image('rgb8', 1, 2, 3, [0] * 3),
    image('yuv422_yuy2', 1, 1, 2, [0, 0]), image('bad', 1, 1, 1, [0]),
])
def test_invalid_image_is_rejected(message):
    with pytest.raises(ValueError):
        image_to_rgb(message)


def test_scan_axes_match_sim_forward_and_counterclockwise():
    ranges = scan_ranges([(5, 0, 1000), (5, 90, 2000), (5, 180, 3000), (5, 270, 4000)])
    assert ranges[180] == 1.0  # 앞
    assert ranges[90] == 2.0   # 오른쪽
    assert ranges[0] == 3.0    # 뒤
    assert ranges[270] == 4.0  # 왼쪽
    assert math.isinf(ranges[1])


def test_scan_invalid_returns_inf_and_duplicate_uses_nearest():
    ranges = scan_ranges([(1, 0, 1000), (1, 0, 500), (0, 90, 1000),
                          (1, 180, 100), (1, 270, 13000), (1, float('nan'), 2000)])
    assert ranges[180] == 0.5
    assert sum(math.isfinite(x) for x in ranges) == 1


def test_scan_mount_rotation_and_wrap():
    ranges = scan_ranges([(1, 0, 1000), (1, 360, 500)], mount_yaw_deg=90)
    assert ranges[270] == 0.5
    assert len(scan_ranges([])) == 360
