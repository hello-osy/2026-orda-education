import pytest

from session_3.auto_center import StartupCenter
from session_3.motor_core import SteeringControl


def feed(gate, start, count=41, raw=501, step=.05):
    results = []
    for i in range(count):
        result = gate.receive(raw if isinstance(raw, int) else raw[i % len(raw)], start+i*step)
        if result is not None:
            results.append(result)
    return results


def test_boot_frames_do_not_register_center_before_full_post_boot_window():
    gate = StartupCenter(opened_at=100)
    assert feed(gate, 100, count=200) == []
    assert feed(gate, 110, count=40) == []
    assert gate.receive(501, 112) == 501
    assert gate.center == 501


def test_median_registration_is_once_and_not_changed_by_later_steering():
    gate = StartupCenter(0)
    assert feed(gate, 10, raw=[501, 502, 501, 503]) == [501]
    assert feed(gate, 12.05, count=100, raw=560) == []
    assert gate.center == 501


def test_motion_and_outliers_restart_the_continuous_stable_window():
    gate = StartupCenter(0, startup_wait=0)
    for i in range(81):
        assert gate.receive(400+i, i*.05) is None
    assert feed(gate, 4.05, count=20, raw=501) == []
    assert gate.receive(510, 5.05) is None
    assert feed(gate, 5.1, count=40, raw=501) == []
    assert gate.receive(501, 7.1) == 501


def test_feedback_gap_discards_old_stable_samples():
    gate = StartupCenter(0, startup_wait=0)
    assert feed(gate, 0, count=30) == []
    assert feed(gate, 2.1, count=40) == []
    assert gate.receive(501, 4.1) == 501


def test_buffered_burst_and_sparse_frames_cannot_fake_settling():
    gate = StartupCenter(0, startup_wait=0)
    assert feed(gate, 0, count=100, step=0) == []
    # 2초 동안 5개만 받은 경우는 최소 8프레임 조건을 만족하지 않는다.
    sparse = StartupCenter(0, startup_wait=0)
    assert feed(sparse, 0, count=10, step=.5) == []
    assert sparse.center is None


@pytest.mark.parametrize('wait', [-1, float('nan'), float('inf')])
def test_invalid_startup_wait_is_rejected(wait):
    with pytest.raises(ValueError):
        StartupCenter(0, startup_wait=wait)


def test_register_median_clears_old_targets_and_uses_latest_raw_for_angle():
    control = SteeringControl(enabled=True, center=512)
    control.receive(502, 20)
    control.heartbeat_stamp = 20
    control.set_target([10, 0], 20)
    control.command(20)
    control.register_center(501)
    assert control.center == 501
    assert control.angle == pytest.approx(-45/80)
    assert control.target is None and control.request_stamp is None and control.last_requested is None
    assert control.motion_sign is None and control.motion_started is None
    assert control.command(20) == b'0 0\n'
    control.set_target([10, 0], 20.1)
    assert control.command(20.1) == b'150 0\n'
