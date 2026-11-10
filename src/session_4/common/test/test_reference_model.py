"""공유할 정답 모델의 누락·손상과 속도 변환을 검사한다."""
import hashlib
import json
import math
from session_4.yolo_node import reference_weights
from session_4.sim_control_node import motor_to_velocity


def test_reference_model_matches_recorded_artifact():
    path = reference_weights()
    metadata = json.loads((path.parent / 'training_info.json').read_text())
    assert path.stat().st_size > 1_000_000
    assert hashlib.sha256(path.read_bytes()).hexdigest() == metadata['sha256']
    assert metadata['classes'] == ['red', 'yellow', 'green', 'cone']


def test_faster_mapping_preserves_turn_radius():
    speed, yaw = motor_to_velocity([18, 255])
    assert speed == 2.0
    assert math.isclose(speed / yaw, .48 / math.tan(math.radians(18)))
