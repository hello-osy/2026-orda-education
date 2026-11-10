"""수동·자동 명령이 섞이지 않고, 키를 놓거나 명령이 끊기면 정지하는지 확인한다."""
import sqlite3
import pytest
from session_4.manual_drive import ManualKeys
from session_4.sim_control_node import ControlSelector
from session_4.recording import bag_summary


def test_only_selected_source_can_drive_and_switch_clears_previous_commands():
    control = ControlSelector()
    control.receive('manual', [0, 200], 10)
    assert control.status(10)['command'] == [0, 0]
    control.receive('auto', [10, 125], 10)
    assert control.status(10.1)['command'] == [10, 125]
    control.set_mode('manual')
    assert control.status(10.1)['command'] == [0, 0]
    control.receive('auto', [0, 255], 10.2)
    assert control.status(10.2)['command'] == [0, 0]
    control.receive('manual', [-18, -100], 10.2)
    assert control.status(10.3)['command'] == [-18, -100]
    assert control.status(10.8)['command'] == [0, 0]
    control.set_mode('auto')
    assert control.status(10.8)['command'] == [0, 0]


def test_invalid_manual_input_stops_and_invalid_mode_does_not_change_selection():
    control = ControlSelector('manual')
    control.receive('manual', [0, 120], 10)
    with pytest.raises(ValueError):
        control.receive('manual', [50, 120], 10.1)
    assert control.status(10.1)['command'] == [0, 0]
    with pytest.raises(ValueError):
        control.set_mode('something_else')
    assert control.mode == 'manual'


def test_manual_key_release_emergency_stop_and_disabled_input():
    keys = ManualKeys()
    keys.pressed.update(['forward', 'left'])
    assert keys.command() == [0, 0]
    keys.enabled = True
    assert keys.command() == [18, 120]
    keys.pressed.discard('forward')
    assert keys.command()[1] == 0
    keys.clear()
    assert keys.command() == [0, 0]
    keys.pressed.update(['forward', 'back'])
    assert keys.command()[1] == 0
    keys.enabled = False
    assert keys.command() == [0, 0]


def test_only_completed_camera_recording_is_usable_for_extract(tmp_path):
    with pytest.raises(ValueError, match='metadata'):
        bag_summary(tmp_path)
    (tmp_path / 'metadata.yaml').write_text('test fixture')
    with sqlite3.connect(tmp_path / 'data.db3') as db:
        db.execute('CREATE TABLE topics(id INTEGER,name TEXT)')
        db.execute('CREATE TABLE messages(id INTEGER,topic_id INTEGER)')
        db.execute("INSERT INTO topics VALUES(1,'/scan')")
        db.execute('INSERT INTO messages VALUES(1,1)')
    with pytest.raises(ValueError, match='카메라'):
        bag_summary(tmp_path)
    with sqlite3.connect(tmp_path / 'data.db3') as db:
        db.execute("INSERT INTO topics VALUES(2,'/camera/high/image_raw')")
        db.executemany('INSERT INTO messages VALUES(?,2)', [(2,), (3,), (4,)])
    assert bag_summary(tmp_path)['/camera/high/image_raw'] == 3
