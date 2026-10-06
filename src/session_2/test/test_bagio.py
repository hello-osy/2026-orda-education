import csv
from pathlib import Path
import time

import numpy as np
from rosbags.rosbag2 import Reader
from session_2.bagio import BagArchive,BagWriter,CAMERA_TOPICS,Recorder,SCAN_TOPIC,extract_frames


def make_bag(path,frames=12):
    writer=BagWriter(path,[*CAMERA_TOPICS,SCAN_TOPIC])
    try:
        for i in range(frames):
            stamp=1_700_000_000_000_000_000+i*100_000_000
            for j,topic in enumerate(CAMERA_TOPICS):
                image=np.full((48,64,3),i+j*80,dtype=np.uint8)
                writer.write(topic,stamp+j*10_000_000,image)
            writer.write(SCAN_TOPIC,stamp,np.array([[10,0,1000],[20,90,2000],[0,10,0]]))
    finally:writer.close()
    return path


def test_standard_bag_roundtrip_and_seek(tmp_path):
    path=make_bag(tmp_path/'recording')
    with Reader(path) as reader:
        assert reader.message_count==36
        assert {c.msgtype for c in reader.connections}=={'sensor_msgs/msg/Image','sensor_msgs/msg/LaserScan'}
    bag=BagArchive(path)
    try:
        snap=bag.snapshot(.55,[*CAMERA_TOPICS,SCAN_TOPIC])
        assert snap[CAMERA_TOPICS[0]]['index']==5
        assert int(snap[CAMERA_TOPICS[1]]['data'][0,0,0])==85
        scan=snap[SCAN_TOPIC]['data']
        assert scan.shape==(2,3)
        assert sorted(scan[:,2])==[1000,2000]
        assert 90 in scan[:,1]  # SLLIDAR bearing +90 -> A1 raw +90.
        assert CAMERA_TOPICS[1] not in bag.snapshot(0,CAMERA_TOPICS)
        assert bag.snapshot(0,[CAMERA_TOPICS[0]])[CAMERA_TOPICS[0]]['index']==0
    finally:bag.close()


def test_extract_every_fifth_per_camera_and_no_overwrite(tmp_path):
    import pytest
    path=make_bag(tmp_path/'bag')
    output=tmp_path/'images'
    counts=extract_frames(path,output,CAMERA_TOPICS)
    assert list(counts.values())==[2,2]
    with (output/'manifest.csv').open() as f: rows=list(csv.DictReader(f))
    assert [int(row['frame_1based']) for row in rows]==[5,10,5,10]
    assert len(list(output.rglob('*.jpg')))==4
    with pytest.raises(FileExistsError):extract_frames(path,output,CAMERA_TOPICS)


def test_recorder_drains_queue_and_finalizes_metadata(tmp_path):
    r=Recorder(tmp_path/'bag',[CAMERA_TOPICS[0]])
    assert r.ready.wait(3)
    for _ in range(6):r.submit(CAMERA_TOPICS[0],time.monotonic(),np.zeros((8,8,3),np.uint8))
    r.stop();r.thread.join(5)
    assert not r.thread.is_alive() and not r.error
    assert r.counts[CAMERA_TOPICS[0]]==6
    with Reader(r.path) as reader:assert reader.message_count==6


def test_capture_with_korean_and_spaces_in_path(tmp_path):
    import cv2
    import json
    path = make_bag(tmp_path/'수업 녹화')
    output = tmp_path/'학습 사진'
    counts = extract_frames(path, output, CAMERA_TOPICS)
    for file in output.rglob('*.jpg'):
        decoded = cv2.imdecode(np.frombuffer(file.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        assert decoded.shape == (48, 64, 3)
    with (output/'manifest.csv').open(encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    assert all('수업 녹화' in row['source_bag'] for row in rows)
    assert json.loads((output/'summary.json').read_text(encoding='utf-8'))['counts'] == counts


def test_competition_scan_cardinal_directions():
    from types import SimpleNamespace
    from session_2.bagio import decode_scan
    from session_2.sensors import scan_xy
    # 대회 LaserScan: 앞 -180, 왼쪽 -90, 뒤 0, 오른쪽 +90.
    msg = SimpleNamespace(angle_min=-np.pi, angle_increment=np.pi/2,
                          ranges=[1., 2., 3., 4.], range_min=.15, range_max=12.)
    np.testing.assert_allclose(scan_xy(decode_scan(msg), 6),
                               [[0, -1], [-2, 0], [0, 3], [4, 0]], atol=1e-6)


def test_recorded_scan_matches_competition_bearings_and_live_view(tmp_path):
    from session_2.bagio import SCAN_TYPE
    from session_2.sensors import scan_xy
    from rosbags.typesys import Stores, get_typestore
    raw = np.array([[10, 0, 1000], [10, 90, 2000],
                    [10, 180, 3000], [10, 270, 4000]])
    writer = BagWriter(tmp_path/'directions', [SCAN_TOPIC])
    writer.write(SCAN_TOPIC, 1_700_000_000_000_000_000, raw)
    writer.close()
    with Reader(tmp_path/'directions') as reader:
        _, _, data = next(reader.messages())
        msg = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(data, SCAN_TYPE)
        np.testing.assert_allclose(msg.ranges[[0, 90, 180, 270]], [3, 2, 1, 4])
    bag = BagArchive(tmp_path/'directions')
    try:
        _, replay = bag.read(SCAN_TOPIC, 0)
        # 거리별로 대응시켜 순서와 무관하게 실시간/재생 좌표를 비교한다.
        replay = replay[np.argsort(replay[:, 2])]
        np.testing.assert_allclose(scan_xy(replay, 6), scan_xy(raw, 6), atol=1e-6)
    finally:
        bag.close()
